import os
import time
import shutil
import threading
import numpy as np
import cv2
from ultralytics import YOLO

try:
    import rclpy
    from std_msgs.msg import String
except ImportError:
    raise SystemExit("rclpy not found. Source ROS 2 first: cd ~/corobo_ws && source install/setup.bash")

# ---------------- CONFIG ----------------
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PT = os.path.join(HERE, "yolov8n-drone.pt")
MODEL_PT_ROI = os.path.join(HERE, "yolov8n-drone-roi.pt")   # auto-copy of the same weights, exported at ROI size

CAMERA_SOURCE = 0
CAP_WIDTH = 640
CAP_HEIGHT = 480

CONF_ROI = 0.15           # confidence inside the ROI (gating protects against false hits)
LOCK_CONF = 0.40          # confidence needed on a full-frame search to start a lock
LOCK_CONFIRM = 2          # consecutive full-frame detections needed to lock
IMG_SIZE = 480            # full-frame search size
ROI_SIZE = 320            # ROI square in camera pixels (multiple of 32); also the ROI model input size
LOST_LIMIT = 4            # ROI misses in a row before going back to full-frame search
GATE_PX = 100             # max distance (px) between a detection and the predicted position
INFER_INTERVAL = 0.05     # seconds to rest between detections

KF_POS_NOISE = 2.0        # Kalman: trust in the motion model for position
KF_VEL_NOISE = 200.0      # Kalman: how fast the drone's speed can change (higher = more agile)
KF_MEAS_NOISE = 16.0      # Kalman: detection noise (higher = smoother dot, slower reaction)

MOVE_SPEED = 50           # px/s to count as movement
STOP_FRAMES = 4           # consecutive still detections before printing STOP
PRINT_INTERVAL = 0.3      # seconds between repeated prints of the same direction
DOT_TTL = 1.0             # seconds to keep drawing the dot after the last update
SHOW_ROI = True           # draw the thin yellow ROI square while tracking
DRONE_POS_TOPIC = "drone_pos"   # ROS 2 topic (std_msgs/String) for LEFT/RIGHT/UP/DOWN/STOP/LOST
# -----------------------------------------

state = {
    "frame": None, "frame_id": 0,
    "pos": None, "vel": (0.0, 0.0), "pos_time": 0.0, "conf": 0.0,
    "status": "SEARCH", "roi": None, "det_seq": 0,
    "infer_ms": 0.0, "running": True,
}
lock = threading.Lock()


def publish_motion(pub, label):
    msg = String()
    msg.data = label
    pub.publish(msg)


def load_model(pt_path, imgsz):
    ov_dir = pt_path.replace(".pt", "_openvino_model")
    try:
        if not os.path.isdir(ov_dir):
            print(f"First run: exporting {os.path.basename(pt_path)} at {imgsz}px (about a minute)...")
            YOLO(pt_path).export(format="openvino", imgsz=imgsz)
        return YOLO(ov_dir, task="detect")
    except Exception as e:
        print(f"OpenVINO failed ({e}); falling back to PyTorch.")
        return YOLO(pt_path)


def load_models():
    if not os.path.isfile(MODEL_PT):
        raise FileNotFoundError(f"Model not found: {MODEL_PT}")
    if not os.path.isfile(MODEL_PT_ROI):
        shutil.copyfile(MODEL_PT, MODEL_PT_ROI)
    return load_model(MODEL_PT, IMG_SIZE), load_model(MODEL_PT_ROI, ROI_SIZE)


def open_camera():
    cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera/video source: {CAMERA_SOURCE}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def kf_create(cx, cy):
    """Constant-velocity Kalman filter. State = [x, y, vx, vy], measurement = [x, y]."""
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
    kf.processNoiseCov = np.diag(
        [KF_POS_NOISE, KF_POS_NOISE, KF_VEL_NOISE, KF_VEL_NOISE]).astype(np.float32)
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * KF_MEAS_NOISE
    kf.errorCovPost = np.eye(4, dtype=np.float32) * 50.0
    kf.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
    kf.transitionMatrix = np.eye(4, dtype=np.float32)
    return kf


def kf_predict(kf, dt):
    """Advance the filter by dt seconds. Returns (x, y, vx, vy)."""
    F = np.eye(4, dtype=np.float32)
    F[0, 2] = dt
    F[1, 3] = dt
    kf.transitionMatrix = F
    s = kf.predict()
    return float(s[0, 0]), float(s[1, 0]), float(s[2, 0]), float(s[3, 0])


def kf_update(kf, cx, cy):
    """Correct the filter with a detection. Returns (x, y, vx, vy)."""
    s = kf.correct(np.array([[cx], [cy]], dtype=np.float32))
    return float(s[0, 0]), float(s[1, 0]), float(s[2, 0]), float(s[3, 0])


def clamp_roi(cx, cy, size, w, h):
    """Square ROI of `size` centered on (cx, cy), kept inside the frame."""
    size = min(size, w, h)
    half = size // 2
    x1 = int(min(max(cx - half, 0), w - size))
    y1 = int(min(max(cy - half, 0), h - size))
    return x1, y1, x1 + size, y1 + size


def detect_pick(model, image, conf, imgsz, ref=None):
    """Return (cx, cy, conf). Picks the most confident box, or the one nearest `ref` if given."""
    results = model.predict(image, conf=conf, imgsz=imgsz, verbose=False)
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    cxs = (xyxy[:, 0] + xyxy[:, 2]) / 2
    cys = (xyxy[:, 1] + xyxy[:, 3]) / 2
    if ref is None:
        i = int(np.argmax(confs))
    else:
        i = int(np.argmin((cxs - ref[0]) ** 2 + (cys - ref[1]) ** 2))
    return float(cxs[i]), float(cys[i]), float(confs[i])


def classify_motion(vx, vy, threshold):
    """LEFT / RIGHT / UP / DOWN / STOP from velocity in px/s (y grows downward)."""
    if abs(vx) < threshold and abs(vy) < threshold:
        return "STOP"
    if abs(vx) >= abs(vy):
        return "RIGHT" if vx > 0 else "LEFT"
    return "DOWN" if vy > 0 else "UP"


def capture_loop(cap):
    fails = 0
    while state["running"]:
        ret, frame = cap.read()
        if ret:
            fails = 0
            with lock:
                state["frame"] = frame
                state["frame_id"] += 1
            continue

        fails += 1
        print(f"Frame grab failed ({fails}/10); reconnecting camera...")
        cap.release()
        time.sleep(1.0)
        try:
            cap = open_camera()
        except RuntimeError as e:
            print(e)
        if fails >= 10:
            print("Camera did not recover. Re-attach the webcam in VirtualBox and restart.")
            state["running"] = False
            break


def infer_loop(model_full, model_roi):
    last_id = -1
    kf = None
    last_t = 0.0
    misses = 0
    cand = None
    cand_count = 0

    while state["running"]:
        with lock:
            frame = state["frame"]
            fid = state["frame_id"]
        if frame is None or fid == last_id:
            time.sleep(0.005)
            continue
        last_id = fid
        h, w = frame.shape[:2]

        t0 = time.time()
        status = "SEARCH"
        est = None          # (x, y, vx, vy) published to the display thread
        roi = None
        conf = 0.0

        if kf is not None:
            # Tracking: predict, crop around the prediction, run the small model there
            dt = min(t0 - last_t, 0.5)
            last_t = t0
            est = kf_predict(kf, dt)
            px, py = est[0], est[1]
            roi = clamp_roi(px, py, ROI_SIZE, w, h)
            rx1, ry1, rx2, ry2 = roi
            d = detect_pick(model_roi, frame[ry1:ry2, rx1:rx2].copy(),
                            CONF_ROI, ROI_SIZE, ref=(px - rx1, py - ry1))
            hit = False
            if d is not None:
                mx, my = d[0] + rx1, d[1] + ry1
                if (mx - px) ** 2 + (my - py) ** 2 <= GATE_PX ** 2:
                    est = kf_update(kf, mx, my)
                    conf = d[2]
                    hit = True
            if hit:
                misses = 0
                status = "TRACK"
            else:
                misses += 1
                if misses >= LOST_LIMIT:
                    kf = None
                    est = None
                    roi = None
                    misses = 0
                    status = "SEARCH"
                else:
                    status = "HOLD"      # coasting on the prediction
        else:
            # Searching: full-frame detection; lock only after LOCK_CONFIRM matches in a row
            d = detect_pick(model_full, frame, LOCK_CONF, IMG_SIZE)
            if d is None:
                cand = None
                cand_count = 0
            else:
                near = cand is not None and \
                    (d[0] - cand[0]) ** 2 + (d[1] - cand[1]) ** 2 <= GATE_PX ** 2
                cand_count = cand_count + 1 if near else 1
                cand = (d[0], d[1])
                if cand_count >= LOCK_CONFIRM:
                    kf = kf_create(d[0], d[1])
                    last_t = t0
                    est = (d[0], d[1], 0.0, 0.0)
                    conf = d[2]
                    status = "TRACK"
                    cand = None
                    cand_count = 0

        infer_ms = (time.time() - t0) * 1000
        with lock:
            if est is not None and status in ("TRACK", "HOLD"):
                state["pos"] = (est[0], est[1])
                state["vel"] = (est[2], est[3])
                state["pos_time"] = t0
                if status == "TRACK":
                    state["conf"] = conf
            state["status"] = status
            state["roi"] = roi
            state["det_seq"] += 1
            state["infer_ms"] = infer_ms

        time.sleep(INFER_INTERVAL)


def main():
    model_full, model_roi = load_models()
    cap = open_camera()

    rclpy.init()
    node = rclpy.create_node("drone_tracker")
    pub = node.create_publisher(String, DRONE_POS_TOPIC, 10)
    print(f"ROS 2 node drone_tracker publishing on /{DRONE_POS_TOPIC}")

    threading.Thread(target=capture_loop, args=(cap,), daemon=True).start()
    threading.Thread(target=infer_loop, args=(model_full, model_roi), daemon=True).start()

    stop_counter = 0
    last_seq = -1
    last_shown_id = -1
    was_lost = False
    last_label = None
    last_print = 0.0

    print("Running. Press 'q' in the video window to quit.\n")

    while state["running"]:
        with lock:
            frame = state["frame"]
            fid = state["frame_id"]
            pos = state["pos"]
            vel = state["vel"]
            pos_time = state["pos_time"]
            conf = state["conf"]
            status = state["status"]
            roi = state["roi"]
            seq = state["det_seq"]
            infer_ms = state["infer_ms"]

        if frame is None:
            time.sleep(0.01)
            continue

        now = time.time()

        # Motion logic runs only when a NEW inference result arrives
        if seq != last_seq:
            last_seq = seq
            if status == "TRACK":
                was_lost = False
                motion = classify_motion(vel[0], vel[1], MOVE_SPEED)
                label = None
                if motion == "STOP":
                    stop_counter += 1
                    if stop_counter >= STOP_FRAMES:
                        label = "STOP"
                else:
                    stop_counter = 0
                    label = motion
                if label is not None and (label != last_label or now - last_print >= PRINT_INTERVAL):
                    print(f"Drone motion: {label}")
                    publish_motion(pub, label)
                    last_label = label
                    last_print = now
            elif status == "SEARCH":
                if not was_lost:
                    print("Drone motion: LOST")
                    publish_motion(pub, "LOST")
                    was_lost = True
                    last_label = "LOST"
                stop_counter = 0
            # HOLD: drone unseen for a moment; coasting, print nothing

        # Redraw only when a new camera frame has arrived
        if fid == last_shown_id:
            if cv2.waitKey(5) & 0xFF == ord("q"):
                break
            continue
        last_shown_id = fid

        frame = frame.copy()

        if SHOW_ROI and roi is not None and status != "SEARCH":
            cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 255), 1)

        if pos is not None and status in ("TRACK", "HOLD") and (now - pos_time) < DOT_TTL:
            lead = min(now - pos_time, 0.3)          # draw slightly ahead to hide model delay
            dx = pos[0] + vel[0] * lead
            dy = pos[1] + vel[1] * lead
            color = (0, 0, 255) if status == "TRACK" else (0, 255, 255)
            cv2.circle(frame, (int(dx), int(dy)), 5, color, -1)
            cv2.circle(frame, (int(dx), int(dy)), 12, (0, 255, 0), 1)
            if status == "TRACK":
                cv2.putText(frame, f"{conf:.2f}", (int(dx) + 15, int(dy) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(frame, f"{status}  infer: {infer_ms:.0f} ms  v=({vel[0]:.0f},{vel[1]:.0f}) px/s",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow("Drone Detection & Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    state["running"] = False
    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
