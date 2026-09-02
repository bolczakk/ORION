"""
HMI ORION - podglad z kamery OAK-D Pro w przegladarce.
YOLO (on-device) + glebia przestrzenna + IMU overlay.

Uruchomienie:
    python hmi_server.py
Potem w przegladarce:  http://localhost:8000
(z innego urzadzenia w sieci:  http://<ip-hosta>:8000)
"""

import threading
import time
import math

import cv2
import numpy as np
import depthai as dai
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse

# ---------------------------------------------------------------------------
# Wspoldzielony stan
# ---------------------------------------------------------------------------
_latest_frame = None
_frame_lock = threading.Lock()
_camera_running = True

PINK = (203, 0, 255)
WHITE = (255, 255, 255)

MODEL_PATH = r"C:\Users\micha\Desktop\ProjektORION\Sekcje\AI_wizja\.depthai_cached_models\98fa008f114a052fc110a400b9f2526e541e7c72\YOLOv6_Nano-R2_COCO_512x288.rvc2.tar.xz"


def quat_to_euler(i, j, k, w):
    sinr = 2.0 * (w * i + j * k)
    cosr = 1.0 - 2.0 * (i * i + j * j)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * j - k * i)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * k + i * j)
    cosy = 1.0 - 2.0 * (j * j + k * k)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def camera_worker():
    global _latest_frame, _camera_running

    modelArchive = dai.NNArchive(MODEL_PATH)

    NN_INPUT_SIZE = (640, 400)
    STEREO_FPS = 15

    with dai.Pipeline() as pipeline:
        # --- Kamery ---
        camRgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A, sensorFps=STEREO_FPS
        )
        monoLeft = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B, sensorFps=STEREO_FPS
        )
        monoRight = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, sensorFps=STEREO_FPS
        )

        # --- Stereo Depth ---
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setExtendedDisparity(True)
        monoLeft.requestOutput(NN_INPUT_SIZE).link(stereo.left)
        monoRight.requestOutput(NN_INPUT_SIZE).link(stereo.right)

        # --- Spatial Detection Network ---
        spatialNN = pipeline.create(dai.node.SpatialDetectionNetwork).build(
            camRgb, stereo, modelArchive
        )
        spatialNN.input.setBlocking(False)
        spatialNN.setConfidenceThreshold(0.4)
        spatialNN.setDepthLowerThreshold(100)
        spatialNN.setDepthUpperThreshold(10000)

        q_rgb = spatialNN.passthrough.createOutputQueue()
        q_det = spatialNN.out.createOutputQueue()

        # --- IMU ---
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)
        q_imu = imu.out.createOutputQueue()

        pipeline.start()
        print("[kamera] pipeline wystartowal (YOLO + depth + IMU)")

        imu_text = ["IMU: brak danych"]
        detections = []

        while _camera_running and pipeline.isRunning():
            in_rgb = q_rgb.tryGet()
            in_det = q_det.tryGet()

            if in_rgb is None:
                time.sleep(0.001)
                continue

            frame = in_rgb.getCvFrame()
            h, w, _ = frame.shape

            # --- Detekcje z głębią ---
            if in_det is not None:
                detections = in_det.detections

            for detection in detections:
                x1 = int(detection.xmin * w)
                y1 = int(detection.ymin * h)
                x2 = int(detection.xmax * w)
                y2 = int(detection.ymax * h)

                label = detection.labelName
                confidence = detection.confidence * 100
                sz = int(detection.spatialCoordinates.z)

                cv2.rectangle(frame, (x1, y1), (x2, y2), WHITE, 2)
                cv2.putText(frame, f"{label} {confidence:.0f}%",
                            (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
                cv2.putText(frame, f"Z: {sz/1000:.2f}m",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, PINK, 1)

            # --- IMU ---
            imu_data = q_imu.tryGet()
            if imu_data:
                packets = imu_data.packets
                if packets:
                    p = packets[-1]
                    acc = p.acceleroMeter
                    gyro = p.gyroscope
                    rot = p.rotationVector
                    roll, pitch, yaw = quat_to_euler(rot.i, rot.j, rot.k, rot.real)
                    imu_text = [
                        f"ACC  [m/s2]: x={acc.x:+7.2f}  y={acc.y:+7.2f}  z={acc.z:+7.2f}",
                        f"GYRO [rad/s]: x={gyro.x:+7.3f}  y={gyro.y:+7.3f}  z={gyro.z:+7.3f}",
                        f"ORIENT [deg]: roll={roll:+7.1f}  pitch={pitch:+7.1f}  yaw={yaw:+7.1f}",
                    ]

            for i, line in enumerate(imu_text):
                cv2.putText(frame, line, (10, 25 + i * 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, PINK, 1, cv2.LINE_AA)

            frame = cv2.resize(frame, (1280, 720))
            
            # --- Kodowanie JPEG i zapis ---
            ok, encoded = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _frame_lock:
                    _latest_frame = encoded.tobytes()

    print("[kamera] pipeline zatrzymany")


# ---------------------------------------------------------------------------
# Serwer FastAPI
# ---------------------------------------------------------------------------
app = FastAPI(title="ORION HMI")


@app.on_event("startup")
def start_camera():
    t = threading.Thread(target=camera_worker, daemon=True)
    t.start()
    print("[serwer] watek kamery uruchomiony")


@app.on_event("shutdown")
def stop_camera():
    global _camera_running
    _camera_running = False


def mjpeg_generator():
    while True:
        with _frame_lock:
            frame = _latest_frame

        if frame is None:
            time.sleep(0.05)
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

        time.sleep(1 / 30)


@app.get("/video")
def video_feed():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/", response_class=HTMLResponse)
def index():
    return """<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="utf-8">
  <title>ORION HMI</title>
  <style>
    body { margin: 0; background: #14171c; color: #d6dae0;
           font-family: monospace; display: flex; flex-direction: column;
           align-items: center; min-height: 100vh; }
    header { padding: 16px; letter-spacing: 2px; font-size: 14px;
             text-transform: uppercase; color: #7d8896; }
    .frame { border: 1px solid #2a2f38; background: #000;
             box-shadow: 0 8px 40px rgba(0,0,0,.6); }
    .frame img { display: block; max-width: 90vw; height: auto; }
    footer { padding: 12px; font-size: 12px; color: #4d5562; }
  </style>
</head>
<body>
  <header>ORION &middot; YOLO + Depth + IMU &middot; podglad na zywo</header>
  <div class="frame">
    <img src="/video" alt="strumien z kamery">
  </div>
  <footer>MJPEG &middot; FastAPI &middot; DepthAI &middot; SpatialDetectionNetwork</footer>
</body>
</html>"""


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)