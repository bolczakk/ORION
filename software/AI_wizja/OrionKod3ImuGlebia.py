#!/usr/bin/env python3

import cv2
import math
import numpy as np
import depthai as dai


def quat_to_euler(i, j, k, w):
    """Quaternion -> roll, pitch, yaw w stopniach."""
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


PINK = (203, 0, 255)
WHITE = (255, 255, 255)

STEREO_FPS = 15
NN_INPUT_SIZE = (640, 400)

MODEL_PATH = r"C:\Users\micha\Desktop\ProjektORION\Sekcje\AI_wizja\.depthai_cached_models\98fa008f114a052fc110a400b9f2526e541e7c72\YOLOv6_Nano-R2_COCO_512x288.rvc2.tar.xz"
modelArchive = dai.NNArchive(MODEL_PATH)

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

    # --- Spatial Detection Network (YOLO on-device + depth) ---
    spatialNN = pipeline.create(dai.node.SpatialDetectionNetwork).build(
        camRgb, stereo, modelArchive
    )
    spatialNN.input.setBlocking(False)
    spatialNN.setConfidenceThreshold(0.4)
    spatialNN.setDepthLowerThreshold(100)    # min 10 cm
    spatialNN.setDepthUpperThreshold(10000)  # max 10 m

    # output queues
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

    imu_text = ["IMU: brak danych"]

    imu_text = ["IMU: brak danych"]
    detections = []  # zapamiętane detekcje

    while pipeline.isRunning():
        in_rgb = q_rgb.tryGet()
        in_det = q_det.tryGet()


        if in_rgb is None:
            continue

        frame = in_rgb.getCvFrame()
        h, w, _ = frame.shape

        # --- Aktualizuj detekcje tylko gdy przyjdą nowe ---
        if in_det is not None:
            detections = in_det.detections

        # --- Rysuj ostatnie znane detekcje ---
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

        # # --- Mapa głębi (debug) ---
        # if in_depth is not None:
        #     depth_frame = in_depth.getCvFrame()
        #     depth_down = depth_frame[::4]
        #     if np.all(depth_down == 0):
        #         min_d = 0
        #     else:
        #         min_d = np.percentile(depth_down[depth_down != 0], 1)
        #     max_d = np.percentile(depth_down, 99)
        #     depth_color = np.interp(depth_frame, (min_d, max_d), (0, 255)).astype(np.uint8)
        #     depth_color = cv2.applyColorMap(depth_color, cv2.COLORMAP_HOT)
        #     cv2.imshow("Depth", depth_color)

        cv2.imshow("ORION Vision", frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()