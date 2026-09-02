import cv2
import math
import depthai as dai
from ultralytics import YOLO


def quat_to_euler(i, j, k, w):
    """Quaternion -> roll, pitch, yaw w stopniach."""
    # roll (X)
    sinr = 2.0 * (w * i + j * k)
    cosr = 1.0 - 2.0 * (i * i + j * j)
    roll = math.atan2(sinr, cosr)

    # pitch (Y)
    sinp = 2.0 * (w * j - k * i)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (Z)
    siny = 2.0 * (w * k + i * j)
    cosy = 1.0 - 2.0 * (j * j + k * k)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    model = YOLO("yolov8n.pt")

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        queue = cam.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

        # --- IMU ---
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)

        imu_queue = imu.out.createOutputQueue()

        pipeline.start()

        imu_text = ["IMU: brak danych"]
        color = (203, 0, 255)  # różowy (BGR)

        while pipeline.isRunning():
            frame = queue.get().getCvFrame()

            imu_data = imu_queue.tryGet()
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

            # YOLO
            results = model(frame, verbose=False)
            annotated = results[0].plot()

            for i, line in enumerate(imu_text):
                cv2.putText(
                    annotated, line, (10, 25 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
                )

            cv2.imshow("result", annotated)
            if cv2.waitKey(30) == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()