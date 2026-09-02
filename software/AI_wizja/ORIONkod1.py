import cv2
import depthai as dai
from ultralytics import YOLO


def main():
    model = YOLO("yolov8n.pt")  # pobierze się automatycznie przy pierwszym uruchomieniu

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        queue = cam.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p).createOutputQueue()

        pipeline.start()
        while pipeline.isRunning():
            frame = queue.get().getCvFrame()

            results = model(frame, verbose=False)
            annotated = results[0].plot()

            cv2.imshow('result', annotated)
            if cv2.waitKey(30) == ord('q'):
                break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()