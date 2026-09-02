"""
Minimalny szkielet HMI - podglad z kamery OAK-D Pro w przegladarce.
 
Architektura:
  - watek w tle: pipeline DepthAI ciagnie klatki z kamery,
    zapisuje najnowsza do wspoldzielonej zmiennej (chronionej Lock-iem)
  - FastAPI: serwuje strone HMI ("/") oraz strumien MJPEG ("/video")
 
Uruchomienie:
    python hmi_server.py
Potem w przegladarce:  http://localhost:8000
(z innego urzadzenia w sieci:  http://<ip-hosta>:8000)
"""
 
import threading
import time
 
import cv2
import depthai as dai
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse
 
# ---------------------------------------------------------------------------
# Wspoldzielony stan miedzy watkiem kamery a serwerem HTTP
# ---------------------------------------------------------------------------
# Tylko jedna klatka naraz - "ostatnia znana". Serwer ja czyta, kamera nadpisuje.
# Lock chroni przed odczytem w polowie zapisu (klatka to spory obiekt).
 
_latest_frame = None          # ostatnia zakodowana klatka JPEG (bytes)
_frame_lock = threading.Lock()
_camera_running = True         # flaga zatrzymania watku kamery
 
 
def camera_worker():
    """Watek w tle: czyta klatki z OAK-D Pro i koduje je do JPEG."""
    global _latest_frame, _camera_running
 
    # --- pipeline DepthAI v3 ---
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        output = cam.requestOutput((1280, 720), dai.ImgFrame.Type.BGR888i)
        queue = output.createOutputQueue()
 
        pipeline.start()
        print("[kamera] pipeline wystartowal")
 
        while _camera_running and pipeline.isRunning():
            frame = queue.get().getCvFrame()      # numpy BGR
 
            # kodujemy do JPEG - mniejszy rozmiar, gotowe do wyslania w MJPEG
            ok, encoded = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
 
            with _frame_lock:
                _latest_frame = encoded.tobytes()
 
    print("[kamera] pipeline zatrzymany")
 
 
# ---------------------------------------------------------------------------
# Serwer FastAPI
# ---------------------------------------------------------------------------
app = FastAPI(title="HMI Skeleton")
 
 
@app.on_event("startup")
def start_camera():
    """Przy starcie serwera odpalamy watek kamery."""
    t = threading.Thread(target=camera_worker, daemon=True)
    t.start()
    print("[serwer] watek kamery uruchomiony")
 
 
@app.on_event("shutdown")
def stop_camera():
    """Przy zamknieciu serwera dajemy watkowi sygnal stop."""
    global _camera_running
    _camera_running = False
 
 
def mjpeg_generator():
    """
    Generator strumienia MJPEG.
    MJPEG = ciag klatek JPEG sklejonych naglowkami multipart.
    Przegladarka w <img> rozumie to natywnie i odswieza obraz na biezaco.
    """
    while True:
        with _frame_lock:
            frame = _latest_frame
 
        if frame is None:
            # kamera jeszcze sie nie rozkrecila - czekamy chwile
            time.sleep(0.05)
            continue
 
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
 
        # ~30 FPS gornego limitu; kamera i tak narzuca swoje tempo
        time.sleep(1 / 30)
 
 
@app.get("/video")
def video_feed():
    """Endpoint strumienia - to do niego celuje <img src="/video">."""
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
 
 
@app.get("/", response_class=HTMLResponse)
def index():
    """Strona HMI - na razie tylko ramka z obrazem."""
    return """<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="utf-8">
  <title>HMI - podglad kamery</title>
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
  <header>OAK-D Pro &middot; podglad na zywo</header>
  <div class="frame">
    <img src="/video" alt="strumien z kamery">
  </div>
  <footer>MJPEG &middot; FastAPI &middot; DepthAI</footer>
</body>
</html>"""
 
 
if __name__ == "__main__":
    # host="0.0.0.0" => serwer widoczny tez z innych urzadzen w sieci
    uvicorn.run(app, host="0.0.0.0", port=8000)