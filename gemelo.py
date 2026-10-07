import argparse, io, math, socket, threading, time
import numpy as np
import pybullet as p
from PIL import Image, ImageDraw
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from maze import MAZE, R, C, START, GOAL, rc

CELL, SPEED, W, H = 1.0, 1.6, 800, 640
COLORS = [(0.9, 0.1, 0.1), (0.1, 0.3, 0.95), (0.1, 0.7, 0.2)]

lock = threading.Lock()
best = {}
heat = [0.0] * (R * C)
frame = None


def pos(i):
    r, c = rc(i)
    return c * CELL, -r * CELL


def listener(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    print(f"[twin] escuchando UDP :{port}")
    while True:
        data, _ = s.recvfrom(2048)
        try:
            t, nid, it, ln, pth = data.decode().strip().split(";")
            if t != "P":
                continue
            nid, it, ln = int(nid), int(it), int(ln)
            path = [int(x) for x in pth.split(",")]
        except ValueError:
            continue
        with lock:
            for i in range(len(heat)):
                heat[i] *= 0.997
            for c in path:
                heat[c] += 1.0 / max(ln, 1)
            if nid not in best or ln <= best[nid]["len"]:
                best[nid] = {"len": ln, "path": path, "iter": it}


class Car:
    def __init__(self, nid, body):
        self.nid, self.body = nid, body
        self.path, self.k, self.t, self.wait = [], 0, 0.0, 0.0

    def update(self, dt):
        if self.wait > 0:
            self.wait -= dt
            return
        if self.k >= len(self.path) - 1:
            with lock:
                new = best.get(self.nid)
            if not new:
                return
            self.path, self.k, self.t = list(new["path"]), 0, 0.0
            if len(self.path) < 2:
                return
        a, b = pos(self.path[self.k]), pos(self.path[self.k + 1])
        self.t = min(1.0, self.t + SPEED * dt)
        x, y = a[0] + (b[0] - a[0]) * self.t, a[1] + (b[1] - a[1]) * self.t
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        p.resetBasePositionAndOrientation(self.body, [x, y, 0.1],
                                          p.getQuaternionFromEuler([0, 0, yaw]))
        if self.t >= 1.0:
            self.k, self.t = self.k + 1, 0.0
            if self.k >= len(self.path) - 1:
                self.wait = 1.0


def box(half, color, xyz):
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=list(color) + [1])
    return p.createMultiBody(0, -1, v, xyz)


def build_scene():
    tiles = {}
    for r in range(R):
        for c in range(C):
            x, y = c * CELL, -r * CELL
            if MAZE[r][c] == "#":
                box([.5, .5, .35], (.25, .25, .3), [x, y, .35])
            else:
                tiles[r * C + c] = box([.48, .48, .005], (.93, .93, .93), [x, y, .005])
    x, y = pos(START); box([.48, .48, .01], (.4, .8, .4), [x, y, .01])
    x, y = pos(GOAL);  box([.48, .48, .01], (1, .85, .2), [x, y, .01])
    box([.03, .03, .4], (.2, .2, .2), [x - .35, y + .35, .4])
    box([.2, .02, .12], (.9, .1, .1), [x - .15, y + .35, .68])
    cars = []
    for i, col in enumerate(COLORS):
        body = box([.22, .14, .08], col, [*pos(START), .1])
        cars.append(Car(i, body))
    return tiles, cars


def render(view, proj):
    img = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER)[2]
    im = Image.fromarray(np.reshape(np.array(img, dtype=np.uint8), (H, W, 4))[:, :, :3])
    d = ImageDraw.Draw(im)
    with lock:
        info = {k: dict(v) for k, v in best.items()}
    d.text((10, 8), "Gemelo digital - enjambre ACO (PyBullet)", fill=(0, 0, 0))
    if not info:
        d.text((10, 24), "Esperando datos de los nodos ESP32 (UDP 4210)...", fill=(150, 0, 0))
    for n in sorted(info):
        col = tuple(int(c * 255) for c in COLORS[n % 3])
        d.text((10, 24 + 14 * n), f"Nodo {n}: mejor ruta = {info[n]['len']} pasos (iter {info[n]['iter']})", fill=col)
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=80)
    return buf.getvalue()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    f = frame
                    if f:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(f))
                        self.wfile.write(f + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            body = b"<html><body style='margin:0;background:#222'><img src='/stream' style='max-width:100%'></body></html>"
            self.send_response(200); self.send_header("Content-Type", "text/html"); self.end_headers()
            self.wfile.write(body)


def main():
    global frame
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--http", type=int, default=8000)
    ap.add_argument("--gui", action="store_true")
    a = ap.parse_args()

    threading.Thread(target=listener, args=(a.port,), daemon=True).start()
    p.connect(p.GUI if a.gui else p.DIRECT)
    p.setGravity(0, 0, 0)
    tiles, cars = build_scene()
    view = p.computeViewMatrixFromYawPitchRoll([(C - 1) / 2, -(R - 1) / 2, 0], 14, 0, -89, 0, 2)
    proj = p.computeProjectionMatrixFOV(45, W / H, 0.1, 50)
    if not a.gui:
        srv = ThreadingHTTPServer(("0.0.0.0", a.http), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[twin] abre http://localhost:{a.http}")

    last, tcol = time.time(), 0
    while True:
        now = time.time(); dt, last = now - last, now
        for car in cars:
            car.update(dt)
        if now - tcol > 0.4:
            tcol = now
            with lock:
                m = max(max(heat), 1e-6)
                for i, t in tiles.items():
                    k = min(1.0, heat[i] / m)
                    p.changeVisualShape(t, -1, rgbaColor=[0.93, 0.93 - 0.55 * k, 0.93 - 0.85 * k, 1])
        if not a.gui:
            frame = render(view, proj)
        time.sleep(0.03)


if __name__ == "__main__":
    main()
