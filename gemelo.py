import argparse
import io
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw

from aco import decode
from maze import C, GOAL, MAZE, R, START, shortest_length, walk

COLORS = {0: (1.0, 0.1, 0.1), 1: (0.1, 0.35, 1.0), 2: (0.1, 0.75, 0.2)}
NAMES = {0: "Nodo 0 (rojo)", 1: "Nodo 1 (azul)", 2: "Nodo 2 (verde)"}
SPEED = 4.0          # celdas por segundo
IMG = 560            # tamano del video en pixeles
FPS = 15

lock = threading.Lock()
best = {}            # id -> {"len", "path", "it"}
heat = {}            # (fila, col) -> intensidad de feromona
latest_jpeg = None


def world(r, c):
    """Celda (fila, col) -> coordenadas del mundo (fila 0 arriba)."""
    return float(c), float(R - 1 - r)


# ------------------------------------------------------------------ red
def udp_listener(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    print(f"Escuchando UDP en el puerto {port}", flush=True)
    while True:
        data, _ = sock.recvfrom(512)
        msg = decode(data.decode(errors="ignore"))
        if not msg:
            continue
        nid, it, path = msg
        cells = walk(path)
        with lock:
            prev = best.get(nid)
            best[nid] = {"len": len(path), "path": path, "it": it}
            if cells and (prev is None or prev["path"] != path or True):
                for cell in cells[1:]:
                    heat[cell] = heat.get(cell, 0.0) + 10.0 / len(path)


# ------------------------------------------------------------------ video
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    if latest_jpeg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(latest_jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path == "/frame.jpg":
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            self.wfile.write(latest_jpeg or b"")
        else:
            html = (
                "<html><head><title>Gemelo digital ACO</title></head>"
                "<body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>"
                "<h2>Enjambre ACO - gemelo digital (PyBullet)</h2>"
                "<img src='/stream' style='max-width:95vw'></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html.encode())


def serve_http(port):
    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    srv.daemon_threads = True
    print(f"Visor web en http://localhost:{port}", flush=True)
    srv.serve_forever()


# ------------------------------------------------------------------ escena
def box(half, pos, rgba):
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs, basePosition=pos)


def build_scene():
    tiles = {}
    for r in range(R):
        for c in range(C):
            x, y = world(r, c)
            ch = MAZE[r][c]
            if ch == "#":
                box([0.5, 0.5, 0.4], [x, y, 0.4], [0.18, 0.2, 0.28, 1])
            else:
                rgba = [0.92, 0.92, 0.92, 1]
                if (r, c) == START:
                    rgba = [0.4, 0.9, 0.4, 1]
                elif (r, c) == GOAL:
                    rgba = [1.0, 0.85, 0.2, 1]
                tiles[(r, c)] = box([0.47, 0.47, 0.02], [x, y, 0.02], rgba)
    return tiles


def make_cart(nid):
    r, g, b = COLORS[nid]
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.28, 0.18, 0.08], rgbaColor=[r, g, b, 1])
    x, y = world(*START)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs, basePosition=[x, y, 0.14])


class Cart:
    def __init__(self, nid):
        self.nid = nid
        self.body = make_cart(nid)
        self.cells = [START]
        self.t = 0.0
        self.wait = 0.0

    def reload(self):
        with lock:
            info = best.get(self.nid)
        cells = walk(info["path"]) if info else None
        self.cells = cells or [START]
        self.t = 0.0

    def update(self, dt):
        if len(self.cells) < 2:
            self.reload()
        else:
            if self.wait > 0:
                self.wait -= dt
                if self.wait <= 0:
                    self.reload()
                return
            self.t += SPEED * dt
            if self.t >= len(self.cells) - 1:
                self.t = len(self.cells) - 1
                self.wait = 1.0
        i = min(int(self.t), len(self.cells) - 1)
        j = min(i + 1, len(self.cells) - 1)
        f = self.t - i
        x0, y0 = world(*self.cells[i])
        x1, y1 = world(*self.cells[j])
        x, y = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
        yaw = math.atan2(y1 - y0, x1 - x0) if (x1, y1) != (x0, y0) else 0.0
        p.resetBasePositionAndOrientation(
            self.body, [x, y, 0.14], p.getQuaternionFromEuler([0, 0, yaw])
        )


def paint_tiles(tiles):
    with lock:
        for k in heat:
            heat[k] *= 0.97
        snapshot = dict(heat)
    top = max(snapshot.values(), default=0.0)
    for cell, body in tiles.items():
        if cell in (START, GOAL):
            continue
        v = min(1.0, snapshot.get(cell, 0.0) / top) if top > 0.01 else 0.0
        rgba = [0.92, 0.92 - 0.55 * v, 0.92 - 0.85 * v, 1]
        p.changeVisualShape(body, -1, rgbaColor=rgba)


def render(view, proj):
    _, _, rgb, _, _ = p.getCameraImage(IMG, IMG, view, proj, renderer=p.ER_TINY_RENDERER)
    arr = np.reshape(np.array(rgb, dtype=np.uint8), (IMG, IMG, 4))[:, :, :3]
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    with lock:
        info = {k: dict(v) for k, v in best.items()}
    d.rectangle([0, 0, IMG, 18 + 14 * 4], fill=(0, 0, 0))
    d.text((6, 4), f"Ruta optima (BFS): {shortest_length()} pasos", fill=(255, 255, 255))
    if not info:
        d.text((6, 18), "Esperando datos de los nodos...", fill=(255, 200, 80))
    for nid in range(3):
        y = 18 + 14 * nid
        if nid in info:
            txt = f"{NAMES[nid]}: {info[nid]['len']} pasos (iter {info[nid]['it']})"
        else:
            txt = f"{NAMES[nid]}: sin datos"
        col = tuple(int(255 * v) for v in COLORS[nid])
        d.text((6, y), txt, fill=col)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def main():
    global latest_jpeg
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="abrir ventana de PyBullet")
    ap.add_argument("--udp", type=int, default=4210)
    ap.add_argument("--http", type=int, default=8000)
    args = ap.parse_args()

    p.connect(p.GUI if args.gui else p.DIRECT)
    tiles = build_scene()
    carts = [Cart(i) for i in range(3)]

    threading.Thread(target=udp_listener, args=(args.udp,), daemon=True).start()
    threading.Thread(target=serve_http, args=(args.http,), daemon=True).start()

    cx, cy = (C - 1) / 2.0, (R - 1) / 2.0
    dist = max(R, C) * 1.05
    view = p.computeViewMatrixFromYawPitchRoll([cx, cy, 0], dist, 0, -89.9, 0, 2)
    proj = p.computeProjectionMatrixFromFOV(60, 1.0, 0.1, 100)
    if args.gui:
        p.resetDebugVisualizerCamera(dist, 0, -89.9, [cx, cy, 0])

    last_paint = 0.0
    last = time.time()
    while True:
        now = time.time()
        dt = now - last
        last = now
        for cart in carts:
            cart.update(dt)
        if now - last_paint > 0.25:
            paint_tiles(tiles)
            last_paint = now
        if not args.gui:
            latest_jpeg = render(view, proj)
        time.sleep(max(0.0, 1.0 / FPS - (time.time() - now)))


if __name__ == "__main__":
    main()
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
