import os
import socket
import time

from aco import ACO, encode
from maze import shortest_length

HOST = os.environ.get("TWIN_HOST", "127.0.0.1")
PORT = int(os.environ.get("TWIN_PORT", "4210"))
DELAY = float(os.environ.get("ITER_DELAY", "0.25"))


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    nodes = [ACO(i) for i in range(3)]
    print(f"Emulador -> {HOST}:{PORT} | ruta optima = {shortest_length()} pasos", flush=True)
    while True:
        for n in nodes:
            n.iterate()
        # intercambio de feromonas: cada nodo refuerza con la mejor ruta de los otros
        for n in nodes:
            for m in nodes:
                if m is not n and m.best_path:
                    n.receive(m.best_path)
        for n in nodes:
            if n.best_path:
                msg = encode(n.node_id, n.iteration, n.best_path)
                try:
                    sock.sendto(msg.encode(), (HOST, PORT))
                except OSError as e:
                    print("No se pudo enviar:", e, flush=True)
        if nodes[0].iteration % 10 == 0:
            print("iter", nodes[0].iteration, [n.best_len for n in nodes], flush=True)
        time.sleep(DELAY)


if __name__ == "__main__":
    main()
import argparse, socket, time
from aco import ACO, Q

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=4210)
ap.add_argument("--period", type=float, default=0.5)
a = ap.parse_args()

nodes = [ACO() for _ in range(3)]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
it = 0
print(f"Emulando 3 nodos -> {a.host}:{a.port}")
while True:
    for i, n in enumerate(nodes):
        path = n.step()
        if not path:
            continue
        ln = len(path) - 1
        sock.sendto(f"P;{i};{it};{ln};{','.join(map(str, path))}".encode(), (a.host, a.port))
        for j, m in enumerate(nodes):
            if j != i:
                m.deposit(path, Q / ln)
    it += 1
    time.sleep(a.period)
