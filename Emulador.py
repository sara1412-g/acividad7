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
