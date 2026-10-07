import random

from maze import DIRS, GOAL, START, R, C, is_open, walk

ALPHA = 1.0      # peso de la feromona
BETA = 1.0       # peso de la cercania a la meta
RHO = 0.10       # evaporacion por iteracion
Q = 10.0         # feromona depositada = Q / largo de la ruta
ANTS = 4         # hormigas por iteracion
TAU_MIN = 0.05
TAU_MAX = 20.0
MAX_STEPS = 60   # largo maximo de una ruta


class ACO:
    def __init__(self, node_id, seed=None):
        self.node_id = node_id
        self.rng = random.Random(seed if seed is not None else node_id + 1)
        self.tau = [[1.0] * C for _ in range(R)]
        self.best_path = None
        self.best_len = 10**9
        self.iteration = 0

    # ---- una hormiga construye una ruta ----
    def run_ant(self):
        r, c = START
        visited = {(r, c)}
        path = []
        while (r, c) != GOAL:
            if len(path) >= MAX_STEPS:
                return None
            options = []
            total = 0.0
            for d, dr, dc in DIRS:
                nr, nc = r + dr, c + dc
                if is_open(nr, nc) and (nr, nc) not in visited:
                    eta = 1.0 / (1 + abs(nr - GOAL[0]) + abs(nc - GOAL[1]))
                    w = (self.tau[nr][nc] ** ALPHA) * (eta ** BETA)
                    options.append((d, nr, nc, w))
                    total += w
            if not options:
                return None  # hormiga atrapada
            pick = self.rng.random() * total
            acc = 0.0
            chosen = options[-1]
            for o in options:
                acc += o[3]
                if pick <= acc:
                    chosen = o
                    break
            d, r, c = chosen[0], chosen[1], chosen[2]
            visited.add((r, c))
            path.append(d)
        return "".join(path)

    def _deposit(self, path, amount):
        cells = walk(path)
        if not cells:
            return
        for (r, c) in cells[1:]:
            self.tau[r][c] = min(TAU_MAX, self.tau[r][c] + amount)

    # ---- una iteracion completa ----
    def iterate(self):
        """Ejecuta ANTS hormigas. Devuelve True si mejoro la mejor ruta."""
        self.iteration += 1
        found = []
        for _ in range(ANTS):
            p = self.run_ant()
            if p:
                found.append(p)

        # evaporacion
        for r in range(R):
            for c in range(C):
                self.tau[r][c] = max(TAU_MIN, self.tau[r][c] * (1 - RHO))

        # deposito
        improved = False
        for p in found:
            self._deposit(p, Q / len(p))
            if len(p) < self.best_len:
                self.best_len = len(p)
                self.best_path = p
                improved = True
        # refuerzo elitista de la mejor ruta
        if self.best_path:
            self._deposit(self.best_path, Q / self.best_len)
        return improved

    # ---- feromona recibida de otro nodo ----
    def receive(self, path):
        """Refuerza la ruta que mando otro nodo (intercambio de feromonas)."""
        if path and walk(path):
            self._deposit(path, Q / len(path))
            if len(path) < self.best_len:
                self.best_len = len(path)
                self.best_path = path


def encode(node_id, iteration, path):
    """Mensaje UDP: 'id,iteracion,largo,ruta'."""
    return f"{node_id},{iteration},{len(path)},{path}"


def decode(text):
    """Inverso de encode(). Devuelve (id, iteracion, ruta) o None."""
    try:
        nid, it, ln, path = text.strip().split(",", 3)
        nid, it, ln = int(nid), int(it), int(ln)
        if ln != len(path) or not walk(path):
            return None
        return nid, it, path
    except ValueError:
        return None
import random
from maze import START, GOAL, R, C, rc, neighbor

ALPHA, BETA, RHO, Q, ANTS = 1.0, 1.0, 0.10, 10.0, 10


class ACO:
    def __init__(self):
        self.tau = [[1.0] * 4 for _ in range(R * C)]

    def ant(self):
        cur, path, vis = START, [START], {START}
        gr, gc = rc(GOAL)
        while cur != GOAL:
            cand = []
            for d in range(4):
                nb = neighbor(cur, d)
                if nb < 0 or nb in vis:
                    continue
                r, c = rc(nb)
                eta = 1.0 / (1 + abs(r - gr) + abs(c - gc))
                cand.append((nb, (self.tau[cur][d] ** ALPHA) * (eta ** BETA)))
            if not cand:
                return None
            x, acc = random.random() * sum(w for _, w in cand), 0.0
            for nb, w in cand:
                acc += w
                if x <= acc:
                    break
            cur = nb
            path.append(cur); vis.add(cur)
        return path

    def deposit(self, path, amt):
        for a, b in zip(path, path[1:]):
            for d in range(4):
                if neighbor(a, d) == b:
                    self.tau[a][d] = min(50.0, self.tau[a][d] + amt)
                    self.tau[b][(d + 2) % 4] = min(50.0, self.tau[b][(d + 2) % 4] + amt)
                    break

    def evaporate(self):
        for row in self.tau:
            for d in range(4):
                row[d] = max(0.05, row[d] * (1 - RHO))

    def step(self):
        best = None
        for _ in range(ANTS):
            p = self.ant()
            if p and (best is None or len(p) < len(best)):
                best = p
        self.evaporate()
        if best:
            self.deposit(best, Q / (len(best) - 1))
        return best
