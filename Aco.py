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
