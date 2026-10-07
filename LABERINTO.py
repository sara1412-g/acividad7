from collections import deque

MAZE = [
    "###########",
    "#S........#",
    "#.##.####.#",
    "#....#....#",
    "#.####.##.#",
    "#.#......##",
    "#.#.####..#",
    "#...#..#.##",
    "#.###.##..#",
    "#.....#..G#",
    "###########",
]

R = len(MAZE)
C = len(MAZE[0])

# Direcciones: (letra, delta fila, delta columna)
DIRS = [("U", -1, 0), ("D", 1, 0), ("L", 0, -1), ("R", 0, 1)]
DELTA = {d: (dr, dc) for d, dr, dc in DIRS}


def _find(ch):
    for r, row in enumerate(MAZE):
        for c, v in enumerate(row):
            if v == ch:
                return (r, c)
    raise ValueError(f"No se encontro '{ch}' en el laberinto")


START = _find("S")
GOAL = _find("G")


def is_open(r, c):
    return 0 <= r < R and 0 <= c < C and MAZE[r][c] != "#"


def walk(path):
    """Convierte un texto de movimientos ('RRDL...') en la lista de celdas.

    Devuelve None si la ruta no es valida (sale del laberinto o cruza una pared).
    """
    r, c = START
    cells = [(r, c)]
    for ch in path:
        if ch not in DELTA:
            return None
        dr, dc = DELTA[ch]
        r, c = r + dr, c + dc
        if not is_open(r, c):
            return None
        cells.append((r, c))
    return cells


def shortest_length():
    """Longitud (en pasos) de la ruta mas corta, calculada con BFS."""
    q = deque([(START, 0)])
    seen = {START}
    while q:
        (r, c), d = q.popleft()
        if (r, c) == GOAL:
            return d
        for _, dr, dc in DIRS:
            n = (r + dr, c + dc)
            if is_open(*n) and n not in seen:
                seen.add(n)
                q.append((n, d + 1))
    return None


if __name__ == "__main__":
    print(f"Laberinto {R}x{C}, inicio={START}, meta={GOAL}, ruta optima={shortest_length()} pasos")
MAZE = [
    "S.........",
    "..#...#.#.",
    "#.#.....#.",
    "#.........",
    "...#..#...",
    "..#..#....",
    "...#.#....",
    ".##.....#.",
    ".....#..#.",
    ".........G",
]
R, C = len(MAZE), len(MAZE[0])
DR = (-1, 0, 1, 0)
DC = (0, 1, 0, -1)


def _find(ch):
    for r, row in enumerate(MAZE):
        if ch in row:
            return r * C + row.index(ch)


START, GOAL = _find("S"), _find("G")


def idx(r, c): return r * C + c
def rc(i): return divmod(i, C)
def free(r, c): return 0 <= r < R and 0 <= c < C and MAZE[r][c] != "#"


def neighbor(i, d):
    r, c = rc(i)
    r += DR[d]; c += DC[d]
    return idx(r, c) if free(r, c) else -1
