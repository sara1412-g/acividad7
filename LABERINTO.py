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
