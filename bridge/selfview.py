"""The avatar itself in its own view (VRChat's third person camera).

Seen from behind, the avatar stands in the lower middle of every frame, much
nearer than what is beside it: depth there is the avatar's back, not the way
ahead, and the camera is some way behind the avatar's feet. find_self finds
that region in a depth grid so the rest of navigation can leave it out (and
measure from the feet, not the camera); None in first person.
"""

from __future__ import annotations

from dataclasses import dataclass

NEAR_SHARE = 0.8  # a body cell is nearer than this share of the scene beside it
CORE_SHARE = 0.5  # of the cells right in the middle that must be body
SAME_BODY = 1.5  # body cells are at most this much farther than the middle of it
# (someone standing beyond the avatar is near too, but farther)
MIN_BODY_CELLS = 5  # per column, to count as body (a neighbour beside it has fewer)
# The camera follows the avatar from straight behind: its body is at most
# this share of the columns either side of the middle (about 10 degrees),
# whoever stands right beside it.
MAX_HALF_SHARE = 0.08


@dataclass(frozen=True)
class Self:
    c0: int  # the grid columns the body covers, inclusive
    c1: int
    r0: int  # its rows: head to feet, inclusive
    r1: int
    # Per row (r0..r1), the columns it covers there, what it holds or wears
    # sticking out included (a weapon on its back, long hair).
    spans: tuple[tuple[int, int], ...] = ()
    far: float = 0.0  # depth (model m) up to which a near thing may be its own

    def covers(self, col: int, row: int | None = None) -> bool:
        """Whether the body is in ``col`` (its trunk), or in that cell."""
        if row is None:
            return self.c0 <= col <= self.c1
        if not self.r0 <= row <= self.r1:
            return False
        if self.spans:
            left, right = self.spans[row - self.r0]
            return left <= col <= right
        return self.c0 <= col <= self.c1


def _near_cells(depth: list[float], cols: int, rows: int) -> list[list[bool]]:
    """Cells much nearer than the scene beside the middle, row by row."""
    sides = [c for c in range(cols) if 0.15 * cols <= c < 0.3 * cols or 0.7 * cols <= c < 0.85 * cols]
    near = []
    for r in range(rows):
        beside = sorted(depth[r * cols + c] for c in sides)
        reference = beside[len(beside) // 2]
        near.append([0 < depth[r * cols + c] < NEAR_SHARE * reference for c in range(cols)])
    return near


def find_self(depth: list[float], cols: int, rows: int) -> Self | None:
    """The avatar's own body in a depth grid seen from behind it, or None."""
    near = _near_cells(depth, cols, rows)
    core_rows = range(int(0.55 * rows), int(0.8 * rows))
    core_cols = range(int(0.45 * cols), int(0.55 * cols) + 1)
    core = [near[r][c] for r in core_rows for c in core_cols]
    if sum(core) < CORE_SHARE * len(core):
        return None
    body = sorted(depth[r * cols + c] for r in core_rows for c in core_cols if near[r][c])
    farthest = SAME_BODY * body[len(body) // 2]
    near = [[near[r][c] and depth[r * cols + c] <= farthest for c in range(cols)]
            for r in range(rows)]
    body_rows = range(int(0.4 * rows), int(0.95 * rows))

    def body_column(c: int) -> bool:
        return sum(near[r][c] for r in body_rows) >= MIN_BODY_CELLS

    half = round(MAX_HALF_SHARE * cols)
    c0 = c1 = cols // 2
    while c0 - 1 >= cols // 2 - half and body_column(c0 - 1):
        c0 -= 1
    while c1 + 1 <= cols // 2 + half and body_column(c1 + 1):
        c1 += 1

    def body_row(r: int) -> bool:
        return any(near[r][c] for c in range(c0, c1 + 1))

    r0 = r1 = core_rows[len(core_rows) // 2]
    while r0 - 1 >= int(0.3 * rows) and body_row(r0 - 1):
        r0 -= 1
    while r1 + 1 < rows and body_row(r1 + 1):
        r1 += 1
    if r1 >= rows - 1:
        return None  # it goes on below the frame: something right in front, not the avatar
    spans = []
    for r in range(r0, r1 + 1):
        left, right = c0, c1
        while left - 1 >= int(0.25 * cols) and near[r][left - 1]:
            left -= 1
        while right + 1 < int(0.75 * cols) and near[r][right + 1]:
            right += 1
        spans.append((left, right))
    return Self(c0, c1, r0, r1, tuple(spans), farthest)
