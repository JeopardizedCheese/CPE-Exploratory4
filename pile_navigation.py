"""Short routes around observed circular obstacles.

The interface accepts arena-millimetre points and obstacle dictionaries with
``x``, ``y`` and optional ``radius_mm`` fields. ``clearance_mm`` is the radius
of the robot's conservative swept envelope, including a safety margin. The
returned points include the exact start and goal; ``None`` means no path was
found inside the supplied bounds.
"""
from heapq import heappop, heappush
import math


def _distance_to_segment(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-9:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
    return math.hypot(point[0] - (start[0] + t * dx), point[1] - (start[1] + t * dy))


def _segment_clear(start, end, circles, bounds, allow_escape=False):
    xmin, ymin, xmax, ymax = bounds
    if not all(xmin - 1e-6 <= p[0] <= xmax + 1e-6 and ymin - 1e-6 <= p[1] <= ymax + 1e-6
               for p in (start, end)):
        return False
    vx, vy = end[0] - start[0], end[1] - start[1]
    for ox, oy, radius in circles:
        sx, sy = start[0] - ox, start[1] - oy
        start_d = math.hypot(sx, sy)
        if start_d < radius:
            # Let a robot already inside a padded obstacle leave it, but never move
            # deeper or slide through the obstacle's centre to reach the far side.
            if not allow_escape or sx * vx + sy * vy < -1e-6:
                return False
            continue
        if _distance_to_segment((ox, oy), start, end) < radius - 1e-6:
            return False
    return True


def _point_clear(point, circles):
    return all(math.hypot(point[0] - x, point[1] - y) >= r - 1e-6 for x, y, r in circles)


def _smooth(points, circles, bounds):
    if len(points) < 3:
        return points
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not _segment_clear(points[i], points[j], circles, bounds,
                                                allow_escape=(i == 0)):
            j -= 1
        result.append(points[j])
        i = j
    return result


def plan_route(start, goal, obstacles, bounds, clearance_mm, grid_mm=50.0):
    """Return a collision-free waypoint route for the robot's swept centre.

    Obstacles are circles inflated by ``clearance_mm``. A direct route is used
    whenever possible; otherwise A* searches a bounded grid and line-of-sight
    smoothing removes unnecessary turns. The route is conservative for rotation
    because callers supply the robot's circumscribed footprint radius.
    """
    start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    xmin, ymin, xmax, ymax = (float(v) for v in bounds)
    if not (xmin <= start[0] <= xmax and ymin <= start[1] <= ymax
            and xmin <= goal[0] <= xmax and ymin <= goal[1] <= ymax):
        return None
    margin = max(0.0, float(clearance_mm))
    circles = []
    for ob in obstacles:
        try:
            x, y = float(ob['x']), float(ob['y'])
            radius = max(0.0, float(ob.get('radius_mm', 20.0))) + margin
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(v) for v in (x, y, radius)):
            circles.append((x, y, radius))
    if not _point_clear(goal, circles):
        return None
    if _segment_clear(start, goal, circles, (xmin, ymin, xmax, ymax), allow_escape=True):
        return [start, goal]

    step = max(20.0, float(grid_mm))
    nx = int(math.floor((xmax - xmin) / step)) + 1
    ny = int(math.floor((ymax - ymin) / step)) + 1
    if nx <= 0 or ny <= 0:
        return None

    def point(node):
        return xmin + node[0] * step, ymin + node[1] * step

    def local_nodes(p):
        ix = int(round((p[0] - xmin) / step))
        iy = int(round((p[1] - ymin) / step))
        for radius in (1, 2):
            found = []
            for i in range(max(0, ix-radius), min(nx, ix+radius+1)):
                for j in range(max(0, iy-radius), min(ny, iy+radius+1)):
                    q = point((i, j))
                    if (math.hypot(q[0]-p[0], q[1]-p[1]) <= radius*step*1.5
                            and _point_clear(q, circles)):
                        found.append((i, j))
            if found:
                return found
        return []

    starts = local_nodes(start)
    goals = set(local_nodes(goal))
    if not starts or not goals:
        return None
    starts = [n for n in starts if _segment_clear(start, point(n), circles,
                                                   (xmin, ymin, xmax, ymax), allow_escape=True)]
    goal_cost = {n: math.hypot(point(n)[0]-goal[0], point(n)[1]-goal[1])
                 for n in goals if _segment_clear(point(n), goal, circles, (xmin, ymin, xmax, ymax))}
    if not starts or not goal_cost:
        return None

    best = {}
    parent = {}
    queue = []
    for node in starts:
        q = point(node)
        cost = math.hypot(q[0]-start[0], q[1]-start[1])
        if cost < best.get(node, float('inf')):
            best[node] = cost
            parent[node] = None
            heappush(queue, (cost + math.hypot(q[0]-goal[0], q[1]-goal[1]), cost, node))

    found = None
    while queue:
        _, cost, node = heappop(queue)
        if cost != best.get(node):
            continue
        if node in goal_cost:
            found = node
            break
        p = point(node)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if not (di or dj):
                    continue
                nxt = node[0] + di, node[1] + dj
                if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny):
                    continue
                q = point(nxt)
                if not _point_clear(q, circles) or not _segment_clear(p, q, circles, (xmin, ymin, xmax, ymax)):
                    continue
                new_cost = cost + math.hypot(q[0]-p[0], q[1]-p[1])
                if new_cost >= best.get(nxt, float('inf')):
                    continue
                best[nxt], parent[nxt] = new_cost, node
                estimate = new_cost + math.hypot(q[0]-goal[0], q[1]-goal[1])
                heappush(queue, (estimate, new_cost, nxt))

    if found is None:
        return None
    grid_path = []
    node = found
    while node is not None:
        grid_path.append(point(node))
        node = parent[node]
    grid_path.reverse()
    return _smooth([start] + grid_path + [goal], circles, (xmin, ymin, xmax, ymax))
