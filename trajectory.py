import typing
import math
import dataclasses

@dataclasses.dataclass
class Point:
    x: float
    y: float

@dataclasses.dataclass
class Pose:
    position: Point
    angle_deg: float

def distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.sqrt((ax-bx)**2+(ay-by)**2)

def sample(pts: typing.Tuple[Point, Point], t: float) -> Point:
    a = pts[0]
    b = pts[1]
    x = (b.x - a.x) * t + a.x
    y = (b.y - a.y) * t + a.y
    return Point(x, y)

class Trajectory:
    STEP_DIST = 0.05 # 5 cm step distance
    def __init__(self, waypoints: typing.List[Point], final_angle: typing.Optional[float]):
        self.waypoints = waypoints
        self.final_angle = final_angle
    def lengths(self) -> typing.List[typing.Tuple[Point, Point]]:
        a = list(self.waypoints[:-1])
        b = list(self.waypoints[1:])
        return list(zip(a, b))
    def total_length(self) -> float:
        total_len = 0
        for cross in self.lengths():
            total_len += distance(cross[0].x, cross[0].y, cross[1].x, cross[1].y)
        return total_len
    def sample(self, t: float) -> Point:
        """ Return the point at a specified distance along the trajectory
        : None if it is outside of the trajectory range"""
        remaining = t
        for length in self.lengths():
            length_distance = distance(length[0].x, length[0].y, length[1].x, length[1].y)
            remaining -= length_distance
            if remaining < 0:
                partial = remaining + length_distance
                total = length_distance
                pt = sample(length, partial / total)
                if partial > total: return None
                return pt
        return None
    def last_point(self) -> Point: return self.waypoints[-1]
    
    def find_closest_t_value(self, point: Point) -> float:
        """Find the closest t value one a curve"""
        total_distance_of_curve = self.total_length()
        step = 0
        smallest_distance = 10000 # Math.infinity
        final_tval = 30
        while True:
            step += 1
            t = total_distance_of_curve - step*self.STEP_DIST
            if t < 0: break # Exit when we are complete
            curve_point = self.sample(t)
            dist = distance(curve_point.x, curve_point.y, point.x, point.y)
            if dist < smallest_distance:
                smallest_distance = dist
                final_tval = t
        return final_tval
    def generate_carrot(self, current_t: float, follow=1.0) -> float:
        """Generate a carrot from a tvalue"""
        target_t = current_t + follow
        while self.sample(target_t) is None: target_t -= self.STEP_DIST
        return target_t

def serialize(traj: Trajectory) -> str:
    string = ""
    for waypoint in traj.waypoints:
        string += str(waypoint.x)
        string += ","
        string += str(waypoint.y)
        string += "#"
    if traj.final_angle:
        string += str(traj.final_angle)
    return string
def deserialize(traj: str) -> Trajectory:
    splits = traj.split("#")
    waypoints = []
    for point_str in splits[:-1]:
        tmp = point_str.split(",")
        point = Point(float(tmp[0]), float(tmp[1]))
        waypoints.append(point)
    fangle = None
    fangle_str = splits[-1]
    if fangle_str:
        fangle = float(fangle_str)
    return Trajectory(waypoints, fangle)
