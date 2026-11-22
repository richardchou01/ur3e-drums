import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# UR3e joint speed limits (rad/s): base/shoulder/elbow 180 deg/s, wrists 360 deg/s.
JOINT_SPEED_LIMITS = [math.radians(v) for v in (180, 180, 180, 360, 360, 360)]

# Smallest time we allow for any single move (seconds). Guards against
# zero/negative segment times when notes are placed too close together.
MIN_MOVE_TIME = 0.02


# ---------------------------------------------------------------------------
# Notes
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Note:
    """One drum stroke the arm knows how to play: top -> bottom -> top."""
    top: tuple             # joint angles (rad) the swing starts from (and rebounds to)
    bottom: tuple          # joint angles (rad) where the stick hits the drum
    downstroke_time: float  # seconds to move from top to bottom
    upstroke_time: float    # seconds to rebound from bottom back to this note's top

    def __post_init__(self):
        for field in ("downstroke_time", "upstroke_time"):
            if getattr(self, field) < MIN_MOVE_TIME:
                raise ValueError(f"{field} must be at least {MIN_MOVE_TIME}s")


def deg(*angles):
    """Joint angles in degrees -> tuple of radians."""
    return tuple(math.radians(a) for a in angles)


# The fixed set of notes the robot can play, looked up by name.
OG_top_no_base = [-10.0, 7.43, -56.13, -1.43, 62.11]
OG_bottom_no_base = [-7.42, 7.6, -43.6, -1.35, 76.5]

OG_top_no_base = [19.15, -16.0, -110.0, -2.0, 68.0]
OG_bottom_no_base = [17.0, -15.6, -89.0, -4.0, 98.28]
base = -186.0

alpha = 0.2
top_no_base = [alpha*i+(1-alpha)*j for i,j in zip(OG_top_no_base, OG_bottom_no_base)]
bottom_no_base = OG_bottom_no_base
v0 = 0.4
v1 = 0.3
v2 = 0.2

NOTES = {
    "r0_v0": Note(
        top=deg(base, *top_no_base),
        bottom=deg(base, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r0_v1": Note(
        top=deg(base, *top_no_base),
        bottom=deg(base, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r0_v2": Note(
        top=deg(base, *top_no_base),
        bottom=deg(base, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r1_v0": Note(
        top=deg(base+2, *top_no_base),
        bottom=deg(base+2, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r1_v1": Note(
        top=deg(base+2, *top_no_base),
        bottom=deg(base+2, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r1_v2": Note(
        top=deg(base+2, *top_no_base),
        bottom=deg(base+2, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r2_v0": Note(
        top=deg(base+4, *top_no_base),
        bottom=deg(base+4, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r2_v1": Note(
        top=deg(base+4, *top_no_base),
        bottom=deg(base+4, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r2_v2": Note(
        top=deg(base+4, *top_no_base),
        bottom=deg(base+4, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r3_v0": Note(
        top=deg(base+6, *top_no_base),
        bottom=deg(base+6, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r3_v1": Note(
        top=deg(base+6, *top_no_base),
        bottom=deg(base+6, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r3_v2": Note(
        top=deg(base+6, *top_no_base),
        bottom=deg(base+6, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r4_v0": Note(
        top=deg(base+8, *top_no_base),
        bottom=deg(base+8, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r4_v1": Note(
        top=deg(base+8, *top_no_base),
        bottom=deg(base+8, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r4_v2": Note(
        top=deg(base+8, *top_no_base),
        bottom=deg(base+8, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r5_v0": Note(
        top=deg(base+10, *top_no_base),
        bottom=deg(base+10, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r5_v1": Note(
        top=deg(base+10, *top_no_base),
        bottom=deg(base+10, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r5_v2": Note(
        top=deg(base+10, *top_no_base),
        bottom=deg(base+10, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
    "r6_v0": Note(
        top=deg(base+12, *top_no_base),
        bottom=deg(base+12, *bottom_no_base),
        downstroke_time=alpha*v0,
        upstroke_time=alpha*v0,
    ),
    "r6_v1": Note(
        top=deg(base+12, *top_no_base),
        bottom=deg(base+12, *bottom_no_base),
        downstroke_time=alpha*v1,
        upstroke_time=alpha*v1,
    ),
    "r6_v2": Note(
        top=deg(base+12, *top_no_base),
        bottom=deg(base+12, *bottom_no_base),
        downstroke_time=alpha*v2,
        upstroke_time=alpha*v2,
    ),
}


# A score is a list of (note name, seconds since the previous hit).
# For the first note, "the previous hit" is the start of the trajectory, so
# its gap is the lead-in time the arm gets to travel from wherever it is.
#
# This one reproduces the old behavior: 20 center hits, 0.25 s apart.
bpm = 120
beat = 60/bpm
#MY_SCORE = [("r0_v0", beat),("r0_v2", beat),("r0_v0", beat),("r0_v2", beat),("r0_v0", beat/2),("r0_v2", beat/2),("r6_v2", beat),("r0_v0", beat/2),("r0_v2", beat/2),("r6_v2", beat)] 
MY_SCORE = [("r0_v0", beat), ("r1_v0", beat/4), ("r1_v1", beat*(3/4)), ("r2_v0", beat), ("r6_v2", beat)] 
MY_SCORE = MY_SCORE * 4



# ---------------------------------------------------------------------------
# Trajectory building (pure Python, no ROS, so it can be tested on its own)
# ---------------------------------------------------------------------------

def _same_pose(a, b, tol=1e-9):
    return math.dist(a, b) < tol


def build_trajectory(score, notes=NOTES):
    """Turn a score into (points, warnings).

    points is a list of (time_from_start, joint_positions). Each note is played
    as a self-contained stroke, then the arm travels to the next note:

      this top --(downstroke)--> this bottom --(upstroke)--> this top
               --(whatever time is left)--> next top

    so every hit lands exactly `gap` seconds after the previous one.

    If there isn't time for the rebound plus a separate move (at least
    MIN_MOVE_TIME) to a *different* next note, the rebound point is skipped and
    the arm goes straight from this bottom to the next top. When the next note
    shares this note's top, the "move" is just a pause, so it only needs the
    rebound to fit. Either fallback is reported in `warnings`, since those
    strokes won't sound quite like the others.

    After the last hit the arm rebounds to that note's top so the stick isn't
    left resting on the drum.
    """
    points, warnings = [], []
    hit_time = 0.0
    prev, prev_hit, prev_name = None, None, None

    for i, (name, gap) in enumerate(score):
        if name not in notes:
            raise KeyError(f"score[{i}]: unknown note {name!r}; known notes: {list(notes)}")
        note = notes[name]

        travel_time = gap - note.downstroke_time  # previous bottom -> this top
        if travel_time < MIN_MOVE_TIME:
            raise ValueError(
                f"score[{i}] ({name!r}): gap of {gap:.3f}s leaves only {travel_time:.3f}s "
                f"to reach the top after the {note.downstroke_time:.3f}s downstroke "
                f"(need at least {MIN_MOVE_TIME:.3f}s)."
            )

        hit_time += gap
        top_time = hit_time - note.downstroke_time

        if prev is not None:
            rebound_time = prev_hit + prev.upstroke_time
            spare = top_time - rebound_time  # time between rebound and next swing

            if _same_pose(prev.top, note.top):
                if spare > 1e-9:
                    points.append((rebound_time, prev.top))  # rebound, then hold
                elif spare < -1e-9:
                    warnings.append(
                        f"score[{i}] ({name!r}): gap too short for the {prev_name!r} "
                        f"upstroke; rebound shortened to {travel_time:.3f}s."
                    )
                # spare == 0: this note's top point doubles as the rebound
            elif spare >= MIN_MOVE_TIME:
                points.append((rebound_time, prev.top))      # rebound to own top
            else:
                warnings.append(
                    f"score[{i}] ({name!r}): no time to rebound to {prev_name!r} top; "
                    f"going straight from its bottom to {name!r} top in {travel_time:.3f}s."
                )

        points.append((top_time, note.top))     # start of swing
        points.append((hit_time, note.bottom))  # hit drum
        prev, prev_hit, prev_name = note, hit_time, name

    if prev is not None:
        points.append((hit_time + prev.upstroke_time, prev.top))  # final rebound

    return points, warnings


def speed_warnings(points, limits=JOINT_SPEED_LIMITS):
    """Return a message for every segment that would exceed a joint speed limit.

    The first point is skipped because we don't know where the arm starts.
    """
    warnings = []
    for (t0, q0), (t1, q1) in zip(points, points[1:]):
        dt = t1 - t0
        for j, (a, b, vmax) in enumerate(zip(q0, q1, limits)):
            v = abs(b - a) / dt
            if v > vmax:
                warnings.append(
                    f"{t0:.3f}s -> {t1:.3f}s: {joint_names[j]} needs "
                    f"{math.degrees(v):.0f} deg/s (limit {math.degrees(vmax):.0f})"
                )
    return warnings


def to_duration(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__("trajectory_client")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

    def send_trajectory(self, score, notes=NOTES):
        points, warnings = build_trajectory(score, notes)
        for w in warnings + speed_warnings(points):
            self.get_logger().warn(w)

        self.client.wait_for_server()  # wait for a connection else the program can hang

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for t, positions in points:
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = to_duration(t)
            goal.trajectory.points.append(point)

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted")
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.get_logger().info("Trajectory done")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrajectoryClient()
    node.send_trajectory(MY_SCORE)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
