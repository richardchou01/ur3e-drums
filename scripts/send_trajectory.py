import rclpy
import math
import random
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

# time, joint positions
# trajectories = [
#     (2.0, [0.0, -1.5, 1.5, -1.5, -1.5, 0.0]),
#     (4.0, [1.5, -1.5, 1.5, -1.5, -1.5, 0.0]),
#     (6.0, [-1.5, -1.5, 1.5, -1.5, -1.5, 0.0]),
#     (8.0, [0.0, -1.5, 1.5, -1.5, -1.5, 0.0]),
# ]

time = 1.0
hz = 8
step = 1.0 / hz

def to_duration(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))

def to_radians(degree):
    return (degree / 360.0) * 2 * math.pi

# trajectories (time, [offsets])
hit = (1 / 8.0, [to_radians(-120.0), to_radians(-7.42), to_radians(7.6), to_radians(-43.6), to_radians(-1.35), to_radians(76.5)])
rebound = (1 / 8.0, [to_radians(-120.0), to_radians(-10.0), to_radians(7.43), to_radians(-56.13), to_radians(-1.43), to_radians(62.11)])

# def construct_nodes(nodes):
    

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__("trajectory_client")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

    def send_trajectory(self):
        self.client.wait_for_server() # wait for a connection else the program can hang

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        
        trajectories = []
        time = 1.0
        hz = 8
        step = 1.0 / hz
        for i in range(100):
            ratio = random.choice([0.5, 1.0, 2.0])
            time = time + (step / ratio)
            # self.get_logger().info("Hz: " + (hz / ratio))
            trajectories.append((time, [to_radians(-120.0), to_radians(-7.42), to_radians(7.6), to_radians(-43.6), to_radians(-1.35), to_radians(76.5)])) # hit drum
            time = time + step
            trajectories.append((time, [to_radians(-120.0), to_radians(-10.0), to_radians(7.43), to_radians(-56.13), to_radians(-1.43), to_radians(62.11)])) # rebound
        for t, positions in trajectories:
            point = JointTrajectoryPoint()
            point.positions = positions
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
    node.send_trajectory()
    rclpy.spin(node)

if __name__ == "__main__":
    main()