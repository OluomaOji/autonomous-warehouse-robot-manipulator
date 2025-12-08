#!/usr/bin/env python3
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import moveit_commander
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
import os

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # MoveIt! initialization
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Gripper service
        self.gripper_service_client = self.create_client(SetBool, 'gripper_control')
        while not self.gripper_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper control service...')

    # ---------------- Navigation ----------------
    def navigate_to(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_action_client.wait_for_server()
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        return result.success

    # ---------------- TF Pose ----------------
    def get_box_pose(self, tag_frame="box1", base_frame="base_link") -> PoseStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tag_frame,
                rclpy.time.Time())
            box_pose = PoseStamped()
            box_pose.header.frame_id = base_frame
            box_pose.header.stamp = self.get_clock().now().to_msg()
            box_pose.pose.position.x = transform.transform.translation.x
            box_pose.pose.position.y = transform.transform.translation.y
            box_pose.pose.position.z = transform.transform.translation.z + 0.1  # hover above box
            box_pose.pose.orientation = transform.transform.rotation
            return box_pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {tag_frame}: {e}")
            return None

    # ---------------- Gripper ----------------
    def control_gripper(self, activate: bool):
        request = SetBool.Request()
        request.data = activate
        future = self.gripper_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.success

    def close_gripper(self):
        return self.control_gripper(True)

    def open_gripper(self):
        return self.control_gripper(False)

    # ---------------- Arm ----------------
    def move_arm_to_pose(self, pose: PoseStamped):
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return plan

    # ---------------- Pick & Place ----------------
    def pick_and_place(self, pick_pose: PoseStamped, place_pose: PoseStamped):
        self.get_logger().info("Moving to pick pose...")
        self.move_arm_to_pose(pick_pose)
        time.sleep(0.5)

        self.get_logger().info("Closing gripper...")
        self.close_gripper()
        time.sleep(0.5)

        self.get_logger().info("Moving to place pose...")
        self.move_arm_to_pose(place_pose)
        time.sleep(0.5)

        self.get_logger().info("Opening gripper...")
        self.open_gripper()
        time.sleep(0.5)

        self.get_logger().info("Returning arm to safe position...")
        safe_pose = PoseStamped()
        safe_pose.header.frame_id = "base_link"
        safe_pose.pose.position.x = 0.4
        safe_pose.pose.position.y = 0.0
        safe_pose.pose.position.z = 0.5
        safe_pose.pose.orientation.w = 1.0
        self.move_arm_to_pose(safe_pose)

# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    # Load boxes and shelves from YAML
    pkg_path = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(pkg_path, "../config/warehouse_layout.yaml")
    with open(yaml_path, 'r') as f:
        layout = yaml.safe_load(f)

    boxes = layout["boxes"]
    shelves = layout["shelves"]

    # Loop through all boxes
    for i, box in enumerate(boxes):
        tag_frame = box["tag"]
        pick_pose = node.get_box_pose(tag_frame, "base_link")
        if pick_pose is None:
            node.get_logger().warn(f"Skipping {tag_frame}, pose not found")
            continue

        place_info = shelves[i]
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = place_info["x"]
        place_pose.pose.position.y = place_info["y"]
        place_pose.pose.position.z = place_info["z"]
        place_pose.pose.orientation.w = 1.0

        node.get_logger().info(f"Navigating to {tag_frame}...")
        if node.navigate_to(pick_pose):
            node.pick_and_place(pick_pose, place_pose)
        else:
            node.get_logger().warn(f"Navigation failed for {tag_frame}, skipping")

    node.get_logger().info("All pick-and-place tasks complete.")
    rclpy.shutdown()
