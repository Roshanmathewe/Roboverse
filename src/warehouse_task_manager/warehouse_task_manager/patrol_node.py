#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory


class WarehousePatrol(Node):
    def __init__(self):
        super().__init__("roboverse_autonomous_patrol")

        self.locations = self.load_locations()
        self.route = [
            name for name in
            ["home", "shelf_a", "shelf_b", "shelf_c", "home"]
            if name in self.locations
        ]

        self.index = 0
        self.goal_handle = None
        self.goal_pending = False
        self.current_goal_name = None
        self.mission_active = False
        self.cancel_requested = False

        self.current_shelf = None
        self.inspect_until = None
        self.camera_return_until = None
        self.phase = "Waiting for navigation"
        self.latest_package_count = 0
        self.max_inspection_count = 0
        self.shelf_counts = {}
        self.inspection_seconds = 8.0
        self.inspection_angles = [0.0, -0.25, -0.50]
        self.inspection_level_names = ["lower", "middle", "upper"]
        self.inspection_level_index = 0
        self.inspection_phase = None
        self.phase_until = None
        self.spin_pending = False
        self.spin_goal_handle = None
        self.spin_purpose = None

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.status_pub = self.create_publisher(String, "/roboverse/status", 10)
        self.inventory_pub = self.create_publisher(
            String, "/roboverse/inventory", 10
        )
        self.camera_command_pub = self.create_publisher(
            Float64,
            "/camera_joint_cmd_pos",
            10,
        )

        self.create_subscription(
            String, "/detections", self.detection_cb, 10
        )
        self.create_subscription(
            GoalStatusArray,
            "/pickup_deliver/_action/status",
            self.mission_status_cb,
            10
        )
        self.create_timer(2.0, self.tick)

        self.publish_inventory()
        self.get_logger().info(
            "Shelf inspection patrol ready: " + " -> ".join(self.route)
        )

    def load_locations(self):
        share = get_package_share_directory("warehouse_simulation")
        path = Path(share) / "config" / "locations.yaml"
        with path.open() as f:
            return yaml.safe_load(f)["locations"]

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def set_camera_angle(self, angle):
        msg = Float64()
        msg.data = float(angle)
        self.camera_command_pub.publish(msg)

    def publish_inventory(self):
        msg = String()
        msg.data = json.dumps({
            "phase": self.phase,
            "current_shelf": self.current_shelf,
            "observed_count": self.max_inspection_count,
            "shelves": self.shelf_counts,
        })
        self.inventory_pub.publish(msg)

    def detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            self.latest_package_count = sum(
                1 for d in detections
                if str(d.get("class_name", "")).lower() == "package"
            )
            if self.inspection_phase is not None:
                self.max_inspection_count = max(
                    self.max_inspection_count,
                    self.latest_package_count
                )
                self.publish_inventory()
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning("Could not parse YOLO detections.")

    def mission_status_cb(self, msg):
        active_codes = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = any(s.status in active_codes for s in msg.status_list)

        if active and not self.mission_active:
            self.get_logger().info("Pickup/delivery active; pausing patrol.")
            self.phase = "Pickup/delivery mission active"
            self.publish_status(self.phase)
            if self.goal_handle is not None and not self.cancel_requested:
                self.cancel_requested = True
                self.goal_handle.cancel_goal_async()

        elif not active and self.mission_active:
            self.get_logger().info("Mission finished; patrol will resume.")
            self.phase = "Autonomous patrol resuming"
            self.publish_status(self.phase)

        self.mission_active = active

    def tick(self):
        if self.mission_active or self.goal_pending or self.goal_handle:
            return

        # Handle shelf inspection before sending another navigation goal.
        if self.inspection_phase is not None:
            now = time.monotonic()
            phase = self.inspection_phase

            if phase == "levels":
                if self.phase_until is not None and now < self.phase_until:
                    level = self.inspection_level_names[self.inspection_level_index]
                    remaining = max(0, int(self.phase_until - now))
                    self.phase = f"Inspecting {self.current_shelf}: {level} level"
                    self.publish_status(f"{self.phase} | {remaining}s remaining")
                    self.publish_inventory()
                    return

                self.inspection_level_index += 1
                if self.inspection_level_index < len(self.inspection_angles):
                    idx = self.inspection_level_index
                    self.set_camera_angle(self.inspection_angles[idx])
                    self.phase_until = now + 2.5
                    self.phase = (
                        f"Inspecting {self.current_shelf}: "
                        f"{self.inspection_level_names[idx]} level"
                    )
                    self.publish_status(self.phase)
                    self.publish_inventory()
                    return

                self.inspection_phase = "spin_out"
                self.phase_until = None
                return

            if phase == "spin_out":
                self._start_inspection_spin(1.57079632679, "spin_out")
                return

            if phase == "spin_out_wait":
                self.phase = f"Rotating 90 degrees at {self.current_shelf}"
                self.publish_status(self.phase)
                return

            if phase == "final_sweep":
                if self.phase_until is not None and now < self.phase_until:
                    self.phase = f"Final camera sweep at {self.current_shelf}"
                    self.publish_status(self.phase)
                    self.publish_inventory()
                    return

                self.inspection_phase = "spin_back"
                self.phase_until = None
                return

            if phase == "spin_back":
                self._start_inspection_spin(-1.57079632679, "spin_back")
                return

            if phase == "spin_back_wait":
                self.phase = f"Restoring heading at {self.current_shelf}"
                self.publish_status(self.phase)
                return

            if phase == "camera_return":
                if self.phase_until is not None and now < self.phase_until:
                    self.phase = "Returning camera to forward view"
                    self.publish_status(self.phase)
                    self.publish_inventory()
                    return

                shelf = self.current_shelf
                if shelf is not None:
                    self.shelf_counts[shelf] = self.max_inspection_count
                    self.get_logger().info(
                        f"{shelf} inspection complete: "
                        f"up to {self.max_inspection_count} package(s) "
                        "visible in one frame."
                    )

                self.inspection_phase = None
                self.phase_until = None
                self.current_shelf = None
                self.phase = f"Completed inspection of {shelf}"
                self.publish_status(self.phase)
                self.publish_inventory()
                return

            if phase == "spin_failed":
                self.phase = (
                    f"Inspection paused at {self.current_shelf}: "
                    "Nav2 rotation failed; robot held for safety"
                )
                self.publish_status(self.phase)
                return

            self.get_logger().error(
                f"Unknown inspection phase {phase!r}; holding patrol."
            )
            self.inspection_phase = "spin_failed"
            return

        if not self.route:
            self.phase = "Patrol unavailable: no configured locations"
            self.publish_status(self.phase)
            return

        if not self.nav.server_is_ready():
            self.phase = "Waiting for Nav2 navigation"
            self.publish_status(self.phase)
            return

        name = self.route[self.index]
        loc = self.locations[name]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc["x"])
        goal.pose.pose.position.y = float(loc["y"])

        yaw = float(loc.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.current_goal_name = name
        self.goal_pending = True
        self.phase = f"Navigating to {name}"
        self.publish_status(self.phase)
        self.publish_inventory()
        self.get_logger().info(f"Patrol goal: {name}")

        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def _start_inspection_spin(self, angle, purpose):
        if self.spin_pending:
            return

        if not self.spin_client.server_is_ready():
            self.phase = "Waiting for Nav2 rotation action"
            self.publish_status(self.phase)
            return

        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        self.spin_pending = True
        self.spin_purpose = purpose
        self.inspection_phase = f"{purpose}_wait"
        degrees = math.degrees(angle)
        self.get_logger().info(
            f"Starting inspection rotation: {degrees:.0f} degrees"
        )
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_goal_response_cb)

    def _spin_goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.spin_pending = False
            self.inspection_phase = "spin_failed"
            self.get_logger().error(f"Could not submit inspection rotation: {e}")
            return

        if not handle.accepted:
            self.spin_pending = False
            self.inspection_phase = "spin_failed"
            self.get_logger().error("Nav2 rejected the inspection rotation.")
            return

        self.spin_goal_handle = handle
        handle.get_result_async().add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future):
        purpose = self.spin_purpose
        self.spin_pending = False
        self.spin_goal_handle = None

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.inspection_phase = "spin_failed"
            self.get_logger().error(f"Inspection rotation result error: {e}")
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.inspection_phase = "spin_failed"
            self.get_logger().error(
                f"Inspection rotation failed with status {status}."
            )
            return

        now = time.monotonic()
        if purpose == "spin_out":
            self.set_camera_angle(-0.25)
            self.inspection_phase = "final_sweep"
            self.phase_until = now + 2.5
            self.phase = f"Final camera sweep at {self.current_shelf}"
        elif purpose == "spin_back":
            self.set_camera_angle(0.0)
            self.inspection_phase = "camera_return"
            self.phase_until = now + 1.5
            self.phase = "Returning camera to forward view"
        else:
            self.inspection_phase = "spin_failed"
            self.get_logger().error(
                f"Unexpected rotation purpose: {purpose!r}"
            )
            return

        self.publish_status(self.phase)
        self.publish_inventory()

    def goal_response_cb(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Could not send patrol goal: {e}")
            self.publish_status("Patrol goal submission failed; retrying")
            return

        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected patrol goal; retrying.")
            self.publish_status("Patrol goal rejected; retrying")
            return

        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"Patrol result error: {e}")
            self.publish_status("Patrol result error; retrying")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            reached = self.current_goal_name
            self.index = (self.index + 1) % len(self.route)

            if self.mission_active:
                self.phase = "Patrol paused for pickup/delivery"
                self.publish_status(self.phase)
                return

            if reached in ("shelf_a", "shelf_b", "shelf_c"):
                self.current_shelf = reached
                self.max_inspection_count = 0
                self.inspection_level_index = 0
                self.inspection_phase = "levels"
                self.phase_until = time.monotonic() + 2.5
                self.set_camera_angle(self.inspection_angles[0])
                self.phase = f"Inspecting {reached}: lower level"
                self.publish_status(
                    f"{self.phase} | robot stopped at inspection waypoint"
                )
                self.publish_inventory()
                self.get_logger().info(
                    f"Stopped at {reached}; beginning three-level inspection."
                )
            else:
                self.phase = f"Reached {reached}"
                self.publish_status(self.phase)
                self.publish_inventory()
        elif self.mission_active:
            self.publish_status("Patrol paused for pickup/delivery")
        else:
            self.get_logger().warning(
                f"Patrol goal ended with Nav2 status {status}; retrying."
            )
            self.publish_status("Navigation interrupted; retrying route")


def main():
    rclpy.init()
    node = WarehousePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
