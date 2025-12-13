#!/usr/bin/env python3
#You have rclpy, nav2_msgs, geometry_msgs, nav_msgs, action_msgs, tf_transformations, websockets, numpy installed on the AMR.
import asyncio
import json
import os
import threading
from typing import Dict, Any, Set

import numpy as np
import websockets #pip install websockets

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.srv import CancelGoal
import tf_transformations


# =========================
#   ROS2 NODE (AMR SIDE)
# =========================

class AMRIPCNode(Node):
    def __init__(self):
        super().__init__("amr_ipc_node")

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pcl_pose_callback, 10
        )

        # Nav2 actions
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_through_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )

        # Cancel services
        self.cancel_nav_pose_srv = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.cancel_nav_through_srv = self.create_client(
            CancelGoal, "/navigate_through_poses/_action/cancel_goal"
        )

        # State
        self.latest_map: OccupancyGrid | None = None
        self.pose = None
        self.saved_stations: Dict[str, Any] = {}

        # File for stations.json
        self.json_path = os.path.join(os.path.dirname(__file__), "stations.json")
        self.load_stations_from_file()

        self.get_logger().info("AMR IPC node started.")

    # ---------- Callbacks ----------

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def pcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        if not self._is_valid_pose(pose):
            self.get_logger().warn("Received invalid pose; skipping.")
            return
        self.pose = pose

    def _is_valid_pose(self, pose) -> bool:
        if any(np.isnan([pose.position.x, pose.position.y])):
            return False
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        norm = np.linalg.norm(q)
        if not np.isclose(norm, 1.0, atol=1e-2):
            return False
        return True

    # ---------- Stations storage ----------

    def load_stations_from_file(self):
        if not os.path.exists(self.json_path):
            self.get_logger().warn(
                f"Station file '{self.json_path}' not found. Starting empty."
            )
            self.saved_stations = {}
            return

        try:
            with open(self.json_path, "r") as f:
                self.saved_stations = json.load(f)
            self.get_logger().info(
                f"Loaded {len(self.saved_stations)} stations from {self.json_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load stations: {e}")
            self.saved_stations = {}

    def save_stations_to_file(self):
        try:
            with open(self.json_path, "w") as f:
                json.dump(self.saved_stations, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.saved_stations)} stations to {self.json_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save stations: {e}")

    def save_current_station(self, name: str):
        if self.pose is None:
            self.get_logger().warn("No pose to save as station.")
            return

        pose = self.pose
        station = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

        if name in self.saved_stations:
            existing = self.saved_stations[name]
            if isinstance(existing, list):
                existing.append(station)
                self.saved_stations[name] = existing
                self.get_logger().info(
                    f"Appended waypoint to station '{name}' (now {len(existing)} waypoints)."
                )
            else:
                self.saved_stations[name] = [existing, station]
                self.get_logger().info(
                    f"Converted station '{name}' to multi-waypoint."
                )
        else:
            self.saved_stations[name] = station
            self.get_logger().info(
                f"Saved new station '{name}' at ({station['x']:.2f}, {station['y']:.2f})"
            )

        self.save_stations_to_file()

    def delete_station(self, name: str):
        if name in self.saved_stations:
            del self.saved_stations[name]
            self.save_stations_to_file()
            self.get_logger().info(f"Deleted station '{name}'")
        else:
            self.get_logger().warn(f"Requested delete for unknown station '{name}'")

    # ---------- Motion & nav ----------

    def move(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot_immediately(self):
        self.move(0.0, 0.0)
        self.cancel_navigation()
        self.get_logger().warn("Emergency stop activated!")

    def cancel_navigation(self):
        self.move(0.0, 0.0)
        self.get_logger().info(
            "Sending cancel for both NavigateToPose and NavigateThroughPoses."
        )

        # Cancel navigate_to_pose
        if self.cancel_nav_pose_srv.wait_for_service(timeout_sec=2.0):
            req = CancelGoal.Request()
            future = self.cancel_nav_pose_srv.call_async(req)

            def _cb(f):
                try:
                    res = f.result()
                    if res and len(res.goals_canceling) > 0:
                        self.get_logger().info("NavigateToPose goal cancelled.")
                    else:
                        self.get_logger().warn(
                            "NavigateToPose cancel: no active goal or failed."
                        )
                except Exception as e:
                    self.get_logger().error(
                        f"Error cancelling NavigateToPose: {e}"
                    )

            future.add_done_callback(_cb)
        else:
            self.get_logger().warn(
                "navigate_to_pose cancel service not available."
            )

        # Cancel navigate_through_poses
        if self.cancel_nav_through_srv.wait_for_service(timeout_sec=2.0):
            req = CancelGoal.Request()
            future = self.cancel_nav_through_srv.call_async(req)

            def _cb2(f):
                try:
                    res = f.result()
                    if res and len(res.goals_canceling) > 0:
                        self.get_logger().info(
                            "NavigateThroughPoses goal cancelled."
                        )
                    else:
                        self.get_logger().warn(
                            "NavigateThroughPoses cancel: no active goal or failed."
                        )
                except Exception as e:
                    self.get_logger().error(
                        f"Error cancelling NavigateThroughPoses: {e}"
                    )

            future.add_done_callback(_cb2)
        else:
            self.get_logger().warn(
                "navigate_through_poses cancel service not available."
            )

    def _dict_to_pose_stamped(self, station_data: Dict[str, Any]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = station_data["x"]
        msg.pose.position.y = station_data["y"]
        msg.pose.position.z = 0.0

        ori = station_data.get("orientation", {})
        msg.pose.orientation.x = ori.get("x", 0.0)
        msg.pose.orientation.y = ori.get("y", 0.0)
        msg.pose.orientation.z = ori.get("z", 0.0)
        msg.pose.orientation.w = ori.get("w", 1.0)

        return msg

    def goto_station(self, name: str):
        if name not in self.saved_stations:
            self.get_logger().warn(f"Station '{name}' not found.")
            return

        data = self.saved_stations[name]

        if isinstance(data, list):
            # Multi-waypoint -> NavigateThroughPoses
            if not self.nav_through_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(
                    "NavigateThroughPoses action server not available."
                )
                return
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._dict_to_pose_stamped(p) for p in data]
            self.nav_through_client.send_goal_async(goal)
            self.get_logger().info(
                f"Sent NavigateThroughPoses goal with {len(goal.poses)} waypoints for station '{name}'"
            )
        else:
            # Single goal -> NavigateToPose (publish topic or use action)
            msg = self._dict_to_pose_stamped(data)
            self.get_logger().info(
                f"Sending navigation goal to station '{name}' "
                f"({data['x']:.2f}, {data['y']:.2f})"
            )

            # Use NavigateToPose action
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(
                    "NavigateToPose action server not available."
                )
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = msg

            self.nav_client.send_goal_async(goal_msg)
            self.get_logger().info(f"NavigateToPose goal sent for '{name}'")

    # ---------- Initial pose from pixels ----------

    def pixel_to_map_coords(
        self, px: float, py: float, widget_w: float, widget_h: float
    ):
        """
        Convert pixel (px,py) from front-end image into map (x,y) using latest map.
        `widget_w` and `widget_h` are the displayed image size on the UI.
        """
        if self.latest_map is None:
            self.get_logger().warn("No map available for pixel→map conversion.")
            return None, None

        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin.position

        map_aspect = width / float(height)
        widget_aspect = widget_w / float(widget_h)

        if widget_aspect > map_aspect:
            # limited by height
            scale = widget_h / float(height)
            offset_x = (widget_w - width * scale) / 2.0
            offset_y = 0.0
        else:
            # limited by width
            scale = widget_w / float(width)
            offset_x = 0.0
            offset_y = (widget_h - height * scale) / 2.0

        img_x = (px - offset_x) / scale
        img_y = (py - offset_y) / scale

        # map coordinate
        map_x = origin.x + img_x * res
        map_y = origin.y + (height - img_y) * res
        return map_x, map_y

    def publish_initial_pose(self, x: float, y: float, theta: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        pub.publish(msg)
        self.get_logger().info(
            f"Published initial pose: x={x:.2f}, y={y:.2f}, θ={theta:.2f} rad"
        )


# =========================
#   WEBSOCKET IPC SERVER
# =========================

connected_clients: Set[websockets.WebSocketServerProtocol] = set()
node: AMRIPCNode | None = None


async def broadcast(msg: Dict[str, Any]):
    """Send JSON message to all connected ground clients."""
    if not connected_clients:
        return
    data = json.dumps(msg)
    dead = []
    for ws in connected_clients:
        try:
            await ws.send(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        connected_clients.discard(ws)


async def handle_client(websocket):
    global node
    connected_clients.add(websocket)
    print("[IPC] command-center connected")

    # On connect: send current stations
    if node is not None:
        await websocket.send(
            json.dumps(
                {"type": "stations_data", "stations": node.saved_stations}
            )
        )

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                print("[IPC] bad JSON from ground:", raw)
                continue

            msg_type = msg.get("type")
            if node is None:
                print("[IPC] Node not ready yet, ignoring message:", msg)
                continue

            # =========== COMMANDS FROM GROUND ===========
            if msg_type == "move":
                lin = float(msg.get("linear", 0.0))
                ang = float(msg.get("angular", 0.0))
                node.move(lin, ang)

            elif msg_type == "emergency_stop":
                node.stop_robot_immediately()

            elif msg_type == "cancel_navigation":
                node.cancel_navigation()

            elif msg_type == "goto_station":
                name = (msg.get("name") or "").strip()
                if name:
                    node.goto_station(name)

            elif msg_type == "save_station":
                name = (msg.get("name") or "").strip()
                if name:
                    node.save_current_station(name)
                    # send updated station list back
                    await broadcast(
                        {"type": "stations_data", "stations": node.saved_stations}
                    )

            elif msg_type == "delete_station":
                name = (msg.get("name") or "").strip()
                if name:
                    node.delete_station(name)
                    await broadcast(
                        {"type": "stations_data", "stations": node.saved_stations}
                    )

            elif msg_type == "set_initial_pose":
                px = float(msg.get("px", 0.0))
                py = float(msg.get("py", 0.0))
                widget_w = float(msg.get("width", 1.0))
                widget_h = float(msg.get("height", 1.0))

                mx, my = node.pixel_to_map_coords(px, py, widget_w, widget_h)
                if mx is not None and my is not None:
                    node.publish_initial_pose(mx, my, theta=0.0)
                else:
                    node.get_logger().warn("Could not compute map coords for initial pose.")

            elif msg_type == "save_stations":
                # Optional: handle bulk update if ground ever sends it
                new_stations = msg.get("stations")
                if isinstance(new_stations, dict):
                    node.saved_stations = new_stations
                    node.save_stations_to_file()
                    await broadcast(
                        {"type": "stations_data", "stations": node.saved_stations}
                    )

            else:
                print("[IPC] unknown msg from ground:", msg_type, msg)

    except websockets.exceptions.ConnectionClosed:
        print("[IPC] command-center disconnected")
    finally:
        connected_clients.discard(websocket)


async def push_updates_task():
    """Periodically send odometry text to ground."""
    global node
    while True:
        if node is not None and connected_clients:
            if node.pose is not None:
                x = float(node.pose.position.x)
                y = float(node.pose.position.y)
                odom_text = f"Position: x={x:.2f}, y={y:.2f}"
            else:
                odom_text = "Waiting for odometry..."

            await broadcast({"type": "odom_station", "text": odom_text})
        await asyncio.sleep(1.0)


async def main_async():
    # Start WebSocket server on AMR
    server = await websockets.serve(handle_client, "0.0.0.0", 8765)
    print("[IPC] WebSocket server listening on ws://0.0.0.0:8765")

    asyncio.create_task(push_updates_task())
    await server.wait_closed()


def ros_spin_thread():
    rclpy.spin(node)


def main():
    global node

    rclpy.init()
    node = AMRIPCNode()

    # Spin ROS in background thread
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("[IPC] Shutting down...")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
