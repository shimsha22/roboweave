import json
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SCADAROSEngine(Node):
    def __init__(self, state_ref):
        super().__init__('scada_master_engine')
        self.state = state_ref
        self.pub_dispatch = self.create_publisher(String, '/warehouse/mission_dispatch', 10)
        self.create_subscription(String, '/warehouse/telemetry', self.telem_cb, 50)
        self.create_subscription(String, '/warehouse/p2p_mesh', self.mesh_cb, 50)

    def telem_cb(self, msg):
        try:
            data = json.loads(msg.data)
            r_id = data["id"]
            new_pos = tuple(data["pos"])
            status = data["status"]
            battery = float(data["battery"])
            path = [tuple(p) for p in data.get("path", [])]
            item = data.get("item", "Cargo Unit")

            with self.state["lock"]:
                if r_id not in self.state["fleet"]:
                    self.state["fleet"][r_id] = {
                        "name": r_id, "start": new_pos, "goal": tuple(data["goal"]),
                        "src_name": "BAY_IN", "dst_name": "STATION_BAY", "item": item,
                        "priority": data["priority"], "status": status, "pos": new_pos,
                        "battery": battery, "path": path, "trail": [new_pos],
                        "ticks": {"cargo": 1, "empty": 0, "docking": 0, "blocked": 0}
                    }
                
                amr = self.state["fleet"][r_id]
                if amr["trail"] and amr["trail"][-1] != new_pos:
                    amr["trail"].append(new_pos)
                
                amr["pos"] = new_pos
                amr["goal"] = tuple(data["goal"])
                amr["status"] = status
                amr["battery"] = battery
                amr["path"] = path
                amr["item"] = item
                
                amr["ticks"]["cargo" if status == "NAVIGATING" else ("docking" if status == "GOAL_REACHED" else "blocked")] += 1

                if status == "GOAL_REACHED" and amr.get("logged_goal") != new_pos:
                    amr["logged_goal"] = new_pos
                    self.state["transport_history"].insert(0, {
                        "Robot": r_id, "Timestamp": time.strftime("%H:%M:%S"),
                        "Source": f"{amr['start']}", "Destination": f"{amr['goal']}",
                        "Item": item, "Ride Time": f"{len(amr['trail']) * 0.5:.1f}s",
                        "Status": "DELIVERED (ROS 2)"
                    })
        except Exception:
            pass

    def mesh_cb(self, msg):
        try:
            data = json.loads(msg.data)
            with self.state["lock"]:
                self.state["mesh_logs"].append(f"[{data['sender_id']}] P{data['priority']} Mesh Token: {data['reservations'][:2]}")
                if len(self.state["mesh_logs"]) > 6:
                    self.state["mesh_logs"].pop(0)
        except Exception:
            pass

    def dispatch_mission(self, target_robot, goal, priority, item_desc):
        payload = {
            "target_robot": target_robot,
            "goal": goal,
            "priority": priority,
            "item": item_desc
        }
        self.pub_dispatch.publish(String(data=json.dumps(payload)))

def init_ros_engine():
    if not rclpy.ok():
        rclpy.init()
    shared_state = {
        "lock": threading.Lock(),
        "fleet": {},
        "mesh_logs": [],
        "transport_history": [],
        "action_point_stats": {"INBOUND_DOCK": 12, "OUTBOUND_BAY": 24, "CENTRAL_CORRIDOR": 18}
    }
    node = SCADAROSEngine(shared_state)
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node, shared_state