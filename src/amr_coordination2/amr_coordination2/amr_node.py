import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import heapq

WAREHOUSE_GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]
GRID_H, GRID_W = len(WAREHOUSE_GRID), len(WAREHOUSE_GRID[0])

class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_peer_node')
        
        self.amr_id = os.getenv('AMR_ID', 'AMR_1')
        self.priority = int(os.getenv('AMR_PRIORITY', '1'))
        start_raw = os.getenv('AMR_START', '0,0').split(',')
        goal_raw = os.getenv('AMR_GOAL', '0,0').split(',')
        
        self.start = (int(start_raw[0]), int(start_raw[1]))
        self.goal = (int(goal_raw[0]), int(goal_raw[1]))
        self.pos = self.start
        self.battery = 100.0
        self.status = "STANDBY"
        self.item_payload = "Idle Bay"
        self.peer_reservations = {}
        self.parked_peers = {}   # Tracks stationary robots {robot_id: (r, c)}
        self.path = []
        self.time_step = 0

        # ROS 2 Subscriptions & Publishers
        self.pub_telemetry = self.create_publisher(String, '/warehouse/telemetry', 10)
        self.pub_mesh = self.create_publisher(String, '/warehouse/p2p_mesh', 10)
        self.sub_mesh = self.create_subscription(String, '/warehouse/p2p_mesh', self.mesh_cb, 10)
        self.sub_dispatch = self.create_subscription(String, '/warehouse/mission_dispatch', self.dispatch_cb, 10)

        # Main Step Execution Loop (2 Hz)
        self.timer = self.create_timer(0.5, self.step_cycle)
        self.get_logger().info(f"[{self.amr_id}] Online | Start: {self.start} -> Goal: {self.goal}")
        self.broadcast_mesh()

    def dispatch_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("target_robot") == self.amr_id:
                self.goal = tuple(data["goal"])
                self.priority = data.get("priority", self.priority)
                self.item_payload = data.get("item", "WMS Order")
                self.get_logger().info(f" [{self.amr_id}] Dispatched to {self.goal} | Priority: {self.priority}")
                self.replan()
        except Exception as e:
            self.get_logger().error(f"Dispatch parse error: {e}")

    def space_time_a_star(self, start, goal, current_t, dynamic_reservations, static_obstacles):
        open_set = [(abs(start[0]-goal[0]) + abs(start[1]-goal[1]), current_t, start, [start])]
        visited = set()
        
        while open_set:
            f, t, curr, path = heapq.heappop(open_set)
            if curr == goal:
                return path
            if t > current_t + 100:
                continue
            if (curr[0], curr[1], t) in visited:
                continue
            visited.add((curr[0], curr[1], t))

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]:
                nr, nc = curr[0] + dx, curr[1] + dy
                nxt_t = t + 1
                
                # Check bounds and physical warehouse walls
                if 0 <= nr < GRID_H and 0 <= nc < GRID_W and WAREHOUSE_GRID[nr][nc] == 0:
                    # 1. Block stationary parked peers (static obstacle)
                    if (nr, nc) in static_obstacles and (nr, nc) != start:
                        continue
                    
                    # 2. Block vertex collision in space-time (x, y, t)
                    if (nr, nc, nxt_t) in dynamic_reservations:
                        continue
                        
                    # 3. Block edge swap collision
                    if (nr, nc, t) in dynamic_reservations and (curr[0], curr[1], nxt_t) in dynamic_reservations:
                        continue
                        
                    h = abs(nr - goal[0]) + abs(nc - goal[1])
                    heapq.heappush(open_set, (nxt_t + h, nxt_t, (nr, nc), path + [(nr, nc)]))
        return [start]

    def mesh_cb(self, msg):
        try:
            data = json.loads(msg.data)
            sender_id = data["sender_id"]
            if sender_id == self.amr_id:
                return
            
            sender_status = data.get("status", "STANDBY")
            sender_pos = tuple(data.get("pos", [0, 0]))
            
            # If peer is parked or idle, record its stationary position as an obstacle
            if sender_status in ["STANDBY", "GOAL_REACHED"] or len(data.get("reservations", [])) <= 1:
                self.parked_peers[sender_id] = sender_pos
                self.peer_reservations.pop(sender_id, None)
            else:
                self.parked_peers.pop(sender_id, None)
                res = [(r[0], r[1], r[2]) for r in data.get("reservations", [])]
                self.peer_reservations[sender_id] = res

            # If a higher priority peer conflicts with our route, re-route around them
            sender_prio = data.get("priority", 1)
            if sender_prio > self.priority and self.status == "NAVIGATING":
                active_conflict = any(
                    (p[0], p[1], self.time_step + idx) in self.peer_reservations.get(sender_id, [])
                    for idx, p in enumerate(self.path)
                ) or any(
                    (p[0], p[1]) == self.parked_peers.get(sender_id)
                    for p in self.path
                )
                if active_conflict:
                    self.get_logger().warn(f"[{self.amr_id}] Conflict detected with {sender_id}! Re-routing...")
                    self.replan()
        except Exception:
            pass

    def replan(self):
        all_reservations = set()
        for res_list in self.peer_reservations.values():
            all_reservations.update(res_list)
            
        static_blocked = set(self.parked_peers.values())
        
        self.path = self.space_time_a_star(self.pos, self.goal, self.time_step, all_reservations, static_blocked)
        self.status = "NAVIGATING" if (self.pos != self.goal and len(self.path) > 1) else "GOAL_REACHED"
        self.broadcast_mesh()

    def broadcast_mesh(self):
        claim_payload = {
            "sender_id": self.amr_id,
            "priority": self.priority,
            "status": self.status,
            "pos": list(self.pos),
            "reservations": [[p[0], p[1], self.time_step + idx] for idx, p in enumerate(self.path)]
        }
        self.pub_mesh.publish(String(data=json.dumps(claim_payload)))

    def step_cycle(self):
        if self.status == "NAVIGATING" and len(self.path) > 1:
            self.path.pop(0)
            self.pos = self.path[0]
            self.battery = max(10.0, round(self.battery - 0.25, 2))
            if self.pos == self.goal:
                self.status = "GOAL_REACHED"
                self.get_logger().info(f" [{self.amr_id}] Arrived at destination {self.goal}!")

        self.time_step += 1
        self.broadcast_mesh()

        telem_payload = {
            "id": self.amr_id,
            "priority": self.priority,
            "pos": list(self.pos),
            "goal": list(self.goal),
            "status": self.status,
            "item": self.item_payload,
            "battery": self.battery,
            "path": [list(p) for p in self.path]
        }
        self.pub_telemetry.publish(String(data=json.dumps(telem_payload)))

def main(args=None):
    rclpy.init(args=args)
    node = AMRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()