import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import heapq
import time

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
MAX_PRIORITY = 10
YIELD_RECHECK_TICKS = 5

class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_peer_node')

        self.amr_id = os.getenv('AMR_ID', 'AMR_1')
        self.priority = int(os.getenv('AMR_PRIORITY', '1'))
        self.base_priority = self.priority
        self.starvation_ticks = 0

        start_raw = os.getenv('AMR_START', '0,0').split(',')
        goal_raw = os.getenv('AMR_GOAL', '0,0').split(',')

        self.start = (int(start_raw[0]), int(start_raw[1]))
        self.goal = (int(goal_raw[0]), int(goal_raw[1]))
        self.pos = self.start
        self.battery = 100.0
        self.status = "STANDBY"
        self.item_payload = "Idle Bay"
        
        self.peer_reservations = {}
        self.parked_peers = {}
        self.live_peers = {}  
        self.peer_priorities = {} # Tracks network SLA claims
        
        self.path = []
        self.time_step = 0
        self.yield_cell = None
        self.yield_ticks = 0

        self.pub_telemetry = self.create_publisher(String, '/warehouse/telemetry', 10)
        self.pub_mesh = self.create_publisher(String, '/warehouse/p2p_mesh', 10)
        self.sub_mesh = self.create_subscription(String, '/warehouse/p2p_mesh', self.mesh_cb, 10)
        self.sub_dispatch = self.create_subscription(String, '/warehouse/mission_dispatch', self.dispatch_cb, 10)

        self.timer = self.create_timer(0.5, self.step_cycle)
        self.broadcast_mesh()

    def dispatch_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("target_robot") == self.amr_id:
                self.goal = tuple(data["goal"])
                self.priority = data.get("priority", self.priority)
                self.base_priority = self.priority
                self.starvation_ticks = 0
                self.item_payload = data.get("item", "WMS Order")
                self.replan()
        except Exception as e:
            pass

    def _id_num(self, id_str):
        try:
            return int(str(id_str).split('_')[-1])
        except (ValueError, IndexError):
            return str(id_str)

    def get_neighbors(self, pos):
        r, c = pos
        return [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]

    def is_walkable(self, cell):
        r, c = cell
        return 0 <= r < GRID_H and 0 <= c < GRID_W and WAREHOUSE_GRID[r][c] == 0

    def all_peer_reservations(self):
        all_res = set()
        for res_list in self.peer_reservations.values():
            all_res.update(res_list)
        return all_res

    def all_static_blocked(self):
        return set(self.parked_peers.values()).union(set(self.live_peers.values()))

    def space_time_a_star(self, start, goal, current_t, dynamic_reservations, static_obstacles):
        open_set = [(abs(start[0] - goal[0]) + abs(start[1] - goal[1]), current_t, start, [start])]
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

                if 0 <= nr < GRID_H and 0 <= nc < GRID_W and WAREHOUSE_GRID[nr][nc] == 0:
                    if (nr, nc) in static_obstacles and (nr, nc) != start:
                        continue
                    if (nr, nc, nxt_t) in dynamic_reservations:
                        continue
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
            sender_prio = data.get("priority", 1)
            
            self.live_peers[sender_id] = sender_pos
            self.peer_priorities[sender_id] = sender_prio

            if sender_status in ["STANDBY", "GOAL_REACHED"] or len(data.get("reservations", [])) <= 1:
                self.parked_peers[sender_id] = sender_pos
                self.peer_reservations.pop(sender_id, None)
            else:
                self.parked_peers.pop(sender_id, None)
                res = [(r[0], r[1], r[2]) for r in data.get("reservations", [])]
                self.peer_reservations[sender_id] = res

            must_yield = False
            if sender_prio > self.priority:
                must_yield = True
            elif sender_prio == self.priority:
                if self._id_num(sender_id) < self._id_num(self.amr_id):
                    must_yield = True

            if must_yield and self.status in ("NAVIGATING", "YIELDING"):
                active_conflict = any(
                    (p[0], p[1], self.time_step + idx) in self.peer_reservations.get(sender_id, [])
                    for idx, p in enumerate(self.path)
                ) or any(
                    (p[0], p[1]) == self.parked_peers.get(sender_id)
                    for p in self.path
                )

                if active_conflict:
                    self.replan()
        except Exception:
            pass

    def replan(self):
        all_reservations = self.all_peer_reservations()
        static_blocked = self.all_static_blocked()

        new_path = self.space_time_a_star(self.pos, self.goal, self.time_step, all_reservations, static_blocked)

        if len(new_path) > 1 or self.pos == self.goal:
            self.path = new_path
            self.yield_cell = None
            self.yield_ticks = 0
            self.status = "NAVIGATING" if (self.pos != self.goal and len(self.path) > 1) else "GOAL_REACHED"
            self.broadcast_mesh()
            return

        escape_cell = self.find_escape_cell(all_reservations, static_blocked)

        if escape_cell:
            self.path = [self.pos, escape_cell]
            self.status = "YIELDING"
            self.yield_cell = escape_cell
            self.yield_ticks = 0
        else:
            self.path = [self.pos]
            self.status = "BLOCKED"

        self.broadcast_mesh()

    def find_escape_cell(self, all_reservations, static_blocked):
        next_t = self.time_step + 1
        forward_cell = self.path[1] if len(self.path) > 1 else None

        candidates = []
        for cell in self.get_neighbors(self.pos):
            if not self.is_walkable(cell):
                continue
            if cell in static_blocked:
                continue
            if (cell[0], cell[1], next_t) in all_reservations:
                continue
            if cell == forward_cell:
                continue
            candidates.append(cell)

        return candidates[0] if candidates else None

    def contested_cell_is_clear(self):
        if not self.goal or self.pos == self.goal:
            return True
        all_reservations = self.all_peer_reservations()
        static_blocked = self.all_static_blocked()
        probe_path = self.space_time_a_star(self.pos, self.goal, self.time_step, all_reservations, static_blocked)
        return len(probe_path) > 1

    def broadcast_mesh(self):
        claim_payload = {
            "sender_id": self.amr_id,
            "priority": self.priority,
            "status": self.status,
            "pos": list(self.pos),
            "reservations": [[p[0], p[1], self.time_step + idx] for idx, p in enumerate(self.path)],
            "time_step": self.time_step
        }
        self.pub_mesh.publish(String(data=json.dumps(claim_payload)))

    def step_cycle(self):
        if self.status == "YIELDING":
            self._step_yielding()
        elif self.status == "BLOCKED":
            self._step_blocked()
        elif self.status == "NAVIGATING" and len(self.path) > 1:
            self._step_navigating()
        else:
            self._step_idle_or_stalled()

        self.time_step += 1
        self.broadcast_mesh()
        self._publish_telemetry()

    def _step_navigating(self):
        next_pos = self.path[1]

        if next_pos in self.live_peers.values():
            self.get_logger().warn(f"[{self.amr_id}] GATE HALT: {next_pos} occupied!")
            self.starvation_ticks += 1
            self.replan()
            return

        for peer_id, res in self.peer_reservations.items():
            conflict_cells = [(r[0], r[1]) for r in res[:2]]
            if (next_pos[0], next_pos[1]) in conflict_cells:
                peer_prio = self.peer_priorities.get(peer_id, 1)
                must_yield = False
                
                if peer_prio > self.priority:
                    must_yield = True
                elif peer_prio == self.priority:
                    if self._id_num(peer_id) < self._id_num(self.amr_id):
                        must_yield = True
                        
                if must_yield:
                    self.get_logger().warn(f"[{self.amr_id}] LOOKAHEAD YIELD: {peer_id} claiming {next_pos}")
                    self.starvation_ticks += 1
                    self.replan()
                    return

        self.path.pop(0)
        self.pos = self.path[0]
        self.battery = max(10.0, round(self.battery - 0.25, 2))
        self.starvation_ticks = 0
        self.priority = self.base_priority

        if self.pos == self.goal:
            self.status = "GOAL_REACHED"

    def _step_yielding(self):
        if self.pos != self.yield_cell:
            if self.yield_cell in self.live_peers.values():
                return
                
            for peer_id, res in self.peer_reservations.items():
                conflict_cells = [(r[0], r[1]) for r in res[:2]]
                if (self.yield_cell[0], self.yield_cell[1]) in conflict_cells:
                    peer_prio = self.peer_priorities.get(peer_id, 1)
                    must_yield = False
                    if peer_prio > self.priority:
                        must_yield = True
                    elif peer_prio == self.priority:
                        if self._id_num(peer_id) < self._id_num(self.amr_id):
                            must_yield = True
                    if must_yield:
                        self.get_logger().warn(f"[{self.amr_id}] ESCAPE ABORT: {peer_id} crossing yield zone.")
                        self.replan()
                        return

            self.pos = self.yield_cell
            self.battery = max(10.0, round(self.battery - 0.25, 2))
            return

        self.starvation_ticks += 1
        if self.starvation_ticks > 10:
            self.priority = min(self.priority + 1, MAX_PRIORITY)
            self.starvation_ticks = 0

        self.yield_ticks += 1
        if self.yield_ticks >= YIELD_RECHECK_TICKS and self.contested_cell_is_clear():
            self.replan()

    def _step_blocked(self):
        self.starvation_ticks += 1
        if self.starvation_ticks > 10:
            self.priority = min(self.priority + 1, MAX_PRIORITY)
            self.starvation_ticks = 0
        self.replan()

    def _step_idle_or_stalled(self):
        if self.status == "NAVIGATING":
            self.starvation_ticks += 1
            if self.starvation_ticks > 10:
                self.priority = min(self.priority + 1, MAX_PRIORITY)
                self.starvation_ticks = 0

    def _publish_telemetry(self):
        telem_payload = {
            "id": self.amr_id,
            "priority": self.priority,
            "pos": list(self.pos),
            "goal": list(self.goal),
            "status": self.status,
            "item": self.item_payload,
            "battery": self.battery,
            "path": [list(p) for p in self.path],
            "time_step": self.time_step,
            "wall_time": time.time()
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