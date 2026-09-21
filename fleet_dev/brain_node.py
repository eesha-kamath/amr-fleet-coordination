#!/usr/bin/env python3
"""
brain_node.py -- FINAL v3

v3 fix: asymmetric stop-or-go. Previously the caution-zone check
stopped BOTH robots whenever they got close, regardless of the
priority tiebreak -- so the "winner" from path planning still froze
in the control loop, producing mutual standoffs instead of one robot
passing. Now only the priority LOSER stops in the caution zone; the
winner keeps moving. A separate, smaller, priority-blind hard-safety
distance still applies to everyone as a true last-resort failsafe.

Usage: python3 brain_node.py <robot_name> <goal_x> <goal_y> [smart|baseline]
"""

import sys, time, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import warehouse_grid as grid
import motion_controller as motion
from path_planner import plan_path, PeerIntent
from intent_codec import encode_intent, decode_intent, is_stale
from metrics import log_run
from lamport_clock import LamportClock
from conflict_resolver import PriorityInfo, wins, find_cycle_from, pick_deadlock_victim

REPLAN_INTERVAL = 3.0
BROADCAST_INTERVAL = 0.5
CONTROL_INTERVAL = 0.1
DEADLOCK_CHECK_INTERVAL = 1.0
PEER_STALE_AGE = 2.0
LIVE_STALE_AGE = 1.5
YIELD_DURATION = 4.0

class FleetBrainNode(Node):
    def __init__(self, robot_name, goal_x, goal_y, mode="smart"):
        super().__init__(f'brain_{robot_name}')
        self.robot_name = robot_name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.mode = mode

        self.x = self.y = self.yaw = 0.0
        self.battery = 100.0
        self._odom_received = False
        self.start_x = self.start_y = None

        self.peers = {}
        self.current_plan_cells = []
        self.current_plan_times = []
        self.task_start_time = time.time()
        self.goal_reached = False

        self.clock = LamportClock()
        self.waiting_for = None
        self.wait_started_at = None
        self._yield_until = 0.0
        self._active_cycle_key = None

        self.collision_count = 0
        self.near_miss_count = 0
        self.emergency_stop_count = 0
        self.replan_count = 0
        self.deadlocks_detected = 0
        self.deadlocks_broken = 0
        self._collision_active = self._near_miss_active = self._estop_active = False

        self.cmd_pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        self.intent_pub = self.create_publisher(String, '/fleet/intent', 10)
        self.create_subscription(Odometry, f'/{robot_name}/odom', self._odom_cb, 10)
        self.create_subscription(String, '/fleet/intent', self._intent_cb, 10)

        self.create_timer(REPLAN_INTERVAL, self._replan)
        self.create_timer(BROADCAST_INTERVAL, self._broadcast_intent)
        self.create_timer(CONTROL_INTERVAL, self._control_step)
        if self.mode == "smart":
            self.create_timer(DEADLOCK_CHECK_INTERVAL, self._check_deadlock)

        self.get_logger().info(f'{robot_name} brain started [{mode}], goal=({goal_x},{goal_y})')

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if not self._odom_received:
            self.start_x, self.start_y = self.x, self.y
            self._odom_received = True
        q = msg.pose.pose.orientation
        self.yaw = motion.yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _intent_cb(self, msg):
        try:
            data = decode_intent(msg.data)
        except Exception:
            return
        if data['robot_id'] == self.robot_name:
            return
        self.clock.update(data.get('lamport_time', 0))
        data['received_at'] = time.time()
        self.peers[data['robot_id']] = data

    def _active_peer_intents(self):
        now = time.time()
        result = {}
        for rid, data in self.peers.items():
            if is_stale(data['sent_at'], now, PEER_STALE_AGE):
                continue
            cells = [tuple(c) for c in data['planned_cells']]
            result[rid] = PeerIntent(robot_id=rid, planned_cells=cells,
                                      planned_times=data['planned_times'],
                                      current_x=data['x'], current_y=data['y'])
        return result

    def _live_peer_positions(self):
        now = time.time()
        return {rid: (d['x'], d['y']) for rid, d in self.peers.items()
                if not is_stale(d['sent_at'], now, LIVE_STALE_AGE)}

    def _peer_priorities(self, now):
        result = {}
        for rid, data in self.peers.items():
            if is_stale(data['sent_at'], now, PEER_STALE_AGE):
                continue
            result[rid] = PriorityInfo(robot_id=rid,
                                        lamport_time=data.get('lamport_time', 0),
                                        wait_started_at=data.get('wait_started_at'))
        return result

    def _my_priority_info(self):
        return PriorityInfo(robot_id=self.robot_name, lamport_time=self.clock.time,
                             wait_started_at=self.wait_started_at)

    def _am_i_yielding_to(self, peer_id, now):
        """True = I lose the tiebreak against peer_id and should stop. Unknown peer -> be safe, yield."""
        peer_data = self.peers.get(peer_id)
        if peer_data is None:
            return True
        peer_info = PriorityInfo(robot_id=peer_id,
                                  lamport_time=peer_data.get('lamport_time', 0),
                                  wait_started_at=peer_data.get('wait_started_at'))
        return not wins(self._my_priority_info(), peer_info, now)

    def _replan(self):
        if self.goal_reached or time.time() < self._yield_until:
            return
        start_cell = grid.world_to_grid(self.x, self.y)
        goal_cell = grid.world_to_grid(self.goal_x, self.goal_y)
        if start_cell == goal_cell:
            self.current_plan_cells = []
            return

        now = time.time()
        if self.mode == "baseline":
            peer_intents, priorities, my_info = {}, None, None
        else:
            peer_intents = self._active_peer_intents()
            priorities = self._peer_priorities(now)
            my_info = self._my_priority_info()

        result = plan_path(start_cell, goal_cell, start_time=now, robot_id=self.robot_name,
                            peer_intents=peer_intents, priorities=priorities,
                            my_priority_info=my_info, now=now)

        if result is None:
            self.get_logger().warn(f'{self.robot_name}: no path found')
            return

        self.current_plan_cells = result.path_cells
        self.current_plan_times = result.arrival_times
        self.replan_count += 1
        self.get_logger().info(
            f'{self.robot_name} [{self.mode}]: replanned, {len(result.path_cells)} cells, '
            f'cost={result.total_cost:.1f}, peers_known={len(peer_intents)}')

    def _broadcast_intent(self):
        self.clock.increment()
        msg = String()
        msg.data = encode_intent(
            robot_id=self.robot_name, x=self.x, y=self.y,
            goal_x=self.goal_x, goal_y=self.goal_y,
            planned_cells=[list(c) for c in self.current_plan_cells],
            planned_times=self.current_plan_times, battery=self.battery,
            lamport_time=self.clock.time, waiting_for=self.waiting_for,
            wait_started_at=self.wait_started_at)
        self.intent_pub.publish(msg)

    def _check_deadlock(self):
        now = time.time()
        graph = {self.robot_name: self.waiting_for}
        for rid, data in self.peers.items():
            if not is_stale(data['sent_at'], now, PEER_STALE_AGE):
                graph[rid] = data.get('waiting_for')

        cycle = find_cycle_from(self.robot_name, graph)
        cycle_key = tuple(sorted(cycle)) if cycle else None
        if cycle is None:
            self._active_cycle_key = None
            return
        if cycle_key == self._active_cycle_key:
            return

        self._active_cycle_key = cycle_key
        self.deadlocks_detected += 1
        priorities = self._peer_priorities(now)
        priorities[self.robot_name] = self._my_priority_info()
        victim = pick_deadlock_victim(cycle, priorities, now)

        self.get_logger().warn(f'{self.robot_name}: DEADLOCK cycle {cycle}, victim={victim}')
        if victim == self.robot_name:
            self.deadlocks_broken += 1
            self._yield_until = now + YIELD_DURATION
            self.get_logger().warn(f'{self.robot_name}: yielding {YIELD_DURATION}s to break deadlock')

    def _on_goal_reached(self):
        self.goal_reached = True
        task_time = time.time() - self.task_start_time
        self.current_plan_cells = []
        self.get_logger().info(
            f'*** {self.robot_name} REACHED GOAL in {task_time:.1f}s [mode={self.mode} '
            f'collisions={self.collision_count} near_misses={self.near_miss_count} '
            f'emergency_stops={self.emergency_stop_count} replans={self.replan_count} '
            f'deadlocks_detected={self.deadlocks_detected} deadlocks_broken={self.deadlocks_broken}] ***')
        log_run(self.mode, self.robot_name, self.start_x, self.start_y, self.goal_x, self.goal_y,
                task_time, self.collision_count, self.near_miss_count, self.emergency_stop_count,
                self.replan_count, self.deadlocks_detected, self.deadlocks_broken)

    def _update_collision_metrics(self, min_dist):
        """Pure metrics -- counts actual proximity events. No stop decisions here."""
        if min_dist is not None and min_dist < motion.COLLISION_DISTANCE:
            if not self._collision_active:
                self.collision_count += 1
                self._collision_active = True
                self.get_logger().error(f'{self.robot_name}: COLLISION (dist={min_dist:.2f}m)')
        else:
            self._collision_active = False

        if min_dist is not None and min_dist < motion.NEAR_MISS_DISTANCE:
            if not self._near_miss_active:
                self.near_miss_count += 1
                self._near_miss_active = True
        else:
            self._near_miss_active = False

    def _set_waiting(self, peer_id, now):
        if not self._estop_active:
            self.emergency_stop_count += 1
            self._estop_active = True
        if self.waiting_for != peer_id:
            self.waiting_for = peer_id
            self.wait_started_at = now

    def _clear_waiting(self):
        self._estop_active = False
        self.waiting_for = None
        self.wait_started_at = None

    def _control_step(self):
        if not self._odom_received:
            return
        dist_to_goal = motion.distance_to(self.x, self.y, self.goal_x, self.goal_y)
        if not self.goal_reached and dist_to_goal < motion.GOAL_TOLERANCE:
            self._on_goal_reached()
        if self.goal_reached:
            self.cmd_pub.publish(Twist())
            return

        now = time.time()
        peer_positions = self._live_peer_positions()
        min_dist, min_id = motion.closest_peer_distance(self.x, self.y, peer_positions)
        self._update_collision_metrics(min_dist)

        # Universal, priority-blind failsafe -- applies to EVERYONE, no exceptions.
        if motion.hard_safety_stop_needed(min_dist):
            self.cmd_pub.publish(Twist())
            self._set_waiting(min_id, now)
            return

        if now < self._yield_until:
            self.cmd_pub.publish(Twist())
            return

        if self.mode == "baseline":
            if motion.stop_and_wait_needed(min_dist):
                self.cmd_pub.publish(Twist())
                self._set_waiting(min_id, now)
                return
            self._clear_waiting()
        else:
            if motion.emergency_stop_needed(min_dist):
                if self._am_i_yielding_to(min_id, now):
                    self.cmd_pub.publish(Twist())
                    self._set_waiting(min_id, now)
                    return
                else:
                    self._clear_waiting()  # I have priority -- keep going
            else:
                self._clear_waiting()

        if not self.current_plan_cells:
            self.cmd_pub.publish(Twist())
            return

        while len(self.current_plan_cells) > 1:
            wx, wy = grid.grid_to_world(self.current_plan_cells[0])
            if math.hypot(wx - self.x, wy - self.y) < 0.3:
                self.current_plan_cells.pop(0)
                self.current_plan_times.pop(0)
            else:
                break

        tx, ty = grid.grid_to_world(self.current_plan_cells[0])
        cmd = motion.compute_velocity_command(self.x, self.y, self.yaw, tx, ty)
        twist = Twist()
        twist.linear.x = cmd.linear_x
        twist.angular.z = cmd.angular_z
        self.cmd_pub.publish(twist)

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 brain_node.py <robot_name> <goal_x> <goal_y> [smart|baseline]")
        sys.exit(1)
    robot_name, goal_x, goal_y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    mode = sys.argv[4] if len(sys.argv) > 4 else "smart"
    rclpy.init()
    node = FleetBrainNode(robot_name, goal_x, goal_y, mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
