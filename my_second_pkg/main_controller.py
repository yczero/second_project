# main_controller.py
import rclpy
from rclpy.node import Node
import numpy as np
import math
import copy 
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

from my_second_pkg.hybrid_a_star import HybridAStarPlanner
from my_second_pkg.dwa import DWAController

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0

        # QoS 설정
        # QoS 설정
        map_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        scan_qos = qos_profile_sensor_data
        
        goal_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        # 구독
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, pose_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, goal_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, pose_qos)

        # 발행
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)

        # 변수 초기화
        self.static_map_data = None 
        self.current_map_data = None 
        self.robot_pose = None
        self.scan_data = [] 
        self.global_path = None
        self.current_goal = None 
        
        self.planner = HybridAStarPlanner()
        self.dwa = DWAController()

        # 타이머
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.replan_loop)
        
        self.get_logger().info("자율주행 노드 준비 완료! (Replanning Enabled)")

    # -------------------
    # 콜백 함수
    # -------------------
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        raw_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.static_map_data = raw_data.T
        self.current_map_data = copy.deepcopy(self.static_map_data)
        self.get_logger().info(f"맵 수신 완료!")

    def pose_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, 0.0, 0.0]

    def init_pose_callback(self, msg):
        self.pose_callback(msg)
        self.get_logger().info(f"초기 위치 설정됨")

    def scan_callback(self, msg):
        if self.robot_pose is None: return
        obstacles = []
        angle = msg.angle_min
        rx, ry, rtheta = self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]
        lidar_offset_x = -0.064 
        lidar_world_x = rx + (lidar_offset_x * math.cos(rtheta))
        lidar_world_y = ry + (lidar_offset_x * math.sin(rtheta))

        for r in msg.ranges:
            if r < msg.range_min or r > msg.range_max or r == 0.0:
                angle += msg.angle_increment
                continue
            if r < 3.5: 
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                mx = lidar_world_x + (ox * math.cos(rtheta) - oy * math.sin(rtheta))
                my = lidar_world_y + (ox * math.sin(rtheta) + oy * math.cos(rtheta))
                obstacles.append([mx, my])
            angle += msg.angle_increment
        self.scan_data = np.array(obstacles)

    def update_map_with_scan(self):
        if self.static_map_data is None or len(self.scan_data) == 0:
            return
        self.current_map_data = copy.deepcopy(self.static_map_data)
        for obs in self.scan_data:
            ix = int((obs[0] - self.map_origin_x) / self.map_resolution)
            iy = int((obs[1] - self.map_origin_y) / self.map_resolution)
            margin = 2
            for i in range(ix-margin, ix+margin+1):
                for j in range(iy-margin, iy+margin+1):
                    if 0 <= i < self.map_width and 0 <= j < self.map_height:
                        self.current_map_data[i][j] = 100 

    # -------------------
    # Goal 처리
    # -------------------
    def goal_callback(self, msg):
        self.current_goal = [msg.pose.position.x, msg.pose.position.y, 0.0]
        self.get_logger().info(f"Goal received: {self.current_goal}, frame_id: {msg.header.frame_id}")
        self.perform_planning()

    def replan_loop(self):
        if self.current_goal is not None and self.robot_pose is not None:
            dist = math.hypot(self.current_goal[0] - self.robot_pose[0], 
                              self.current_goal[1] - self.robot_pose[1])
            if dist > 0.5: 
                self.perform_planning()

    # -------------------
    # 경로 계획
    # -------------------
    def perform_planning(self):
        if self.static_map_data is None or self.robot_pose is None or self.current_goal is None:
            return

        self.update_map_with_scan()

        start_map_x = self.robot_pose[0] - self.map_origin_x
        start_map_y = self.robot_pose[1] - self.map_origin_y
        goal_map_x = self.current_goal[0] - self.map_origin_x
        goal_map_y = self.current_goal[1] - self.map_origin_y
        
        start_node = [start_map_x, start_map_y, self.robot_pose[2]]
        goal_node = [goal_map_x, goal_map_y, 0.0]

        self.get_logger().info(f"Planning from {start_node} to {goal_node}")

        path_in_map = self.planner.plan(start_node, goal_node, self.current_map_data)
        if not path_in_map:
            self.get_logger().warn("Hybrid A* 경로 생성 실패! 좌표나 map 확인 필요.")
            self.global_path = None
            return

        # 경로 변환 및 시각화
        final_path = [[p[0] + self.map_origin_x, p[1] + self.map_origin_y] for p in path_in_map]
        self.global_path = final_path

        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in final_path:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)
        self.get_logger().info(f"Global path published: {len(final_path)} points")

    # -------------------
    # Local goal 계산
    # -------------------
    def get_local_goal(self):
        if self.global_path is None or len(self.global_path) == 0: 
            return None
        search_dist = 0.8
        min_idx = -1
        min_dist = float('inf')
        for i, point in enumerate(self.global_path):
            dist = math.hypot(point[0] - self.robot_pose[0], point[1] - self.robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        curr_dist = 0
        for i in range(min_idx, len(self.global_path)-1):
            p1 = self.global_path[i]
            p2 = self.global_path[i+1]
            curr_dist += math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if curr_dist > search_dist:
                return self.global_path[i+1]
        return self.global_path[-1]

    # -------------------
    # 제어 루프
    # -------------------
    def control_loop(self):
        if self.robot_pose is None: return
        if self.global_path is None:
            self.cmd_pub.publish(Twist()) 
            return

        local_goal = self.get_local_goal()
        if local_goal is None:
            self.cmd_pub.publish(Twist())
            return

        dist_to_final = math.hypot(self.global_path[-1][0] - self.robot_pose[0], 
                                   self.global_path[-1][1] - self.robot_pose[1])
        if dist_to_final < 0.2:
            self.get_logger().info("도착 완료!")
            self.global_path = None
            self.current_goal = None
            self.cmd_pub.publish(Twist())
            return

        # 긴급 후진
        if len(self.scan_data) > 0:
            dx = self.scan_data[:, 0] - self.robot_pose[0]
            dy = self.scan_data[:, 1] - self.robot_pose[1]
            min_dist = np.min(np.hypot(dx, dy))
            if min_dist < 0.18:
                cmd = Twist()
                cmd.linear.x = -0.15
                self.cmd_pub.publish(cmd)
                return

        # 방향 정렬
        curr_yaw = self.robot_pose[2]
        goal_dx = local_goal[0] - self.robot_pose[0]
        goal_dy = local_goal[1] - self.robot_pose[1]
        target_yaw = math.atan2(goal_dy, goal_dx)
        yaw_diff = target_yaw - curr_yaw
        while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi: yaw_diff += 2 * math.pi

        if abs(yaw_diff) > 0.26:
            cmd = Twist()
            cmd.linear.x = 0.0
            turn_speed = 0.5 * (yaw_diff / abs(yaw_diff))
            if abs(yaw_diff) < 0.5: turn_speed *= 0.6
            cmd.angular.z = float(turn_speed)
            self.cmd_pub.publish(cmd)
            return

        # DWA 제어
        obstacles = self.scan_data if len(self.scan_data) > 0 else np.array([[-100,-100]])
        current_state = [self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], 0.0, 0.0] 
        best_u, traj = self.dwa.dwa_control(current_state, local_goal, obstacles)

        cmd = Twist()
        if best_u[0] == 0.0 and best_u[1] == 0.0:
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = best_u[0]
            cmd.angular.z = best_u[1]
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
