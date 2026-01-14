# hybrid_a_star.py
import numpy as np
import math
import heapq

class HybridAStarPlanner:
    def __init__(self, resolution=0.1, grid_width=100, grid_height=100):
        self.reso = resolution
        # 로봇의 회전 반경과 이동 단위 설정
        self.steer_set = [-20, 0, 20]  # 조향 각도 (도) - 좌, 직진, 우
        self.step_size = 0.5           # 한 번에 이동하는 거리 (m)
        self.grid_w = grid_width
        self.grid_h = grid_height

    class Node:
        def __init__(self, x_ind, y_ind, theta_ind, parent, cost, x, y, theta):
            self.x_ind = x_ind
            self.y_ind = y_ind
            self.theta_ind = theta_ind
            self.parent = parent
            self.cost = cost      # G cost
            self.x = x            # 실제 좌표
            self.y = y
            self.theta = theta    # 실제 각도
            self.f = 0            # F = G + H

        def __lt__(self, other):
            return self.f < other.f

    def calc_heuristic(self, node, goal_node):
        # 유클리드 거리 (더 복잡한 Reeds-Shepp 곡선을 쓰기도 함)
        dx = node.x - goal_node.x
        dy = node.y - goal_node.y
        return math.hypot(dx, dy)

    def is_collision(self, x, y, grid_map):
        # 1. 맵 범위 체크
        ix = int(x / self.reso)
        iy = int(y / self.reso)
        if ix < 0 or ix >= self.grid_w or iy < 0 or iy >= self.grid_h:
            return True
            
        # [수정] 단순히 점 하나만 보는 게 아니라 주변 픽셀도 봅니다 (간이 Inflation)
        # 로봇이 지나가려는 점(ix, iy) 주변에 장애물이 있으면 충돌로 처리
        
        margin = 6  # 맵 해상도가 0.05m라면 4칸 = 20cm 여유
        
        # 주변 픽셀 검사 (속도를 위해 사각형 범위로 검사)
        x_min = max(0, ix - margin)
        x_max = min(self.grid_w, ix + margin + 1)
        y_min = max(0, iy - margin)
        y_max = min(self.grid_h, iy + margin + 1)
        
        # 해당 범위 내에 50 이상의 값(장애물)이 하나라도 있으면 True
        # (numpy 슬라이싱을 쓰면 빠르지만, 리스트라면 반복문 필요)
        # 여기서는 리스트라고 가정하고 간단히 구현:
        
        for i in range(x_min, x_max):
            for j in range(y_min, y_max):
                if grid_map[i][j] > 50: # 장애물 발견
                    return True
                    
        return False

    def plan(self, start, goal, grid_map):
        # start/goal: [x, y, theta]
        start_node = self.Node(int(start[0]/self.reso), int(start[1]/self.reso), 0,
                               None, 0, start[0], start[1], start[2])
        goal_node = self.Node(int(goal[0]/self.reso), int(goal[1]/self.reso), 0,
                              None, 0, goal[0], goal[1], goal[2])

        open_set = []
        closed_set = {} # (x_ind, y_ind, theta_ind)를 키로 사용

        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            # 목표 도달 확인 (거리 0.5m 이내면 도착 처리)
            dist_to_goal = math.hypot(current.x - goal_node.x, current.y - goal_node.y)
            if dist_to_goal < 0.5:
                print("Goal Found!")
                return self.reconstruct_path(current)

            # Closed Set 확인
            c_id = (current.x_ind, current.y_ind, int(current.theta / math.radians(10)))
            if c_id in closed_set:
                continue
            closed_set[c_id] = current

            # 다음 노드 확장 (Motion Primitives)
            for steer in self.steer_set:
                steer_rad = math.radians(steer)
                
                # 로봇 운동 모델 (Bicycle Model or Differential Drive)
                next_theta = current.theta + steer_rad
                next_x = current.x + self.step_size * math.cos(next_theta)
                next_y = current.y + self.step_size * math.sin(next_theta)

                if self.is_collision(next_x, next_y, grid_map):
                    continue

                # 노드 생성 및 비용 계산
                next_g = current.cost + self.step_size
                new_node = self.Node(int(next_x/self.reso), int(next_y/self.reso), 0,
                                     current, next_g, next_x, next_y, next_theta)
                new_node.f = new_node.cost + self.calc_heuristic(new_node, goal_node)
                
                heapq.heappush(open_set, new_node)

        return None # 경로 못 찾음

    def reconstruct_path(self, current):
        path = []
        while current:
            path.append([current.x, current.y])
            current = current.parent
        return path[::-1] # 역순 반환