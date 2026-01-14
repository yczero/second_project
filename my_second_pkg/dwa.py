

# dwa.py

import math
import numpy as np

class DWAController:
    def __init__(self):
        # 로봇 파라미터 (TurtleBot3 기준)
        self.max_speed = 0.18  # m/s
        self.min_speed = 0.0
        self.max_yaw_rate = 1.0 # rad/s
        self.max_accel = 0.2
        self.max_dyaw_rate = 1.0 # rad/s^2
        
        self.v_reso = 0.01      # 속도 해상도
        self.yaw_rate_reso = 0.05 # 각속도 해상도
        self.dt = 0.1           # 시뮬레이션 시간 간격
        self.predict_time = 2.0 # 예측 시간 (초)

        # 가중치 (튜닝 필요!)
        self.to_goal_cost_gain = 0.20
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.5

        self.robot_radius = 0.25

    def motion_model(self, x, u):
        # x = [x, y, theta, v, w]
        # u = [v, w]
        x[2] += u[1] * self.dt # theta
        x[0] += u[0] * math.cos(x[2]) * self.dt # x
        x[1] += u[0] * math.sin(x[2]) * self.dt # y
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_dynamic_window(self, current_state):
        # 현재 속도에서 물리적으로 도달 가능한 속도 범위 계산
        # Vs: [min_v, max_v, min_w, max_w]
        
        # 1. 로봇 스펙상 한계
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]

        # 2. 현재 속도 기준 가속도 한계 고려
        Vd = [current_state[3] - self.max_accel * self.dt,
              current_state[3] + self.max_accel * self.dt,
              current_state[4] - self.max_dyaw_rate * self.dt,
              current_state[4] + self.max_dyaw_rate * self.dt]

        # 교집합 구하기
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_trajectory(self, x_init, v, w):
        x = np.array(x_init)
        traj = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion_model(x, [v, w])
            traj = np.vstack((traj, x))
            time += self.dt
        return traj

    def calc_to_goal_cost(self, trajectory, goal):
        # 예측 경로의 마지막 점과 목표점 사이의 거리 (혹은 헤딩 각도 차이)
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        return math.hypot(dx, dy)

    def calc_obstacle_cost(self, trajectory, obstacles):
        # obstacles: [[ox, oy], ...]
        if len(obstacles) == 0: return 0.0

        min_r = float("inf")
        
        # 모든 경로 점에 대해 가장 가까운 장애물 거리 계산
        # (Numpy 브로드캐스팅을 사용하여 고속 연산)
        ox = obstacles[:, 0]
        oy = obstacles[:, 1]
        
        for i in range(len(trajectory)):
            dx = trajectory[i, 0] - ox
            dy = trajectory[i, 1] - oy
            r = np.hypot(dx, dy)
            current_min = np.min(r)
            
            if current_min < min_r:
                min_r = current_min

        # [수정] 충돌 판정 거리(반지름) 늘리기
        # 터틀봇 반지름이 약 0.1~0.15m 정도인데, 0.2로 하면 너무 타이트합니다.
        # 0.35m 정도로 설정해서 벽에서 최소 35cm는 떨어지게 만드세요.
        # robot_radius = 0.40  
        
        if min_r <= self.robot_radius:
            return float("inf") # 충돌로 간주 (무한대 비용)
            
        return 1.0 / min_r

    # ------------------
    # DWA 계산량 최소화
    # ------------------
    def dwa_control(self, x, goal, obstacles):
        # x: [x, y, theta, v, w], goal: [gx, gy]
        dw = self.calc_dynamic_window(x)
    
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_traj = None
    
        # 속도/각속도 전수 조사 대신 단계 증가 조정
        v_step = max(self.v_reso, 0.02)
        w_step = max(self.yaw_rate_reso, 0.05)
    
        # 장애물이 없으면 속도만 최대한
        if len(obstacles) == 0:
            return [self.max_speed, 0.0], np.array([x])
    
        for v in np.arange(dw[0], dw[1]+v_step, v_step):
            for w in np.arange(dw[2], dw[3]+w_step, w_step):
                traj = self.calc_trajectory(x, v, w)
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(traj, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - traj[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(traj, obstacles)
                final_cost = to_goal_cost + speed_cost + ob_cost
    
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj
    
        if best_traj is None:
            best_traj = np.array([x])
        return best_u, best_traj
