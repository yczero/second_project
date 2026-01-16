1️⃣ RPP란 

RPP = Regulated Pure Pursuit Controller입니다.
Nav2에서 제공하는 로컬 경로 추종(Local Controller) 알고리즘 중 하나입니다.

기본 개념 (Pure Pursuit)

전역 경로(Global Path) 위에서

**앞쪽 일정 거리(lookahead)**에 있는 목표점을 하나 잡고

그 점을 향하도록 원호(곡선)로 따라가며 주행합니다.

계산이 단순하고 안정적이라 실제 로봇에서 많이 사용됩니다.

Regulated가 붙은 이유

기본 Pure Pursuit의 단점을 보완한 버전입니다.

문제	                      RPP에서의 개선
장애물 근처에서도 속도가 유지됨	장애물·곡률·정렬 상태에 따라 속도 자동 감소
좁은 공간에서 불안정	       충돌 시간(Time to Collision) 기반 감속
곡선에서 과속	               경로 곡률 기반 감속

즉,

**RPP는 “경로를 따라가되, 상황에 맞게 속도를 스스로 조절하는 컨트롤러”**입니다.

2️⃣ nav2_rpp.launch.py 

이 launch 파일의 목적은:

AMCL로 위치 추정 → Nav2 전체 실행 → 컨트롤러를 RPP로 사용

입니다.

① Launch Argument 선언
use_sim_time = LaunchConfiguration('use_sim_time') ->  시뮬레이션 시간 사용 여부
map_yaml = LaunchConfiguration('map')   -> 사용할 맵 yaml
params_file = LaunchConfiguration('params_file') ->  Nav2 전체 파라미터 파일



② Localization 실행 (map_server + AMCL)
localization_launch = IncludeLaunchDescription(
    localization_launch.py
)
포함되는 노드:
map_server
amcl
lifecycle_manager_localization
역할:
/map 퍼블리시
로봇 위치 추정 (map ↔ odom ↔ base_link TF)



③ Navigation 실행 (Nav2 핵심)
navigation_launch = IncludeLaunchDescription(
    navigation_launch.py
)
이 안에서 실행되는 것들:
planner_server → 전역 경로 생성
controller_server → RPP로 경로 추종
bt_navigator → NavigateToPose 액션 처리
behavior_server → recovery 동작



④ RViz 실행
rviz2 -d nav2_default_view.rviz    ->Goal 찍고 상태 확인용

✔ 이 launch 파일의 핵심 요약
Nav2 기본 구조 그대로 사용
컨트롤러만 RPP로 설정
별도 커스텀 주행 노드 없음
RViz에서 Goal 찍으면 그대로 RPP 주행

3️⃣ rpp_params.yaml 
이 파일이 실질적인 주행 성능을 결정합니다.

🔹 Lifecycle Manager
lifecycle_manager_navigation:
  node_names:
    ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
Nav2 노드들을 자동으로 configure → activate
이게 없으면 직접 lifecycle 명령을 줘야 합니다

🔹 Map Server
map_server:
  yaml_filename: "/home/zero/maze.yaml"
정적 맵 제공
글로벌 코스트맵의 기반

🔹 AMCL
amcl:
  global_frame_id: "map"
  odom_frame_id: "odom"
  base_frame_id: "base_link"

LiDAR 기반 위치 추정
TF 트리 정상 구성의 핵심

🔹 Global Costmap (전역 계획용)
plugins: ["static_layer", "inflation_layer"]
static_layer → 맵 기반 장애물
inflation_layer → 벽 주변 비용 증가
👉 전역 경로는 “맵 기준으로만” 생성

🔹 Local Costmap (로컬 회피용)
plugins: ["obstacle_layer", "inflation_layer"]
obstacle_layer → /scan 기반 실시간 장애물
rolling_window → 로봇 중심으로 움직이는 맵
👉 RPP가 참고하는 실시간 환경 정보

🔹 Planner Server
NavfnPlanner
단순한 전역 경로 생성기
RPP와 궁합이 좋음


🔹 Controller Server (핵심)
plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
주요 파라미터 의미
desired_linear_vel: 0.22  목표 속도 (상황 안 좋으면 자동 감속됨)

lookahead_dist: 0.6
min_lookahead_dist: 0.3
max_lookahead_dist: 0.9
경로를 “얼마나 멀리 보고” 따라갈지
속도에 따라 자동 조절

use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.0
충돌 예측 기반 감속
RPP의 가장 큰 장점

allow_reversing: false
후진 금지 (전진 전용)

🔹 BT Navigator
navigate_to_pose_w_replanning_and_recovery.xml
주행 중 경로 재계획 가능
막히면 recovery 수행

4️⃣ 전체 흐름 한 줄 요약
AMCL이 위치 추정
Planner가 전역 경로 생성
RPP가 경로를 부드럽게 추종
장애물·곡률·충돌 위험에 따라 속도 자동 조절
Goal 도달 또는 recovery 수행
