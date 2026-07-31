# Ackermann SLAM workspace

ROS 2 Jazzy, Gazebo Harmonic, `gz_ros2_control`, Ackermann steering
controller, 2D LiDAR, SLAM Toolbox를 묶은 실행 가능한 워크스페이스다.

> 현재 모델은 제공된 STL에서 추론한 **가정 기반 시뮬레이션 모델**이다.
> `src/ackermann_car_description/config/vehicle_geometry.yaml`의
> `cad_stop_gate: not_passed` 항목을 실제 CAD 기준값과 질량 특성으로
> 교체하기 전에는 실차·생산 용도로 사용하지 않는다.

## 필수 실행 환경

이 워크스페이스에서는 모든 빌드·테스트·launch·`ros2 topic/service`
터미널에서 다음 파일을 먼저 source한다.

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
```

`setup_local.bash`는 순서대로 시스템 Jazzy, `local_ros` overlay, 빌드된
workspace를 source하고 다음 RMW를 고정한다.

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

현재 호스트의 시스템 Fast DDS/Fast-CDR 조합은 일부 패키지만 갱신된
상태에서 ABI가 맞지 않을 수 있다. 검증된 경로는 이 조합을 로드하지 않고
workspace-local 공식 CycloneDDS overlay를 사용하는 것이다. 따라서
`/opt/ros/jazzy/setup.bash`나 `install/setup.bash`만 따로 source하거나,
실행 터미널에서 RMW를 Fast DDS로 다시 바꾸지 않는다.

`local_ros`는 공식 Jazzy `.deb`에서 구성한 workspace-local overlay다.
여기에는 source build에 필요한 `ros2_control_cmake` bootstrap,
Fast-CDR, CycloneDDS/RMW 및 관련 런타임 의존성이 들어 있다. 최종
controller/plugin은 다음 공식 소스를 현재 시스템 ABI에 맞춰 이
workspace에서 다시 빌드하며, `install` overlay가 `local_ros`보다
우선한다.

- `src/gz_ros2_control`: `1.2.19`
- `src/steering_controllers_library`: `4.40.1`
- `src/ackermann_steering_controller`: `4.40.1`

일반적인 깨끗한 Jazzy 호스트에서는 상위 워크플로우의 APT 설치 절차를
사용할 수 있다. 다만 이 저장소와 현재 호스트에서는 위
`setup_local.bash` 경로가 검증된 우선 실행 경로다.

## 구성

- `ackermann_car_description`: 정규화 mesh, Xacro/URDF, Gazebo world,
  ros2_control, bridge, SLAM/RViz 설정과 launch
- `ackermann_command_guard`: `TwistStamped` 입력을 조향각·조향속도·
  후륜 속도/가속도 제약 안으로 투영하고 timeout 시 정지시키는 노드
- `gz_ros2_control`, `steering_controllers_library`,
  `ackermann_steering_controller`: 현재 ABI로 재빌드하는 고정 버전 공식
  소스
- `local_ros`: 빌드 bootstrap과 Fast-CDR/CycloneDDS 공식 `.deb`
  overlay
- `colcon.meta`: 위 세 upstream 패키지만 `BUILD_TESTING=OFF`로 빌드

## 생성과 빌드

`vehicle_geometry.yaml`만 사람이 수정하는 기하 정본이다. 값을 바꾸면
mesh와 세 생성물을 다시 만들고 검사한 뒤 빌드한다.

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash

python3 src/ackermann_car_description/scripts/normalize_meshes.py \
  --input /home/hoodinga/Documents/SLAM/slam_files \
  --geometry src/ackermann_car_description/config/vehicle_geometry.yaml \
  --output src/ackermann_car_description/meshes
python3 src/ackermann_car_description/scripts/normalize_meshes.py \
  --input /home/hoodinga/Documents/SLAM/slam_files \
  --geometry src/ackermann_car_description/config/vehicle_geometry.yaml \
  --output src/ackermann_car_description/meshes \
  --check
python3 src/ackermann_car_description/scripts/generate_geometry.py
python3 src/ackermann_car_description/scripts/generate_geometry.py --check
python3 src/ackermann_car_description/scripts/validate_yaml.py \
  src/ackermann_car_description/config

colcon build --symlink-install \
  --allow-overriding \
    gz_ros2_control \
    steering_controllers_library \
    ackermann_steering_controller
source setup_local.bash
colcon test
colcon test-result --verbose
```

워크스페이스 루트의 `colcon.meta`는 자동으로 읽힌다. 이 파일은 가져온
세 upstream 패키지의 자체 test suite만 끄며,
`ackermann_car_description`과 `ackermann_command_guard`의 프로젝트
테스트는 계속 실행한다.

## 시뮬레이션과 주행

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
ros2 launch ackermann_car_description simulation.launch.py rviz:=true
```

GUI가 필요 없는 검증은 `headless:=true rviz:=false`를 사용한다. 다른
터미널에서도 반드시 같은 setup을 source한 뒤 안전 입력 토픽으로
명령한다.

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 topic pub --rate 10 /cmd_vel_raw geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}, angular: {z: 0.3}}}"
```

`/cmd_vel_raw`을 controller에 직접 remap하지 않는다. guard가 검증한
`/cmd_vel`만 `ackermann_steering_controller`가 소비한다.

## SLAM과 지도 저장

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 launch ackermann_car_description slam.launch.py rviz:=true
```

주행을 마친 뒤 occupancy map과 pose graph를 각각 저장한다. 시뮬레이션
환경에서는 기본 제한이 촉박할 수 있으므로 map saver timeout은
`10.0`초를 권장한다.

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
mkdir -p /home/hoodinga/Documents/SLAM/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/hoodinga/Documents/SLAM/maps/ackermann_slam \
  --ros-args -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/hoodinga/Documents/SLAM/maps/ackermann_slam'}"
```

mapping launch를 완전히 종료한 다음 새 터미널에서 저장 graph를 다시
불러온다.

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 launch ackermann_car_description localization.launch.py \
  map_file_name:=/home/hoodinga/Documents/SLAM/maps/ackermann_slam \
  rviz:=true
```

현재 `slam_localization.yaml`은 `map_start_pose: [0.0, 0.0, 0.0]`와
`map_start_at_dock: true`를 함께 명시한다. 현재 Jazzy의 localization
parameter validator가 빈 `map_start_pose`를 잘못 처리하는 오류를
피하기 위한 조합이며, 현재 고정 버전에서 graph 원점/dock round-trip을
통과했다. 하나만 독립적으로 삭제하지 않는다. 임의 위치에서 시작하려면
사용 중인 Jazzy 버전의 validator 동작을 먼저 확인한 뒤 두 값을 함께
재검증한다.

설계 판단, 교정 근거, 단계별 판정 기준은 상위
`ACKERMANN_SLAM_WORKFLOW.md`에 정리되어 있다.

## 검증 기록

2026-07-31 현재 이 호스트에서 다음을 통과했다.

- 정규화 STL 9개와 생성 파일 3개 재현성 검사, YAML 6개 strict 검사
- Xacro/URDF와 robot/world SDF 검사; 총 질량 `6.2 kg`
- 5개 package 빌드와 프로젝트 test `23/23`
- 두 controller 활성화, hardware interface claim, 직선·원호 주행,
  좌우 조향각/후륜속도 차동, guard 곡률 clamp와 watchdog 정지
- `/clock`, `/scan`, `/odom`, TF와 `/cmd_vel` 단일 guard→controller 경로
- SLAM map `237×175 @ 0.05 m`, occupancy map/pose graph 저장,
  새 localization 프로세스에서 active·동일 map·`map→odom` 재확인

## 재배치와 라이선스

`setup_local.bash` 자체는 현재 위치를 동적으로 찾지만, `--symlink-install`
결과는 빌드 당시 절대경로를 포함한다. 워크스페이스를 옮긴 뒤에는 새
위치에서 다시 빌드한다. 자체 작성 부분은 루트 `LICENSE`의
Apache-2.0을 따르며, 가져온 upstream 소스와 `local_ros` 구성요소는 각
패키지에 보존된 원 저작권·라이선스 고지를 따른다. 정확한 local overlay
버전은 `local_ros/README.md`에 기록되어 있다.
