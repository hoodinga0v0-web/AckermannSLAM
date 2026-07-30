# ROS 2 Jazzy + Gazebo Harmonic Ackermann SLAM 수정 워크플로우

검토 기준일: 2026-07-31 (Asia/Seoul)<br>
작업공간: `/home/hoodinga/Documents/SLAM`<br>
원본 계획: `slam_files/ros2_jazzy_gazebo_ackermann_slam_workflow (1).pdf`

## 0. 문서의 지위와 완료 기준

이 문서는 원본 PDF를 실제 자산과 ROS 2 Jazzy 공식 인터페이스에 맞게 교정한 구현 정본이다. 원본 PDF는 보존한다.

이 결과 문서를 제외한 원래 입력 자산은 PDF 1개와 STL 10개뿐이며 구현 코드는 없다. 따라서 이 문서에서 말하는 “오류 없음”은 다음 두 조건을 뜻한다.

1. 계획 안의 파일명, 좌표계, 기구학, ROS/Gazebo 인터페이스가 서로 모순되지 않는다.
2. 아직 제공되지 않은 CAD 질량·관성·정확한 피벗값을 임의로 만들지 않고, 측정 완료를 실행 전 필수 게이트로 둔다.

런타임 완료 판정은 마지막의 단계별 검증 게이트를 모두 통과했을 때만 내린다.

## 1. 전체 파일 감사 결과

### 1.1 PDF

- `pdfinfo`와 페이지 분리 결과 실제 페이지 수는 16쪽이다.
- 암호화, JavaScript, 서명, 첨부파일은 없고 텍스트를 정상 추출할 수 있다.
- 기본 아키텍처 선택은 타당하지만 실제 STL과 맞지 않는 치수·원점, 불완전한 launch/world, 잘못된 기준 프레임과 명령 범위가 포함되어 있다.

### 1.2 STL

모든 STL은 binary STL이며 좌표와 normal이 유한하고 winding이 일관된다. 퇴화 삼각형, 열린 경계, non-manifold edge는 발견되지 않았고 각 파일은 닫힌 단일 shell이다.

STL 형식에는 단위 메타데이터가 없다. 아래 `u`는 원시 좌표 단위이며 형상 비율상 `1 u = 1 mm`가 강하게 추정될 뿐, CAD export 설정으로 확인하기 전에는 mm로 확정하지 않는다. ROS bbox는 `Rz(-π/2)`를 적용했을 때의 축 순서이며, 단위 크기는 아직 같다.

| 실제 파일 | 삼각형 수 | 원시 CAD XYZ bbox (u) | ROS XYZ bbox (u) | 처리 |
|---|---:|---:|---:|---|
| `chassis.stl` | 632 | 110 × 176 × 10 | 176 × 110 × 10 | 차대 visual, 단순 collision의 치수 근거 |
| `exterior.stl` | 107,692 | 158.161 × 305.993 × 74.294 | 305.993 × 158.161 × 74.294 | 고해상도 visual 전용 |
| `LFWheel.stl` | 1,002 | 28 × 50 × 49.927 | 50 × 28 × 49.927 | 좌전륜 |
| `RFWheel.stl` | 1,002 | 28 × 50 × 49.927 | 50 × 28 × 49.927 | 우전륜 |
| `LRWheel.stl` | 1,050 | 34 × 50 × 49.927 | 50 × 34 × 49.927 | 좌후륜 |
| `RRWheel.stl` | 1,050 | 34 × 50 × 49.927 | 50 × 34 × 49.927 | 우후륜 |
| `LFKnuckle.stl` | 380 | 17.5 × 33 × 6 | 33 × 17.5 × 6 | 좌측 킹핀/너클 |
| `RFKnuckle.stl` | 380 | 17.5 × 33 × 6 | 33 × 17.5 × 6 | 우측 킹핀/너클 |
| `KnuckleLink.stl` | 300 | 100 × 5 × 2.5 | 5 × 100 × 2.5 | 타이로드 추정; 1차 모델에서는 제외 |
| `LiDER.stl` | 828 | 24.963 × 24.982 × 15 | 24.982 × 24.963 × 15 | LiDAR housing visual |

`exterior.stl`을 collision으로 사용하지 않는다. 10만 개가 넘는 삼각형을 동적 충돌 계산에 넣을 이유가 없고 물리 안정성도 나빠진다.

## 2. 원본 계획에서 교정한 핵심 오류

| 원본 계획의 문제 | 교정 |
|---|---|
| 존재하지 않는 snake_case 메시 파일명 사용 | 실제 이름을 그대로 쓰거나 패키지 복사 시 명시적으로 이름을 정규화 |
| 7개 STL 및 부품별 local origin 가정 | 실제 10개 STL은 CAD 조립 좌표를 보존하므로 각 피벗 기준 재수출/정규화 |
| STL 단위를 확인하지 않고 `scale=0.001` 확정 | CAD export 단위를 먼저 확인하고 그 단위의 m 변환계수를 한 곳에서만 적용 |
| CAD 축과 ROS 축이 같다고 가정 | 형상상 CAD `+Y=전방, +X=오른쪽, +Z=위`가 강하게 추정됨; CAD에서 확인한 뒤 ROS로 후보 `Rz(-π/2)` 변환 |
| 예시 치수 `0.42/0.34/0.34/0.065 m` 사용 | 이 차량에는 사용 금지; CAD 측정값을 정본으로 사용 |
| 차체 중심을 controller base frame으로 허용 | controller base는 후륜 구동축 중앙의 지면 투영점으로 고정 |
| 실제 킹핀과 앞바퀴 중심을 같은 점으로 가정 | 표준 controller용 가상 조향축 모델과 실제 킹핀 모델을 분리 |
| `KnuckleLink.stl` 처리 누락 | 표준 URDF의 폐루프를 피하기 위해 1차 모델에서 제외 |
| 일부 링크만 질량·관성·collision 보유 | `base_link`와 모든 비고정 물리 링크에 양의 질량·유효 관성을 주고, 지면/차체 접촉 링크에 단순 collision 부여; massless fixed frame과 A 가상 steering collision만 예외 |
| CAD 관성을 축 변환 없이 사용 | CoM과 관성 기준점을 확인하고 `I_ros = R I_cad Rᵀ` 적용 |
| LiDAR housing 중심을 광학 원점으로 간주 | `lidar_body_link` visual과 실측 ray origin인 `lidar_link`를 분리 |
| world에 시스템 플러그인 조각만 존재 | 완전한 SDF, ground, light, physics, 비대칭 장애물 작성 |
| bridge/launch가 파일을 실제로 연결하지 않음 | `config_file`, world 경로, spawn 이벤트, spawner timeout을 명시 |
| `enable_odom_tf`만으로 `/tf`가 생기는 것처럼 설명 | 실제 `<controller>/tf_odometry`를 `/tf`로 remap |
| 속도와 yaw rate를 독립 범위로 제시 | `|ω| ≤ |v|/R_min`, `v=0 && ω≠0` 금지 |
| command limit이 자동 적용된다고 암묵적으로 가정 | `controller_manager.enforce_command_limits: true` 설정 |
| stamped teleop에 wall time 사용 | teleop, RSP, controller, RViz, SLAM 모두 simulation time 사용 |
| SLAM이 odometry 토픽을 직접 요구한다고 설명 | SLAM Toolbox의 핵심 입력은 `/scan`과 `odom→base` TF |
| `nav2_map_server`만으로 Nav2까지 가능한 것처럼 표현 | SLAM 본체와 선택적 Nav2 단계를 분리 |

## 3. 구현 전 게이트: CAD 기준표 확정

### 3.1 좌표계와 기준 프레임

다음 정의를 고정하고 이후 단계에서 바꾸지 않는다.

- `base_footprint`: 후륜 구동축 중앙을 지면에 투영한 점
- `base_link`: 후륜 구동축 중앙, 바퀴 축 높이
- `base_footprint → base_link`: `xyz="0 0 REAR_AXLE_HEIGHT"`
- ROS 링크 축: `+X` 전방, `+Y` 왼쪽, `+Z` 위
- 조향축: `+Z`
- 바퀴 회전축: `+Y`

Ackermann controller는 후륜축 중앙의 속도와 포즈를 적분하며 임의의 종방향 base offset 파라미터를 제공하지 않는다. 따라서 차체 중심을 odometry child frame으로 사용하면 안 된다. 차체 중심 프레임이 필요하면 `base_link`의 고정 자식으로 따로 둔다.

형상에서 추정한 CAD→ROS 회전 후보는 다음이다. CAD source의 axis convention으로 확인해 `R_cad_to_ros`에 기록하기 전에는 확정 변환으로 사용하지 않는다.

```text
R_candidate = Rz(-π/2)
            = [[ 0,  1, 0],
               [-1,  0, 0],
               [ 0,  0, 1]]
```

### 3.2 조향 모델 선택

현재 STL에서 실제 킹핀은 앞바퀴 rolling center보다 원시 좌표 약 27 u 뒤, 각 바퀴보다 약 25 u 안쪽에 있다. `1 u=1 mm`가 확인되면 각각 약 0.027 m와 0.025 m다. 반면 Jazzy의 표준 `ackermann_steering_controller` 운동학과 공식 예제는 조향 joint를 앞바퀴 rolling center에 둔 이상화 모델과 정확히 맞는다. 두 모델을 섞지 않는다.

이 워크플로우의 기본 경로는 다음 **A 경로**다.

| 경로 | 조향 joint 위치 | 용도와 제약 |
|---|---|---|
| A. controller-compatible | 앞바퀴 rolling center, `x=rolling_wheelbase`, `y=±steering_track_width/2` | 이 문서의 SLAM 기본 경로. controller의 `wheelbase=rolling_wheelbase`. 앞바퀴 spin joint도 같은 점에 둔다. 너클은 생략하거나 이 가상축에 붙인 visual로만 사용한다. |
| B. mechanically faithful | 실제 킹핀, `x=kingpin_x`, `y=±kingpin_track/2`; wheel은 `knuckle_to_wheel` offset을 가짐 | 실제 tie-rod/폐루프와 조향 시 moving wheel center를 다뤄야 한다. 사용자 정의 비선형 기구학·constraint·controller를 검증하기 전에는 표준 controller와 “정확히 일치”한다고 판정하지 않는다. |

A 경로에서는 물리 킹핀 측정값을 controller joint origin에 사용하지 않는다. B 경로를 선택하면 이 문서의 URDF와 controller 절을 그대로 실행하지 말고 별도 설계 검토로 되돌아간다.

### 3.3 반드시 CAD에서 확정할 값

| 항목 | 정의 |
|---|---|
| `cad_axis_convention`, `R_cad_to_ros` | CAD 원축 의미와 검증된 3×3 회전행렬 |
| `mesh_export_unit` | STL 원시 좌표 1 u가 몇 m인지 나타내는 변환계수 |
| `rolling_wheelbase` | 후륜 rolling axis에서 0 조향의 전륜 rolling center까지 X 거리 |
| controller `wheelbase` | A 경로에서 `rolling_wheelbase`와 같은 생성값; B 경로에는 이 표준 controller를 사용하지 않음 |
| `traction_track_width` | 좌우 후륜 rolling center 사이 거리 |
| `steering_track_width` | 두 모델링된 steering axis(virtual/model kingpin) 사이 거리; A 경로에서는 가상축이므로 전륜 rolling-center track과 같음 |
| `traction_wheels_radius` | 후륜 collision 반지름과 같은 유효 rolling radius |
| `front_wheels_radius` | 앞바퀴 collision 반지름 |
| `rear/front_wheel_width` | 각 원통 collision 길이 |
| `rear/front_axle_height` | `base_footprint`에서 각 rolling center까지의 Z 거리 |
| `kingpin_x`, `kingpin_track` | B 경로 및 너클 visual 검사용 실제 조향축 위치 |
| `knuckle_to_wheel_xyz/rpy` | 킹핀 링크에서 앞바퀴 회전축까지의 변환 |
| 좌/우 steering limit | `front_left/right` 각 joint의 `lower`, `upper`, `velocity`, `effort` |
| 좌/우 rear wheel limit | `rear_left/right` 각 joint의 `velocity`, `effort` |
| `lidar_body_xyz/rpy` | `base_link`에서 housing visual 기준점까지의 변환 |
| `lidar_ray_xyz/rpy` | `lidar_body_link`에서 실제 레이저 방사 원점까지의 변환 |
| mesh별 `pivot_cad` | `base_link`, 네 wheel center, 두 A virtual steering/knuckle visual 기준점, LiDAR housing 기준점의 원시 CAD 좌표 |
| 질량·CoM·관성 | 차체, 두 너클, 네 바퀴 각각의 CAD 값 |
| footprint | `base_footprint` 기준 차량 외곽 polygon |

`1 u = 1 mm`라고 가정해 STL 형상에서 얻은 아래 값은 CAD 측정 교차검사용 추정치일 뿐 최종 설정값이 아니다.

| 항목 | 추정치 |
|---|---:|
| rolling wheelbase | 약 0.171 m |
| 전륜 rolling-center track | 약 0.130 m |
| 후륜 rolling-center track | 약 0.124 m |
| 바퀴 반지름 | 약 0.025 m |
| 실제 kingpin X, rear axle 기준 | 약 0.144 m |
| 실제 kingpin track | 약 0.080 m |
| 좌측 킹핀→앞바퀴 중심, ROS XYZ | 약 `(+0.027, +0.025, 0) m` |
| 우측 킹핀→앞바퀴 중심, ROS XYZ | 약 `(+0.027, -0.025, 0) m` |
| rear axle→LiDAR housing bbox 중심 | 전방 약 0.066 m, 위 약 0.057 m |

마지막 LiDAR 값은 housing 형상의 중심이지 ray origin이 아니다. `lidar_ray_xyz/rpy`를 센서 도면이나 CAD datum으로 별도 측정한다.

좌후륜 배치는 우후륜의 정확한 mirror에서 원시 좌표 약 0.366 u 벗어나 보인다. A 경로는 두 후륜축이 같은 X/Z 선상이고 `base_link` 좌우에 대칭이어야 한다. CAD가 대칭 설계를 확인하면 정규화 전에 mirror 기준으로 보정한다. 종·수직 비대칭이 의도된 설계라면 A 경로를 STOP하고 별도 비대칭 기구학 모델로 보낸다.

### 3.4 회전 가능 영역과 후륜 속도

A 경로의 controller wheelbase `L=rolling_wheelbase`, model steering-axis track을 `W_s`, rear-axle-center 곡률을 `κ=ω/v`라 하면 controller와 같은 좌우 목표각은 다음이다.

```text
δ_left(κ)  = atan2(L κ, 1 - W_s κ/2)
δ_right(κ) = atan2(L κ, 1 + W_s κ/2)
```

generator는 각 방향에서 두 식을 좌·우 joint의 실제 lower/upper에 모두 대입해 feasible 곡률의 경계를 수치적으로 구한다. 양의 최대 feasible 곡률을 `κ_max,left`, 음의 최소 feasible 곡률의 절댓값을 `|κ_min,right|`라 두면 다음과 같다.

```text
R_min,left  = 1 / κ_max,left
R_min,right = 1 / |κ_min,right|
```

두 joint가 같은 대칭 한계를 갖고 안쪽 wheel limit가 지배할 때만 다음 축약식을 교차검사에 쓴다.

```text
R_min = L / tan(δ_inner,max) + W_s / 2
```

모든 명령은 곡률 방향에 맞는 값을 사용해 다음을 만족해야 한다.

```text
v = 0 이면 ω = 0
κ > 0: |ω| <= |v| / R_min,left
κ < 0: |ω| <= |v| / R_min,right
```

후륜 track을 `W_t`, 유효 반지름을 `r`, joint 속도 한계를 각각 `q̇_max`라 하면 다음도 동시에 만족해야 한다.

```text
q̇_left  = (v - ω W_t/2) / r
q̇_right = (v + ω W_t/2) / r

|q̇_left|  <= q̇_left,max
|q̇_right| <= q̇_right,max
```

URDF joint limit, ros2_control command limit, controller 설정, command guard, 향후 Nav2의 `minimum_turning_radius`에 같은 측정값을 사용한다.

### 3.5 게이트 통과 조건

현재 작업공간에는 CAD 원본, export 단위, 질량 특성표, 센서 datum이 없다. 따라서 이 게이트는 현재 **STOP** 상태다. 다음 조건을 CAD/도면으로 채우거나, 검토된 피벗·단위·질량 값을 입력받는 재현 가능한 변환 도구가 준비되기 전에는 메시 변환과 URDF 구현을 시작하지 않는다.

원시 좌표의 m 변환계수를 `s`, CAD에서 확인한 `R=R_cad_to_ros`(후보는 3.1절의 `Rz(-π/2)`), ROS link 원점에 대응하는 CAD 점을 `p_link`라 하면 CoM과 관성은 다음처럼 함께 변환한다.

```text
p_com,ros = s R (p_com,cad - p_link)
I_com,ros = s² R I_com,cad Rᵀ
```

두 번째 식은 CAD 관성의 길이 단위가 `u²`일 때다. CAD가 이미 `kg·m²`를 출력했다면 `s²`을 다시 곱하지 않는다. URDF inertial origin을 CoM이 아닌 다른 점에 둘 때만 평행축 정리를 적용하며, 어느 기준점과 축에서 산출한 tensor인지 표에 기록한다.

- 모든 필수 값의 단위가 m, kg, rad, kg·m²로 기록됨
- STL export 단위와 단 하나의 m 변환계수가 확정됨
- `front_rolling_center_x - rear_traction_axis_x = rolling_wheelbase`
- 좌우 rear rolling center가 `traction_track_width`, 좌우 model steering axis가 `steering_track_width`와 일치
- A 경로에서 각 가상 steering joint가 앞바퀴 rolling center와 일치하고 controller `wheelbase=rolling_wheelbase`
- B 경로라면 0 조향에서 `kingpin_x + knuckle_to_wheel_x = rolling_wheelbase`이고 사용자 정의 기구학 검토가 완료됨
- `cad_axis_convention`, `R_cad_to_ros`, 모든 mesh별 `pivot_cad`가 CAD 근거로 확정됨
- wheel collision radius와 controller radius가 일치
- `lidar_body`와 실제 ray origin의 변환이 각각 확정됨
- A 경로에서 후륜축의 동일 X/Z와 좌우 대칭을 CAD로 확인하고 필요한 mesh 보정을 완료함
- CoM 기준 관성을 ROS 축으로 `I_ros = R I_cad Rᵀ` 변환하고, 다른 기준점이면 평행축 정리를 적용함
- 모든 관성 행렬이 대칭이며 양의 정부호
- `R_min`이 계산됨
- `TODO_CAD`, `REQUIRED`, PDF 예시 치수가 구현 파일에 남지 않음

## 4. 메시 정규화

원본 `slam_files/*.stl`은 수정하지 않는다. 정규화한 사본만 패키지 `meshes/`에 넣는다.

원본과 대상의 대소문자를 포함한 매핑은 다음으로 고정한다.

| 원본 | 패키지 대상 | 처리 |
|---|---|---|
| `chassis.stl` | `chassis.stl` | `base_link` 기준 |
| `exterior.stl` | `exterior.stl` | `base_link` 기준, visual 전용 |
| `LFWheel.stl` | `front_left_wheel.stl` | 좌전륜 rolling center 기준 |
| `RFWheel.stl` | `front_right_wheel.stl` | 우전륜 rolling center 기준 |
| `LRWheel.stl` | `rear_left_wheel.stl` | 좌후륜 rolling center 기준 |
| `RRWheel.stl` | `rear_right_wheel.stl` | 우후륜 rolling center 기준 |
| `LFKnuckle.stl` | `front_left_knuckle.stl` | A 경로에서는 선택 visual, 가상 조향축 기준 |
| `RFKnuckle.stl` | `front_right_knuckle.stl` | A 경로에서는 선택 visual, 가상 조향축 기준 |
| `LiDER.stl` | `lidar_body.stl` | housing visual 기준점 |
| `KnuckleLink.stl` | 복사하지 않음 | 폐루프 타이로드 추정; A 경로에서 제외 |

A 경로에서는 앞바퀴와 가상 steering link 모두 앞바퀴 rolling center를 원점으로 한다. 실제 너클 visual을 넣으면 가상축 주위로 회전하므로 기계적으로 충실한 표현은 아니다. SLAM 1차 모델에서는 너클 visual/collision을 생략해도 된다. LiDAR housing mesh는 housing 기준점으로 정규화하고, 실제 `lidar_link`는 그 고정 자식으로 둔다.

`scripts/normalize_meshes.py`는 `vehicle_geometry.yaml`의 `R_cad_to_ros`, `mesh_export_unit`, mesh별 `pivot_cad`와 위 매핑을 유일한 입력으로 사용한다. 원본 10개의 SHA-256을 먼저 기록하고 9개 대상만 임시 파일에 쓴 뒤 원자적으로 교체한다. CAD 점 `p_cad`, 해당 기준점 `p_pivot`, CAD normal `n_cad`에는 다음을 적용한다.

```text
p_ros_u = R_cad_to_ros (p_cad - p_pivot)
n_ros   = normalize(R_cad_to_ros n_cad)
```

CAD에서 후보 `Rz(-π/2)`가 확인된 경우에만 성분식은 다음이 된다.

```text
x_ros_u =  y_cad - y_p
y_ros_u = -(x_cad - x_p)
z_ros_u =  z_cad - z_p
n_ros   = normalize((n_cad_y, -n_cad_x, n_cad_z))
```

vertex와 normal을 함께 변환하고 normal을 재계산해 winding과 일치하는지 다시 검사한다. 좌표를 원시 단위로 보존하면 CAD에서 확정한 `mesh_export_unit`을 URDF의 세 축 `scale`에 정확히 한 번만 적용한다. 예를 들어 단위가 실제로 mm일 때만 `scale="0.001 0.001 0.001"`이다.

스크립트는 `meshes/mesh_manifest.json`에 canonical geometry SHA-256, 원본/대상 hash, pivot, 회전, scale, triangle/topology/bbox 결과를 저장한다. `--check`는 쓰기 없이 이를 재계산해 stale output, 10→9 매핑, normal/winding, watertight/non-manifold, 계획된 joint transform으로 재조립한 assembly 오차를 검사한다.

정규화 후 각 mesh visual origin은 원칙적으로 `0 0 0 / 0 0 0`이다. 정규화와 URDF origin 보정을 동시에 적용해 이중 이동시키지 않는다. 모든 기준점과 bbox가 CAD 허용오차 안에 들어야 Gate A를 통과한다.

`KnuckleLink.stl`은 두 너클을 잇는 폐루프 타이로드로 보인다. 1차 모델에서는 제외하고 두 steering joint를 controller가 직접 구동한다. 실제 폐루프 기구가 필요할 때만 별도의 Gazebo constraint 설계를 추가한다.

## 5. 워크스페이스와 패키지

### 5.1 필요한 패키지

일반적인 깨끗한 ROS 2 Jazzy 호스트에서는 다음 APT 설치를 기본 경로로
사용한다. 이 일반 설치 안내는 새 환경을 위한 것이며, 아래에 설명하는
현재 워크스페이스의 검증된 local overlay보다 우선하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-nav2-map-server \
  liburdfdom-tools
```

확인:

```bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix gz_ros2_control
ros2 pkg prefix ackermann_steering_controller
ros2 pkg prefix slam_toolbox
gz sim --versions
```

Jazzy의 기본 Gazebo 조합은 Harmonic이며 Gazebo Sim major version은 8이다.

현재 `/home/hoodinga/Documents/SLAM/ackermann_ws`에서는 시스템
Fast DDS와 일부 갱신된 Fast-CDR 사이의 ABI 불일치를 회피하기 위해
workspace-local overlay를 검증 경로로 고정했다. `local_ros`에는 공식
Jazzy `.deb`에서 가져온 `ros2_control_cmake` build bootstrap,
Fast-CDR, CycloneDDS/RMW와 관련 런타임 의존성이 들어 있다. 다음 공식
소스 패키지는 `src`에 고정하고 현재 시스템 ABI로 다시 빌드한다.

| 패키지 | 고정 버전 |
|---|---:|
| `gz_ros2_control` | `1.2.19` |
| `steering_controllers_library` | `4.40.1` |
| `ackermann_steering_controller` | `4.40.1` |

이 워크스페이스의 모든 빌드·테스트·launch·`ros2 topic/service`
터미널에서는 `/opt/ros/jazzy/setup.bash`나 `install/setup.bash`만
직접 source하지 않고 다음 하나를 사용한다.

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
test "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp"
```

`setup_local.bash`는 시스템 Jazzy → `local_ros` → 빌드된 `install`
순서로 overlay하고 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를
내보낸다. 실행 터미널에서 이를 Fast DDS로 다시 바꾸면 검증된 ABI
경로를 벗어난다. 최종 source build의 controller/plugin은
`install`에서 `local_ros`의 bootstrap 자산보다 우선한다.

### 5.2 생성 위치

현재 프로젝트 아래에 워크스페이스를 둔다. 아래의
`/opt/ros/jazzy/setup.bash`는 아직 `setup_local.bash`가 존재하지 않는
빈 workspace를 만드는 최초 bootstrap에만 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hoodinga/Documents/SLAM
mkdir -p ackermann_ws/src
cd ackermann_ws/src
ros2 pkg create --build-type ament_cmake ackermann_car_description
ros2 pkg create ackermann_command_guard \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs
```

`ackermann_command_guard`는 teleop/Nav2와 controller 사이에서 3.4절의
결합 제약을 강제하는 작은 노드다. 검증된 최종 구조에는 local
overlay와 고정 버전 upstream source도 포함된다.

```text
ackermann_ws/
├── setup_local.bash
├── colcon.meta
├── local_ros/                        # bootstrap/Fast-CDR/CycloneDDS deb overlay
└── src/
    ├── ackermann_car_description/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── meshes/
    │   │   ├── (4절의 정규화 STL 9개)
    │   │   └── mesh_manifest.json
    │   ├── urdf/
    │   │   ├── ackermann_car.urdf.xacro
    │   │   ├── vehicle_geometry.generated.xacro
    │   │   ├── inertial_macros.xacro
    │   │   ├── lidar.xacro
    │   │   └── ros2_control.xacro
    │   ├── config/
    │   │   ├── vehicle_geometry.yaml
    │   │   ├── controllers.yaml
    │   │   ├── command_guard.yaml
    │   │   ├── bridge.yaml
    │   │   ├── slam.yaml
    │   │   ├── slam_localization.yaml
    │   │   └── ackermann_car.rviz
    │   ├── scripts/
    │   │   ├── normalize_meshes.py
    │   │   ├── generate_geometry.py
    │   │   └── validate_yaml.py
    │   ├── launch/
    │   │   ├── display.launch.py
    │   │   ├── simulation.launch.py
    │   │   ├── slam.launch.py
    │   │   └── localization.launch.py
    │   ├── test/
    │   │   └── test_simulation_launch.py
    │   └── worlds/
    │       └── slam_world.sdf
    ├── ackermann_command_guard/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── ackermann_command_guard
    │   ├── ackermann_command_guard/
    │   │   ├── __init__.py
    │   │   └── command_guard.py
    │   └── test/
    │       └── test_command_guard.py
    ├── gz_ros2_control/              # official 1.2.19
    ├── steering_controllers_library/ # official 4.40.1
    └── ackermann_steering_controller/ # official 4.40.1
```

### 5.3 manifest와 설치

`config/vehicle_geometry.yaml`만 치수·한계·질량 특성의 사람이 편집하는 정본으로 둔다. 스키마에는 3.3절의 모든 값과 mesh별 `source/target/pivot_cad`, A/B model 선택, 좌우 steering limit/velocity, 좌우 rear-wheel velocity/acceleration, command timeout, position gain 산식 입력, 각 link의 mass/CoM/inertia를 넣는다. `scripts/generate_geometry.py`는 duplicate key를 거부한 뒤 schema와 단위를 검증하고 다음 파일을 원자적으로 생성한다. 각 생성물에는 정규화한 입력의 SHA-256을 기록하며, `--check` 모드에서는 파일을 쓰지 않고 digest 또는 예상 byte가 다르면 비정상 종료한다. `validate_yaml.py`도 duplicate key를 오류로 취급해 모든 config를 검사한다.

이 generator와 이후 URDF/controller는 A 경로 전용이다. `model != controller_compatible_A`이면 생성하지 않고 명시적으로 실패시켜 3.2절의 B 경로 설계 검토로 돌려보낸다.

- `urdf/vehicle_geometry.generated.xacro`: link/joint/mesh/inertial 숫자와 `steering_position_gain` property
- `config/controllers.yaml`: controller manager와 Ackermann 숫자
- `config/command_guard.yaml`: `R_min,left/right`, track/radius와 joint 속도 한계

생성 파일은 직접 편집하지 않는다. `display.launch.py`는 생성 Xacro와 RViz만 실행하고, `simulation.launch.py`는 world·robot·bridge·guard·controller를 실행하며, `slam.launch.py`는 `simulation.launch.py`와 mapping용 SLAM Toolbox를 포함한다. `localization.launch.py`는 simulation과 `slam_localization.yaml`을 포함하고 `map_file_name` launch argument를 SLAM Toolbox에 전달한다. 이 소비 관계를 launch test에서 확인한다.

`package.xml`에는 최소한 다음 runtime dependency를 선언한다.

```xml
<exec_depend>ament_index_python</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>ackermann_steering_controller</exec_depend>
<exec_depend>slam_toolbox</exec_depend>
<exec_depend>nav2_map_server</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>teleop_twist_keyboard</exec_depend>
<exec_depend>ackermann_command_guard</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_tools</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>rosgraph_msgs</exec_depend>
<exec_depend>python3-yaml</exec_depend>
<exec_depend>python3-numpy</exec_depend>

<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>

<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros gazebo_model_path="${prefix}/.."/>
</export>
```

`ros2 pkg create`가 만든 `CMakeLists.txt`의 기존 header와 마지막 `ament_package()`는 유지하고, `ament_package()` 바로 앞에 다음을 추가한다.

```cmake
install(
  DIRECTORY launch urdf meshes config worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/normalize_meshes.py
    scripts/generate_geometry.py
    scripts/validate_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_simulation_launch.py TIMEOUT 120)
endif()
```

`ackermann_command_guard/package.xml`에는 `rclpy`, `geometry_msgs` 실행 의존성과 `python3-pytest` test dependency를 둔다. `setup.py`의 `console_scripts`에는 `command_guard = ackermann_command_guard.command_guard:main`을 등록한다. `test_command_guard.py`는 Gate F의 정상·경계·비정상 입력 표를 자동 검사하고, `test_simulation_launch.py`는 정상 startup과 각 spawner 실패 주입을 검사한다. 테스트가 0개 수집되면 Gate C 실패로 처리한다.

워크스페이스 루트의 `colcon.meta`는
`gz_ros2_control`, `steering_controllers_library`,
`ackermann_steering_controller`에만 `-DBUILD_TESTING=OFF`를 적용한다.
가져온 upstream test의 추가 의존성을 끄기 위한 것이며 두 프로젝트
패키지의 테스트는 끄지 않는다. 모든 파일을 작성한 뒤에만 빌드한다.

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
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
colcon build --symlink-install \
  --allow-overriding \
    gz_ros2_control \
    steering_controllers_library \
    ackermann_steering_controller
source setup_local.bash
```

## 6. URDF/Xacro

### 6.1 링크 트리

```text
base_footprint                         # 후륜축 중앙의 지면 투영
└── base_link                         # 후륜축 중앙, 축 높이
    ├── rear_left_wheel_link
    ├── rear_right_wheel_link
    ├── front_left_steering_link       # A 경로의 가상 steering axis
    │   └── front_left_wheel_link
    ├── front_right_steering_link      # A 경로의 가상 steering axis
    │   └── front_right_wheel_link
    └── lidar_body_link                # housing visual
        └── lidar_link                 # 실측 ray origin
```

이 링크 트리는 3.2절의 A 경로 전용이다. 조인트 배치 원칙:

- 후륜 joint: `x=0`, `y=±traction_track_width/2`, `z=0` (`base_link`가 후륜축 높이)
- 전륜 steering joint: `x=rolling_wheelbase`, `y=±steering_track_width/2`, `z=front_axle_height-rear_axle_height`
- 앞바퀴 rolling joint: 해당 가상 steering link와 같은 원점 `0 0 0`
- LiDAR housing fixed joint: `base_link` 기준 `lidar_body_xyz/rpy`
- LiDAR ray-origin fixed joint: `lidar_body_link` 기준 `lidar_ray_xyz/rpy`

B 경로의 실제 킹핀 모델에서만 `front_*_wheel_joint`에 `knuckle_to_wheel_xyz/rpy`를 적용한다. A 경로와 B 경로의 origin 규칙을 혼합하지 않는다.

### 6.2 물리 링크

- `chassis.stl`과 `exterior.stl`은 `base_link`의 visual로 사용할 수 있다.
- 차체 collision은 CAD 치수에 맞춘 1~3개의 box로 단순화한다.
- 바퀴 collision은 실제 반지름과 폭을 갖는 cylinder로 만든다.
- URDF cylinder의 기본 축 `+Z`를 바퀴 joint의 `+Y`와 나란하게 하도록 collision origin에 `rpy="1.57079632679 0 0"`을 적용하고 Gazebo에서 축을 확인한다.
- A 경로의 장식용 너클 visual에는 별도 collision을 두지 않는다. B 경로를 새로 설계할 때만 실제 너클 collision을 단순 primitive로 추가한다.
- `base_link`에는 차체 조립체의 mass/inertia를 반드시 넣고, 네 wheel link와 두 virtual steering link에도 각각 양의 mass와 유효 inertia를 넣는다. `base_footprint`, `lidar_link` 같은 massless fixed frame만 예외다.
- inertial origin은 변환된 CoM에 두고 관성 tensor는 같은 ROS link 축으로 표현한다. 원시 관성 단위가 `kg·u²`이면 `mesh_export_unit²`을 곱해 `kg·m²`로 변환한다.
- joint `<dynamics friction>`은 타이어-지면 접촉 마찰이 아니다. Gazebo surface friction/slip은 별도로 조정한다.

바닥 접점 조건:

```text
base_footprint.z = 0
base_link.z = rear axle height
rear wheel center.z - rear radius = 0
front wheel center.z - front radius = 0
```

### 6.3 joint limit

- steering revolute joint: CAD의 `lower`, `upper`, `effort`, `velocity`
- rear continuous joint: 실제 `effort`, `velocity`
- ros2_control command interface의 min/max도 같은 한계와 일치

## 7. ros2_control과 Ackermann controller

### 7.1 인터페이스

```text
rear_right_wheel_joint: velocity command + position/velocity state
rear_left_wheel_joint:  velocity command + position/velocity state
front_right_steering_joint: position command + position/velocity state
front_left_steering_joint:  position command + position/velocity state
front_right_wheel_joint: position/velocity state only
front_left_wheel_joint:  position/velocity state only
```

수동 앞바퀴도 state-only로 `ros2_control`에 등록한다. 밖으로 빼면 `joint_state_broadcaster`가 앞바퀴 상태를 발행하지 않아 동적 TF가 빠진다.

하드웨어 플러그인:

```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```

Gazebo 플러그인:

```xml
<xacro:arg name="controllers_file" default=""/>
<xacro:arg name="mesh_prefix" default=""/>

<gazebo>
  <plugin
    filename="libgz_ros2_control-system.so"
    name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(arg controllers_file)</parameters>
    <position_proportional_gain>${steering_position_gain}</position_proportional_gain>
    <ros>
      <remapping>/ackermann_steering_controller/reference:=/cmd_vel</remapping>
      <remapping>/ackermann_steering_controller/odometry:=/odom</remapping>
      <remapping>/ackermann_steering_controller/tf_odometry:=/tf</remapping>
    </ros>
  </plugin>
</gazebo>
```

모든 mesh URI도 `$(arg mesh_prefix)/파일명.stl` 형태로 만든다. runtime launch는 설치된 package share에서 `controllers.yaml`의 절대경로와 `package://ackermann_car_description/meshes`를 Xacro mapping으로 전달한다. Gate B는 아직 설치되지 않은 source 경로를 전달하므로 첫 빌드 전에도 정적 검사가 가능하다.

remap이 실제 적용됐는지는 토픽 목록과 타입으로 검증한다. `enable_odom_tf: true`만으로 표준 `/tf` 토픽에 변환이 나오지는 않는다.

`gz_ros2_control`의 position interface는 다음 속도를 내부 생성하므로 gain을 임의로 고정하지 않는다.

```text
q̇_command = position_proportional_gain × position_error × update_rate
```

가능한 최대 명령–측정 position 오차를 `e_err,max`로 둔다. 이는 한 step의 target 변화량이 아니라 지연 중 누적될 수 있는 오차이며, 검증된 tracking-error bound가 없으면 보수적으로 전체 joint range를 쓴다. steering joint 속도 한계를 `q̇_steer,max`, controller update rate를 `f`라 할 때 `gain ≤ q̇_steer,max/(e_err,max f)`와 `gain ≤ 1`을 모두 만족하도록 안전여유를 두고 정한다. 실제 joint velocity와 타이어 slip을 Gate F에서 확인한다.

### 7.2 `controllers.yaml`

아래 대문자 값은 CAD 측정 후 실제 숫자로 치환한다. 문자열 그대로 남겨 두면 안 된다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true
    enforce_command_limits: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    ackermann_steering_controller:
      type: ackermann_steering_controller/AckermannSteeringController

ackermann_steering_controller:
  ros__parameters:
    use_sim_time: true
    reference_timeout: 0.5

    traction_joints_names:
      - rear_right_wheel_joint
      - rear_left_wheel_joint
    steering_joints_names:
      - front_right_steering_joint
      - front_left_steering_joint

    wheelbase: MODELED_WHEELBASE_M
    traction_track_width: REAR_TRACK_M
    steering_track_width: MODELED_STEERING_AXIS_TRACK_M
    traction_wheels_radius: REAR_EFFECTIVE_RADIUS_M

    base_frame_id: base_footprint
    odom_frame_id: odom
    enable_odom_tf: true

    open_loop: false
    position_feedback: false
    reduce_wheel_speed_until_steering_reached: true
    velocity_rolling_window_size: 10

    # [x, y, z, roll, pitch, yaw]; 실제 모델에 맞게 튜닝한다.
    pose_covariance_diagonal: [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]
    twist_covariance_diagonal: [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]
```

joint 이름 순서는 공식 인터페이스가 요구하는 `right, left`이다.

### 7.3 필수 command guard

`controller_manager.enforce_command_limits`는 joint interface 한계를 처리하지만 body 명령의 곡률 결합 조건을 대신 보장하지 않는다. 따라서 모든 운전 모드에서 controller의 `/cmd_vel` 바로 앞에 `ackermann_command_guard`를 하나만 둔다.

```text
teleop 또는 Nav2 최종 출력
  → /cmd_vel_raw (TwistStamped)
  → ackermann_command_guard
  → /cmd_vel (TwistStamped)
  → ackermann_steering_controller
```

guard의 필수 동작:

1. `linear.x`, `angular.z` 이외 성분을 0으로 만들고 NaN/Inf를 거부한다. freshness는 simulation-clock 기준 수신 시각으로 관리하며, nonzero input stamp가 별도 허용오차보다 오래됐거나 미래면 거부한다.
2. `|v| < ε`이면 `v=0, ω=0`으로 만든다.
3. `κ=ω/v`의 부호에 맞는 `R_min,left/right`로 `|κ|≤1/R_min`을 강제한다.
4. 계산된 좌우 후륜 `q̇`가 한계를 넘으면 `v`와 `ω`에 같은 비율을 곱해 곡률을 보존하며 낮춘다.
5. 이전 출력 곡률에서 새 곡률까지의 구간에서 `δ_left(κ)`, `δ_right(κ)`를 각각 계산하고, 두 angle 변화율이 모두 joint 속도 한계 안인 새 `κ`를 구한 뒤 `ω=vκ`로 재구성한다.
6. 검증된 acceleration/ramp를 적용하고, 바뀐 `v,κ`에 대해 곡률·좌우 steering angle/rate·rear-wheel speed/acceleration을 다시 계산한다. 모든 조건이 동시에 만족될 때까지 유계 반복/projection하고, 해가 없으면 정지 명령을 낸다.
7. CLI 시험의 zero input stamp는 수신 시각으로 취급한다. 출력에는 항상 node clock의 새 stamp와 `frame_id=base_footprint`를 넣는다.
8. `/cmd_vel`의 유일한 publisher가 guard인지 검증하고 모든 시험도 `/cmd_vel_raw`을 통해 수행한다.

정지 명령과 watchdog은 충돌 회피를 위해 ramp보다 우선해 즉시 0을 허용한다. controller와 guard는 `vehicle_geometry.yaml`을 직접 따로 해석하지 않고 generator가 만든 각자의 config를 사용하며, SHA-256과 `--check`로 같은 정본에서 생성됐음을 보장한다.

## 8. LiDAR, bridge, world

### 8.1 LiDAR

`lidar_body_link`에는 `lidar_body.stl` housing visual만 두고, 아래 센서는 CAD/도면으로 측정한 광학 원점 `lidar_link`에 붙인다.

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <topic>scan</topic>
    <gz_frame_id>lidar_link</gz_frame_id>
    <lidar>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159265</min_angle>
          <max_angle>3.14159265</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
  </sensor>
</gazebo>
```

레이저 평면이 차량 collision과 교차하지 않아야 한다.

### 8.2 `bridge.yaml`

```yaml
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  qos_profile: CLOCK

- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
```

Gazebo가 `/clock` 대신 `/world/slam_world/clock`만 발행하면 실제 Gazebo 토픽을 `gz_topic_name`으로 사용하고 ROS 쪽 이름은 `/clock`으로 유지한다.

### 8.3 `slam_world.sdf`

완전한 SDF에는 다음이 모두 있어야 한다.

- `<sdf>`와 `<world name="slam_world">`
- gravity
- physics: 초기값 `max_step_size=0.001`, `real_time_factor=1.0`
- Physics, UserCommands, SceneBroadcaster, Sensors 시스템 플러그인
- Sensors 플러그인의 `<render_engine>ogre2</render_engine>`
- inline ground plane collision/visual
- light
- 길이가 서로 다른 벽, 기둥, 모서리, 통로
- 대칭성을 깨는 비대칭 landmark
- ground, 벽, 기둥, landmark 모델의 `<static>true</static>`

Fuel 모델을 내려받아야만 생기는 ground plane에 의존하지 말고 월드에 plane을 inline으로 정의한다. 물리 step 1 ms는 100 Hz controller 주기 10 ms보다 충분히 짧다.

## 9. Launch

`simulation.launch.py`는 다음 순서와 의존 관계를 구현한다.

1. Xacro 평가
2. `robot_state_publisher` (`use_sim_time=true`)
3. `ros_gz_sim/launch/gz_sim.launch.py`에 `-r`, verbosity, 설치된 world 절대 경로 전달
4. `ros_gz_bridge/parameter_bridge`에 `parameters=[{'config_file': bridge_yaml}]` 전달
5. `ackermann_command_guard`를 `use_sim_time=true`, input `/cmd_vel_raw`, output `/cmd_vel`로 실행
6. `ros_gz_sim/create`로 `-world slam_world -topic robot_description -name ackermann_car` 실행
7. spawn process가 성공 종료한 뒤 `joint_state_broadcaster` spawner 실행
8. 그 spawner가 성공 종료한 뒤 Ackermann controller spawner 실행
9. RViz를 선택적으로 `use_sim_time=true`로 실행

Timer만 믿지 말고 `OnProcessExit`로 순서를 묶는다. 각 spawner에는 다음 timeout을 둔다.

```text
--controller-manager /controller_manager
--controller-manager-timeout 60
--switch-timeout 60
```

Gazebo는 `-r`로 시작해 controller switch가 paused simulation 때문에 timeout되지 않게 한다. `OnProcessExit`는 성공과 실패 모두에서 호출되므로 단순히 `on_exit=[next_action]`을 쓰면 안 된다. spawn→JSB와 JSB→Ackermann 두 연결 모두 callable handler에서 성공 여부를 판정한다.

```python
def next_only_on_success(next_action, label):
    def _on_exit(event, context):
        if event.returncode == 0:
            return [next_action]
        return [
            LogError(msg=f"{label} failed: rc={event.returncode}"),
            EmitEvent(event=Shutdown(reason=f"{label} failed")),
        ]
    return _on_exit
```

필요한 `LogError`, `EmitEvent`, `Shutdown`을 import하고 이 handler를 `OnProcessExit(on_exit=...)`에 전달한다. 비정상 return code에서 후속 controller가 시작되지 않는 것을 launch test로 확인한다.

마지막 Ackermann spawner에도 별도 `OnProcessExit` 검사를 붙인다. return code 0이면 빈 action을 반환하고, 0이 아니면 같은 `LogError + Shutdown`을 반환해야 launch가 controller 없이 살아남지 않는다.

## 10. 단계별 구현·검증 게이트

오류가 나오면 다음 단계로 넘어가지 않는다. 수정 후 현재 게이트와 영향을 받는 이전 게이트를 다시 수행한다.

### Gate A — 정적 자산

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
python3 src/ackermann_car_description/scripts/normalize_meshes.py \
  --input /home/hoodinga/Documents/SLAM/slam_files \
  --geometry src/ackermann_car_description/config/vehicle_geometry.yaml \
  --output src/ackermann_car_description/meshes \
  --check
```

- CAD 단위와 `mesh_export_unit`이 일치하는지 확인
- 원본→대상 매핑이 정확히 9개이고 제외 파일이 `KnuckleLink.stl` 하나인지 확인
- 정규화 mesh를 계획된 joint transform으로 재조립해 원본 CAD assembly와 허용오차 내에서 일치하는지 비교
- vertex뿐 아니라 normal, winding, watertight/non-manifold 검사를 다시 수행
- 모든 filename의 대소문자와 package URI 확인
- `exterior.stl`은 visual에만 존재
- `KnuckleLink.stl`은 1차 URDF에서 제외

### Gate B — 생성물과 정적 문법

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
set -euo pipefail
python3 src/ackermann_car_description/scripts/generate_geometry.py --check
set +e
rg -n \
  'TODO_CAD|MODELED_WHEELBASE_M|MODELED_STEERING_AXIS_TRACK_M|REAR_TRACK_M|REAR_EFFECTIVE_RADIUS_M|STEERING_POSITION_GAIN|SAFE_(LINEAR_SPEED|YAW_RATE)' \
  src/ackermann_car_description/config \
  src/ackermann_car_description/launch \
  src/ackermann_car_description/urdf \
  src/ackermann_car_description/worlds \
  src/ackermann_command_guard/ackermann_command_guard
placeholder_rc=$?
set -e
test "$placeholder_rc" -eq 1

python3 -m py_compile \
  src/ackermann_car_description/launch/*.py \
  src/ackermann_command_guard/ackermann_command_guard/*.py
python3 src/ackermann_car_description/scripts/validate_yaml.py \
  src/ackermann_car_description/config

xacro src/ackermann_car_description/urdf/ackermann_car.urdf.xacro \
  controllers_file:=/home/hoodinga/Documents/SLAM/ackermann_ws/src/ackermann_car_description/config/controllers.yaml \
  mesh_prefix:=file:///home/hoodinga/Documents/SLAM/ackermann_ws/src/ackermann_car_description/meshes \
  > /tmp/ackermann_car.urdf
check_urdf /tmp/ackermann_car.urdf
gz sdf -p /tmp/ackermann_car.urdf > /tmp/ackermann_car.sdf
gz sdf -k /tmp/ackermann_car.sdf
gz sdf --inertial-stats /tmp/ackermann_car.sdf
gz sdf -k src/ackermann_car_description/worlds/slam_world.sdf
```

통과 조건:

- 모든 command가 exit code 0이고 placeholder 검색은 출력 없음
- root와 링크 트리가 의도와 일치
- `base_link`, 네 wheel, 두 steering link에 inertial 존재
- A 경로의 가상 피벗, 앞뒤 axle height와 회전축이 정확
- 바퀴 최저점이 `base_footprint`의 z=0
- 생성 파일이 `vehicle_geometry.yaml`과 일치하고 직접 수정되지 않음

### Gate C — 빌드·리소스

```bash
cd /home/hoodinga/Documents/SLAM/ackermann_ws
source setup_local.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install \
  --allow-overriding \
    gz_ros2_control \
    steering_controllers_library \
    ackermann_steering_controller
source setup_local.bash
colcon test
colcon test-result --verbose
ros2 pkg prefix --share ackermann_car_description
```

`colcon.meta`가 세 upstream package의 test만 비활성화하고 프로젝트
테스트가 실제 수집되는지 확인한다. 설치된 share 아래에 `urdf`,
`meshes`, `config`, `launch`, `worlds`가 모두 있어야 하고 `ros2 pkg
executables ackermann_command_guard`에 `command_guard`가 정확히 하나
보여야 한다. launch test는 spawn, JSB spawner, Ackermann spawner
각각에 의도적인 실패를 주입해 error log와 shutdown event가 발생하고
후속 action이 시작되지 않는지 assert한다. `Shutdown` event 자체의
shell exit code가 0일 수 있으므로 이를 실패 판정으로 대신 쓰지 않는다.

### Gate D — Gazebo 정지 물리

앞선 launch가 남아 있지 않은 상태에서 한 terminal에 다음을 실행하고 Gate E~H 동안 유지한다.

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 launch ackermann_car_description simulation.launch.py rviz:=true
```

이후 검증 명령을 여는 모든 새 terminal에서도 같은
`setup_local.bash`를 먼저 source한다. 이는 package 경로뿐 아니라
검증된 CycloneDDS RMW와 ABI overlay까지 함께 고정한다.

통과 조건:

- 차량이 ground를 뚫거나 튀지 않음
- 차체 collision이 바닥에 닿지 않음
- 바퀴와 차체 collision이 깊게 겹치지 않음
- 무명령 상태에서 떨림과 drift가 없음
- LiDAR가 차체 collision 내부에 있지 않음

### Gate E — controller

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 param get /controller_manager use_sim_time
ros2 param get /controller_manager enforce_command_limits
ros2 param get /ackermann_steering_controller use_sim_time
ros2 topic info -v /joint_states
ros2 topic info -v /cmd_vel_raw
ros2 topic info -v /cmd_vel
```

통과 조건:

- `joint_state_broadcaster`와 `ackermann_steering_controller`가 active
- 후륜 velocity 및 조향 position command interface가 claimed
- 수동 앞바퀴 position/velocity state가 존재
- `/joint_states` 발행자는 simulation 중 하나
- `/cmd_vel`의 publisher는 command guard 하나
- `/cmd_vel` 타입은 `geometry_msgs/msg/TwistStamped`

### Gate F — 저속 직진·조향

직진:

```bash
ros2 topic pub --rate 10 /cmd_vel_raw geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

별도 terminal에서 guard 출력을 관찰한다.

```bash
ros2 topic echo /cmd_vel
```

회전 테스트 raw 입력의 `angular.z`를 일부러 경계 안과 밖에서 각각 보내 guard의 통과·제한을 확인한다. `ros2 topic pub`의 zero stamp는 guard 단순 시험에만 사용하고 실제 teleop은 simulation clock으로 stamp한다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p use_sim_time:=true \
  -p stamped:=true \
  -p frame_id:=base_footprint \
  -p speed:=SAFE_LINEAR_SPEED \
  -p turn:=SAFE_YAW_RATE \
  -r cmd_vel:=/cmd_vel_raw
```

실제 패키지에서는 `SAFE_*`를 CAD 확정 숫자로 치환한다. keyboard의 속도/회전 scale 키와 제자리 회전 키가 raw 한계를 넘어도 guard 출력은 항상 feasible set 안이어야 한다. 처음에는 낮은 명령으로 slip을 줄인다.

통과 조건:

- 직진, 좌/우 원호, 전/후진, `v=0·ω≠0`, 과도 곡률, 후륜 과속, timeout/NaN 입력을 자동 시험
- 모든 guard 출력이 방향별 `R_min`, 좌우 rear joint 속도, 좌우 steering 속도 한계를 만족
- 양의 `linear.x`에서 차량이 ROS `+X` 방향으로 전진
- 양의 `angular.z`에서 좌회전
- 좌우 후륜 속도와 좌우 steering angle 차이가 Ackermann 방향과 일치
- 조향/바퀴 명령이 joint limit 안에 있음
- 측정 steering joint velocity가 gain 산식의 한계 안이고 저속 원호에서 과도한 tire slip이 없음

### Gate G — odometry와 TF

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 run tf2_tools view_frames
```

SLAM 전 TF:

```text
odom → base_footprint → base_link → lidar_body_link → lidar_link
```

통과 조건:

- controller의 `tf_odometry` remap 결과가 `/tf`에 있음
- `odom→base_footprint`는 controller 하나만 책임짐
- RSP는 `base_footprint` 아래만 책임짐
- 직선과 원호 주행에서 odometry 기준점이 후륜축 중앙과 일치

### Gate H — clock과 scan

```bash
gz topic -l | rg 'clock|scan'
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic echo /scan --once
```

통과 조건:

- `/clock`이 지속 발행
- `/scan`이 약 10 Hz
- `header.frame_id == lidar_link`
- scan timestamp에서 `odom→lidar_link` TF를 조회 가능
- scan 값에 NaN 폭주나 차량 자체의 360도 ring이 없음

## 11. SLAM Toolbox

`slam.yaml`은 공식 `mapper_params_online_async.yaml`을 출발점으로 삼고 다음 핵심값을 명시한다.

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    mode: mapping

    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    scan_topic: /scan

    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    throttle_scans: 1
    scan_queue_size: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    resolution: 0.05
    min_laser_range: 0.10
    max_laser_range: 12.0
    minimum_time_interval: 0.1

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
```

실행:

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 launch ackermann_car_description slam.launch.py rviz:=true
```

Gate D에서 실행한 `simulation.launch.py`는 먼저 종료한다. `slam.launch.py`가 simulation을 정확히 한 번 include하고 내부에서 SLAM Toolbox `online_async_launch.py`, 설치된 `slam.yaml`, `use_sim_time=true`를 연결한다.

RViz:

- `use_sim_time=true`
- Fixed Frame `map`
- Map `/map`
- LaserScan `/scan`
- RobotModel `/robot_description`
- TF 전체
- controller odometry `/odom`

SLAM 통과 조건:

- `/map` 발행
- `map→odom`은 SLAM Toolbox 하나만 발행
- `odom→base_footprint`는 controller 하나만 발행
- RViz에서 scan, robot, map이 같은 시각에 정렬
- 저속 원호 주행과 loop closure 후 지도가 갑자기 이중화되지 않음

## 12. 지도 저장

occupancy map과 pose graph는 목적이 다르므로 둘 다 저장한다.

### 12.1 AMCL용 occupancy map

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
mkdir -p /home/hoodinga/Documents/SLAM/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/hoodinga/Documents/SLAM/maps/ackermann_slam \
  --ros-args -p save_map_timeout:=10.0
```

시뮬레이션 clock과 큰 map 응답에는 기본 제한이 촉박할 수 있으므로
`save_map_timeout=10.0`초를 권장한다.

생성물:

```text
ackermann_slam.yaml
ackermann_slam.pgm
```

### 12.2 이어 그리기/SLAM localization용 pose graph

```bash
ros2 service call \
  /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/hoodinga/Documents/SLAM/maps/ackermann_slam'}"

test -s /home/hoodinga/Documents/SLAM/maps/ackermann_slam.posegraph
test -s /home/hoodinga/Documents/SLAM/maps/ackermann_slam.data
```

서비스 응답의 `result`가 `0`이어야 하며 `ackermann_slam.posegraph`와 `ackermann_slam.data`가 모두 비어 있지 않아야 한다. 둘은 한 쌍이므로 함께 보관한다. occupancy map만으로는 SLAM pose graph를 이어서 편집할 수 없다.

저장 직후 `slam.launch.py` 전체를 종료해 mapping SLAM과 그 안의 simulation을 모두 내린다. 잔류 node가 없는지 확인한 뒤, simulation을 다시 포함하는 새 localization 프로세스로 round-trip을 수행한다.

```bash
source /home/hoodinga/Documents/SLAM/ackermann_ws/setup_local.bash
ros2 launch ackermann_car_description localization.launch.py \
  map_file_name:=/home/hoodinga/Documents/SLAM/maps/ackermann_slam \
  rviz:=true
```

`slam_localization.yaml`은 `mode: localization`,
`map_start_pose: [0.0, 0.0, 0.0]`, `map_start_at_dock: true`와 mapping
때와 같은 frame/scan 설정을 사용한다. 현재 Jazzy의 localization
parameter validator는 비어 있는 `map_start_pose`를 처리할 때 오류를
낼 수 있으므로 명시적인 영점 pose를 함께 둔다. 이 두 값의 조합은 현재
고정 버전에서 graph 원점/dock round-trip으로 검증된 계약이므로 하나만
독립적으로 삭제하지 않는다. 다른 시작점을 쓸 때는 사용 중인 Jazzy
버전의 validator 동작을 확인하고 두 파라미터 조합을 함께 재검증한다.
`/slam_toolbox`가 active가 되고 저장된 graph를 오류 없이 읽으며
`/map`과 `map→odom`을 다시 제공하는지 확인하는 round-trip까지 통과해야
저장 완료다.

## 13. 선택 단계: Nav2

이 절은 SLAM 지도 작성 완료 뒤의 별도 범위다. 필요할 때 다음 패키지를 추가한다.

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### 13.1 운용 모드 분리

동시에 `map→odom`을 발행하는 노드는 정확히 하나여야 한다.

1. Mapping: SLAM Toolbox mapping 실행, AMCL/SLAM localization 중지
2. 정적 지도 navigation: mapping 중지, map server + AMCL + Nav2 실행
3. Pose graph localization: mapping/AMCL 중지, SLAM Toolbox localization + Nav2 실행

### 13.2 Jazzy message와 odometry

- Jazzy Nav2의 기본 `cmd_vel`은 `Twist`이고 Ackermann controller 입력은 `TwistStamped`이다.
- `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`에 `enable_stamped_cmd_vel: true`를 일관되게 설정한다.
- 이 워크플로우의 odometry 정본은 controller에서 remap한 `/odom`이다. Nav2의 모든 `odom_topic`도 `/odom`을 사용한다.
- Nav2 전체에 `use_sim_time=true`를 적용한다.
- 표준 Jazzy `navigation_launch.py`가 포함하는 `docking_server`는 이 1차 구성의 custom launch에서 제외한다. Ackermann-safe docking과 stamped 출력, rotate-to-dock 제거를 별도 검증한 뒤에만 추가한다.

토픽 연결은 다음 하나로 고정한다.

```text
controller_server 또는 behavior_server
  → /cmd_vel_nav
  → velocity_smoother
  → /cmd_vel_smoothed
  → collision_monitor
  → /cmd_vel_raw
  → ackermann_command_guard
  → /cmd_vel
  → ackermann_steering_controller
```

Nav2 중에는 keyboard teleop을 종료한다. 수동 override가 필요하면 두 source 앞에 명시적인 mux를 두고 한 source만 선택되게 하며 `/cmd_vel_raw`에 다중 publisher를 허용하지 않는다. `velocity_smoother.scale_velocities: true`로 축 비율 보존을 돕고 Jazzy의 `stamp_smoothed_velocity_with_smoothing_time: true`를 사용하되, 최종 hard constraint는 항상 guard가 책임진다.

### 13.3 비홀로노믹 설정

- Global planner: Smac Hybrid-A*
- `motion_model_for_search`: 전진만이면 `DUBIN`, 후진 허용이면 `REEDS_SHEPP`
- Nav2의 단일 `minimum_turning_radius`: `max(R_min,left, R_min,right)`
- 기본 local controller: Jazzy MPPI의 `motion_model: "Ackermann"`과 `AckermannConstraints.min_turning_r: max(R_min,left, R_min,right)`
- RPP를 대신 쓸 경우 `use_rotate_to_heading: false`; `REEDS_SHEPP`이면 `allow_reversing: true`도 설정한다. RPP 자체는 hard 최소회전반경을 보장하지 않으므로 guard를 제거하지 않는다.
- `behavior_server.behavior_plugins`에서 Spin을 제거하고 rotation shim을 사용하지 않는다.
- 기본 BT XML의 Spin recovery action도 남지 않도록 `default_nav_to_pose_bt_xml`과 `default_nav_through_poses_bt_xml`을 non-spin custom BT로 교체한다.
- global/local costmap 모두 후륜축 기준의 실측 비대칭 polygon footprint 사용
- 모든 costmap에 `robot_base_frame: base_footprint`; global frame은 `map`, local frame은 `odom`; `/scan`을 obstacle/voxel layer source로 명시

## 14. 최종 반복 검증 규칙

각 수정은 다음 순서로 재검증한다.

```text
mesh/geometry 변경
  → generator → Gate A → B → C → D → F → G → H → SLAM

URDF/inertia/collision 변경
  → Gate B → C → D → E → F → G → H → SLAM

controller 변경
  → generator → Gate B → C → E → F → G → SLAM

command guard 변경
  → Gate B → C → E → F → G → SLAM

sensor/bridge/time 변경
  → Gate B → C → D → H → G → SLAM

SLAM parameter 변경
  → Gate B → C → G → H → SLAM → map/posegraph round-trip
```

다음 항목이 하나라도 남으면 “오류 없음”으로 판정하지 않는다.

- 실제 파일과 다른 대소문자/경로
- `setup_local.bash`를 거치지 않은 터미널, Fast DDS 재선택 또는 고정
  upstream 세 패키지와 다른 overlay 버전
- 미통과 CAD STOP gate 또는 CAD 미확정 placeholder
- A/B 조향 모델이 섞인 joint origin과 controller 치수
- stale generated config, SHA-256 불일치 또는 YAML duplicate key
- 두 번 적용된 mesh offset/rotation
- 차체 중심에 잘못 붙인 Ackermann odometry frame
- 중복 `/joint_states` 발행
- 중복 TF authority
- wall time과 simulation time 혼용
- `Twist`/`TwistStamped` 타입 불일치
- guard 우회 또는 `/cmd_vel`·`/cmd_vel_raw`의 의도하지 않은 다중 publisher
- `v=0, ω≠0`, 곡률 초과, rear wheel/steering joint 속도 초과
- 실패한 spawn/spawner 뒤에 계속 진행하는 launch
- ground/light/sensor system이 빠진 불완전 world
- occupancy map과 pose graph의 용도 혼동, `.posegraph`/`.data` 쌍 누락 또는 round-trip 실패
- Nav2의 pure-spin BT/behavior, 미검증 docking 또는 command chain 우회

## 15. 공식 근거

- [Gazebo Harmonic과 ROS 2 Jazzy 조합](https://gazebosim.org/docs/harmonic/ros_installation/)
- [gz_ros2_control Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Ackermann Steering Controller](https://control.ros.org/jazzy/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)
- [Steering Controllers Library 인터페이스](https://control.ros.org/jazzy/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html)
- [ros2_controllers 차량 운동학](https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html)
- [공식 Jazzy Ackermann Gazebo 모델](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control_demos/urdf/test_ackermann_drive.xacro.urdf)
- [ros_gz_bridge 설정과 clock](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)
- [Gazebo Harmonic GPU LiDAR와 `gz_frame_id`](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 SLAM 운용](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [Nav2 Velocity Smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)
- [Nav2 Behavior Server](https://docs.nav2.org/configuration/packages/configuring-behavior-server.html)
- [Jazzy Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/README.md)
- [Jazzy MPPI](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/README.md)
