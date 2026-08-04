# Ackermann SLAM 정량 실험 결과

> 실행일: 2026-08-04 (Asia/Seoul)<br>
> 환경: ROS 2 Jazzy, Gazebo Harmonic, `rmw_cyclonedds_cpp`<br>
> 결과 성격: 보고서 작성을 위한 신속 정량 평가

## 1. 실행 가능성 판정

| 실험 | 판정 | 실행 결과 |
|---|---|---|
| Fusion 이론–실측 조향각 | 실행 불가 | Fusion 360 조립 모델과 실제 tie-rod joint 상태가 없어 실측 생략 |
| 직선 거리 오차 | 실행 가능 | Gazebo 재시작 조건으로 5회 완료 |
| 좌·우 원호 반경 | 실행 가능 | 좌 5회, 우 5회 완료 |
| 지도 거리 오차 | 실행 가능 | 단일 SLAM 지도에서 3개 거리 측정 완료 |
| pose graph localization 재로딩 | 실행 가능 | 새 프로세스에서 graph·active·map·TF 확인 완료 |
| Watchdog 정량 지연 | 제외 | 요청대로 반복 지연 시험은 하지 않고 timeout 기능만 확인 |

Fusion 결과는 현재 simulation의 virtual steering-axis 측정값으로 대체하지 않는다. 두 모델의 기구학 구조가 다르기 때문이다.

## 2. 공통 측정 조건

- 직선·원호 실험은 매회 simulation을 완전히 종료하고 새 Gazebo world에서 시작했다.
- 각 run은 고유 `ROS_DOMAIN_ID`와 `GZ_PARTITION`으로 격리했다.
- 입력 명령은 `/cmd_vel_raw`에 10 Hz로 발행했다.
- command guard 출력 `/cmd_vel`, `/odom`, `/joint_states`를 simulation time으로 수집했다.
- Gazebo GT는 `/world/slam_world/dynamic_pose/info`의 `ackermann_car` model pose를 사용했다.
- 직선 명령: `v=0.2 m/s`, `ω=0 rad/s`, 5.0 s.
- 원호 명령: `v=0.1 m/s`, `ω=±0.2 rad/s`, 8.0 s.
- 원호 fitting은 조향 과도구간을 제외한 `명령 시작+0.75 s`부터 `명령 종료-0.1 s`까지 최소제곱 원 fitting을 사용했다.
- 표준편차는 표본 표준편차(`ddof=1`)다.

이번 신속 시험은 rosbag 대신 실행 중 sample을 수집해 run별 요약 JSON을 남겼다. 공식 제출용 원시 trajectory가 필요하면 같은 스크립트와 함께 rosbag을 추가 기록한다.

## 3. 직선 거리 오차

가속도 제한 때문에 이론적 `1.0 m` 대신 실제 guard 출력 적분값을 함께 기록했다.

| Run | GT 거리 (m) | Odom 거리 (m) | Odom–GT 절대오차 (m) | 상대오차 (%) |
|---:|---:|---:|---:|---:|
| 1 | 0.964726 | 0.961000 | 0.003726 | 0.3863 |
| 2 | 0.964751 | 0.961050 | 0.003701 | 0.3836 |
| 3 | 0.964726 | 0.960800 | 0.003926 | 0.4070 |
| 4 | 0.964766 | 0.960907 | 0.003859 | 0.4000 |
| 5 | 0.964741 | 0.960846 | 0.003895 | 0.4038 |
| 평균 | **0.964742** | **0.960921** | **0.003822** | **0.3961** |
| 표준편차 | 0.000017 | 0.000104 | 0.000102 | 0.0106 |

추가 결과:

- `/cmd_vel` 적분 기준거리: `0.960941 ± 0.000098 m`
- GT 횡방향 편차: 최대 `1.80×10⁻¹⁵ m`, 수치 정밀도 범위에서 0
- Odom은 GT보다 평균 `3.82 mm` 짧게 계산했다.

## 4. 좌·우 원호 반경

Steady-state `/cmd_vel`은 10회 모두 `v=0.1 m/s`, `|ω|=0.2 rad/s`였으므로 guard clamp는 발생하지 않았고 기준반경은 정확히 `0.5 m`다.

### 4.1 요약

| 방향 | GT 반경 (m) | 기준 대비 절대오차 (m) | 기준 대비 상대오차 (%) | Odom 반경 (m) |
|---|---:|---:|---:|---:|
| 좌, 평균±표준편차 | **0.522104 ± 0.000263** | 0.022104 ± 0.000263 | 4.4208 ± 0.0526 | 0.505794 ± 0.000074 |
| 우, 평균±표준편차 | **0.522762 ± 0.001290** | 0.022762 ± 0.001290 | 4.5523 ± 0.2579 | 0.505895 ± 0.000065 |

- 좌·우 평균반경 대칭 오차: `0.000658 m`
- 기준반경 0.5 m 대비 대칭 오차: `0.1315%`
- GT circle-fit residual RMS: 좌 `0.000617 ± 0.000016 m`, 우 `0.000626 ± 0.000033 m`

### 4.2 Run별 GT fitted radius

| Run | 좌회전 (m) | 우회전 (m) |
|---:|---:|---:|
| 1 | 0.522061 | 0.522800 |
| 2 | 0.521815 | 0.522673 |
| 3 | 0.522399 | 0.524716 |
| 4 | 0.522350 | 0.521099 |
| 5 | 0.521894 | 0.522519 |

### 4.3 Gazebo steering joint 측정

이 값은 Fusion 360 tie-rod 측정값이 아니라 현재 virtual steering-axis simulation의 steady-state joint 값이다.

| 방향 | 내측각 (deg) | 외측각 (deg) |
|---|---:|---:|
| 좌, 평균±표준편차 | 20.6550 ± 0.0102 | 16.2141 ± 0.0077 |
| 우, 평균±표준편차 | 20.6427 ± 0.0097 | 16.2084 ± 0.0056 |

`κ=2.0 m⁻¹`의 이상적 target은 내측 `21.4600°`, 외측 `16.8387°`다. 실제 joint 각도가 target보다 작아 GT 반경이 0.5 m보다 약 4.4~4.6% 커진 것으로 해석할 수 있다.

## 5. Fusion 조향각

### 5.1 실측 생략 사유

현재 저장소에는 Fusion 360 assembly, revolute joint, tie-rod constraint 또는 측정 가능한 design file이 없다. STL은 정적 triangle mesh이므로 조향 linkage를 구동하거나 외측각을 측정할 수 없다.

### 5.2 이상적 외측각 계산값

`L=171 mm`, `W=130 mm`에서 계산한 값이다.

| 내측각 (deg) | 이상적 외측각 (deg) |
|---:|---:|
| 10 | 8.8378 |
| 20 | 15.9121 |
| 30 | 21.8625 |
| 40 | 27.1260 |

Fusion file을 확보하면 실제 외측각과 위 값의 절대·상대·좌우 대칭 오차를 채운다. 현재는 Fusion 오차값을 생성하지 않는다.

## 6. 지도 거리 오차

### 6.1 지도 조건

- 주행: `v=0.1 m/s`, `ω=-0.2 rad/s`, 10 Hz, 31.5 s
- 결과: `237 × 217 cell`, `0.05 m/cell`
- mapping `/map` origin: `(-5.946090, -4.947763, 0)`
- 측정 기준: 저장 PGM의 occupied cell 중심
- 반복 구분: **단일 지도 기하 오차 평가**, 반복 mapping 실험 아님

### 6.2 측정 결과

| 측정 대상 | World 기준 (m) | Map 측정 (m) | 절대오차 (m) | 상대오차 (%) |
|---|---:|---:|---:|---:|
| 서–동 외벽 안쪽 면 간격 | 11.8000 | 11.7500 | 0.0500 | 0.4237 |
| 남–북 외벽 안쪽 면 간격 | 10.8000 | 10.7500 | 0.0500 | 0.4630 |
| 내부 긴 벽 면 길이 | 4.2000 | 4.1839 | 0.0161 | 0.3841 |
| 평균 | 8.9333 | 8.8946 | **0.0387** | **0.4236** |

첫 두 오차는 정확히 map resolution 1 cell이다. 벽 두께와 cell 중심을 사용하는 trinary map의 양자화 영향을 결과 해석에 포함해야 한다.

## 7. Localization pose-graph 재로딩

Mapping 프로세스와 Gazebo를 완전히 종료하고, 다른 domain과 partition의 새 프로세스에서 확장자 없는 base path를 전달했다.

```text
/home/hoodinga/Documents/SLAM/maps/ackermann_eval_20260804
```

### 7.1 3중 판정 결과

| 판정 항목 | 관측 결과 | 판정 |
|---|---|---|
| 새 프로세스에서 pose graph 열기 | `Load From File ...ackermann_eval_20260804.posegraph`, Dataset load 완료 | PASS |
| SLAM Toolbox lifecycle | `active [3]` | PASS |
| `/map` 재생성 | publisher=`/slam_toolbox`, `237×217`, `0.05 m/cell`, mapping과 동일 origin | PASS |
| `map → odom` TF 재생성 | time 27~30 s에서 반복 출력 | PASS |

관측된 `map → odom` 예:

```text
Translation: [0.010, 0.000, 0.000]
Quaternion (x,y,z,w): [0.000, 0.000, -0.000, 1.000]
```

`tf2_echo` 시작 직후 한 번은 map frame 대기 메시지가 나왔고, 첫 TF publication 이후 정상 변환이 반복됐다.

### 7.2 저장 파일

| 파일 | 크기 (byte) | SHA-256 |
|---|---:|---|
| `ackermann_eval_20260804.pgm` | 51,444 | `295bd076234d7f75881f1fea90a4f080e928eb94190ebba84ac4c368109d0460` |
| `ackermann_eval_20260804.yaml` | 145 | `dc66bdc08b6c61724a429443dc32d82db7696f9afe783d081da74cbfa2e192ac` |
| `ackermann_eval_20260804.posegraph` | 8,718,336 | `742f7bd13af7a0ac2d07577e02fe5ba27ff6c3303b90a48c4621ceff80380f72` |
| `ackermann_eval_20260804.data` | 1,747,604 | `ed5cce9888b36676357b74bc19ad377a26f326aa03b0e0298be3a797573a1e44` |

## 8. Watchdog 기능

주행 명령 발행 종료 뒤 command guard에서 `watchdog_timeout` 상태가 관측되고 0 명령이 출력됐다. 설정값 0.5 s의 기능 확인만 유지하며 지연 평균·표준편차는 계산하지 않았다.

## 9. 결과 파일

- `experiment_results/2026-08-04/ackermann_quant_summary.json`
- `experiment_results/2026-08-04/ackermann_map_measurements.json`
- `experiment_results/2026-08-04/quant/*.json`
- `experiment_results/2026-08-04/ackermann_quant_run.py`
- `experiment_results/2026-08-04/run_ackermann_quant_suite.sh`
- `experiment_results/2026-08-04/summarize_ackermann_quant.py`
- `experiment_results/2026-08-04/measure_ackermann_map.py`
- `maps/ackermann_eval_20260804.{pgm,yaml,posegraph,data}`

## 10. 보고서 반영 시 제한

- Fusion 측정값은 아직 없으므로 이상적 계산값과 Gazebo joint 값을 Fusion 결과로 표시하지 않는다.
- 지도 결과는 한 지도에서 3개 거리를 잰 결과이므로 mapping 반복성으로 확대 해석하지 않는다.
- 직선·원호 run별 요약은 남았지만 raw rosbag은 저장하지 않았다.
- 모든 값은 가정 기반 simulation 모델의 결과이며 CAD/실차 정확도를 뜻하지 않는다.
