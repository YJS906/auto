# 카메라 기반 CAD 지도 측위

## 구현 범위

`limo_localization`은 ROS 2 Jazzy / C++17 / OpenCV / Eigen으로 작성한 **첫 단계 기하 정합 베이스라인**이다.
노면의 밝은 도색을 관측하여 기존 트랙 지도에 정합하고, 휠 속도·IMU 요레이트 예측을 영상 위치 관측으로 보정한다.
측위 노드는 Ground Truth, `/cmd_vel`, 미션 순서에 의존하지 않는다.

현재 구현:

- RGB IPM: 보정된 광선을 지면과 교차시켜 BEV 좌표의 희소 특징점 생성. BEV 전체 영상을 매번 만들지 않는다.
- RGB-D: 깊이 픽셀을 역투영하고 RGB 카메라로 재투영하여 도색 선택. 서로 다른 해상도·내부/외부 파라미터를 처리한다.
- `32FC1` 깊이는 optical Z 단위 m, `16UC1`은 mm. IR 영상은 현재 입력으로 사용하지 않는다.
- 도색 지도까지의 2D Distance Transform 또는 격자 최근접 대응점을 사용하는 point-to-point ICP 비교.
- 상태 `(x,y,yaw)`의 planar EKF: 휠 선속도와 휠/IMU 요레이트 가중 결합으로 예측, 영상 pose로 보정.
- 영상 촬영 시각에 보정한 뒤 이후 휠/IMU 예측 재실행. 최대 5초 예측 이력 유지.
- RGB-D는 bounded 프레임 버퍼에서 촬영 시각 기준 짝을 찾고, 두 촬영 시각 사이의 평면 운동을 보상한다.
- 관측점 수, 정합 잔차, 인라이어 비율, 보정량, Mahalanobis innovation gate로 잘못된 보정 제한.
- 도로 형상의 약한 방향을 거리장 Hessian으로 확인하고 관측 covariance를 크게 설정.
- 최신 영상 처리 워커와 ROS 수신 분리, 영상 버퍼 각 6개, 최대 출력 특징점 수 설정, 픽셀 광선 캐시.
- 처리 단계별 wall-clock 시간, 촬영 시각부터 결과까지 ROS-clock 지연, 처리/수락/거부 횟수 진단.
- 전역 초기화: 지도 전체 x/y/yaw 격자에서 다중 후보를 찾고 기존 거리장 정합기로 정밀화.
- 후보 점수 차이·관측 가능성·서로 다른 촬영 시각 3회의 운동 일관성을 확인한 뒤 전역 위치 확정.
- 관측 상실·기울기·정합 연속 거부 시 위치 무효화와 재측위. 추적 중에도 주기적 전역 검색으로 기존 위치 검사.
- `ready`/상태/후보 출력 및 시뮬레이터 주행 명령의 선택적 측위 연동.

**아직 구현하지 않은 것:** 노면 문자 OCR/클래스 분류, 횡단보도·차선·방지턱을 구별하는 학습 기반 의미 분할,
의미 클래스별 지도 정합, 물리 방지턱의 3D 검출, IMU bias 추정,
전체 코스·실차·Orin Nano/Pi5 성능 인증.
문자·횡단보도는 현재 흰 도색의 **기하 형상**으로 사용한다. 의미 인식 성능으로 해석하지 않는다.
기본 시뮬레이터에는 물리 방지턱이 없다. 평면 IPM 가정이 깨지는 큰 기울기에서는 영상 보정을 보류한다.

## LIMO Pro 기준 장비

2026-11-01 제조사 제품 페이지와 사용자 매뉴얼을 확인했다.

| 항목 | 제조사 문서 값 | 적용 방침 |
|---|---|---|
| 컴퓨터 | NVIDIA Jetson **Orin Nano**, 6-core Cortex-A78AE, 8GB LPDDR5 | 구형 Jetson Nano와 구분. 우선 CPU 경로로 개발·계측 |
| 깊이 카메라 | Orbbec **DaBai** | 실차 CameraInfo, RGB/depth TF와 timestamp 사용 |
| IMU | HI226 | 실제 frame, gyro 축·bias·timestamp 확인 |
| LiDAR | EAI T-mini Pro | 이 측위 베이스라인의 입력에는 포함하지 않음 |
| 휠베이스 | 200mm | 후륜축 base_link와 실차 차체 기준점 차이 보정 |
| 출고 OS 예시 | Ubuntu 20.04 | 출고 이미지에 Jazzy가 있다고 가정하지 않음 |

제조사 문서에도 구형 장비 표현이 일부 혼재하므로 실물 장비/OS/메모리는 현장에서 최종 확인해야 한다.
Jazzy의 표준 Ubuntu 바이너리 대상은 Ubuntu 24.04 amd64/arm64이다.
JetPack BSP와 카메라 드라이버를 확인하고 Jazzy 사용자 공간(네이티브 또는 적절한 컨테이너)을 검증해야 한다.
이 작업은 실차 OS나 카메라 드라이버를 변경하지 않는다.

출처:

- [제조사 LIMO Pro 제품](https://global.agilex.ai/products/limo-pro)
- [제조사 매뉴얼 1.3 / 1.4](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20user%20manual(EN).md)
- [ROS Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 실행

기존 시뮬레이터 의존성이 설치된 환경:

```bash
cd /home/a/2026autorace
sudo apt install libopencv-dev libeigen3-dev ros-jazzy-cv-bridge ros-jazzy-diagnostic-msgs
source tools/env.sh
colcon build --base-paths src --packages-select limo_localization --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source tools/env.sh
```

터미널 1에서 측위 확인 전 주행을 막는 옵션으로 시뮬레이터를 실행한다:

```bash
./tools/run_sim.sh require_localization:=true
```

터미널 2에서:

```bash
source tools/env.sh
ros2 launch limo_localization localization.launch.py projection:=ipm matcher:=distance
```

RGB-D는 깊이 카메라를 활성화한 시뮬레이션에서 실행한다.
깊이 활성화를 위해 기본 센서 파일을 변경할 필요는 없다. 아래 자동 비교 도구가 별도 파일을 생성한다.

```bash
ros2 launch limo_localization localization.launch.py projection:=rgbd matcher:=icp
```

실차에서는 실제 드라이버와 RSP/TF를 실행한 상태에서:

```bash
ros2 launch limo_localization localization.launch.py use_sim_time:=false \
  config:=/absolute/path/calibrated-real-vehicle.yaml \
  image:=/actual/color/image camera_info:=/actual/color/camera_info \
  depth:=/actual/depth/image depth_info:=/actual/depth/camera_info \
  odom:=/actual/odom imu:=/actual/imu projection:=rgbd matcher:=distance
```

`config`는 기본 `config/localization.yaml`을 복사해 실제 장비 값으로 보정한다.
`ground_z`는 **base_link 좌표계의 바닥 높이**이다. 시뮬레이션 기본은 -0.045m이며 실차에 그대로 사용하지 않는다.
기본 `global_initialization:=true`에서는 초기 `(x,y,yaw)`를 정답으로 사용하지 않는다.
초기 값은 내부 예측 이력을 만들기 위한 값이며, 전역 검색이 확정되기 전에는 지도 pose/TF를 발행하지 않는다.
정확한 시작 위치를 알고 국소 정합으로 시작하려면 `global_initialization:=false`와
설정 파일의 initial_x/y/yaw를 사용한다. 이 경우에도 서로 다른 영상 3회의 확인을 거친다.

`initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)로 초기 위치를 다시 설정할 수 있다.
header.frame_id는 map_frame이며 x/y/yaw의 3x3 covariance는 유한한 양의 정부호여야 한다.
이는 전역 검색이 애매할 때 제공하는 수동 사전 정보이다. 받은 직후 `ready=false`가 되며 영상 확인이 필요하다.
차량 이동 후 반드시 수동 초기 위치를 보내야 하는 것은 아니다. 시뮬레이션 시간이 역행하거나 odom에
0.5초 초과 공백이 생기면 위치를 무효화하고 전역 검색으로 돌아간다.

## 초기화와 차량을 들어 옮겼을 때

상태 흐름은 `SEARCHING → CONFIRMING → TRACKING`, 상실 시 `LOST → CONFIRMING → TRACKING`이다.

1. 지도 전체를 기본 20cm / 15° 간격으로 검사하고 최대 32개 후보만 거리장 정합기로 정밀화한다.
2. 최상위 후보의 점수·Hessian 최소 고유값·두 번째 후보와의 점수 차이를 검사한다.
3. 휠·gyro 상대 운동으로 최근 영상의 특징점을 누적한다. 최대 3프레임/3초, 정합점 1,600개로 제한한다.
4. 서로 다른 촬영 시각의 후보가 상대 운동과 일치하는지 3회 확인한다. 확인 전에는 `ready=false`이다.
5. 확정된 pose로 영상 촬영 시각의 필터를 다시 고정하고 이후 휠·IMU 예측을 재실행한다.
   전역 재고정은 국소 EKF innovation gate와 분리한다. 큰 위치 이동을 국소 보정으로 억지로 통과시키지 않는다.

전역 검색은 미확정 상태에서 기본 wall 1Hz, 추적 중에는 ROS 시간 2초마다 수행한다.
추적은 기존 15Hz 처리 제한을 유지한다. 검색도 같은 영상 워커에서 실행하므로 검색 회차의 지연은 커진다.
`global_search_ms`로 검색 자체의 wall 시간을, `matching_ms`로 검색을 포함한 정합 시간을 확인한다.

추적 중 정합 5회 연속 거부, 영상 보정 age 0.8초 초과, odom age 0.3초 초과,
유효 IMU의 큰 tilt, 또는 확정 가능한 전역 후보와 기존 pose의 0.3m/0.3rad 초과 차이가 상실 조건이다.
상실 즉시 `ready=false`로 바꾸고 이후 지도 pose/TF 발행을 중단한다. 마지막 TF는 소비자의 버퍼에 남을 수 있으므로
**TF 존재 여부 대신 ready heartbeat와 최신 timestamp로 판단해야 한다.**

반복되는 직선·횡단보도만 보이면 위치를 확정하지 않을 수 있다. 후보 빔은 유한한 근사 검색이므로
전역 최적성이나 모든 위치에서의 초기화를 보장하지 않는다. 지도 밖, 잘못된 보정, 동일 무늬로의 이동도 별도 한계다.
특히 수평으로 들어 동일하게 보이는 구간에 옮기면 휠/gyro/카메라만으로 즉시 구분할 수 없다.
고유한 문자 형상·교차점 등이 보이거나 수동 initialpose가 들어올 때까지 대기해야 할 수 있다.
자동 탐색 주행은 구현하지 않았다. 대회 미션 완료 이력의 초기화/복원 역시 미션 관리자가 별도로 결정해야 한다.

시뮬레이터 `require_localization:=true`는 `/localization/ready`가 false이거나 wall heartbeat가
`command_timeout`(기본 0.35초)을 넘으면 설정된 감속 한도로 정지시킨다. 재확정 후에는 새로운 명령이 필요하다.
기본값 false는 기존 수동 시뮬레이션 용도이다. namespace를 사용하면 guard의 ready 구독도 같은 namespace로 맞춰야 한다.
실차 제어기/미션 관리자에도 동등한 유효성·heartbeat gate를 연결해야 하며, 이 변경만으로 실차 정지가 연결되지는 않는다.

## 입력, 출력과 TF

| 입력 | 타입 | 조건 |
|---|---|---|
| image / camera_info | Image / CameraInfo | BGR 변환 가능한 RGB, 올바른 K/D, 같은 optical frame |
| depth / depth_info | Image / CameraInfo | 원 깊이 자체의 K/D와 optical TF. RGB에 정렬되어 있다고 가정하지 않음 |
| odom | Odometry | twist가 child_frame_id 기준, child_frame_id가 설정 base_frame과 같아야 함 |
| imu | Imu | gyro covariance[0] >= 0, IMU->base TF 필요 |
| initialpose | PoseWithCovarianceStamped | 지도 좌표의 초기 위치와 covariance |

RGB/depth의 distortion model은 `plumb_bob`/`rational_polynomial` 또는 이미 보정한 영상이다.
다른 모델은 외부에서 올바르게 rectify해야 한다. 장착 외부 파라미터는 TF에서 가져온다.
기본 RGB 20Hz / depth 10Hz의 위상 차이를 수용하도록 최대 timestamp 차이는 60ms이며
두 촬영 시각 사이의 휠·gyro 운동을 보상한다. 실차에서는 실제 동기화·최대 운동 오차에 맞게 제한해야 한다.
IMU의 gyro를 TF로 base 축으로 회전한다. orientation이 유효하면 평면 가정을 벗어나는 tilt를 검사한다.
단, 고정한 WEGO driver의 `publishImu`는 `setRPY(0,0,yaw)`로 orientation을 만들므로,
그 드라이버의 quaternion만으로 실제 pitch/roll을 알 수 없다. 방지턱 tilt 검증에는 실제 roll/pitch를
제공하는 IMU 출력 또는 별도 자세 추정이 필요하다. 현재 tilt 보류 기능을 실차 방지턱 대응 완료로 해석하지 않는다.
절대 IMU yaw를 지도 yaw로 간주하지 않으며, 가속도 이중 적분이나 15상태 INS는 사용하지 않는다.
휠/IMU 속도 공분산은 현재 설정 잡음으로 모델링하며 장비 noise/bias 식별이 남아 있다.

| 출력 | 용도 |
|---|---|
| localization/pose | odom 촬영 시각의 지도 기준 필터 pose와 covariance |
| localization/visual_pose | 수락한 영상 관측 pose, 원 영상 촬영 timestamp 유지 |
| localization/ready | Bool, 10Hz wall heartbeat, transient local. TRACKING에서만 true |
| localization/state | String: SEARCHING / CONFIRMING / TRACKING / LOST |
| localization/candidates | PoseArray, 최근 전역 검색의 상위 8개 후보, 진단용 |
| localization/diagnostics | 특징 수·잔차·약한 방향·실측 처리 시간·영상 보정 경과시간 |
| map -> odom | 지도 보정. `publish_tf:=false`로 비활성화 가능 |

기존 odom->base_link 및 차체->카메라 TF 발행자는 유지한다. map->odom은 한 노드만 발행해야 한다.
Ground Truth `/evaluation/ground_truth`는 **평가 Python 도구에만** 연결된다.
짧은 관측 공백에는 예측을 발행하고 상실 기준을 넘으면 발행을 멈춘다. 소비자는 ready heartbeat,
covariance, diagnostics의 `visual_age_sim_s` 및 pose timestamp를 함께 확인해야 한다.
측위 노드 자체는 cmd_vel을 발행하지 않는다.

지도 원점은 기존 CAD 변환의 좌측 아래 (0,0), +X 오른쪽, +Y 위쪽이다.
원본 texture는 image top=+Y로 읽고 1cm 격자로 축소한다. map_width/height/resolution은 설정 가능하다.
현재 별도 map origin 파라미터는 없으므로 실차 지도도 이 원점 정의에 맞춘다.
노면 도색 지도는 장애물 costmap이 아니다.

## 알고리즘 비교와 제한

| 항목 | IPM | RGB-D |
|---|---|---|
| 지면 가정 | 카메라 높이/각도로 평면 교차 | 측정 깊이로 지면 근처 점 선택 |
| 장점 | 깊이 노이즈·depth 누락에 독립, 연산량 작음 | 실제 3D 관측으로 지면 외 물체 제거 가능 |
| 약점 | pitch/roll·방지턱·외부 보정 오차 | depth 유효거리·반사·정렬·동기화에 의존 |

| 항목 | ICP | Distance Transform |
|---|---|---|
| 대응/목적 | 격자 최근접 도색점과 2D 좌표 오차 | 도색까지의 거리장 잔차 |
| 구현 | 지도 최근접 label field, O(1) 조회 | 거리장 bilinear 조회 + 수치 gradient |
| 공통 | SE(2) Gauss-Newton, robust weighting, 최대 iteration/보정량 제한 | 동일 |

최근접 label field는 격자 근사이다. PCL 3D KD-tree ICP와 동일 구현/성능으로 주장하지 않는다.
직선에서 진행 방향 위치, 반복 패턴에서 잘못된 대응은 여전히 문제가 된다.
잔차가 작다는 이유만으로 전역 위치가 맞다고 판단하지 않는다. 후보 차이와 시간 일관성도 검사하지만
반복 형상의 모호성이 완전히 해결되는 것은 아니다.
현재 covariance는 거리장 형상과 잔차로 근사하며 실차에서 통계적으로 검증된 정확도 보증이 아니다.

## 검증과 성능 측정

```bash
source tools/env.sh
colcon test --packages-select limo_localization
colcon test-result --verbose
python3 tools/run_localization_benchmark.py --output /tmp/localization-new-run
python3 tools/check_global_localization.py --output /tmp/global-localization-new-run
```

비교 도구는 독립 domain/partition에 새 Gazebo와 네 측위 노드를 실행한다.
출력 폴더가 이미 있으면 중단한다. 기본 시나리오:

1. `global_initialization:=false`로 초기 위치 오차를 주고 정지 상태에서 정합.
2. 휠 선속도 +3% 오차 (`--wheel-scale 1.0`으로 제거 가능).
3. 전진 중 RGB를 2초간 검게 만들어 관측 상실. 어느 한 조합이라도 ready=false이면 모두 정지.
4. 영상 회복과 재측위 후 전진, 정지, 작은 좌우 원호, 후진, 최종 정지.

전진 구간은 3초이며 motion 구간의 시간은 모두 ready인 동안만 누적한다.
정지 상태에서 20초 동안 재확정되지 않으면 시험을 실패 처리하고 부분 결과도 남긴다.
`--blank-seconds 0`으로 영상 가림을 제외할 수 있다. 반복 무늬 구간에서 멈춰 재측위할 수 없는 경우는
오차가 작은 출력만 추려 통과시키지 않는다. `phase_ready_fraction`과 구간별 출력 유무도 확인한다.

네 조합은 동일한 relayed RGB, 원 depth, 보정된 wheel 입력을 관측한다.
동시에 실행하므로 **연산 자원은 경합하며**, 정합 호출 시점과 샘플 선택은 완전히 같지 않다.
Ground Truth는 timestamp 선형/각도 보간해 비교한다. 시작 초기화 구간은 주행 오차 집계에서 제외하고 따로 남긴다.
`summary.json`과 시각별 `samples.json`, 각 실행 로그를 보존한다.
10cm p95 검사 한계는 초기 통합 회귀 기준이며 **대회 ±5cm 정지 규정의 충족을 뜻하지 않는다**.

- 위치: 평균, RMSE, 중앙값, p95, 최대 오차와 구간별 오차.
- yaw: degree 단위 동일 집계.
- 특징 추출/정합: `steady_clock` wall ms. 입력 대기/ROS 전달 시간은 포함하지 않음.
- capture-to-result: ROS clock 기준 age. 시뮬레이션에서는 sim ms로 wall latency와 다름.
- 처리 Hz: wall-clock 기준 실제 처리율. 카메라 sim 주파수와 RTF, 설정 processing_rate의 영향을 받음.
- 실시간성은 Orin Nano에서 단일 노드 + 실제 카메라로 측정해야 최종 판단할 수 있음.

실제 측정 결과는 `reports/localization/`의 실행 요약을 참고한다.
장기 과제는 전체 트랙 반복/조명/카메라 오차/깊이 누락/시간 지연/재시작 시험, ARM 빌드와 thermal soak이다.

전역 회귀 도구는 별도로 초기 영상 가림/차단 명령, 초기 위치 없는 검색, 영상 가림 후 재측위,
Gazebo `set_pose`로 위치·방향을 바꾸는 고장 주입, 측위 프로세스 정지 시 heartbeat 차단을 검사한다.
`set_pose`는 평가 도구에만 있고 측위 입력이 아니다. `--start X Y YAW`, `--relocate X Y YAW`로 시험 위치를 바꿀 수 있다.
단위 시험은 동일 형상이 두 곳에 있는 지도, 직선의 관측 불가능 방향, 빈 영상, 후보 확인 실패,
지연된 큰 위치 재고정과 예측 재실행도 검사한다.

### 2026-11-02 전역 초기화·재측위 시험

최종 소스의 IPM + 거리장, 단일 측위 노드와 Gazebo, x86 호스트 시험이다.
`reports/localization/2026-11-02-global-reverse/summary.json`에 결과를 보존했다.
전역 위치를 주지 않고 `(4.5, 4.65, π)`에서 시작해 `(1.9, 5.57, 0.05)`로 강제 이동했다.
시작 좌표·이동 서비스·Ground Truth는 평가 도구에서만 사용했다. 반대 순서의 선행 시험은 `2026-11-02-global/`에 있다.

| 항목 | 확정까지 ROS 시간 | 확정 후 1초 정지 시 위치 오차 | yaw 오차 |
|---|---:|---:|---:|
| 초기 좌표 없는 시작 | 5.07초 | 0.69cm | 0.18° |
| 영상 가림 후 복구 | 4.95초 | 0.68cm | 0.18° |
| 강제 이동 감지 후 재측위 | 2.62초 | 1.01cm | 0.62° |

이동 감지 시간은 0.56 ROS초였으며 위 재측위 시간은 감지 이후부터이다.
전역 검색 wall p95는 219.2ms, 최대 236.8ms였다. 확정 시간은 시작 시점의 영상 처리 주기와 후보 상태에 영향을 받는다.
빈 영상 초기 pose 미발행, 미확정/상실 시 주행 차단, 측위 프로세스 SIGSTOP 후 heartbeat 차단 등 9개 검사를 통과했다.
단위·회귀 CTest 집계도 9개 통과했다. 물리적으로 들어 옮기는 동역학이나 ARM 성능 시험은 아니다.

**연속 주행 네 조합 회귀는 미통과했다.** `reports/localization/2026-11-02-tracking/`에 부분 결과와 실패 로그를 보존했다.
전진 후 영상 가림으로 정지한 지점에서 IPM 두 조합의 후보 점수 차이가 약 2.29mm로 4mm 기준보다 작아
위치를 확정하지 못했고, recovery에서 20초 대기 한계로 종료했다. 마지막 RGB-D 두 조합은 TRACKING이었다.
출력된 위치의 오차만으로 이 시험을 통과로 처리하지 않았다. 고유한 관측이 부족한 구간의 자동 재출발은 남은 과제다.

### 2026-11-01 호스트 통합 시험

전역 재측위를 추가하기 전 베이스라인 소스의 짧은 전진·좌우 원호·후진 시험. 휠 속도 +3%, 초기 오차 약 7.2cm / 0.03rad,
RGB 2초 가림을 포함한다. 시작 초기화·settle을 제외한 주행 집계이며 초기 구간도 원시 자료에는 남아 있다.

| 조합 | 위치 RMSE | 위치 p95 | yaw p95 | 추출+정합 wall p95 | 실제 처리 wall Hz |
|---|---:|---:|---:|---:|---:|
| IPM + Distance Transform | 1.35cm | 2.53cm | 0.092° | 1.94ms | 9.99 |
| IPM + ICP | 1.74cm | 3.01cm | 0.283° | 1.70ms | 10.19 |
| RGB-D + Distance Transform | 1.77cm | 3.46cm | 0.266° | 1.59ms | 4.94 |
| RGB-D + ICP | 2.17cm | 3.79cm | 0.405° | 1.39ms | 4.65 |

`reports/localization/2026-11-01/summary.json`이 수치의 근거이며,
`samples.json.gz`에는 시각별 오차와 진단을 보존했다. 빌드·테스트 로그도 같은 폴더에 있다.
RGB 20Hz / depth 10Hz는 **시뮬레이션 시간** 설정이고 표의 Hz는 **wall 시간** 처리율이다.
Gazebo RTF와 네 노드의 동시 실행 부하가 포함되므로 RGB-D 알고리즘의 최대 처리율로 해석하지 않는다.
추출+정합 시간은 EKF 보정·ROS 송수신·영상 대기 시간을 포함하지 않는다. end-to-end ROS-clock 지연은 JSON에 별도 기록한다.

호스트 단위/회귀 검사는 CTest 집계 7개 통과. 기본 map->odom TF의 초기 위치와 바닥 높이 보정도
별도 ROS 입출력 확인으로 검사했다. 이 결과로 실차 ±5cm 정지나 Orin Nano 실시간 실행을 보증하지 않는다.

## 참고 오픈소스와 후속 단계

- [AutoMiny Road Marking Localization](https://autominy.github.io/AutoMiny/docs/road-marking-localization/):
  IR/depth 도색 점군과 사전 지도 ICP. 관측-지도 정합 구성의 참고.
- [YabLoc 현재 Autoware 문서](https://autowarefoundation.github.io/autoware_universe/main/localization/yabloc/yabloc_particle_filter/):
  예측기와 영상 보정기를 분리한 particle filter. 다중 가설/지연 관측 참고.
  [이전 standalone 저장소](https://github.com/tier4/YabLoc)는 archived이며 현재 개발 위치와 구분한다.
- [RoadLib master](https://github.com/GREAT-WHU/RoadLib/tree/master):
  의미 인스턴스 도색, 불확실성, 지도 정합 참고. main README는 placeholder이므로 master를 확인한다.
  Ceres/PCL/GLFW와 사전 학습 분할 모델을 그대로 의존성으로 넣지 않았다.

위 구현의 소스 코드를 복사하거나 링크하지 않고 프로젝트용 코어를 새로 작성했다.
학습 기반 의미 분할을 추가하려면 실제 대회 노면 데이터를 수집·검증하고,
클래스별 관측/지도 distance field를 비교하여 반복 도색 오정합이 실제로 줄어드는지 평가한다.
현재 RGB-D/IPM × ICP/거리장 비교 결과를 기준으로 기본 경로를 먼저 선택한다.
