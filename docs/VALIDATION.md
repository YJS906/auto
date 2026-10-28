# 실제 실행 검증 결과

2026-10-28, 로컬 Ubuntu 24.04 / Jazzy / Harmonic 8.15, AMD 780M 렌더링에서 수행했습니다. 최신 요청에 따라 자율주행 알고리즘은 포함하지 않습니다.

| 항목 | 결과 | 근거와 범위 |
|---|---|---|
| ROS 패키지 빌드 | 통과 | `reports/build.log`, 패키지 1개 |
| 단위 검사 | 통과 | `test_core.py` 5개 pytest, CTest wrapper 포함 집계 6개 |
| Ackermann / diff URDF→SDF | 통과 | `reports/ackermann_sdf_check.txt`, `diff_sdf_check.txt` |
| 도면 축척 | 통과 | PDF 10000×6000, DXF 333.333333×200와 dimlfac 30 교차 확인 |
| 트랙/차량 모델 비율 | 모델 기준 통과 | 트랙 10×6 m, 바퀴 반지름 0.045 m, 휠베이스 0.2 m. 실제 Pro 및 출력물 실측은 미검증 |
| 트랙 전체 GUI 표시 | 통과 | `reports/gazebo_overview.png`, 기본 스폰 차량 포함 |
| 접지·바닥 관통·튀기 | 통과 | 아래 15초 시퀀스에서 후륜 중심 Z 약 0.04498~0.04500 m |
| 직진·좌회전·우회전·정지 | 통과 | 실제 관절 회전/조향과 ground truth 이동 확인 |
| 앞바퀴 수동 회전과 조향 | 통과 | JointState에서 전륜 회전, 좌우 조향 부호 변화 확인 |
| 입력 끊김 후 정지 | 통과 | timeout 구간의 최종 위치 변화 2 cm 미만 |
| 전방 RGB 영상 | 통과 | 640×480에서 양쪽 차선·노면 문자·횡단보도 실제 렌더링. `reports/smoke/settle.png` |
| Image/CameraInfo | 통과 | frame, 해상도, K와 hfov, 일치하는 촬영 timestamp 확인 |
| ROS TF·주행계·IMU·clock | 통과 | 필수 메시지 수신, optical frame 연결, clock 단조 증가 |
| TF 중복 책임 | 구성 점검 통과 | `/tf`의 두 publisher는 RSP와 bridge이며 서로 다른 edge 담당. `reports/tf_publishers.txt` |
| 차선 충돌 | 통과 | paint model에 collision 0개; 기본 world에는 바닥 collision만 존재 |
| 완전 초기화·반복 시험 | 통과 | Gazebo와 ROS 노드를 새 프로세스로 두 번 실행, 아래 결과 |
| rosbag 기록 | 통과 | trial별 MCAP 및 metadata 생성, 첫 기록 15.409초 / 약 263 MiB |
| 차동구동 실제 주행 | 수정 후 통과 | 직진·좌우 회전·제자리 회전·입력 중단 정지 등 23개 검사. 아래 추가 검증 참조 |
| 선택 depth/LiDAR | 시뮬레이션 통과 | 활성화 후 14개 검사: 거리, 발행률, 해상도, CameraInfo, 프레임·타임스탬프. 실차 센서 보정은 미검증 |
| LiDAR에서 차선 무검출 | 시험 구간 통과 | 기본 스폰 전방 ±20°에서 평면 트랙 반환점 0개, 시험 상자 추가 후 약 0.8 m 검출 |
| RViz 화면 | 통과 | 기본 설정의 RGB 영상·차량 모델·TF·Scan 실제 표시와 Global Status: Ok 확인. `reports/extended/rviz/` |
| 센서 지연/노이즈 보정 | 미검증 | 설정 가능하지만 실차 통계로 보정하지 않음 |
| 실차 성능·대회 완주 | 미검증/범위 밖 | 현재 환경의 성공으로 실차 결과를 보장하지 않음 |

## 반복 시험 수치

시험: 2초 접지 대기 → 0.25 m/s 직진 3초 → 정지 2초 → 0.2 m/s, +0.3 rad/s 3초 → −0.3 rad/s 3초 → 입력 중단 2초. 가감속·조향 응답 제한이 적용되므로 단순 `속도×시간`과 거리가 다릅니다.

| 시험 | 직진 거리 | 결과 |
|---|---:|---|
| 새 서버 1 | 0.6777965 m | 15개 런타임 check 통과 |
| 새 서버 2 | 0.6843090 m | 15개 런타임 check 통과 |

근거: `reports/repeat/summary.json`, 각 `trial_*/smoke.json`, `trajectory.json`, `sim.log`, `bag/`. ROS 시작 시점과 통신 스케줄링으로 결과가 완전히 같지는 않습니다. 결정론적 모터/센서 시뮬레이션으로 주장하지 않습니다. 초기 스폰 위치와 전체 노드 상태를 다시 만드는 재현 절차입니다.

최종 GUI 실행 상태에서도 `reports/final_smoke/smoke.json`의 모든 런타임 check가 통과했습니다. 시험 명령 발행률을 시뮬레이션 시간 기준 최대 50 Hz로 제한했습니다. 초기 반복 기록은 제한 전 시험 도구로 수집되어 `/cmd_vel` 기록 빈도가 높습니다.

처음 발견한 텍스처 흰색 렌더링은 기본 plane UV로 수정했습니다. 수정 전 시험은 `reports/smoke_first`에 남아 있으므로 영상 검증 증거로 사용하지 않습니다. `reports/smoke`와 이후 결과는 수정된 노면입니다.

## NVIDIA 드라이버 복구와 재검증 (2026-10-28)

현재 커널은 `7.0.0-34-generic`인데 설치된 NVIDIA 모듈은 이전 `7.0.0-31`용뿐이었습니다. Secure Boot는 꺼져 있었고 `modinfo nvidia`는 모듈을 찾지 못했습니다. 패키지 목록 갱신 후 기존 커널용 사전 제작 모듈이 요구하는 NVIDIA 버전과 배포 중인 버전이 달라, `nvidia-driver-595-open`, `nvidia-dkms-595-open`, 현재 커널 헤더로 `595.99.02` 모듈을 빌드했습니다.

`modprobe nvidia` 후 재부팅 없이 `nvidia-smi`가 정상 동작했습니다. 의존성으로 새 커널 `7.0.0-42`와 같은 버전의 사전 제작 NVIDIA 모듈도 설치됐지만 복구 직후에는 새 커널로 부팅한 검증을 수행하지 않았습니다. 이후 검증은 아래 추가 검증 절에 기록했습니다.

별도 ROS 도메인 74에서 NVIDIA EGL을 선택해 기본 아커만 시험을 수행했습니다. **15개 검사 모두 통과**, RGB 296개 수신, 직진 거리 0.684349 m. Ogre2 로그의 `GL_RENDERER`는 `NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE2`, `nvidia-smi`에는 Gazebo 렌더링 프로세스와 GPU 메모리 사용 약 422 MiB가 기록됐습니다. 이는 headless 센서 렌더링 검증이며 RViz 화면 검증과는 별개입니다.

근거: `reports/nvidia/smoke/smoke.json`, `reports/nvidia/nvidia-smi.txt`, `reports/nvidia/ogre2.log`, 해당 폴더의 실행 로그와 이미지. 시스템 전체 GPU 설정이나 기본 실행 명령의 GPU 선택은 변경하지 않았습니다.

## 차동구동·선택 센서·RViz 추가 검증 (2026-10-28)

재부팅 후 커널 `7.0.0-42-generic`, NVIDIA 드라이버 `595.99.02`에서 검증했습니다. 따라서 위 NVIDIA 복구 기록 이후 새 커널에서의 GPU 인식과 실제 센서 렌더링까지 확인된 상태입니다. `tools/run_extended_checks.py --nvidia`는 검사마다 새 Gazebo/ROS 프로세스를 사용합니다.

### 차동구동에서 발견하고 수정한 문제

수정 전에는 직진과 바퀴 회전은 정상인데, 좌우 회전의 실제 차체 yaw 변화가 약 ±0.000001 rad이고 제자리 회전은 0 rad였습니다. 바퀴 회전량만 확인하면 통과할 수 있는 결함이라 ground truth 차체 방향과 차체 중심 이동도 함께 검사했습니다. 수정 전 수치는 `reports/extended/diff/before_fix.json`에 보존했습니다.

네 고정 바퀴가 회전할 때 필요한 측면 미끄럼이 0으로 설정되어 있었습니다. collision 좌표계의 마찰 방향 `fdir1`을 바퀴 축과 평행한 `0 0 1`로 지정하고, 차동모드의 측면 slip compliance를 별도 파라미터 `diff_lateral_slip: 1.0`으로 분리했습니다. 종방향 미끄럼과 아커만 측면 미끄럼은 기존 `wheel_slip: 0.0`을 유지합니다. 이 파라미터는 물리 모델의 동작을 위한 임시값이며 실차 응답을 보정한 결과는 아닙니다.

수정 후 **23개 검사 통과**: 직진, 좌우 회전, 정지, 입력 중단 정지, 앞바퀴 회전, 조향 관절 부재, 좌우 제자리 회전과 바퀴 반대 회전, 제자리 회전 중 입력 중단 후 정지. 후륜 기준점은 회전 중 이동하므로 휠베이스 중점의 drift를 8 cm 이내로 검사합니다. 기존 아커만도 **15개 회귀 검사 통과**했습니다. 실행 JSON과 궤적은 각 `reports/extended/diff/`, `ackermann/`에 있습니다.

구현 근거: [Gazebo 8 WheelSlip](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WheelSlip.html)의 마찰 방향·측면 미끄럼 정의, [공식 4륜 skid-steer 예제](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive_skid.sdf). 예제의 수치를 실차 측정값으로 사용하지 않았습니다.

### 깊이 카메라와 LiDAR

기본 스폰에서 정지 상태로 관측한 뒤, 중심 `(2.6, 5.57, 0.5)` m, 크기 `(0.2, 0.8, 1.0)` m의 시험 상자를 추가했습니다. 기대 거리는 설정 파일에서 계산한 센서 좌표와 상자 전면 위치로 정했습니다.

- 깊이 영상: `32FC1`, 320×240, 10 Hz. 중앙 바닥 깊이는 약 1.110 m, 상자 추가 후 약 0.743 m로 변했습니다. 카메라 pitch를 포함한 기대 상자 깊이는 `0.72/cos(0.25)` m입니다.
- LiDAR: 360 sample, −π~π, 0.1~12 m, 10 Hz. 전방 상자 거리 약 0.802 m로 기대값 0.8 m와 일치했습니다. ±6 cm 허용 오차를 사용합니다.
- 기본 스폰 전방 ±20°에서는 평면 도색에 의한 장애물 반환점이 없었습니다. 이는 시험 구간의 결과이며 모든 위치·각도에 대한 전수 검사는 아닙니다.
- depth Image/CameraInfo의 촬영 시각·해상도·내부 파라미터, depth optical frame·laser_link의 TF, 촬영 시각 증가를 검사했습니다.

총 **14개 검사 통과**. `reports/extended/sensors/sensors.json`, `baseline_*`, `target_*`에 원시 depth 배열(NPZ), 거리 배열(JSON), RGB·깊이 미리보기, 시험 상자 SDF를 보존했습니다. 깊이 미리보기는 0~5 m를 0~255로 선형 표시하며 무효 픽셀은 검정입니다. 센서 사양·노이즈·왜곡·전달 지연의 실차 일치는 여전히 미검증입니다.

### RViz

검증용 depth/LiDAR 활성 설정과 저장소의 기본 `autorace.rviz`를 사용했습니다. 차량 메시, TF 축, 전방 RGB 영상, 시험 상자의 LaserScan 점군과 `Global Status: Ok`를 실제 화면으로 확인했습니다. 장애물 제거 후에도 이전 LaserScan 점들이 남는 현상을 발견해 `Scan`의 `Decay Time`을 0.2초로 설정했습니다. 센서에서 전방 반환점이 0으로 바뀐 것과 화면에서 점군이 사라지는 것을 함께 확인했습니다. 수정 전 화면은 `before_decay_fix.png`에 보존했습니다. 동작 근거는 [RViz 점군 표시 구현](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp)의 decay 처리입니다. 화면 증거는 `reports/extended/rviz/screen.png`(시험 상자 포함)와 `track_screen.png`(상자 제거 후 트랙)입니다. 기본 설정에는 깊이 Image 패널이 없으므로 깊이 영상의 RViz 표시까지 검증했다고 주장하지 않습니다. 깊이 데이터 자체는 위 거리 검사에서 검증했습니다.

빌드와 기존 pytest 5개(CTest wrapper 포함 집계 6개)도 통과했습니다. 검증 도구는 자율주행 알고리즘이 아닌 고정 명령·관측 시험이며 대회 완주나 실차 성능을 보장하지 않습니다.
