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
| 차동구동 실제 주행 | 미검증 | 모델 변환만 검사; 물리 반복 시험은 Ackermann으로 수행 |
| 선택 depth/LiDAR | 미검증 | 구현·파라미터 제공, 기본 off. 실기 센서/범위 일치와 런타임 발행 시험 미실시 |
| LiDAR에서 차선 무검출 | 구조상 평면 구현, 실행 미검증 | 수평 LiDAR로 실제 scan을 측정하지 않음. 차선에 벽/collision은 없음 |
| RViz 화면 | 미검증 | 설정 파일 제공, Gazebo GUI와 ROS 데이터는 확인 |
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
