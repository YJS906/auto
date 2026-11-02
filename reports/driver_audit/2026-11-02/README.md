LIMO Pro ROS 2 드라이버 평가 — 2026-11-02
================================================

평가 대상은 WEGO 저장소 커밋 `7ef355cd515e806b26f498d38bfe2abe4f3a9728`의 `limo_base`이다. 로컬 파일, 로컬 Git 객체, GitHub raw 다운로드의 바이트를 직접 비교했다. 세 파일은 동일하며 `limo_driver.cpp` SHA-256은 `1dd98fca8500a65f5ff7e3ef342c0331ced620a817dda923f752d49d060ec979`, 크기는 24,661 bytes이다. [동일성 증거](identity.json)

**평가 결론:** 이 드라이버는 작은 규모의 통신·ROS 연결 코드이지만, 시스템 식별 데이터 수집과 피드백 제어에 그대로 사용하기에는 시간·초기화·수치 처리·종료 관련 결함이 있다. 정상 입력의 계산량은 작다. 우선순위는 계산 수식 최적화보다 정확한 상태·시간을 보존하고 오류 시 반복 실행을 막는 것이다. 제조사 하위 보드의 실제 동작이나 설치된 실차 드라이버 버전은 이번에 확인하지 않았다.

**시험 범위와 방법.** Intel Core i5-1135G7, Ubuntu 24.04, ROS 2 Jazzy, VTune 2026.4에서 실행했다. 실차·센서 하드웨어는 연결하지 않았다. 원본 드라이버 실행파일을 Linux PTY에 연결하고 정상·손상·단절 프레임을 주입했다. ROS 도메인 184와 `/dev/shm/ttyLimoAudit_*` 임시 링크를 사용했다. 정지·비정상 명령도 이 격리된 가상 차량에만 전송했다. `probe.py`는 도메인 잠금으로 동시 실행을 금지한다.

원본은 Jazzy에서 `tf2_geometry_msgs/tf2_geometry_msgs.h`가 없어 그대로 빌드되지 않았다. `/tmp`의 호환 헤더가 새 `.hpp`를 include하도록 하여 원본 CPP·헤더를 수정하지 않고 빌드했다. 일반 빌드는 `RelWithDebInfo`; 수치 검사 빌드는 `-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all`이다. 런타임 검사기가 없는 일반 실행에서 UB가 반드시 즉시 크래시한다는 뜻은 아니다.

VTune은 `hotspots`, `sampling-mode=sw`, `enable-characterization-insights=false`로 실행했다. 별도로 `/proc/PID/stat`의 user+system CPU 시간과 RSS, `perf stat`, `strace -f -c`를 사용했다. `/usr/bin/perf` 래퍼는 현재 커널용 바이너리가 없어 실패했지만 설치된 `/usr/lib/linux-tools/7.0.0-38-generic/perf`는 동작했다. perf 하드웨어 카운터는 사용자 영역 `:u` 결과이며 context-switches=0을 실제 문맥 전환이 없었다고 해석하면 안 된다.

테스트 주파수는 **임의로 정한 합성 부하**이다. 50Hz마다 acceleration/gyro/Euler/motion 프레임을 각각 보내고, system 프레임은 10Hz로 보낸다. 14-byte 프레임 210개/s, 2,940 bytes/s이다. 500Hz 시험은 각 센서 500Hz·system 100Hz의 10배 부하다. 실제 펌웨어 발행률이나 UART 지터를 재현한 것은 아니다. PTY는 460800 baud의 물리적 바이트 전송 지연을 재현하지 않는다.

**우선 수정할 결함과 근거**

| 번호 | 우선순위·확인 수준 | 위치 | 문제·영향·최소 개선 |
|---|---|---|---|
| F01 | P1, UBSan 재현 | `limo_driver.cpp:429`, `:392`, `:389` | 아커만 정지 `(0,0)`에서 `r=0/0` → NaN 조향값 → `int16_t` 변환 UB. 외부 NaN/Inf도 정수 변환까지 통과한다. 유한성·범위 검사와 정지/직진 분기를 먼저 처리하고, 프로토콜 정수 범위와 실차 허용 범위를 각각 제한해야 한다. |
| F02 | P1, 정적 확정 + 동작 재현 | `limo_driver.h:104`, `:124`; `limo_driver.cpp:485` | 첫 IMU 처리의 지역변수 `present_theta_`, `last_theta_`가 멤버를 가린다. `last_theta_`, `real_theta_`, `ImuData` 등이 미초기화 상태에서 읽힐 수 있다. 명시적 초기값·최초 유효 표본 플래그가 필요하다. 이번 실행의 우연한 0 초기 상태는 언어상 보장이 아니다. |
| F03 | P1, 재현 | `limo_driver.cpp:495` | 매 표본 yaw 차이가 0.1도 미만이면 버리고 마지막 yaw는 갱신한다. 0.05도씩 300개를 보내면 IMU는 최종 14.95도이지만 `/odom` yaw는 0도였다. 50Hz 기준 약 5도/s 미만 회전이 사라질 수 있다. 표본별 데드밴드를 제거하고 필요하면 명시적 잡음 모델/필터로 처리한다. |
| F04 | P1, 재현 | `limo_driver.cpp:627` | 초 단위 `double stamp`를 나노초 생성자 `rclcpp::Time(stamp)`에 전달한다. 실제 약 1,793,550,419초의 수신 시각이 메시지에서는 약 1.793550419초가 되었다. 정수 나노초를 저장·전달하는 방식으로 통일한다. |
| F05 | P1, SIGABRT 재현 | `limo_driver.cpp:90`, `:125`, `:143`; `serial_port.cpp:67` | 소멸자가 비어 있고 joinable 수신 스레드를 join/detach하지 않아 SIGINT 종료 시 `terminate called without an active exception`, SIGABRT가 발생했다. `keep_running_`도 수신 루프에서 읽지 않는다. 종료 플래그 + 유한 시간 poll + join + port close 순서가 필요하다. blocking read 상태에서 단순 join만 추가하면 종료가 멈출 수 있다. |
| F06 | P1, VTune·CPU 재현 | `limo_driver.cpp:145` | EOF/오류를 구별하지 않고 `len<1`이면 즉시 continue한다. PTY 단절 후 코어 하나 약 99.8% 사용. EINTR 재시도와 EOF/EIO 분리, 연결 상실 상태 발행, 중단 또는 제한된 재연결 대기가 필요하다. 고속 반복 재시도는 삭제한다. |
| F07 | P1, 정적 확정; race detector 미실행 | `limo_driver.h:125`; `limo_driver.cpp:261`, `:423` | `motion_mode_`는 초기화되지 않고 수신 스레드가 쓰며 ROS callback 스레드가 읽는다. 최초 상태 수신 전 잘못된 분기와 C++ data race 가능. UNKNOWN 초기화, 지원 모드 확인, atomic 또는 일관된 동기화가 필요하다. 1-byte 필드라는 이유로 C++ race가 해결되지는 않는다. |
| F08 | P1, 재현 | `limo_driver.cpp:304`, `:311`, `:321`, `:471` | accel/gyro/Euler가 별도 프레임인데 Euler 도착 시 이전 accel·gyro를 묶어 최신 timestamp로 발행한다. gyro/accel 전송을 끊어도 `/imu`는 계속 최신 stamp로 발행됐다. 필드별 수신시각·유효성·age를 보존하고 허용 시차 또는 개별 raw 출력을 둔다. 오래된 값을 새 측정으로 위장하면 시간지연 식별이 왜곡된다. |
| F09 | P1/P2, 수치 재현; 물리 기준점 확인 필요 | `limo_driver.cpp:548` | 같은 조향각에서 전진과 후진을 비교하면 `vx,wz`는 반전되지만 `vy`는 모두 +0.0362576m/s였다. `v<0`일 때 `sin(-central_angle)`로 바꾸는 분기 때문이다. 부호 있는 속도에 맞는 운동학과 기준점을 먼저 확정하고 고친다. 재현된 부호 비대칭과 실제 센서 속도 정의의 정확성은 별개다. |
| F10 | P1/P2, 전송 관측 | `limo_driver.cpp:70`, `:385` | 명령 유효기간·timeout·송신 실패 처리가 없다. 한 번의 전진 명령 후 약 3초 동안 새 명령이 없어도 드라이버는 정지 프레임을 보내지 않았다. 드라이버 또는 명시적으로 연결된 guard가 유효기간을 관리해야 한다. 하위 보드 watchdog의 유무·정지 시간은 실차 시험 대상이며, 이번 결과로 차량이 계속 움직인다고 단정하지 않는다. |
| F11 | P2, 정적 확정 | `serial_port.h:58`; `limo_driver.cpp:419` | write 반환값을 무시한다. short write/EINTR/EIO가 발생하면 실제 전송된 명령을 알 수 없다. 14 bytes 전체 송신 확인과 중단 가능한 write 루프, 오류 진단, 요청값·인코딩값·송신 결과 기록이 필요하다. write 성공은 펌웨어 수락 ACK와 다르다. |
| F12 | P2, 정적 확정·합성 입력 재현 | `limo_driver.cpp:483`, `:505`, `:613` | roll/pitch를 수신하고도 quaternion은 `(0,0,yaw)`로 만든다. roll=10도/pitch=5도 입력에서 qx=qy=0이었다. 자세 공분산은 1e-6으로 확신하고 twist 공분산은 기본 0이다. 제공하지 않은 자세를 정확한 측정으로 해석하게 만들 수 있다. 축·단위 검증 후 전체 자세 사용 또는 미제공/미상 불확실성의 표준 표현이 필요하다. |
| F13 | P2, 정적 확정 | `limo_driver.cpp:170`, `:524`, `:580`, `:597` | 적분에는 프레임 헤더 수신시각, odom/TF 출력에는 별도로 읽은 now를 사용한다. status/IMU/odom 시간 정책도 다르다. `last_stamp`가 함수 static이며 dt의 음수·큰 공백 검사도 없다. 수신 timestamp를 일관되게 유지하고 적분 상태를 인스턴스 멤버로 둔다. 장치 capture timestamp가 없으면 host receive time임을 표시해야 한다. |
| F14 | P2, 교정 제한 | `limo_driver.h:127`; `limo_driver.cpp:445` | L=0.2, track=0.172, 좌우 scale=2.47이 컴파일 상수다. 양쪽 송신 분기 모두 `right_angle_scale_`을 쓴다. 현재 좌우 scale이 같아 현 설정에서 수치 차이는 없으나 좌우 교정 시 불일치한다. 실측 가능한 ROS 파라미터와 좌우 역변환 일치를 확인한다. 모델 상수로 만든 odom을 그대로 교정 정답으로 쓰지 않는다. |
| F15 | P2, 정적 확정 | `limo_driver.cpp:79`, `:83`, `:369` | 시작 시 `/dev/`를 무조건 붙이고 `tty` 부분문자로 포트 사용을 결정한다. 또한 관측만 하려는 실행에서도 command-mode 설정 프레임을 보낸다. 절대 경로를 그대로 받고 수동 조종 데이터 수집에는 송신하지 않는 관측 옵션을 둔다. 설정 프레임과 물리 SWB의 우선권은 실차/펌웨어 확인이 필요하다. |
| F16 | P2/P3, 정적 확정 | `limo_driver.cpp:156`, `:208`, `:221`; `serial_port.h:43`, `:63` | 파서 static 상태가 인스턴스 간 공유되고 부분 프레임 timeout/sequence gap 진단이 없다. checksum 실패마다 WARN을 출력한다. fd는 생성 시 초기화하지 않고 close 후 -1로 바꾸지 않는다. 파서 상태를 멤버로, 오류 로그는 throttle/카운터로, fd는 -1로 초기화·close 뒤 무효화한다. 단일 장치에서 모든 잠재 문제가 발생했다는 의미는 아니다. |
| F17 | P2/P3, 빌드 실패 재현 | `limo_driver.h:45`; `package.xml:11`; `CMakeLists.txt:33` | Jazzy 헤더 호환 문제가 있고 package.xml의 sensor_msgs/geometry_msgs/tf2_geometry_msgs 의존성 선언이 빠져 있다. CMake의 DEPENDENCIES 목록은 사용되지 않는다. 선언과 target dependencies를 일치시키고 사용하는 배포판에서 별도 빌드를 검증한다. |
| F18 | P3, 정적 확정 | `limo_driver.cpp:44`, `:66`; `launch/limo_base.launch.py:22`; `limo_base_node.cpp:51` | control_rate는 사용되지 않는다. launch의 odom_topic_name도 실제 topic 설정에 연결되지 않고 절대 topic 이름 때문에 namespace가 적용되지 않는다. spin 뒤 Rate(100).sleep()은 제어 주기를 만들지 않는다. 상대 topic+ROS remap 등 기존 기능을 쓰고 실제로 작동하지 않는 설정은 삭제하거나 연결한다. |

위치 링크의 공통 기준은 [원본 driver](/home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base/src/limo_driver.cpp), [driver header](/home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base/include/limo_base/limo_driver.h), [serial header](/home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base/include/limo_base/serial_port.h), [serial implementation](/home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base/src/serial_port.cpp)이다. P1은 제어·관측 신뢰성을 우선 복구할 항목, P2는 데이터 품질/견고성, P3는 정리·사용성이다. 하드웨어 동작을 증명하지 않은 항목을 P0로 단정하지 않았다.

**동적 재현 증거.** [정지 UB](ubsan_stop.log), [NaN/Inf UB](ubsan_invalid.log), [정상 입력과 종료](plain_50.json), [단절 CPU](disconnect_plain.json), [후진 부호](reverse.json), [명령 중단](watchdog.json), [오래된 IMU 혼합](mixed_imu.json), [checksum 거부](checksum.json). checksum 손상 프레임은 정상적으로 거부되어 ROS 출력이 0개였다. 프로토콜을 임의로 추측해 수정한 결과가 아니다.

종료코드는 일반 직접 실행에서 -6(SIGABRT), perf 래퍼에서 134, VTune에서 4였다. 이는 이 시험에서 원본의 종료 결함을 재현한 결과이며, **정상 종료 검사가 통과한 것으로 처리하지 않았다.** VTune 원시 결과와 CSV는 생성됐다. UBSan 실행은 예상대로 오류 지점에서 종료코드 1로 중단했다.

**VTune과 성능 해석.** 정상 수신 VTune 결과는 elapsed 8.146922초, sampled CPU 0.320000초였다. 약 32개의 10ms CPU 표본 수준으로, 부팅·동적 로딩·프로파일러 관련 함수가 섞인다. 이 결과만으로 `operator new`가 실차 드라이버의 최우선 병목이라고 결론내릴 수 없다. 드라이버 계산이 정상 부하에서 작다는 관측에 가깝다.

단절 결과는 elapsed 5.893552초, sampled CPU 4.190000초였으며 `__GI___libc_read` 2.469945초(58.9%), 드라이버에 귀속된 read wrapper 1.120094초(26.7%)였다. 두 hotspot은 exclusive CPU 표본 기준 합계 약 85.7%다. 별도 비-VTune 실행의 단절 후 정상상태 CPU는 코어 하나 기준 약 99.8%였다. 이는 I/O 실패 busy loop라는 코드 분석과 일치한다.

원본의 정상 입력에서는 RSS가 대략 31MiB였다. VTune 실행에서는 profiler 주입 비용 때문에 약 275MiB로 커졌으므로 실차 기본 메모리 사용량으로 해석하면 안 된다. DDS 등을 포함한 프로세스 스레드는 VTune 기준 15개였고, 드라이버가 직접 만든 수신 스레드는 1개다.

**연산 효율 후보 비교.** `/tmp`의 별도 복사본에서만 `read(fd,buf,1)`을 최대 256-byte read로 바꾸고, 기존 byte 파서에 순서대로 넣었다. VMIN=1을 유지해 256 bytes가 찰 때까지 기다리지 않게 했다. 파서·운동학·ROS 출력은 바꾸지 않았다. 원본 vendor 파일은 변경하지 않았다. [정확한 후보 diff](batch_read_candidate.patch)

| 합성 입력 | 원본 CPU 중앙값 | 묶음 읽기 CPU 중앙값 | 상대 감소 | 합성 입력→IMU 수신 p95, 원본/후보 |
|---|---:|---:|---:|---:|
| 약 50Hz | 1.35% | 1.12% | 16.7% | 2.06/1.85ms |
| 약 500Hz | 8.92% | 7.17% | 19.6% | 2.19/2.16ms |

CPU는 코어 하나=100%이고 각 조건 6초 시험 3회의 중앙값이다. 각 시험의 입력 시작 후 첫 1.5초를 제외한 `/proc` CPU 변화량을 계산했다. 10ms tick 해상도와 짧은 측정 구간 때문에 소수점 두 자리의 정확도를 보증하지 않는다. 50Hz 절감은 약 0.23%p에 불과하며, 500Hz의 상대 감소도 이 합성 환경에 한정된다. p95는 각 시험 p95의 중앙값으로, 부트 이후 호스트 PTY write에서 Python IMU subscriber callback까지의 지연이다. 장치 센서 capture-to-control 지연이 아니다. 지연 개선이나 hard real-time 보장을 주장하지 않는다.

시간 제한 feeder가 목표 3000회 대신 2997~2998회를 보낸 시험이 있다. 이는 실제 write byte 수로 확인했고, 수신 손실과 구분했다. 최종 12회 비교는 포트 open 이후 입력을 시작하고 수신을 drain했으며, **실제 주입한 모든 motion/IMU/system 프레임에 대응하는 출력 수가 일치했다.** [수치·반복별 범위](performance_summary.json), [근거 검사](evidence_checks.json). 초기 탐색 실행(`plain_*`, `batch_*`, `stress_*`)은 별도로 보존했고 최종 성능 집계는 `ready_*` 실행만 사용한다.

strace의 프로세스 전체 read 호출 수는 같은 6초·17,640-byte 입력에서 22,795 → 5,455로 약 76.1% 감소했다. 이 수치에는 DDS·동적 로딩 등의 read도 포함되며, 모두 UART read라고 주장하지 않는다. strace 실행시간의 futex 비율은 blocking wait와 계측 비용을 포함하므로 CPU 병목 비율로 읽으면 안 된다.

PTY는 프레임을 묶어 도착시키므로 실 UART보다 batching에 유리할 수 있다. 본 결과는 Intel 호스트의 합성 시험이며 Jetson ARM이나 실제 센서·bag 저장 동시 실행의 개선율로 전이할 수 없다. 정상 50Hz에서 절약하는 CPU의 절대량은 작다. 삼각함수 근사, GPU화, lock-free queue, 별도 프레임워크 도입을 정당화하는 결과는 없다. 수신 오류 처리와 시간·상태 정확성이 먼저다.

**유용하지만 보존되지 않는 정보.**

| 정보 | 현재 상황 | 실용적 활용과 주의점 |
|---|---|---|
| 보드 보고 조향각 | 0x221에서 수신, 2.47 교정 후 odom 계산에만 사용 | raw 값과 교정값을 함께 기록하면 좌우 비대칭·조향 응답·데드존 비교에 유용. 실제 각도 센서값인지 목표값 echo인지 이번 시험에서는 알 수 없다. |
| 보드 보고 요레이트·횡속도 | 수신하지만 아커만 odom에서는 모델 계산값으로 대체 | IMU yaw rate와 모델값을 비교할 자료. 서로 독립된 측정이라고 미리 가정하면 안 된다. |
| 좌우 누적 주행량 | 0x311에서 int32로 읽은 뒤 버림 | 단위·부호·wrap 규칙 확인 후 속도/누적 이동·슬립 비교에 활용. 이 값만으로 개별 4개 휠 엔코더가 있다고 말할 수 없다. |
| IMU roll/pitch | 0x323에서 수신하지만 자세 출력에 미사용 | 카메라 IPM의 평면 가정 검사와 기울기 관측에 유용. 축·보정·유효성을 먼저 확인. |
| 액추에이터 HS/LS 상태 | 0x251~254, 0x261~264 ID만 정의, switch에서 즉시 break | 펌웨어가 실제로 보내는지와 payload 명세부터 확인. 이름만 보고 RPM·전류·온도를 특정 byte에 배정하면 안 된다. |
| count와 수신 품질 | count byte 저장 후 미사용, 오류/간격 통계 없음 | protocol 의미 확인 후 gap·duplicate·dropout 진단. host 도착 시각도 저장해 버퍼 지연을 파악. |
| 명령 변환·송신 결과 | /cmd_vel 요청만 외부에서 볼 수 있고 인코딩·write 결과 기록 없음 | 요청 속도/요레이트, 변환 조향값, clipping 여부, 송신 성공·시각을 남겨 식별 입력의 범위를 명시. 펌웨어 수락·실제 실행의 증거는 별도. |
| RC 스틱/목표 명령 | 이 코드와 프로토콜 header에 RC 채널 출력이 없음 | 보드 보고 운동 상태를 RC 입력으로 바꿔 부르면 안 된다. 펌웨어 프로토콜의 노출 여부를 확인하고, 없으면 ROS 명령 모드로 식별하는 것이 명확하다. |
| 배터리·mode·error | 이미 /limo_status에 있음 | timestamp를 고친 뒤 배터리 전압과 운용 모드별로 데이터 구분. 새로운 센서가 필요한 정보는 아니다. |

현재 `/odom`의 yaw는 gyro 적분만으로 만든 것이 아니라 Euler yaw 차이를 이용하고, 아커만 twist도 상수 운동학을 거쳐 재계산된다. 따라서 `/odom`을 독립적인 ground truth로 두고 동일 운동학을 식별하면 순환 검증이 된다. 온보드 카메라·LiDAR의 이동 추정도 편향·드리프트를 가지므로, 독립성·시간 정렬·관측 가능성을 평가하면서 교차 검증해야 한다.

**Ponytail 관점의 최소 변경 순서.**

1. 유한성/정지 분기, 최초 상태 초기화, yaw 데드밴드, timestamp 단위를 수정한다. 작은 재현 검사를 유지한다.
2. 스레드 종료·EOF/EIO 처리·송신 성공 확인·모드 동기화를 복구한다. 하위 보드 watchdog과 RC 우선권은 실차에서 검증한다.
3. 기존 원시 보드 피드백을 소수의 명확한 상태 메시지에 보존하고 validity/수신시각을 넣는다. 펌웨어 센서값과 목표값을 구별한다.
4. 실측 교정 파라미터·기준점을 확정한다. 실차 base_link를 후륜축으로 자동 간주하지 않는다. rigid-body twist는 기준점 변화에 따라 `v_B = v_A + omega × r_AB`로 변하므로 카메라 외부 파라미터와 함께 맞춘다.
5. RX batching은 작은 diff로 적용 가능한 효율 후보지만 실제 UART에서 재측정한다. 더 큰 구조 변경은 필요성이 측정될 때만 한다.
6. ROS1 잔여 주석·미사용 함수/변수·가짜 설정만 정리한다. 패킷 validation이나 오류 처리를 ‘코드가 길다’는 이유로 삭제하지 않는다.

복잡성만 보는 보조 review 결과: `limo_driver.cpp:445 shrink:` 동일한 좌우 분기를 교정 의미에 맞게 정리; `:94 delete:` 빈 run 함수와 선언; `:108 delete:` 미호출 normalizeAngle 또는 실제 각도 처리에서 사용; `limo_base_node.cpp:32 delete:` 미사용 global robot; `:51 delete:` spin 뒤 Rate/sleep; `CMakeLists.txt:33 delete:` 미사용 DEPENDENCIES 목록. 코드만 세는 보수적 정리 가능량은 약 30~40줄이며 실제 patch는 적용하지 않았다. 실행 비용 개선과 주석 삭제를 같은 성과로 합산하지 않았다. 보수적으로 `net: -30 lines possible.`

**재현 방법.** 시스템 패키지나 커널 설정을 바꾸지 않았다. 아래 빌드는 `/tmp`에 생성되며 vendor 소스는 읽기만 한다. 호환 헤더는 이름 연결만 제공한다.

```bash
mkdir -p /tmp/limo-driver-audit/compat/tf2_geometry_msgs
printf '#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>\n' \
  > /tmp/limo-driver-audit/compat/tf2_geometry_msgs/tf2_geometry_msgs.h
source /opt/ros/jazzy/setup.bash
cd /tmp
colcon build \
  --base-paths /home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_msgs \
               /home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base \
  --packages-select limo_msgs limo_base \
  --build-base /tmp/limo-driver-audit/build \
  --install-base /tmp/limo-driver-audit/install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  '-DCMAKE_CXX_FLAGS=-I/tmp/limo-driver-audit/compat -Wshadow'
source /tmp/limo-driver-audit/install/setup.bash
python3 /home/a/2026autorace/reports/driver_audit/2026-11-02/probe.py \
  --case normal --profile perf --duration 6 --tag new_nominal
python3 /home/a/2026autorace/reports/driver_audit/2026-11-02/probe.py \
  --case disconnect --profile vtune --duration 8 --tag new_disconnect
```

UBSan은 별도 build/install 경로와 `-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all` 컴파일 옵션으로 빌드한 뒤 `probe.py --case stop --binary /tmp/limo-driver-audit/ubsan_install/limo_base/lib/limo_base/limo_base`로 재현한다. batch 후보는 `batch_read_candidate.patch`를 `/tmp`의 원본 복사본에만 적용해 별도 빌드했다.

기존 결과의 VTune DB는 [정상 결과](/tmp/limo-driver-audit/vtune_vtune_nominal_50/vtune_vtune_nominal_50.vtune), [단절 결과](/tmp/limo-driver-audit/vtune_vtune_disconnect/vtune_vtune_disconnect.vtune)에 있다. `/tmp`는 영구 보존 경로가 아니므로 장기 공유용 증거는 이 폴더의 [정상 summary](vtune_nominal_summary.csv), [정상 hotspots](vtune_nominal_hotspots.csv), [단절 summary](vtune_disconnect_summary.csv), [단절 hotspots](vtune_disconnect_hotspots.csv)와 JSON/log를 사용한다. VTune CSV 확장자는 `.csv`이지만 기본 출력 구분자는 TAB이다.

참고: [Intel VTune collect 문서](https://www.intel.com/content/www/us/en/docs/vtune-profiler/user-guide/2023-1/collect.html), [rclcpp Time API의 나노초 생성자](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Time.html), [제조사 매뉴얼](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20user%20manual%28EN%29.md). 매뉴얼에 기능 이름이 있다는 사실과 현재 ROS 2 코드가 실제로 해당 값을 발행한다는 사실을 구분했다.
