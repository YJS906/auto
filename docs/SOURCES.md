# 원본·버전·라이선스

## 차량

- 저장소: https://github.com/WeGo-Robotics/limo_pro
- 고정 커밋: `7ef355cd515e806b26f498d38bfe2abe4f3a9728`
- 확인 경로: `limo_ros2/limo_description/urdf/limo_ackerman.xacro`, `limo_four_diff.xacro`, `limo_steering_hinge.xacro`, `limo_ackerman.gazebo`, `limo_xacro.xacro`, `meshes/`, `limo_ros2/limo_base/src/limo_driver.cpp`, `include/limo_base/limo_driver.h`.
- 메시 사용: `limo_base.dae`, `limo_wheel.dae`. 저장소를 별도 vendor dependency로 받아 CMake install에서 참조합니다.
- **라이선스 미확인**: 해당 `limo_description/package.xml`은 `TODO: License declaration`이고 해당 패키지에 유효한 LICENSE가 없습니다. 저장소의 다른 하위 패키지 LICENSE를 이 메시와 Xacro의 허가로 간주하지 않습니다. 로컬 구현에서 출처를 보존했으며 재배포 라이선스는 원저작자 확인이 필요합니다. 이 워크스페이스의 신규 코드 라이선스는 upstream 자산에 적용되지 않습니다.

## Gazebo·ROS

- 환경 선택 근거: https://gazebosim.org/docs/harmonic/ros_installation/ — Jazzy와 Harmonic 권장 조합.
- 구동 API: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html
- 구동 구현 확인: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/ackermann_steering/AckermannSteering.cc
- ros_gz bridge: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge
- Gazebo Sim / ros_gz: Apache-2.0. 실제 설치 Gazebo Sim 8.15.0, ros_gz 1.0.24.
- EzDXF 1.4.5: MIT, Shapely 2.2.0: BSD-3-Clause. 트랙 생성시에만 사용.

## 사용자 자료와 새 코드

- 제공 도면과 사진은 사용자 자료이며 별도 재배포 허가를 추정하지 않습니다. DWG는 변환하지 않고 원본 보존했습니다.
- 이 작업에서 신규 작성한 시뮬레이터 연결·생성·검증 코드는 루트 `LICENSE`의 Apache-2.0을 적용합니다. 공식 자산과 사용자 자료는 제외합니다.
- AI 생성 이미지로 트랙을 만들지 않았습니다. 도면 형상을 직접 샘플링하고 텍스처로 래스터화했습니다.
