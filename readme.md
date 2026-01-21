# HAMR 시뮬레이터 (v1.1.0 → ETRI 업데이트)

## 패키지 구성
- description/urdf/testbot.urdf.xacro : 로봇 모델 (URDF, Xacro 기반)
- worlds/samhyun/samhyun.sdf : samhyun 시뮬레이션 월드 파일
- data/maps/samhyun_map.yaml : 2D 네비게이션용 지도 (Occupancy Grid Map)
- launch/hamr30_simulator_launch.py : GZ Sim 실행 및 로봇 스폰
- launch/hamr30_nav_sim_launch.py : Navigation2 실행
- parma/nav2_params.yaml : Nav2 파라미터 설정 파일

## 주요 업데이트
1. testbot.urdf.xacro 적용  
   - Jazzy 기반 gz Sim 플러그인 적용 되어있으므로, ignition 버전으로 변경 필요
   - gzsim bridge 파라미터 또한 ignition 버전으로 수정 필요
2. samhyun_map.sdf 적용
3. 새로운 노드 파일 생성
4. nav 파라미터 조정  
   - 최대 선속도 2.0 m/s  
   - 최대 각속도 1.0 rad/s
5. 에러 확인 및 업데이트  
   - 진행 노드리스트 및 전체 경로 노드리스트 # 한번 더 확인 
   - 로봇의 진행 방향 (앞/뒤) 위치가 맞지 않음  # 확인 필요
   - 로봇의 상태 관련 : 로봇에 명령을 주면 잠깐 주행 상태로 바뀌었다 다시 idle로 변함. # 확인 필요

## 실행 방법
1. 시뮬레이터 실행
```bash
ros2 launch hamr30_sim hamr30_simulator_launch.py
ros2 launch hamr30_sim hamr30_nav_sim_launch.py
```

20251201.ver 시뮬레이터 실행

```bash
(GZ Sim On)
ros2 launch amr_sim amr_simulator_launch.py 

(Localization)
ros2 launch amr_sim amr_hdl_localization_simulation.launch.py

(Bringup for Merge, etc..)
ros2 launch amr_sim amr_bringup_launch.py
```

## 실행 방법(robot localization)
```bash
ros2 launch amr_sim amr_simulator_launch.py
------------------------------------------------------------------
ekf_node bcrbot only
ros2 run robot_localization ekf_node --ros-args --params-file /root/ros2_ws/src/amr_sim/param/bcr_bot/ekf.yaml
------------------------------------------------------------------
ros2 launch amr_sim amr_bringup_launch.py
ros2 launch amr_sim rviz_launch.py
```