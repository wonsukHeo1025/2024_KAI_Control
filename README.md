# KAI Driverless car Control part
건국대학교 자작자동차 동아리 'Team K.A.I'에서 2024 대학생 창작 모빌리티 경진대회에 사용했던 제어 파트의 모든 것


![image](https://github.com/user-attachments/assets/d0d17866-be94-43c7-be99-e9ae061086d7)



https://github.com/user-attachments/assets/eb7b4ba7-077a-4692-982d-707644006857


https://github.com/user-attachments/assets/cfde363a-4079-4ee5-ae85-68c2481a473a





## 대회 개요
자율주행 차량 설계부터 제작, 시스템 구현까지 통합된  혁신적인 모빌리티 솔루션을 개발


## 각 패키지 설명

### 1. `control_setup`
- **1_open_can_port.sh**: CAN 포트를 활성화하는 스크립트입니다.
- **2_activate_can_bridge.sh**: CAN 브릿지 노드를 실행하는 스크립트입니다.
- **3_activate_rosserial.sh**: ROS Serial을 실행하는 스크립트입니다.
- **사용 방법**: 위 스크립트들은 **이름 순서대로 실행**하면 됩니다.

---

### 2. `integrated_controller`
- **esp32_control/esp32_control.ino**: 종방향 제어기 코드입니다.
- 대회때 사용한 종방향 제어, 모드 전환 스위치,각종 액추에이터 제어 코드.

---

### 3. `ros_kai_ws/src`
#### **can_bridge_pkg**
- **스티어링 모터와 ROS 간 통신**을 위한 CAN 브릿지 패키지입니다.
- 주요 구성:
  - `CMakeLists.txt` 및 `package.xml`: 패키지 설정 파일.
  - `launch/hardware_activate.launch`: CAN 브릿지 실행을 위한 런치 파일.
  - `script/open_can_port.sh`: CAN 포트를 활성화하는 스크립트.
  - `src`: CAN 브릿지 소스 파일들(`can_bridge.cpp`, `can_bridge_estop_deactivate.cpp` 등).

#### **custom_msg_pkg**
- 테스트 및 제어 메시지 관련 패키지입니다.
- 주요 구성:
  - `msg/ControlMsg.msg`: 제어 메시지 정의.
  - `msg/FeedbackMsg.msg`: 피드백 메시지 정의.
  - `scripts/sine_input.py`: 테스트용 사인 입력 스크립트.
  - `scripts/step_input.py`: 테스트용 스텝 입력 스크립트.
  - `scripts/tune_pid.sh`: PID 튜닝 스크립트.
  - `scripts/view_rqt_plot.sh`: RQT 플롯 뷰어 실행 스크립트.

---

## 프로젝트 목적
이 프로젝트는 자율주행 차량의 통신과 제어를 다룹니다.  
- **CAN 브릿지**: 스티어링 모터와 ROS 간의 통신을 중계.
- **ESP32 제어기**: 차량의 종방향 제어, 모드 전환, 속도계 출력 등.
- **테스트 스크립트**: 사인파 및 스텝 입력을 통해 제어기 성능 테스트.

---

### 사용 방법
1. `control_setup` 내부 스크립트를 **순서대로 실행**:
   ```bash
   ./1_open_can_port.sh
   ./2_activate_can_bridge.sh
   ./3_activate_rosserial.sh
   
2. `ROS 노드 실행` 
    ```bash
    roslaunch can_bridge_pkg hardware_activate.launch
    
3. `테스트 스크립트 실행`

    ```bash
    python3 ros_kai_ws/src/custom_msg_pkg/scripts/sine_input.py
    python3 ros_kai_ws/src/custom_msg_pkg/scripts/step_input.py

## 역할 : 제어 파트
- 하드웨어 제어기 설계 및 제어 알고리즘 개발
- 통신 프로토콜 설계 및 구현
- ASMS 및 E-Stop 안전 시스템 구현

## 주요 기능

### ✅ 하드웨어 제어기 설계 및 제어 알고리즘 개발

- 아두이노(ESP32)를 기반으로 듀얼 코어 제어기 설계 및 PID 알고리즘 구현
- 차량 속도 및 방향 제어를 위한 PID 알고리즘 설계 및 튜닝

![image](https://github.com/user-attachments/assets/085dd79c-d229-4ea7-a9a8-a05b332f5d5a)



< 종방향 제어 task 코드 >


- 감속을 위한 브레이크 시스템 구현

![image](https://github.com/user-attachments/assets/d2509ea1-a71d-450f-b6e2-f4468f7276a8)



<pid 아웃풋 - 브레이크 제어 코드>

![image](https://github.com/user-attachments/assets/4411073c-82b7-4bb9-a836-78a07963ced5)



<지수함수로 나타낸 브레이크 함수>

- Cubic Spline 을 활용해 입력값의 비선형성을 보정하여 조향 안정성 향상
- 증분형 엔코더 + ESP32의 하드웨어 펄스 카운터를 활용해 고속 및 저속 환경에서 정밀한 속도 측정 구현

### ✅ 통신 프로토콜 설계 및 구현

- CAN, UART, PWM 통신 방식을 활용한 액추에이터 제어
- ROS 기반 CAN Bridge Node 구현
1. ROS 메시지를 CAN 프레임으로 변환하여 스티어링 모터 제어
2. 특정 CAN ID에 따른 활성화/비활성화 신호 처리 및 실시간 피드백 수신
3. 목표 각도를 -45도에서 45도 범위로 스케일링하여 CAN 메시지로 전송
- ROS publisher-subscriber 구조를 통해 제어값 전송 및 데이터 로깅

![image](https://github.com/user-attachments/assets/92c8a1f6-411f-43d8-9938-da0c358d46da)



<커스텀 메세지 형식을 제어기와 각 액추에이터에 전달하는 통합 통신 브릿지 노드>

![image](https://github.com/user-attachments/assets/d071a63b-c1f5-4f2f-8c7c-f1b40e29f55f)



<원하는 제어 토픽값을 비트 연산을 통해 적절한 명령어 형식으로 변환하는 코드>

![image](https://github.com/user-attachments/assets/c7c4801e-d705-4495-9d7e-b573eb426c83)



<CAN 통신에서 하트비트 신호를 수신하여 현재 각도 정보, 전류 정보, 모터 활성화 정보와 같은 데이터를 처리하는 코드>

<빅 엔디안 방식이기에 데이터를 뒤집고 이를 실제 각도로 변환하는 내용 포함>

![image](https://github.com/user-attachments/assets/49a681bf-f3d5-4a88-a257-2fc3e889d57f)


<목표 스티어링 값을 CAN 신호로 변환하여 제어 토픽을 전송하는 모습>

![image](https://github.com/user-attachments/assets/88d1cd5a-1381-48d1-94df-8d7ded2dbca3)


<스티어링 모터 PID 튜닝>


### ✅ ASMS 및 E-Stop 안전 시스템 구현

- 자율주행 모드와 수동 주행 모드 간의 전환 시스템 및 Emergency Stop 기능 구현
- 스위치 핸들링과 디바운싱 처리를 통해 안정적인 모드 전환 구현
- ROS기반으로 E-Stop 활성화 상태 및 현재 모드를 실시간 publish
- 비상 정지 신호 수신 시 모터 출력 차단 및 브레이크 최대 작동 ( 리모컨 및 물리적 버튼을 활용한 다중 입력 방식 구현 )

### ✅ 노이즈 방지 및 신호 안정화 기법

- RC 회로를 이용한 하드웨어 디바운싱

![image](https://github.com/user-attachments/assets/16901453-8a1c-475c-9528-4b910c146638)



<RC 회로를 이용한 하드웨어 디바운싱>

- 소프트웨어 디바운싱 구현

![image](https://github.com/user-attachments/assets/e412d1ab-1fcf-45db-a218-308453f28402)



<소프트웨어 디바운싱 코드>

- 포토커플러 기반 신호 절연

![image](https://github.com/user-attachments/assets/499d00a0-e23d-412d-9292-6ba038cb8e24)



<포토커플러 회로도>

- 사용하지 않는 GPIO 핀의 풀다운 모드 지정

![image](https://github.com/user-attachments/assets/874b9d8a-78be-4b8e-8471-b611b0181018)



<풀다운 처리를 통한 노이즈 방지 코드>

### ✅ 제어 및 테스트 성능 평가

- ROSbag 데이터 + GPS 기반 측정값을 비교하며 PID 제어 알고리즘 성능 테스트 → 상승 시간 2초, 오버슈트 5% 미만으로 제어 성능 최적화
- 속도 및 조향 데이터를 디스플레이로 시각화하여 피드백 구현 및 속도 간 오차 데이터를 활용한 동적 튜닝

![image](https://github.com/user-attachments/assets/69dd6e6d-f758-48bd-bf05-525b151b5d28)



<스텝입력을 넣었을 때 관측된 그래프>

![image](https://github.com/user-attachments/assets/5b3f48ae-23b8-43e2-b4be-70aa119d15e5)



<Sine wave 입력에 대한 반응 그래프>

![image](https://github.com/user-attachments/assets/ebd53c77-f1da-40a8-8df9-4d85089a2181)



<ROSbag을 통해 성능 테스트 데이터 로깅 및 분석>

## 기술 스택

### ✅ 아두이노 + ESP32

- 제어기 설계 및 신호 처리
- PID 제어 알고리즘 및 E-Stop 시스템 구현
- PWM 제어, 디지털 입력 핸들링, 서보모터 및 모터 제어

### ✅ CAN 통신

- 스티어링 및 액추에이터와의 데이터 통신
- ROS 기반의 CAN 브리지 노드 설계 및 구현 → ROS 메시지와 CAN 프레임 간 데이터 변환
- 스티어링 모터의 상태 수신, 제어 명령 전송

### ✅ ROS

- 자율주행 알고리즘 통합 및 데이터 관리
- 제어 메시지 전송 및 피드백 데이터 수신
- ROSbag을 통해 성능 테스트 데이터 로깅 및 분석

### ✅ 파이썬

- CAN 브리지 노드를 파이썬으로 구현하여 각도 및 속도 데이터를 처리하고 ROS 메시지로 퍼블리시

## 아키텍처 구조

![image](https://github.com/user-attachments/assets/51111ca7-c6f7-4436-b412-340815785d2c)


![image](https://github.com/user-attachments/assets/37adb44a-4f39-4ade-b255-0523ee273684)



