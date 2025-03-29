# KAI Driverless car Control part
건국대학교 자작자동차 동아리 'Team K.A.I'에서 2024 대학생 창작 모빌리티 경진대회에 사용했던 제어 파트의 모든 것

![image](https://github.com/user-attachments/assets/82c385fa-c066-47d9-99b1-d67dbfa1670e)


https://github.com/user-attachments/assets/194afa3c-7ac1-49f6-bc8c-7f1969c28a11


https://github.com/user-attachments/assets/85d22046-8056-4a7d-915f-4da4d3968cdf





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

![image](https://github.com/user-attachments/assets/f4c43015-6875-4505-8a3e-4c3368484aeb)


< 종방향 제어 task 코드 >



https://github.com/user-attachments/assets/a4baaec2-f8bf-4fa3-8111-f228f14c4b1e



<종방향 테스트 영상>

- 감속을 위한 브레이크 시스템 구현

![image](https://github.com/user-attachments/assets/afd55431-9cab-4241-8fd7-1f92d95f4a00)


<pid 아웃풋 - 브레이크 제어 코드>

![image](https://github.com/user-attachments/assets/4094dbc9-d40b-4ee6-8cb3-82094fa66fe3)


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

![image](https://github.com/user-attachments/assets/6f9ee93a-6c11-46db-866a-d50e7e3679ed)


<커스텀 메세지 형식을 제어기와 각 액추에이터에 전달하는 통합 통신 브릿지 노드>

![image](https://github.com/user-attachments/assets/cc6cd8b8-1277-4ff0-8369-8ea21305c707)


<원하는 제어 토픽값을 비트 연산을 통해 적절한 명령어 형식으로 변환하는 코드>

![image](https://github.com/user-attachments/assets/159396dc-18f3-4342-a12e-d0bf0922ee43)


<CAN 통신에서 하트비트 신호를 수신하여 현재 각도 정보, 전류 정보, 모터 활성화 정보와 같은 데이터를 처리하는 코드>

<빅 엔디안 방식이기에 데이터를 뒤집고 이를 실제 각도로 변환하는 내용 포함>

![image](https://github.com/user-attachments/assets/226c09d0-91b1-4450-a4d7-d3e2438e4044)

<목표 스티어링 값을 CAN 신호로 변환하여 제어 토픽을 전송하는 모습>

![image](https://github.com/user-attachments/assets/1fb23a87-64fe-4d60-8587-cacf73edb8cc)


<스티어링 모터 PID 튜닝>


### ✅ ASMS 및 E-Stop 안전 시스템 구현

- 자율주행 모드와 수동 주행 모드 간의 전환 시스템 및 Emergency Stop 기능 구현
- 스위치 핸들링과 디바운싱 처리를 통해 안정적인 모드 전환 구현
- ROS기반으로 E-Stop 활성화 상태 및 현재 모드를 실시간 publish
- 비상 정지 신호 수신 시 모터 출력 차단 및 브레이크 최대 작동 ( 리모컨 및 물리적 버튼을 활용한 다중 입력 방식 구현 )

### ✅ 노이즈 방지 및 신호 안정화 기법

- RC 회로를 이용한 하드웨어 디바운싱

![image](https://github.com/user-attachments/assets/cab8fde2-8375-4989-9f69-da67aa0bc9f0)


<RC 회로를 이용한 하드웨어 디바운싱>

- 소프트웨어 디바운싱 구현

![image](https://github.com/user-attachments/assets/cbc4f778-ee02-4172-b31f-9fa7a4f16bd4)


<소프트웨어 디바운싱 코드>

- 포토커플러 기반 신호 절연

![image](https://github.com/user-attachments/assets/2574ff12-fad9-4496-8400-bc898fecc766)


<포토커플러 회로도>

- 사용하지 않는 GPIO 핀의 풀다운 모드 지정

![image](https://github.com/user-attachments/assets/25ff5ad2-7785-4755-a7ba-9ba901a00a6e)


<풀다운 처리를 통한 노이즈 방지 코드>

### ✅ 제어 및 테스트 성능 평가

- ROSbag 데이터 + GPS 기반 측정값을 비교하며 PID 제어 알고리즘 성능 테스트 → 상승 시간 2초, 오버슈트 5% 미만으로 제어 성능 최적화
- 속도 및 조향 데이터를 디스플레이로 시각화하여 피드백 구현 및 속도 간 오차 데이터를 활용한 동적 튜닝

![image](https://github.com/user-attachments/assets/02a0b084-8215-4425-8e02-72acecd8e52f)


<스텝입력을 넣었을 때 관측된 그래프>

![image](https://github.com/user-attachments/assets/b5dc5db6-30af-4e5c-bcdb-16f85383924f)


<Sine wave 입력에 대한 반응 그래프>

![image](https://github.com/user-attachments/assets/9b3f2e8e-7318-492c-98e5-56bf81af7ae4)


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



