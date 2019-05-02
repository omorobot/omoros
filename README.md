# OMOROS

오모로봇 제품군의 ROS 지원 드라이버입니다.
이 드라이버를 사용하여 자율주행, 원격 주행에 필요한 엔코더, Odometry정보를 가져오고 주행 명령을 내릴 수 있습니다.
지원 모델: 

[R1](https://www.omorobot.com/omo-r1)

[R1-mini](https://www.omorobot.com/omo-r1-mini)

## 1. 설치방법

### 1.1 ROS-Kinetic on Ubuntu 16.04 PC

ROS패키지들이 설치되어있는 ros_catkin_ws/src 에서 git clone하여 소스를 복사하면 됩니다.

```
$ cd to catkin_ws/src
$ git clone https://github.com/omorobot/omoros.git
$ cd to catkin_ws
$ catkin_make
```
### 1.2 ROS-Kinetic on Raspbian 

로봇을 라즈베리파이에 연결하여 구동하는 경우 아래 설치 과정을 따라 설치 후 진행하시기 바랍니다.
tf2 등의 라이브러리가 필요하므로 Desktop 버전으로 설치하는 것을 권장합니다.

[Installing ROS Kinetic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

** 이미지 파일로부터 Raspberry Pi 3 적용**

위 설치과정이 번거롭다면 Raspberry Pi 3를 위해 만들어진 전체 이미지를 다운로드 하고 etcher와 같은 이미지 쓰기 도구로 microSD에 구워서 사용하여 설치합니다.

[Raspian stretch image with ROS-kinetic installed](https://drive.google.com/open?id=1jAGlkIUAB_SLq0WCe1G4SktzwUm7abHW)

처음 microSD로 부팅한 후에는 터미널에서 raspi-config 를 실행하여 파일시스템을 SD카드 전체로 확장합니다.

<div align="center">
<img src="https://geek-university.com/wp-content/images/raspberry-pi/expand_filesystem_raspbian.jpg?x66712">
</div>

### 1.1 Dependency

드라이버를 구동하기 위해서는 기본적으로 다음과 같은 패키지들이 필요합니다.

자세한 설치 방법은 아래 링크를 참조하세요.

Joy: [ROS JOY](http://wiki.ros.org/joy)

tf: [ROS TF](http://wiki.ros.org/tf)

## 2. How to use

### 2.1 Launching Driver Node

roscore가 실행중인 상태에서 joy 노드를 실행합니다.
```
$ rosrun joy joy_node
```

R1 및 R1-mini 버전에 따라 아래와 같이 입력하여 OMOROS 드라이버를 실행합니다. 

**주의** 실행 전에 PC의 Serial port와 로봇의 연결상태를 확인하세요.

시리얼 포트 관련 문제는 [다음](#serial)을 참조 바랍니다.

R1 로봇은 아래 명령으로 실행합니다.
```
$ rosrun omoros driver_r1.py r1
```

R1-mini 버전은 다음 명령어로 실행합니다.
```
$ rosrun omoros driver_r1.py mini
```

### 2.1 Messages
이 드라이버는 다음과 같은 메세지들을 Publish 혹은 Subscribe 합니다.
```
$ rostopic list
/R1Command
/diagnostics
/joint_state
/joy
/joy/set_feedback
/motor/encoder/left
/motor/encoder/right
/motor/status
/odom
/tf
```

**Subscribed message**

* joy 
  - Axis: Joystick의 스틱 입력을 받아 좌/우 바퀴의 회전 속도를 제어합니다.
  - BUttons: 1번 버튼(A) 입력으로 조이스틱 조종 및 자동주행을 선택합니다.

* R1Command 
  - mode : 제어 방식을 선택합니다. 0은 속도와 회전속도(Vl, Vr) 1은 좌/우 바퀴 속도(WheelL, WheelR)로 제어하는 방식입니다.
  - Vl, Vr : 로봇의 속도와 회전속도를 설정합니다. 단위는 각각 mm/s, mrad/s 입니다.
  - WheelL, WheelR: 바퀴의 속도를 설정합니다. 단위는 mm/s 입니다.

**Publish message**

* motor/encoder/left or right: 모터 엔코더의 누적된 카운트를 출력합니다.

* motor/status 
   - 좌/우 모터의 엔코더, RPM, ODO, speed(mm/s)값을 전송합니다.
<div align="center">
  <img src="images/topic_motor_status.png">
</div>

* odom: Navigation에 필요한 속도/회전속도 등을 전송합니다.
<div align="center">
  <img src="images/topic_odom.png">
</div>



## 3. Trouble shooting

### 3.1 <a name="serial"> Serial Port Error </a>
* 퍼미션 오류: Add user dialout

아래와 같은 메세지가 뜨면서 시리얼 포트를 열 수 없는 경우

[Errno 13] Permission denied: '/dev/ttyUSB0'
사용자 그룹에 dialout을 추가합니다.

'''
$ sudo gpasswd -a UserName dialout
'''

로그아웃 후 재 로그인을 하면 정상적으로 실행됩니다.

* 시리얼 포트를 열 수 없는 경우

시리얼 포트의 경로를 확인합니다.

driver_r1.py를 열고 다음 코드를 수정합니다.

```
ser = serial.Serial('/dev/ttyUSB0', 115200)
```

Raspberry PI의 내장 시리얼포트를 사용하기 위해서는 경로를 아래와같이 설정합니다.

'/dev/ttyS0'



Copyright (c) 2019, OMOROBOT Inc.,

