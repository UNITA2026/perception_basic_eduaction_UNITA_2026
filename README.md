# UNITA 2026.01.02 2차 기본교육

본 교육은 본격적인 ROS 2 학습에 앞서 **ROS 2 기반 자율주행 시스템의 전체 구조를 이해**하고,  
시뮬레이션 환경에서 구현 가능한 **간단한 자율주행 실습을 통해 흥미와 동기 부여를 강화**하는 것을 목표로 한다.

특히 자율주행 구현에 활용되는 **인지, 판단, 제어, 시뮬레이션** 등 다양한 기술 요소를  
직접 다뤄봄으로써 이후 심화 학습을 위한 기초 역량을 확보하고자 한다.

---
## 시뮬레이션 영상 (YouTube)

아래 썸네일을 클릭하면 실제 주행 영상을 확인할 수 있습니다.

[![Simulation Video](http://img.youtube.com/vi/2VR7mfooOVU/0.jpg)](https://www.youtube.com/watch?v=2VR7mfooOVU)

---

## 참여 인원 및 일시 & 장소

- **강의자**: 김형진  
- **교육 조교**: 이기현, 이다빈, 윤제호  
- **교육 참여자**:  
  강민수, 김민서, 박근호, 성현영, 윤지윤, 이석빈,  
  이원종, 장동혁, 정가용, 정규민, 한주형  

- **교육 일시**: 2025년 12월 26일 10:00 ~ 17:00  
  (점심시간 제외, 총 6시간)  
- **교육 장소**: 공과대학 8호관 117호  

---

### 0. 팀 구성

- **1팀**: 성현영, 정가용, 이석빈, 이다빈  
- **2팀**: 장동혁, 이원종, 윤제호  
- **3팀**: 윤지윤, 박근호, 정규민, 김형진  
- **4팀**: 한주형, 김민서, 강민수, 이기현  

---

### 1. 개발 환경 세팅 확인

- **운영체제**: Ubuntu 22.04 *(사전 설치 공지)*  
- **ROS 2**: Humble Hawksbill *(사전 설치 공지)*  
- **그래픽 드라이버**: NVIDIA Graphic Driver *(필요 시)*  

#### 실습 코드 링크

- **실습 코드 (실차/기본)**  
  https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course

- **실습 코드 (시뮬레이션 ver.)**  
  https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course-Simulation

```bash
# 본인이 받고자 하는 GitHub 레포지토리 클론
git clone <repository_url>

# 예시
git clone https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course.git
```
두 개의 실습 코드를 모두 홈 디렉터리(~)에 설치한다.

---

### 2. 리눅스 기초 커맨드 강의
linux command(기본 명령어)
```bash
man     # 명령어 매뉴얼 확인
cd      # 디렉터리 이동
pwd     # 현재 경로 출력
ls      # 디렉터리 목록 출력
touch   # 파일 생성
mkdir   # 디렉터리 생성
rm -rf  # 파일 및 폴더 삭제
exit    # 터미널 나가기
clear   # 터미널 깔끔하게 밀기
```

ros2 command(기본 명령어)
```bash
ros2 node list                           # 실행 중인 노드 목록 확인
ros2 topic list                          # 퍼블리시 중인 토픽 목록 확인
ros2 topic echo <topic_name>             # 토픽 데이터 확인
colcon build --symlink-install --packages-select <package_name>       # 선택 패키지 빌드

```

.bashrc 설정 실습
자주 사용하는 source 및 명령어 alias 설정
```bash
cd ~; code ./.bashrc
source /opt/ros/humble/setup.bash
alias 'c'=clear
alias 'e'=exit
alias 'cb'=colcon build --symlink-install
```

rqt tool 사용방법 간단 리뷰.
- **동그란 도형**: 노드(Node)  
  > 실제로 동작하는 프로그램(프로세스)  
- **네모난 도형**: 토픽(Topic)  
  > 노드 간에 데이터를 주고받는 통신 채널

![Node and Topic Example](img/nodeandtopic_example.png)

동아리원들이 아직 ROS에 대해 익숙하지 않기에, ROS2 라는 것이 간단히 말해 노드 간의 통신을 관리하는 미들 웨어라는 것을 시각적으로 이해시키기 위해 설명함

---
### 3. Roboflow · GitHub 계정 생성 및 간단한 사용 방법 안내

**Roboflow**는 데이터 라벨링(Labeling), 데이터 증강(Augmentation),  
유료 플랜 사용 시 모델 학습까지 지원하는 **컴퓨터 비전 올인원 플랫폼**이다.

금번 실습에서는 Roboflow의 다양한 기능 중  
**데이터 라벨링 과정에 초점을 맞추어 실습을 진행**하였다.

- Roboflow 공식 웹사이트: https://roboflow.com/

> 본 실습에서는 객체 인식 기반 자율주행 구현의 첫 단계로,  
> **라벨링 품질이 인지 성능에 미치는 영향**을 직접 확인하는 것을 목표로 한다.

---

또한 팀 단위 협업 및 코드 관리를 위해  
**모든 교육 참여자에게 GitHub 계정 생성을 필수로 안내**하였다.

- GitHub 공식 웹사이트: https://github.com/

> GitHub를 활용하여 코드 공유, 변경 이력 관리, 협업 흐름을 경험하도록 구성하였다.

---

### 4. 팀별 영상 시청 후 코드 리뷰

각 팀은 **3인 단위로 역할을 분담**하여 강의 영상을 시청하고,  
제한된 시간 내 학습 --> 설명 --> 토론 과정을 통해 이해도를 높인다.

#### 진행 방식

1. 팀당 3명이 **서로 다른 영상 범위**를 분담하여 시청
2. **영상 시청 시간: 1시간 제한**
3. 시청 종료 후, 팀 내에서 **총 30분간 내용 공유**
   - 인당 10분씩 설명
4. 설명 이후 코드 구조 및 동작 방식에 대한 토론 진행
5. 교육조교는 옆에서 정상적으로 학습이 진행되는지 확인

#### 영상 분담 및 역할

##### 📷 카메라 인지부 담당자 1
- **영상 번호**: 18 ~ 22  
- **주요 노드**
  - `image_publisher_node`
  - `yolov8_node`

##### 🚦 카메라 인지부 담당자 2
- **영상 번호**: 23 ~ 25  
- **주요 노드**
  - `traffic_light_detector_node`
  - `lane_info_extractor_node`

##### 🧠 차량 판단부 담당자
- **영상 번호**: 27 ~ 28  
- **주요 노드**
  - `path_planner_node`
  - `motion_planner_node`

#### 참고 영상 링크

[![Simulation Video](http://img.youtube.com/vi/DgEK8iT9QeA/0.jpg)](https://www.youtube.com/watch?v=DgEK8iT9QeA&list=PLIyoAG_PPqRfgHJl1UnMCg3h4-mE_eBfF&index=19)

---

### 5. 초단기 자율주행 차량 해커톤

초기 제공된 프로젝트 코드를 그대로 실행할 경우,  
**차량 제어부의 불안정성으로 인해 좌우 진동(Oscillation)이 발생**하는 문제가 존재한다.

본 해커톤에서는 해당 문제를 해결하기 위해  
**차량 제어부 코드 수정 및 튜닝을 수행**한다.

#### 제한 조건

- **코드 수정 제한 시간**: 1시간
- **트랙**: 동일 트랙 사용
- **실행 횟수 제한 없음**

#### 평가 기준

- **랩 타임 점수**
  - 1등: 20점
  - 2등: 18점
  - 3등: 16점
  - 4등: 14점

- **감점 요소**
  - 차량 바퀴가 라인을 밟을 때마다 **–1점**

> 속도뿐만 아니라 **안정성 및 제어 품질**을 함께 평가한다.

![Lap Time Comparison](img/UNITA_2025_12_26_scoreboard.jpg)