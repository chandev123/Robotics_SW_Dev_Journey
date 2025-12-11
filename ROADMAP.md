# 🗺️ Robotics Software Developer Journey Roadmap

이 로드맵은 지능형 로보틱스 소프트웨어 개발자가 되기 위한 학습 여정과 마일스톤을 기록합니다.
**Python & AI 기초**를 다진 후, 현재 **ROS2 기반의 로봇 제어 시스템**을 집중적으로 학습하고 있습니다.
또한, **Kaggle 경진대회**와 **개인 프로젝트**를 통해 실전 데이터 처리 및 소프트웨어 구현 역량을 지속적으로 강화하고 있습니다.

## 📌 Legend
- ✅ **Done**: 학습 및 실습 완료
- 🏃 **In Progress**: 현재 집중 학습 중
- 🔜 **Upcoming**: 학습 예정

---

## 🚀 Practical Experience & Side Projects (Highlights)
정규 과정 외에 주도적으로 수행한 프로젝트와 성과입니다.

### 🏆 Kaggle Competitions
- **Ongoing Challenge ($500 Tier)** 🏃
  - **Subject:** Geospatial Object Detection using YOLOv8s
  - **Focus:** 컴퓨팅 리소스 제약 환경(Low-spec)에서의 모델 최적화 및 24시간 연속 학습 파이프라인 구축
- ✅ **Mini-Kaggle Challenge 1** 
  - **Scale:** Training & Prediction on **85,000+** Image Datasets
  - **Role:** 대용량 이미지 전처리 자동화 및 분류 모델 성능 개선
- ✅ **Mini-Kaggle Challenge 2** 
  - **Scale:** **13,000+** Image Datasets
  - **Role:** Baseline 모델 구축 및 데이터 증강(Augmentation) 기법 적용

### 💻 Application Development
- ✅ **Project SignLens** 
  - **Tech Stack:** Python, PyQt5, OpenCV, OCR
  - **Description:** 간판 이미지 인식 및 텍스트 추출을 위한 GUI 애플리케이션 개발
  - **Key Learning:** 사용자 인터페이스(GUI) 설계 및 딥러닝 모델의 실제 서비스(Serving) 연동 경험

### 📚 Intensive Study
- **Peer Learning Group** (Weekly 10h+) 🏃
  - 매주 10시간 이상 동료들과 코드 리뷰 및 신기술(Paper Review) 토론 진행
  - CS 기초(Network, OS) 및 알고리즘 문제 해결 능력 배양

---

## 🚀 Phase 1: Foundation (Python & DevOps)
> **Period:** 2025.09.08 ~ 2025.10.22

로봇 소프트웨어 개발을 위한 프로그래밍 언어 숙달 및 협업/배포 환경 구축 기초를 다졌습니다.

- ✅ **Python Programming**
  - Python 기초 문법, 객체지향 프로그래밍(Class) 및 예외 처리
  - 데이터 분석(Pandas, Numpy) 및 시각화(Matplotlib) 활용
  - 자료구조(Stack, Queue) 및 탐색 알고리즘(DFS/BFS) 구현

- ✅ **DevOps Implementation**
  - DBMS(SQL) 활용 및 Python 연동
  - Git 형상관리 (Branch 전략, 협업 워크플로우)
  - Docker 컨테이너 환경 구축 및 배포 실습

---

## 🧠 Phase 2: AI Core (Computer Vision)
> **Period:** 2025.10.23 ~ 2025.12.09

로봇의 "눈"이 되는 컴퓨터 비전 기술과 딥러닝 모델의 원리를 학습하고 구현했습니다.

- ✅ **Deep Learning & Modeling**
  - CNN 아키텍처(VGG, ResNet) 이해 및 모델 빌드
  - Object Detection (YOLO, SSD, Faster R-CNN) 원리 학습
  - Transformer & ViT(Vision Transformer) 구조 이해

- ✅ **Computer Vision Application**
  - OpenCV 기반 영상 처리(전처리, 필터링, 기하학적 변환)
  - Object Tracking 알고리즘 구현 (Optical Flow, Meanshift)
  - **Project:** Custom Dataset을 활용한 객체 탐지 및 성능 튜닝

---

## 🤖 Phase 3: ROS2 Robotics Core (Current Focus)
> **Period:** 2025.12.10 ~ 2026.01.09

로봇 운영체제(ROS2)의 핵심 통신 미들웨어와 패키지 구조를 익히고, 시뮬레이션 환경을 구축합니다.

### 🏃 Step 1: ROS2 Basics (12.10 ~ 12.16)
- ✅ **Linux & Network Foundation**
  - Linux CLI 숙련 및 프로세스/스레드 관리
  - Network Socket Programming (TCP/UDP, Multi-client Chatting) 구현
  - 하드웨어 통신 인터페이스(RS-485, Ethernet) 이해
- 🏃 **ROS2 Communication**
  - ROS2 Humble 환경 설정 및 CLI 도구 활용
  - Node, Topic, Service, Action 개념 이해 및 통신 실습
  - Turtlesim 제어 및 RQT 디버깅 툴 활용

### 🔜 Step 2: ROS2 Programming (12.17 ~ 12.30)
- [ ] **Python Package Development**
  - `rclpy` 기반 커스텀 노드 및 인터페이스(msg/srv) 개발
  - MultiThreadedExecutor를 활용한 동시성 제어
  - Launch 파일을 이용한 시스템 실행 최적화

### 🔜 Step 3: Simulation & Integration (12.31 ~ 01.09)
- [ ] **Advanced ROS2 Features**
  - DDS QoS 설정 및 네트워크 최적화
  - Node Lifecycle 관리 및 ROS2 Security 적용
- [ ] **Simulation & Navigation**
  - URDF/TF2 로봇 모델링 및 Gazebo 시뮬레이션 연동
  - SLAM 및 Navigation2(Nav2) 실습
  - ROS2 + OpenCV 연동 자율주행 기능 구현 (Lane Detect)

---

## 🏆 Phase 4: Capstone Projects (Real-world Application)
> **Period:** 2026.01.13 ~ 2026.03.16

현업 수준의 하드웨어와 최신 기술 스택을 활용한 4단계 실무 프로젝트를 수행합니다.
난이도가 높은 **Isaac Sim**과 **SLAM** 프로젝트에 대비해 선행 학습을 병행합니다.

### 🔜 Project 1: ROS2 Robotic Process Automation
> **Period:** 2026.01.13 ~ 2026.01.26
- **Goal:** 두산 협동로봇(M0609) 제어 및 자동화 공정 시스템 구현
- **Key Tech:** ROS2 Humble, MoveIt2, Doosan DART-Platform
- **Focus:** Manipulator Kinematics 이해 및 안전한 하드웨어 제어 로직 구현

### 🔜 Project 2: Digital Twin Simulation System (High Difficulty 🔥)
> **Period:** 2026.01.27 ~ 2026.02.09
- **Goal:** NVIDIA Isaac Sim 활용 가상 제조 환경(Virtual Commissioning) 구축
- **Key Tech:** NVIDIA Isaac Sim, Omniverse, ROS2 Bridge
- **Challenge:** 고사양 시뮬레이션 환경 최적화 및 ROS2 통신 동기화 해결
- **Prep:** USD 파일 포맷 이해 및 Isaac Sim 튜토리얼 선행 학습

### 🔜 Project 3: Vision & Voice AI Collaborative Robot
> **Period:** 2026.02.10 ~ 2026.02.27
- **Goal:** Vision AI(객체 인식)와 LLM(음성/언어) 기술을 융합한 차세대 서비스 로봇 개발
- **Key Tech:** PyTorch, YOLO, Langchain, OpenAI Whisper
- **Advantage:** Kaggle 및 SignLens 프로젝트 경험을 바탕으로 **고도화된 파이프라인(Pipeline) 구축** 및 **실시간 처리**에 집중

### 🔜 Project 4: SLAM Autonomous Driving System (Final & Integration)
> **Period:** 2026.03.03 ~ 2026.03.16
- **Goal:** 멀티 로봇 협업을 통한 물류/보안 자율주행 시스템 통합 구축
- **Key Tech:** ROS2 Nav2, SLAM, TurtleBot4, Fleet Management
- **Challenge:** 실제 환경(Real-world)에서의 센서 노이즈 처리 및 Nav2 파라미터 튜닝
- **Features:** 다중 로봇 관제 시스템 및 통합 GUI 개발