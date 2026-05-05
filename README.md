# 🕊️ Beli
Unreal Engine Boids SimulationAn Unreal Engine project focused on the implementation and performance optimization of the Boids flocking algorithm.(언리얼 엔진 기반의 Boids 군집 알고리즘 구현 및 성능 최적화 프로젝트입니다.)<br><br>


# 📖 Project Overview
<img width="1188" height="1159" alt="image" src="https://github.com/user-attachments/assets/0bbfe52d-3a99-43b7-8213-669792d1b42d" />
Beli 프로젝트는 수많은 객체(Boids)가 유기적으로 무리 지어 이동하는 군집 시뮬레이션을 언리얼 엔진(Unreal Engine) 환경에서 C++로 구현하는 프로젝트입니다.<br>
단순한 로직 구현을 넘어, 수천개의 객체를 렌더링하고 연산하기 위한 최적화 기법을 점진적으로 적용하고 연구하는 것을 목표로 합니다.<br><br>


# 🚀 Roadmap & Features
현재 프로젝트는 기본 시스템을 구축하는 초기 단계이며, 다음과 같은 로드맵을 따라 고도화될 예정입니다.<br><br>


## ✅ Phase 1:
Basic Flocking System Boids 알고리즘 기초 구현<br>
크레이그 레이놀즈(Craig Reynolds)의 3대 규칙(Separation, Alignment, Cohesion) 적용.<br>
Instanced Static Mesh (ISM) 도입: 개별 액터를 스폰하는 대신, ISM 컴포넌트(UInstancedStaticMeshComponent)를 사용하여 드로우 콜(Draw Call)을 줄이고 수천 개의 기본 메시(Cone)를 렌더링.<br>
C++ 기반의 코어 연산 설계.<br>

## 🛠️ Phase 2: 
크레이그 레이놀즈의 3개 규칙을 기반으로 목적지 추적, 포식자 회피 알고리즘 적용.<br>
Performance Optimization **O(N^2)** 시간 복잡도를 가지는 거리 계산 로직의 최적화. 공간 분할(Spatial Partitioning)<br>
Grid, Quadtree, 또는 Octree 구조를 도입하여 불필요한 이웃 탐색 연산 비용 감소. 멀티스레딩(Multi-threading)을 활용한 병렬 처리 연산 검토.<br>

## ✨ Phase 3:
Advanced Rendering & Animation VAT (Vertex Animation Texture) 적용<br>
스켈레탈 메시(Skeletal Mesh)의 무거운 본 연산을 배제하고, 텍스처 데이터를 이용해 날갯짓 등의 유기적인 애니메이션을 가볍게 인스턴싱.<br>
Niagara System 통합 검토: 언리얼 엔진의 강력한 파티클 시스템인 나이아가라를 활용한 초대규모 군집 렌더링 및 GPU 연산 테스트.<br>


# 💻 Tech StackEngine
Unreal Engine 5 (C++)Rendering: Instanced Static Mesh Component (ISMC)<br>
Algorithm: Boids (Flocking Simulation)<br><br>


# ⚙️ Getting Started
본 프로젝트는 Unreal Engine 환경에서 빌드되었습니다.<br>
Repository 클론: git clone https://github.com/storhouser/unreal_tech_boids<br>
uproject 파일을 우클릭하여 Generate Visual Studio project files 실행<br>
Visual Studio에서 솔루션 빌드 후 언리얼 에디터 실행<br>
BP_BeliSwarm 액터를 레벨에 배치하여 시뮬레이션 테스트<br>
