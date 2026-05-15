
# 🕊️ Beli
Unreal Engine Boids SimulationAn Unreal Engine project focused on the implementation and performance optimization of the Boids flocking algorithm.<br>
언리얼 엔진 기반의 [Boids 군집 알고리즘](https://en.wikipedia.org/wiki/Boids) 구현 및 성능 최적화 프로젝트입니다. 
<br><br>


# 📖 Project Overview
<table style="width: 100%;">
  <tr>
    <td style="width: 50%;"><img src="https://github.com/user-attachments/assets/0bbfe52d-3a99-43b7-8213-669792d1b42d" width="100%"></td>
    <td style="width: 50%;"><img src="https://github.com/user-attachments/assets/b0b97c2c-4f15-4c3f-a48c-d0adac9b85d9" width="100%"></td>
  </tr>
  <tr>
    <td style="width: 50%;"><img src="https://github.com/user-attachments/assets/472132ab-3618-41f2-839b-508e0122a11e" width="100%"></td>
    <td style="width: 50%;"><img src="https://github.com/user-attachments/assets/95e90cdb-c90c-44af-9050-0460d9aa59a0" width="100%"></td>
  </tr>
</table>
Beli 프로젝트는 수많은 객체(Boids)가 유기적으로 무리 지어 이동하는 군집 시뮬레이션을 언리얼 엔진(Unreal Engine) 환경에서 C++로 구현하는 프로젝트입니다.<br>
단순한 로직 구현을 넘어, 수천개의 객체를 렌더링하고 연산하기 위한 최적화 기법을 점진적으로 적용하고 연구하는 것을 목표로 합니다.<br><br><br>


# 🛠️ 구현 및 성능 최적화 내역 (Implemented & Optimized)<br>
기본적인 Boids 군집 알고리즘 구현을 넘어, 1만 마리 이상의 객체를 데스크탑 환경에서 프레임 드랍 없이 연산하기 위해 엔진의 메모리 구조와 CPU 아키텍처를 고려한 최적화 기법들이 적용되어 있습니다.<br><br>

## ISMC (Instanced Static Mesh Component) 렌더링:<br>
수만 개의 개별 액터를 스폰하는 대신, ISMC를 도입하여 단일 드로우 콜(Draw Call)로 군집을 렌더링하여 GPU 파이프라인의 오버헤드를 극적으로 제거했습니다.<br><br>

## 멀티스레딩 병렬 연산 (Multi-threading):<br>
언리얼 엔진의 ParallelFor를 적극 활용하여, 1만 마리의 Boids 조향(Steering - Separation, Alignment, Cohesion) 연산을 CPU의 여러 코어에 효율적으로 분산 처리합니다.<br>
이때 읽기(Read) 버퍼와 쓰기(Write) 버퍼를 분리하는 더블 버퍼링(Double Buffering) 기반의 Lock-Free 아키텍처를 적용하여, 무거운 동기화(Lock/Mutex) 오버헤드나 데이터 경합(Race Condition) 없이 여러 스레드가 하나의 버퍼에 안전하고 빠르게 접근할 수 있도록 설계했습니다.<br><br>

## 비정렬 공간 해싱 (Unsorted Spatial Hashing):<br>
O(N²) 의 무거운 이웃 탐색 시간 복잡도를 공간 해시 그리드를 통해 O(1) 수준으로 압축했습니다.<br><br>

## 데이터 지향 설계 (DOD) & SoA 패턴 적용:<br>
객체 지향(AoS) 구조에서 발생하는 캐시 미스(Cache Miss)를 해결하기 위해, 조향 연산에 필요한 데이터(위치, 속도 등)를 분리된 배열 기반인 SoA(Structure of Arrays) 로 재설계했습니다. 이를 통해 L1/L2 캐시 히트율을 비약적으로 끌어올렸습니다.<br><br><br>

# 🚀 TODO: 비동기 공간 충돌 및 회피 시스템 (Async Spatial Collision)
순수 GPU 기반 파티클(Niagara) 시뮬레이션이 가지는 '게임플레이 로직과의 상호작용 부재'라는 한계를 극복하기 위해, CPU 환경에서 동적 객체(플레이어, 움직이는 기믹)와의 실시간 상호작용 및 지능적 회피를 처리하는 최적화 시스템을 개발 중입니다.
- 피보나치 구면 레이캐스트 (Fibonacci Sphere Raycasting): 1만 마리가 개별적으로 물리 연산을 수행하는 병목을 막기 위해, 보이드가 존재하는 '활성화된 그리드(Active Grid)'의 중앙점에서 피보나치 16방향으로만 LineTrace를 발사하여 연산량을 1/20 이하로 압축합니다.<br>
- 공간 후보정 (Grid Smoothing / Blur Pass): 충돌이 감지된 그리드의 회피 벡터 값을 인접한 이웃 그리드들과 평균 내어 전파하는 평준화(Smoothing) 로직을 적용합니다. 이를 통해 무리가 장애물에 닿기 전부터 유기적이고 부드럽게 갈라지는 고품질 시각 연출을 달성합니다.<br><br>


# ⚙️ Getting Started
본 프로젝트는 Unreal Engine 5.7.4 환경에서 빌드되었습니다.<br>
Repository 클론: git clone https://github.com/storhouser/unreal_tech_boids<br>
uproject 파일을 우클릭하여 Generate Visual Studio project files 실행<br>
Visual Studio에서 솔루션 빌드 후 언리얼 에디터 실행<br>
BP_BeliSwarm 액터를 레벨에 배치하여 시뮬레이션 테스트<br>
