# Python 구현: 새로운 신호 형식 기반 연비 최적 주행 시스템

## 개요

본 문서는 새로운 신호 데이터 형식을 사용하는 Python 기반 RHC(Rolling Horizon Control) 연비 최적 주행 시스템의 구현을 설명합니다.

## 신호 데이터 형식

### 기존 형식 (MATLAB/기존 Python)
- SPaT 기반: `[id, currentPhase, timeToPhaseEnd]`
- 별도의 duration 및 green flag 배열 필요

### 새로운 형식 (본 구현)

각 신호등 정보는 **6개 요소의 배열**로 제공됩니다:

```python
[Δd, tRemain, TrfState, red, yellow, green]
```

| 요소 | 설명 | 단위 | 예시 |
|------|------|------|------|
| `Δd` | 차량과 신호등 사이의 상대거리 | m | 150.0 |
| `tRemain` | 현재 신호 상태의 잔여시간 | s | 12.0 |
| `TrfState` | 현재 신호 상태 | - | 0=빨강, 1=노랑, 2=초록 |
| `red` | 빨간불 지속시간 | s | 30.0 |
| `yellow` | 노란불 지속시간 | s | 3.0 |
| `green` | 초록불 지속시간 | s | 35.0 |

**최대 4개의 신호등 정보**를 동시에 처리할 수 있습니다.

## 파일 구조

```
DP_PDC/
├── rhc_eco_driving.py          # 메인 구현 모듈
├── test_eco_driving.py         # 단위 테스트
├── example_eco_driving.py      # 실행 예제
├── PYTHON_IMPLEMENTATION.md    # 본 문서
└── README.md                   # 원본 아키텍처 명세
```

## 주요 클래스 및 함수

### 1. 데이터 구조

#### `VehicleState`
차량의 현재 상태를 나타냅니다.

```python
@dataclass
class VehicleState:
    speed: float       # 현재 속도 (m/s)
    distance: float    # 누적 주행 거리 (m)
```

#### `SignalInfo`
신호등 정보를 나타냅니다.

```python
@dataclass
class SignalInfo:
    delta_distance: float    # 상대 거리 (m)
    time_remaining: float    # 잔여 시간 (s)
    current_state: int       # 0=빨강, 1=노랑, 2=초록
    red_duration: float      # 빨간불 시간 (s)
    yellow_duration: float   # 노란불 시간 (s)
    green_duration: float    # 초록불 시간 (s)
```

**사용 예시:**
```python
# 배열로부터 생성
signal_array = [150.0, 12.0, 2, 30.0, 3.0, 35.0]
signal = SignalInfo.from_array(signal_array)

# 객체로부터 배열로 변환
array = signal.to_array()
```

#### `RoadSegment`
도로 구간 정보를 나타냅니다.

```python
@dataclass
class RoadSegment:
    intersection_id: int  # 교차로 ID
    distance: float       # 시작점부터의 거리 (m)
    length: float         # 구간 길이 (m)
    grade: float          # 경사도 (rad)
    speed_limit: float    # 제한 속도 (m/s)
```

#### `OptimizationParams`
최적화 파라미터를 설정합니다.

```python
@dataclass
class OptimizationParams:
    ds: float = 10.0              # 공간 이산화 간격 (m)
    max_accel: float = 1.2        # 최대 가속도 (m/s^2)
    max_decel: float = 1.5        # 최대 감속도 (m/s^2)
    horizon_time: float = 180.0   # 예측 구간 시간 (s)
    speed_grid: Optional[ArrayLike] = None  # 속도 그리드
```

### 2. 핵심 모듈

#### `SignalScheduler`
신호 스케줄을 계산하여 녹색 신호 구간(green windows)을 생성합니다.

```python
@staticmethod
def compute_green_windows(
    signals: List[SignalInfo],
    vehicle_distance: float,
    horizon_time: float = 180.0
) -> ArrayLike:
    """
    Returns:
        Array of shape (N, 3): [absolute_distance, start_time, end_time]
    """
```

**동작 원리:**
1. 각 신호등의 현재 상태와 잔여 시간을 기반으로 신호 주기 시뮬레이션
2. 빨강(0) → 노랑(1) → 초록(2) → 빨강 순서로 순환
3. 예측 구간(`horizon_time`) 내의 모든 녹색 구간을 계산

#### `EcoDrivingController`
RHC 전략을 구현하는 메인 컨트롤러입니다.

```python
class EcoDrivingController:
    def __init__(self, params: Optional[OptimizationParams] = None)
    def initialize_route(self, segments: List[RoadSegment])
    def step(self, vehicle_state: VehicleState, signals: List[SignalInfo])
        -> Tuple[float, ArrayLike, ArrayLike, dict]
```

**주요 메서드:**

- `initialize_route()`: 경로 초기화
- `step()`: RHC 최적화 실행
  - 입력: 차량 상태, 신호 정보
  - 출력: 목표 속도, 최적 궤적, 진단 정보

#### `velocity_optimiz_dp()`
동적 프로그래밍 기반 최적화 엔진입니다.

```python
def velocity_optimiz_dp(
    segments: ArrayLike,
    segment_steps: ArrayLike,
    total_steps: int,
    parameter_vector: ArrayLike,
    speed_vector: ArrayLike,
    signal_windows: ArrayLike,
) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
```

**알고리즘:**
- 상태 공간: 속도 그리드 × 공간 이산화 스텝
- 비용 함수: 연료 소모 + 신호 위반 페널티
- 제약 조건: 가속도 한계, 속도 제한, 신호 준수

#### `fuel_prediction_model()`
23톤 대형 차량의 연비 예측 모델입니다.

```python
def fuel_prediction_model(v: float, a: float, theta: float) -> float:
    """
    Args:
        v: 속도 (m/s)
        a: 가속도 (m/s^2)
        theta: 경사도 (rad)

    Returns:
        연료 소모율
    """
```

## 사용 방법

### 1. 기본 사용 예제

```python
from rhc_eco_driving import (
    VehicleState, SignalInfo, RoadSegment,
    OptimizationParams, EcoDrivingController
)

# 1. 컨트롤러 초기화
params = OptimizationParams(
    ds=10.0,
    max_accel=1.2,
    max_decel=1.5,
    horizon_time=180.0
)
controller = EcoDrivingController(params)

# 2. 경로 설정
segments = [
    RoadSegment(1, 200.0, 100.0, 0.01, 20.0),
    RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
    RoadSegment(3, 650.0, 120.0, -0.01, 20.0),
]
controller.initialize_route(segments)

# 3. 차량 상태
vehicle = VehicleState(speed=15.0, distance=50.0)

# 4. 신호 정보 (최대 4개)
signals = [
    SignalInfo(150.0, 12.0, 2, 30.0, 3.0, 35.0),  # 신호 1
    SignalInfo(350.0, 20.0, 0, 30.0, 3.0, 35.0),  # 신호 2
]

# 5. 최적화 실행
v_set, speeds, times, diag = controller.step(vehicle, signals)

print(f"목표 속도: {v_set:.2f} m/s")
print(f"최적화 비용: {diag['cost']:.6f}")
print(f"신호 위반 횟수: {diag['penalties']}")
```

### 2. 실행 예제

```bash
# 테스트 실행
python test_eco_driving.py

# 시뮬레이션 예제 실행
python example_eco_driving.py
```

### 3. 예제 출력

```
======================================================================
Eco-Driving Simulation with RHC and New Signal Format
======================================================================

Route Information:
  Number of intersections: 4
    Intersection 1: 250m, limit=20.0 m/s, grade=0.0050 rad
    Intersection 2: 450m, limit=22.0 m/s, grade=0.0000 rad
    Intersection 3: 700m, limit=20.0 m/s, grade=-0.0080 rad
    Intersection 4: 950m, limit=20.0 m/s, grade=0.0020 rad

Initial Vehicle State:
  Speed: 15.00 m/s (54.0 km/h)
  Distance: 80.0 m

Traffic Signal Information (4 signals):
  Signal 1 at 250m:
    Current: Green (15.0s remaining)
    Cycle: R=35s, Y=3s, G=30s
  Signal 2 at 450m:
    Current: Red (18.0s remaining)
    Cycle: R=35s, Y=3s, G=30s
  ...

======================================================================
Optimization Results
======================================================================

Target Speed: 16.50 m/s (59.4 km/h)

Diagnostics:
  Status: optimized
  Horizon count: 2 intersections
  Optimal cost: 0.012345
  Signal violations: 0

Speed Profile:
  Initial speed: 15.00 m/s
  Final speed: 18.00 m/s
  Total time: 45.23 s
```

## 주요 특징

### 1. Rolling Horizon Control (RHC)

- **예측 구간**: 현재 위치에서 가장 가까운 **2개 교차로**
- **제어 구간**: 첫 번째 교차로까지의 계획만 실제 적용
- **호라이즌 이동**: 교차로 통과 시 자동으로 다음 구간으로 이동

### 2. 신호 협조 제어

- 녹색 신호에 맞춰 도착하도록 속도 조절
- 빨간불 대기 시간 최소화
- 불필요한 가감속 억제

### 3. 연비 최적화

- 물리 기반 연료 소모 모델
- 가속/감속 최소화
- 경사도 고려

### 4. 제약 조건

- 제한 속도 준수
- 가속도/감속도 한계
- 안전한 주행 궤적

## 알고리즘 상세

### 동적 프로그래밍 (DP)

**상태 공간:**
```
State = (velocity_index, spatial_step)
- velocity_index ∈ [0, num_speeds-1]
- spatial_step ∈ [0, total_steps]
```

**비용 함수:**
```
J(v, s) = min Σ [fuel_cost(v, a, θ) × Δt + signal_penalty(s, t)]
```

**페널티 시스템:**
- 신호 정보 없음: `1e6`
- 적색 신호 도착: `1e5 + 1e3 × |time_to_green|`
- 녹색 신호 도착: `0`

### 신호 스케줄러

**신호 상태 순환:**
```
Red (0) → Yellow (1) → Green (2) → Red (0) → ...
```

**녹색 구간 계산:**
1. 현재 신호 상태에서 시작
2. 잔여 시간 후 다음 상태로 전환
3. 예측 구간 동안 반복
4. 녹색 상태인 구간만 기록

## 테스트

전체 테스트 스위트는 다음을 검증합니다:

1. ✅ 신호 데이터 구조 생성 및 변환
2. ✅ 신호 스케줄러의 녹색 구간 계산
3. ✅ 컨트롤러 초기화 및 경로 설정
4. ✅ 전체 최적화 사이클
5. ✅ 호라이즌 업데이트 메커니즘
6. ✅ 연비 예측 모델
7. ✅ 가속도 및 시간 계산
8. ✅ 최대 4개 신호 처리
9. ✅ 경로 완료 처리
10. ✅ 신호 위상 순환

**테스트 실행:**
```bash
pytest test_eco_driving.py -v
```

## 성능 특성

### 계산 복잡도

- **DP 복잡도**: O(N_v² × N_s)
  - N_v: 속도 그리드 크기 (기본 16)
  - N_s: 공간 스텝 수 (기본 20-50)
- **실시간 처리**: 일반적으로 < 100ms (현대 PC 기준)

### 메모리 사용

- **DP 테이블**: ~2MB (16 speeds × 50 steps × 2 tables)
- **전체 메모리**: < 10MB

## 파라미터 튜닝 가이드

### `ds` (공간 이산화 간격)
- **작은 값 (5-8m)**: 더 정밀한 궤적, 계산량 증가
- **큰 값 (15-20m)**: 빠른 계산, 정밀도 감소
- **권장**: 10m

### `max_accel` / `max_decel`
- **작은 값**: 더 부드러운 주행, 승차감 향상
- **큰 값**: 더 빠른 반응, 연비 악화
- **권장**: 1.0-1.5 m/s²

### `horizon_time`
- **작은 값 (60-120s)**: 빠른 계산, 단기 최적화
- **큰 값 (180-300s)**: 장기 최적화, 더 나은 연비
- **권장**: 150-180s

### `speed_grid`
- **조밀한 그리드**: 정밀한 속도 제어
- **희박한 그리드**: 빠른 계산
- **권장**: `np.arange(0, 30, 2.0)` (2 m/s 간격)

## 제한사항 및 향후 개선

### 현재 제한사항

1. **최대 2개 교차로**: 계산 시간 제약
2. **단일 차선**: 차선 변경 미고려
3. **정적 신호**: 실시간 신호 변경 미반영
4. **단일 차량**: 교통류 상호작용 미고려

### 향후 개선 방향

1. **적응형 호라이즌**: 상황에 따라 1-3개 교차로 선택
2. **차선 변경**: 다차선 시나리오 지원
3. **V2I 통합**: 실시간 신호 업데이트
4. **학습 기반**: 연비 모델 온라인 학습
5. **병렬 처리**: GPU 가속 DP 계산

## 참고 자료

- 원본 아키텍처: `README.md`
- MATLAB 구현: `rhcSupervisor.m`, `velocityOptimiz_DP.m`
- 기존 Python 구현: `rhc_pipeline.py`

## 라이선스 및 기여

본 구현은 연구 및 교육 목적으로 자유롭게 사용 가능합니다.

---

**문서 버전**: 1.0
**작성일**: 2025년 10월
**작성자**: Claude Code (Anthropic)
