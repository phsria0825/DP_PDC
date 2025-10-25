# 시뮬레이션 가이드

## 개요

본 문서는 RHC 기반 연비 최적 주행 시스템의 완전한 시뮬레이션 환경 사용법을 설명합니다.

## 시뮬레이션 구성 요소

### 1. 핵심 모듈

| 파일 | 설명 |
|------|------|
| `simulator.py` | 시뮬레이션 프레임워크 |
| `run_simulation.py` | 시나리오 실행 스크립트 |
| `analyze_results.py` | 결과 분석 도구 |

### 2. 주요 클래스

#### `EcoDrivingSimulator`
완전한 closed-loop 시뮬레이션 환경을 제공합니다.

**주요 기능:**
- 시간에 따른 차량 동역학 시뮬레이션
- 실시간 신호 상태 업데이트
- RHC 컨트롤러 주기적 호출
- 궤적 및 성능 메트릭 기록
- 결과 시각화

#### `VehicleDynamics`
간단한 차량 동역학 모델입니다.

```python
class VehicleDynamics:
    mass: float = 23000.0        # 차량 질량 (kg)
    max_accel: float = 1.2       # 최대 가속도 (m/s²)
    max_decel: float = 1.5       # 최대 감속도 (m/s²)
    speed: float = 15.0          # 현재 속도 (m/s)
    distance: float = 0.0        # 누적 거리 (m)
```

#### `TrafficLight`
실시간으로 상태가 변하는 신호등입니다.

```python
class TrafficLight:
    intersection_id: int         # 교차로 ID
    distance: float              # 절대 거리 (m)
    red_duration: float          # 빨간불 시간 (s)
    yellow_duration: float       # 노란불 시간 (s)
    green_duration: float        # 초록불 시간 (s)
    current_state: int           # 현재 상태 (0/1/2)
    time_in_state: float         # 현재 상태 경과 시간 (s)
```

#### `SimulationRecord`
시뮬레이션 이력을 기록하고 분석합니다.

**기록 데이터:**
- 시간, 거리, 속도, 목표 속도
- 가속도, 연료 소모율
- RHC 최적화 비용 및 페널티
- 신호 상태

**메트릭 계산:**
- 총 연료 소모량
- km당 연료 소모량
- 평균 속도
- 최대 가속/감속
- RMS Jerk (승차감 지표)
- 신호 위반 횟수

## 빠른 시작

### 1. 기본 실행

```bash
# 모든 시나리오 실행 (비교 포함)
python run_simulation.py --compare

# 특정 시나리오만 실행
python run_simulation.py --scenario 1
python run_simulation.py --scenario 2
python run_simulation.py --scenario 3
```

### 2. 결과 확인

시뮬레이션 완료 후 `simulation_results/` 디렉토리에 다음이 생성됩니다:

```
simulation_results/
├── scenario_1/
│   ├── simulation_results.png    # 종합 결과 그래프
│   └── metrics.txt                # 성능 메트릭
├── scenario_2/
│   ├── simulation_results.png
│   └── metrics.txt
├── scenario_3/
│   ├── simulation_results.png
│   └── metrics.txt
└── scenario_comparison.png        # 시나리오 비교 (--compare 시)
```

## 시나리오 설명

### Scenario 1: 잘 조율된 신호

**특징:**
- 3개 신호등
- 조율된 green wave 패턴
- 최적 속도 유지 시 부드러운 통과 가능

**목적:** 이상적인 조건에서의 시스템 성능 검증

**예상 결과:**
- 신호 위반: 0-1회
- 연비: 최적
- 평균 속도: 높음

### Scenario 2: 어려운 조율

**특징:**
- 4개 신호등
- 어려운 타이밍 패턴
- 제약 조건 내 최적화 능력 테스트

**목적:** 도전적인 조건에서의 적응력 검증

**예상 결과:**
- 신호 위반: 1-3회 (불가피)
- 연비: 양호
- 가감속: 빈번

### Scenario 3: 장거리 혼합 조건

**특징:**
- 4개 신호등, 긴 구간
- 다양한 경사도 (오르막/내리막)
- 혼합 신호 패턴

**목적:** 장기 최적화 및 지형 적응 검증

**예상 결과:**
- 신호 위반: 2-4회
- 연비: 지형 영향 반영
- 장기 안정성 확인

## 커스텀 시나리오 만들기

### 1. 기본 템플릿

```python
from simulator import EcoDrivingSimulator, SimulationConfig
from rhc_eco_driving import RoadSegment

# 1. 설정 생성
config = SimulationConfig(
    dt=0.1,                    # 시간 스텝 (s)
    total_time=200.0,          # 총 시뮬레이션 시간 (s)
    control_interval=1.0,      # RHC 업데이트 주기 (s)
    initial_speed=15.0,        # 초기 속도 (m/s)
    initial_distance=0.0,      # 시작 위치 (m)
    ds=10.0,                   # 공간 이산화 (m)
    max_accel=1.2,             # 최대 가속도 (m/s²)
    max_decel=1.5,             # 최대 감속도 (m/s²)
    horizon_time=180.0,        # 예측 구간 (s)
    save_plots=True,
    output_dir="my_scenario"
)

# 2. 시뮬레이터 생성
simulator = EcoDrivingSimulator(config)

# 3. 신호등 추가
simulator.add_traffic_light(
    intersection_id=1,
    distance=300.0,           # 절대 거리 (m)
    red=30.0,                 # 빨간불 시간 (s)
    yellow=3.0,               # 노란불 시간 (s)
    green=35.0,               # 초록불 시간 (s)
    initial_state=2,          # 초기 상태 (0=빨강, 1=노랑, 2=초록)
    initial_time=10.0         # 현재 상태 경과 시간 (s)
)

# 4. 경로 설정
segments = [
    RoadSegment(
        intersection_id=1,
        distance=300.0,       # 교차로까지 거리 (m)
        length=200.0,         # 구간 길이 (m)
        grade=0.01,           # 경사도 (rad, 양수=오르막)
        speed_limit=20.0      # 제한 속도 (m/s)
    ),
    # 더 많은 구간 추가...
]
simulator.setup_route(segments)

# 5. 실행
record = simulator.run()

# 6. 시각화
simulator.visualize_results(save=True, show=False)
```

### 2. 신호 타이밍 설계 팁

#### 녹색 파동 (Green Wave)
연속된 신호등이 순차적으로 녹색이 되도록 설계:

```python
# 차량이 v_avg = 16 m/s로 주행한다고 가정
# 신호등 간 거리: 300m → 이동 시간: 300/16 = 18.75s

# 신호 1: 초기 녹색
simulator.add_traffic_light(1, 300.0, 30, 3, 35, initial_state=2, initial_time=5)

# 신호 2: 약 19초 후 녹색이 되도록 조정
simulator.add_traffic_light(2, 600.0, 30, 3, 35, initial_state=0, initial_time=14)
# 신호 2는 14초 경과 후 → 16초 후 녹색 (30-14=16)
```

#### 도전적인 패턴
최적화가 필요한 어려운 타이밍:

```python
# 빨간불이 막 시작된 신호
simulator.add_traffic_light(1, 200.0, 35, 3, 25, initial_state=0, initial_time=2)

# 곧 끝날 녹색 신호
simulator.add_traffic_light(2, 450.0, 40, 3, 30, initial_state=2, initial_time=28)
```

## 결과 분석

### 1. 기본 메트릭

시뮬레이션 완료 시 자동으로 계산되는 메트릭:

| 메트릭 | 설명 | 단위 |
|--------|------|------|
| `total_fuel` | 총 연료 소모량 | - |
| `fuel_per_km` | km당 연료 소모량 | /km |
| `total_distance` | 총 주행 거리 | m |
| `avg_speed` | 평균 속도 | m/s |
| `max_accel` | 최대 가속도 | m/s² |
| `min_decel` | 최대 감속도 | m/s² |
| `rms_jerk` | RMS Jerk (승차감) | m/s³ |
| `total_violations` | 신호 위반 횟수 | 회 |
| `avg_cost` | 평균 최적화 비용 | - |

### 2. 시각화 그래프

`simulation_results.png`는 다음을 포함합니다:

1. **Speed Profile**: 속도 vs 시간
2. **Position vs Time**: 거리 vs 시간 (신호등 위치 표시)
3. **Acceleration Profile**: 가속도 vs 시간
4. **Fuel Consumption Rate**: 순간 연료 소모율
5. **Cumulative Fuel**: 누적 연료 소모량
6. **RHC Optimization Cost**: 최적화 비용
7. **Signal Violations**: 신호 위반 발생 시점

### 3. 상세 분석

`analyze_results.py`를 사용하여 추가 분석 가능:

```bash
python analyze_results.py --scenario simulation_results/scenario_1
```

## 시뮬레이션 파라미터 가이드

### SimulationConfig

#### 시간 관련
- **`dt`** (0.05 - 0.2s)
  - 작을수록: 정밀, 느림
  - 클수록: 빠름, 부정확
  - 권장: 0.1s

- **`total_time`** (60 - 500s)
  - 경로 길이에 따라 조정
  - 권장: 경로 길이(m) / 평균속도(m/s) + 여유 30%

- **`control_interval`** (0.5 - 2.0s)
  - RHC 업데이트 주기
  - 작을수록: 반응 빠름, 계산량 증가
  - 권장: 1.0s

#### 최적화 파라미터
- **`ds`** (5 - 20m)
  - 공간 이산화 간격
  - 작을수록: 정밀, 느림
  - 권장: 8-12m

- **`max_accel/max_decel`** (0.5 - 2.0 m/s²)
  - 승차감과 연비 균형
  - 작을수록: 부드러움, 연비 향상
  - 권장: 1.0-1.5 m/s²

- **`horizon_time`** (60 - 300s)
  - 예측 구간 시간
  - 클수록: 장기 최적화, 계산량 증가
  - 권장: 150-180s

## 성능 벤치마크

일반적인 PC (Intel i7, 16GB RAM) 기준:

| 시나리오 | 총 시간 | 시뮬레이션 시간 | 실시간 비율 |
|----------|---------|-----------------|-------------|
| Scenario 1 | 150s | ~10s | 15x |
| Scenario 2 | 200s | ~15s | 13x |
| Scenario 3 | 250s | ~20s | 12x |

**최적화 팁:**
- `ds` 증가 → 2-3배 속도 향상
- `control_interval` 증가 → 선형적 속도 향상
- `horizon_time` 감소 → 비용 함수 계산량 감소

## 문제 해결

### 시뮬레이션이 너무 느려요
```python
config = SimulationConfig(
    ds=15.0,              # 10 → 15로 증가
    control_interval=2.0, # 1 → 2로 증가
    horizon_time=120.0    # 180 → 120으로 감소
)
```

### 신호 위반이 너무 많아요
```python
config = SimulationConfig(
    horizon_time=200.0,   # 예측 구간 증가
    ds=8.0,               # 더 정밀한 이산화
    max_accel=1.5,        # 더 적극적인 가속
)
```

### 결과가 부자연스러워요
```python
config = SimulationConfig(
    max_accel=0.8,        # 가속도 제한 완화
    max_decel=1.2,        # 감속도 제한 완화
    control_interval=0.5  # 더 빈번한 업데이트
)
```

## 고급 활용

### 1. 커스텀 차량 모델

```python
from simulator import VehicleDynamics

# 경량 차량
light_vehicle = VehicleDynamics(
    mass=15000.0,
    max_accel=1.8,
    max_decel=2.5
)
simulator.vehicle = light_vehicle
```

### 2. 동적 신호 변경

```python
# 시뮬레이션 중 신호 타이밍 변경
for light in simulator.traffic_lights:
    if light.intersection_id == 2:
        light.green_duration = 40.0  # 녹색 시간 증가
```

### 3. 데이터 저장 및 재분석

```python
import pickle

# 시뮬레이션 실행 및 저장
record = simulator.run()
with open('my_results.pkl', 'wb') as f:
    pickle.dump(record, f)

# 나중에 불러와서 재분석
with open('my_results.pkl', 'rb') as f:
    loaded_record = pickle.load(f)
    metrics = loaded_record.compute_metrics()
```

## 참고 자료

- **기본 사용법**: `QUICKSTART.md`
- **API 문서**: `PYTHON_IMPLEMENTATION.md`
- **아키텍처**: `README.md`

---

**문서 버전**: 1.0
**마지막 업데이트**: 2025-10
