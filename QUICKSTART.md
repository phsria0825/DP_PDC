# 빠른 시작 가이드 (Quick Start Guide)

## 설치

### 1. Python 환경 설정

Python 3.8 이상이 필요합니다.

```bash
# Python 버전 확인
python --version

# 필요한 패키지 설치
pip install -r requirements.txt
```

### 2. 파일 확인

```bash
ls -l *.py
# rhc_eco_driving.py        - 메인 모듈
# test_eco_driving.py       - 테스트
# example_eco_driving.py    - 예제
```

## 사용 방법

### 기본 예제

```python
from rhc_eco_driving import (
    VehicleState, SignalInfo, RoadSegment,
    EcoDrivingController, OptimizationParams
)

# 1. 컨트롤러 생성
controller = EcoDrivingController()

# 2. 경로 설정 (3개 교차로)
route = [
    RoadSegment(1, 200.0, 100.0, 0.01, 20.0),  # (ID, 거리, 길이, 경사, 제한속도)
    RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
    RoadSegment(3, 650.0, 120.0, -0.01, 20.0),
]
controller.initialize_route(route)

# 3. 차량 상태
vehicle = VehicleState(speed=15.0, distance=50.0)  # 15m/s, 50m 주행

# 4. 신호 정보 (최대 4개)
# 형식: [상대거리(m), 잔여시간(s), 현재상태, 빨강(s), 노랑(s), 초록(s)]
#       현재상태: 0=빨강, 1=노랑, 2=초록
signals = [
    SignalInfo(150.0, 12.0, 2, 30.0, 3.0, 35.0),  # 200m 위치, 초록불 12초 남음
    SignalInfo(350.0, 20.0, 0, 30.0, 3.0, 35.0),  # 400m 위치, 빨간불 20초 남음
]

# 5. 최적화 실행
v_set, speeds, times, diagnostics = controller.step(vehicle, signals)

# 6. 결과 출력
print(f"목표 속도: {v_set:.2f} m/s ({v_set * 3.6:.1f} km/h)")
print(f"최적화 비용: {diagnostics['cost']:.6f}")
print(f"신호 위반: {diagnostics['penalties']}회")
```

## 신호 데이터 형식

각 신호는 **6개 요소의 배열**로 제공:

```python
[Δd, tRemain, TrfState, red, yellow, green]
```

| 인덱스 | 이름 | 설명 | 예시 |
|--------|------|------|------|
| 0 | Δd | 차량과 신호등 사이 거리 (m) | 150.0 |
| 1 | tRemain | 현재 신호 잔여시간 (s) | 12.0 |
| 2 | TrfState | 현재 신호 (0/1/2) | 2 (초록) |
| 3 | red | 빨간불 지속시간 (s) | 30.0 |
| 4 | yellow | 노란불 지속시간 (s) | 3.0 |
| 5 | green | 초록불 지속시간 (s) | 35.0 |

### 예시

```python
# 방법 1: 배열로 생성
signal_array = [150.0, 12.0, 2, 30.0, 3.0, 35.0]
signal = SignalInfo.from_array(signal_array)

# 방법 2: 직접 생성
signal = SignalInfo(
    delta_distance=150.0,
    time_remaining=12.0,
    current_state=2,  # 0=빨강, 1=노랑, 2=초록
    red_duration=30.0,
    yellow_duration=3.0,
    green_duration=35.0
)

# 배열로 변환
array = signal.to_array()
```

## 예제 실행

### 1. 전체 시뮬레이션 예제

```bash
python example_eco_driving.py
```

**출력:**
- 경로 정보
- 차량 상태
- 신호 정보
- 최적화 결과
- 속도 프로파일 그래프 (eco_driving_result.png)

### 2. 테스트 실행

```bash
# pytest 사용 (설치된 경우)
pytest test_eco_driving.py -v

# 또는 직접 실행
python test_eco_driving.py
```

## 파라미터 설정

### OptimizationParams

```python
params = OptimizationParams(
    ds=10.0,              # 공간 간격 (m) - 작을수록 정밀
    max_accel=1.2,        # 최대 가속도 (m/s²)
    max_decel=1.5,        # 최대 감속도 (m/s²)
    horizon_time=180.0,   # 예측 구간 (s) - 클수록 장기 최적화
)
controller = EcoDrivingController(params)
```

### 권장 값

| 파라미터 | 최소 | 권장 | 최대 | 설명 |
|----------|------|------|------|------|
| ds | 5 | 10 | 20 | 작으면 정밀, 크면 빠름 |
| max_accel | 0.8 | 1.2 | 2.0 | 작으면 부드러움 |
| max_decel | 1.0 | 1.5 | 3.0 | 안전 고려 |
| horizon_time | 60 | 150-180 | 300 | 장기 계획 범위 |

## 주요 기능

### 1. Rolling Horizon Control
- 가장 가까운 **2개 교차로**만 최적화
- 교차로 통과 시 자동으로 다음 구간으로 이동
- 실시간 처리 가능

### 2. 신호 협조 제어
- 녹색 신호에 맞춰 도착 시간 조절
- 불필요한 정지 최소화
- 연비 최적화

### 3. 물리 기반 연비 모델
- 23톤 대형 차량 기준
- 공기/구름/등판 저항 고려
- 실험 데이터 기반 모델

## 출력 해석

### v_set (목표 속도)
다음 제어 주기에 적용할 목표 속도 (m/s)

### speeds (속도 궤적)
최적화된 전체 속도 프로파일 배열

### times (시간 배열)
각 속도에 대응하는 시간 (s)

### diagnostics (진단 정보)
```python
{
    'status': 'optimized',           # 최적화 상태
    'current_idx': 0,                 # 현재 교차로 인덱스
    'horizon_count': 2,               # 예측 구간 교차로 개수
    'cost': 0.012345,                 # 최적 비용
    'total_steps': 25,                # 총 이산화 스텝
    'penalties': 0,                   # 신호 위반 횟수
}
```

## 문제 해결

### ImportError: No module named 'numpy'
```bash
pip install numpy matplotlib
```

### "Route not initialized" 오류
```python
# initialize_route()를 먼저 호출
controller.initialize_route(segments)
```

### 신호 위반이 많이 발생
- `horizon_time` 증가 (180 → 240)
- `ds` 감소 (10 → 8)
- 속도 그리드 조밀화

### 계산 시간이 너무 길어요
- `ds` 증가 (10 → 15)
- `horizon_time` 감소 (180 → 120)
- 속도 그리드 간격 증가 (2 → 3)

## 다음 단계

1. **고급 사용법**: `PYTHON_IMPLEMENTATION.md` 참조
2. **아키텍처 이해**: `README.md` 참조
3. **코드 커스터마이징**: `rhc_eco_driving.py` 수정

## 지원

이슈나 질문은 GitHub Issues에 등록해주세요.

---

**버전**: 1.0
**마지막 업데이트**: 2025-10
