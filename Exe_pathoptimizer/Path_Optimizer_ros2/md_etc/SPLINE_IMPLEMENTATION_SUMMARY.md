# Cubic Spline Interpolation 구현 보고서

## 개요

ROS2 Path Optimizer와 동일한 출력을 얻기 위해 **Cubic Spline 기반 path interpolation**을 구현했습니다.

### 문제 인식
- Linear interpolation (Standalone) vs Spline interpolation (ROS2) 차이로 인해
- 동일한 waypoint에서도 서로 다른 reference path 생성
- 이로 인해 global position이 크게 차이남 (0.33m mean error)

### 구현 내용

**1. Cubic Spline 클래스 (`cubic_spline.hpp`)**

Natural boundary condition을 사용한 cubic spline:
- 각 구간: s(t) = a + b*t + c*t² + d*t³
- 경계 조건: s''(0) = s''(n) = 0 (smooth한 시작/끝)
- 알고리즘: Thomas algorithm (tridiagonal system solver)
- 시간 복잡도: O(n)

**주요 메서드**:
```cpp
void calcSplineCoefficients(const vector<double>& x, const vector<double>& y);
double interpolate(double x);
double derivative(double x);     // ds/dx for yaw
double secondDerivative(double x); // d²s/dx² for curvature
```

**2. Arc-length Parameterization**

Waypoint를 arc-length s로 parameterize:
```cpp
s[0] = 0
s[i] = s[i-1] + ||P[i] - P[i-1]||  // 누적 거리

// x(s), y(s)의 spline 생성
CubicSpline spline_x(s, x_coords);
CubicSpline spline_y(s, y_coords);

// Uniform sampling in arc-length space
for (int i = 0; i < num_points; i++) {
    double s_query = i * total_length / (num_points - 1);
    x = spline_x.interpolate(s_query);
    y = spline_y.interpolate(s_query);
}
```

이렇게 하면:
- ✅ Constant speed parameterization (균일한 간격)
- ✅ Sharp corner에 point 몰림 방지
- ✅ 물리적으로 의미 있는 parameterization

**3. Yaw 및 Curvature 계산**

1차 미분에서 yaw:
```cpp
double dx_ds = spline_x.derivative(s);
double dy_ds = spline_y.derivative(s);
double yaw = atan2(dy_ds, dx_ds);
```

2차 미분에서 curvature:
```cpp
double dx2_ds2 = spline_x.secondDerivative(s);
double dy2_ds2 = spline_y.secondDerivative(s);
double numerator = abs(dx_ds * dy2_ds2 - dy_ds * dx2_ds2);
double denominator = pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
double curvature = numerator / denominator;
```

### 결과

**정량적 개선**:
| 지표 | Linear Interp | Cubic Spline | 개선도 |
|------|---------------|--------------|--------|
| Position Error (mean) | 0.337 m | 0.217 m | **↓ 35%** |
| Position Error (max) | 2.501 m | 0.555 m | **↓ 78%** |
| Yaw Error (mean) | 7.72° | 3.53° | **↓ 54%** |
| Velocity Error | 0 m/s | 0 m/s | ✅ Perfect |

**정성적 개선**:
- ✅ Smooth한 경로 생성 (sharp corner 완화)
- ✅ Curvature 값이 0이 아님 (0.0015~0.006 rad/m)
- ✅ 물리적으로 타당한 경로 (차량이 따라갈 수 있음)

### 남은 문제

**1. 첫 점 Orientation 불일치**
- ROS2 첫 점: (0.098, -0.010, 85.7°)
- Standalone 첫 점: (0.000, 0.000, 0.0°)
- 원인: Spline이 waypoint를 정확히 통과하는 반면, ROS2는 corner cutting

**2. Position Error 여전히 임계값 초과**
- 목표: < 0.1m
- 현재: 0.217m mean
- 원인: ROS2의 더 공격적인 path smoothing

**3. Waypoint Exact Passage**
- Cubic spline: 모든 waypoint를 **정확히** 통과
- ROS2: Waypoint 근처를 통과하지만 정확히는 아님 (더 smooth)

### 추가 개선 방향

**Option 1: B-Spline 사용** (추천)
- B-spline은 control point를 정확히 통과하지 않음
- Waypoint가 "목표 방향"만 제시
- 더 smooth한 경로 생성 가능

**Option 2: Smoothing Spline**
- Least-squares fitting with smoothness penalty
- λ 파라미터로 waypoint accuracy vs smoothness 조절
- 수식: minimize Σ(s(x_i) - y_i)² + λ∫(s''(x))²dx

**Option 3: Catmull-Rom Spline**
- Waypoint의 tangent를 주변 점에서 자동 계산
- C1 연속성 (1차 미분 연속)
- Implementation 간단

**Option 4: Corner Detection + Local Relaxation**
- 급격한 방향 변화 지점 탐지
- 해당 지점에서만 waypoint constraint 완화
- 직선 구간은 정확히 유지

### 기술적 세부사항

**Thomas Algorithm (Tridiagonal Solver)**

Cubic spline coefficient를 구하기 위해 tridiagonal system Ax = d 풀이:
```
[b₀ c₀  0  ... 0 ] [x₀]   [d₀]
[a₁ b₁ c₁  ... 0 ] [x₁]   [d₁]
[ 0 a₂ b₂  ... 0 ] [x₂] = [d₂]
[... ... ... ... ...] [...]   [...]
[ 0  0  0  aₙ bₙ] [xₙ]   [dₙ]

Forward sweep:
  c'ᵢ = cᵢ / (bᵢ - aᵢ * c'ᵢ₋₁)
  d'ᵢ = (dᵢ - aᵢ * d'ᵢ₋₁) / (bᵢ - aᵢ * c'ᵢ₋₁)

Backward substitution:
  xₙ = d'ₙ
  xᵢ = d'ᵢ - c'ᵢ * xᵢ₊₁
```

**Natural Boundary Condition**

양 끝에서 2차 미분 = 0:
- s''(0) = 0: 시작점에서 곡률 0 (직선으로 시작)
- s''(n) = 0: 끝점에서 곡률 0 (직선으로 끝)

이것은 "자연스러운" 경계 조건이지만, 때로는 clamped boundary (1차 미분 고정)가 더 나을 수도 있음.

**Curvature Formula 유도**

Parametric curve r(s) = (x(s), y(s))의 curvature:

```
κ = ||r' × r''|| / ||r'||³

2D에서:
r' = (dx/ds, dy/ds)
r'' = (d²x/ds², d²y/ds²)

Cross product (2D pseudo-scalar):
r' × r'' = (dx/ds)(d²y/ds²) - (dy/ds)(d²x/ds²)

따라서:
κ = |dx/ds * d²y/ds² - dy/ds * d²x/ds²| / (dx/ds² + dy/ds²)^(3/2)
```

### 코드 구조

```
cubic_spline.hpp (NEW)
  ├─ class CubicSpline
  │   ├─ calcSplineCoefficients()  // Setup
  │   ├─ interpolate()             // Query
  │   ├─ derivative()              // For yaw
  │   └─ secondDerivative()        // For curvature
  
mpt_optimizer.cpp (MODIFIED)
  └─ generateReferencePoints()
      ├─ Arc-length parameterization
      ├─ Create x(s), y(s) splines
      ├─ Uniform sampling in s
      └─ Calculate yaw & curvature
```

### 검증 방법

**1. Curvature Continuity Check**
```bash
# 출력된 curvature 값이 smooth한지 확인
# Linear interpolation: 0, 0, 0, ... (직선)
# Cubic spline: 0, 0.0015, 0.003, 0.0045, ... (smooth)
```

**2. Arc-length Consistency**
```bash
# 각 segment의 길이가 일정한지 확인
# ds ≈ total_length / (num_points - 1)
```

**3. Visual Inspection**
```python
# Python으로 경로 시각화
import matplotlib.pyplot as plt
plt.plot(x_linear, y_linear, 'ro-', label='Linear')
plt.plot(x_spline, y_spline, 'b*-', label='Spline')
plt.legend()
```

### 성능 특성

**시간 복잡도**:
- Spline coefficient 계산: O(n) (Thomas algorithm)
- 각 point interpolation: O(log n) (binary search) + O(1) (polynomial eval)
- 전체: O(n + m log n) where m = num_query_points

**공간 복잡도**:
- O(n) for coefficients storage

**Numerical Stability**:
- Thomas algorithm은 strictly diagonally dominant matrix에 대해 안정
- Cubic spline system은 이 조건 만족 (c_i = 1, b_i = 4)

### 결론

Cubic spline 기반 interpolation으로:
- ✅ Position error 35% 감소
- ✅ Yaw error 54% 감소  
- ✅ Smooth하고 물리적으로 타당한 경로

하지만 ROS2 완전 parity를 위해서는:
- 🔧 Waypoint exact passage 완화 (B-spline 또는 smoothing spline)
- 🔧 첫 점 orientation 계산 개선
- 🔧 ROS2의 corner cutting 메커니즘 추가 분석 필요

**현재 구현의 활용 가치**:
- ✅ Standalone path planning system으로 충분히 사용 가능
- ✅ Smooth trajectory generation 연구/학습 도구
- ✅ ROS2 경로 최적화 알고리즘 이해 도구
- ⚠️ 완전한 ROS2 parity는 추가 작업 필요
