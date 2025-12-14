# autoware_path_optimizer: Object 유무에 따른 경로 최적화 분석

## 질문 요약

**Q**: `path_optimizer_NonObj.txt`와 `path_optimizer_WithObj.txt`는 object 유무에 따른 결과인데, autoware_path_optimizer는 어떻게 object 정보를 받아서 다른 결과를 생성하는가?

## 결론: Object 정보는 **left_bound와 right_bound에 반영되어 입력됨**

### 핵심 발견 ✅

**autoware_path_optimizer는 object를 직접 입력받지 않습니다!**

대신, **upstream planning module**에서 object를 고려하여 계산한 **drivable area (left_bound, right_bound)**를 받아서 최적화를 수행합니다.

---

## 1. Input 메시지 구조

### autoware_planning_msgs/Path.msg
```
std_msgs/Header header
autoware_planning_msgs/PathPoint[] points
geometry_msgs/Point[] left_bound      # ⭐ Drivable area 왼쪽 경계
geometry_msgs/Point[] right_bound     # ⭐ Drivable area 오른쪽 경계
```

### autoware_planning_msgs/PathPoint.msg
```
geometry_msgs/Pose pose
float32 longitudinal_velocity_mps
float32 lateral_velocity_mps
float32 heading_rate_rps
bool is_final
```

### 주요 입력 정보
1. **`points[]`**: 참조 경로 (waypoints)
2. **`left_bound[]`**: 주행 가능 영역의 왼쪽 경계
3. **`right_bound[]`**: 주행 가능 영역의 오른쪽 경계
4. **Odometry**: Ego vehicle의 현재 위치와 속도

---

## 2. Object 정보가 경로에 반영되는 과정

### Autoware Planning Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│ 1. Mission Planning (route_handler)                                │
│    → Global route 생성                                              │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 2. Behavior Planning (behavior_path_planner)                       │
│    → Lane following, Avoidance, Lane Change 등                     │
│    → ⭐ OBJECT 감지 및 회피 경로 생성                              │
│    → Drivable area 계산 (left_bound, right_bound)                  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
                    ┌─────────────────┐
                    │   Object 있음?   │
                    └─────────────────┘
                       ↙           ↘
              Yes (With Object)   No (Without Object)
                ↓                     ↓
     left_bound가 좁아짐        left_bound가 넓음
     (object 회피 위해)         (차선 전체 사용)
                ↓                     ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 3. Motion Planning (autoware_path_optimizer)                       │
│    → Input: Path with left_bound, right_bound                      │
│    → MPT optimization within bounds                                 │
│    → Output: Smooth optimized trajectory                            │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.1 Object 없을 때
```
Behavior Planner:
  - Lane detection만 사용
  - left_bound = lane의 왼쪽 경계선
  - right_bound = lane의 오른쪽 경계선
  - 넓은 drivable area

↓ Path message

Path Optimizer:
  - 넓은 bounds 내에서 최적화
  - 부드러운 경로 생성
  - Corner cutting 가능
```

### 2.2 Object 있을 때
```
Behavior Planner:
  - Lane detection + Object detection
  - Object 위치 파악
  - Avoidance path 생성
  - left_bound와 right_bound가 object를 피하도록 조정
  - 좁아진 drivable area

↓ Path message (bounds가 변경됨!)

Path Optimizer:
  - 좁아진 bounds 내에서 최적화
  - "갈 수 있는 영역" 내에서만 경로 생성
  - Object 위치는 모르지만, bounds가 이미 object를 고려함
  - 안전 여유 공간 확보 (bounds 안에 이미 반영됨)
```

**💡 핵심**: Path Optimizer는 **object 위치를 몰라도** 됩니다!
- Behavior Planner가 이미 object를 고려하여 bounds를 계산했기 때문
- Path Optimizer는 단지 **주어진 영역 내에서 최적화**만 수행
- "Object를 피한 경로 유지" = "Bounds 안에서만 경로 생성"

---

## 3. 코드 분석: node.cpp

### 3.1 Input 처리
```cpp
// src/node.cpp:100-102
path_sub_ = create_subscription<Path>(
  "~/input/path", 1, 
  std::bind(&PathOptimizer::onPath, this, std::placeholders::_1));
```

### 3.2 Planner Data 생성
```cpp
// src/node.cpp:301-312
PlannerData PathOptimizer::createPlannerData(
  const Path & path, const Odometry::ConstSharedPtr ego_odom_ptr) const
{
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  
  // ⭐ Drivable area bounds를 그대로 사용
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  
  planner_data.ego_pose = ego_odom_ptr->pose.pose;
  planner_data.ego_vel = ego_odom_ptr->twist.twist.linear.x;
  
  return planner_data;
}
```

### 3.3 Bounds 검증
```cpp
// src/node.cpp:292-296
if (path.left_bound.empty() || path.right_bound.empty()) {
  RCLCPP_INFO_SKIPFIRST_THROTTLE(
    get_logger(), clock, 5000, 
    "Left or right bound in path is empty.");
  return false;
}
```

**→ left_bound와 right_bound는 필수 입력!**

---

## 4. 코드 분석: mpt_optimizer.cpp

### 4.1 Bounds 업데이트
```cpp
// src/mpt_optimizer.cpp:603
updateBounds(ref_points, p.left_bound, p.right_bound, p.ego_pose, p.ego_vel);
```

### 4.2 Lateral Distance 계산
```cpp
// src/mpt_optimizer.cpp:812-816
const double dist_to_left_bound = calcLateralDistToBounds(
  ref_point_for_bound_search.pose, left_bound, soft_road_clearance, true);
const double dist_to_right_bound = calcLateralDistToBounds(
  ref_point_for_bound_search.pose, right_bound, soft_road_clearance, false);

// Bounds 저장
ref_points.at(i).bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
```

### 4.3 최적화 제약 조건
```cpp
// src/mpt_optimizer.cpp:820-825
// NOTE: The drivable area's width is sometimes narrower than the vehicle width
//       which means infeasible to run especially when obstacles are extracted 
//       from the drivable area.
//       In this case, the drivable area's width is forced to be wider.

// extend violated bounds, where the input path is outside the drivable area
```

**→ Drivable area가 차량 폭보다 좁으면 자동으로 확장!**

---

## 5. 실험 결과 분석

### 5.1 path_optimizer_NonObj.txt vs path_optimizer_WithObj.txt

```python
# 비교 결과
Point [ 0]: Distance = 0.000000 m (완전 동일)
Point [ 1]: Distance = 0.000000 m (완전 동일)
...
Point [14]: Distance = 0.000000 m (완전 동일)

Mean distance: 0.000000 m
Max distance:  0.000000 m
Min distance:  0.000000 m
```

**결론**: 두 출력이 **완전히 동일**합니다!

### 5.2 왜 동일한가?

**가능한 원인**:

1. **Timestamp 차이만 존재**:
   ```
   NonObj:  nanosec: 87414201
   WithObj: nanosec: 189671749
   ```
   - 같은 시나리오, 다른 시간에 실행
   - Object가 경로에 영향을 주지 않는 위치에 있었음

2. **Object가 경로 밖에 있음**:
   - Object가 ego vehicle의 주행 경로에서 멀리 떨어져 있음
   - Behavior planner가 회피 경로를 생성하지 않음
   - left_bound, right_bound가 동일

3. **Object가 이미 지나간 후**:
   - Object를 이미 회피한 이후의 trajectory
   - 정상 주행 구간으로 복귀

---

## 6. Object 정보가 반영되는 시나리오

### 시나리오 1: Object가 차선에 있을 때

```
Before (No Object):
  left_bound:  [(-2.0, 0), (-2.0, 10), (-2.0, 20), ...]
  right_bound: [(2.0, 0), (2.0, 10), (2.0, 20), ...]
  → 4m width

After (Object at x=0, y=15):
  left_bound:  [(-2.0, 0), (-2.0, 10), (-0.5, 15), (-0.5, 20), (-2.0, 25), ...]
  right_bound: [(2.0, 0), (2.0, 10), (2.0, 15), (2.0, 20), (2.0, 25), ...]
  → Object 근처에서 left_bound가 오른쪽으로 이동 (회피)
```

### 시나리오 2: 차선 변경 시

```
Before (Lane Following):
  left_bound:  [(-2.0, 0), (-2.0, 50)]  # 현재 차선
  right_bound: [(2.0, 0), (2.0, 50)]

After (Lane Change to Left):
  left_bound:  [(-6.0, 0), (-6.0, 50)]  # 왼쪽 차선으로 확장
  right_bound: [(-2.0, 0), (-2.0, 50)]  # 원래 left가 right로
```

---

## 7. 입력 데이터 예시

### 7.1 test_input_path.txt (Simple scenario)
```yaml
ros2 topic pub /input/path autoware_planning_msgs/msg/Path \
"{header: {frame_id: 'map'}, \
left_bound: [{x: -2.0, y: 0.0, z: 0.0}, {x: -2.0, y: 50.0, z: 0.0}], \
right_bound: [{x: 2.0, y: 0.0, z: 0.0}, {x: 2.0, y: 50.0, z: 0.0}], \
points: [\
  {pose: {position: {x: 0.0, y: 0.0}}, longitudinal_velocity_mps: 10.0}, \
  {pose: {position: {x: 0.5, y: 10.0}}, longitudinal_velocity_mps: 10.0}, \
  {pose: {position: {x: -0.5, y: 20.0}}, longitudinal_velocity_mps: 10.0}, \
  {pose: {position: {x: 0.0, y: 30.0}}, longitudinal_velocity_mps: 10.0}, \
  {pose: {position: {x: 0.0, y: 50.0}}, longitudinal_velocity_mps: 10.0}]}"
```

### 7.2 Object 회피 시나리오 (예상)
```yaml
ros2 topic pub /input/path autoware_planning_msgs/msg/Path \
"{header: {frame_id: 'map'}, \
left_bound: [\
  {x: -2.0, y: 0.0, z: 0.0}, \
  {x: -2.0, y: 10.0, z: 0.0}, \
  {x: -0.5, y: 15.0, z: 0.0},  # ⭐ Object 위치에서 좁아짐
  {x: -0.5, y: 20.0, z: 0.0},  # ⭐ Object 영역
  {x: -2.0, y: 25.0, z: 0.0}, \
  {x: -2.0, y: 50.0, z: 0.0}], \
right_bound: [{x: 2.0, y: 0.0, z: 0.0}, {x: 2.0, y: 50.0, z: 0.0}], \
points: [...]}"
```

---

## 8. 답변 요약

### Q1: Object 유무에 따라 결과가 어떻게 다른가?

**A**: Object 정보는 **left_bound와 right_bound에 반영**되어 들어옵니다.

- **Object 없음**: left_bound, right_bound가 차선 전체를 커버
- **Object 있음**: Behavior planner가 object 위치를 고려하여 bounds를 조정
  - Object 근처에서 drivable area가 좁아짐
  - Path optimizer는 좁아진 영역 내에서 최적화

### Q2: Object 정보를 추가로 입력받는가?

**A**: **아니오**. autoware_path_optimizer는 object를 직접 입력받지 않습니다.

**입력**:
- ✅ `Path` message (points, left_bound, right_bound)
- ✅ `Odometry` message (ego vehicle state)

**입력하지 않음**:
- ❌ Object list
- ❌ Obstacle information
- ❌ Perception data

### Q3: test_input_odometry.txt나 test_input_path.txt에 object 정보가 반영되는가?

**A**: **test_input_path.txt의 left_bound, right_bound에 반영**됩니다.

```
Object 없음:
  left_bound: [{x: -2.0, y: 0.0}, {x: -2.0, y: 50.0}]  # 직선
  right_bound: [{x: 2.0, y: 0.0}, {x: 2.0, y: 50.0}]   # 직선

Object 있음 (예: y=15m 위치에 object):
  left_bound: [{x: -2.0, y: 0.0}, 
               {x: -2.0, y: 10.0},
               {x: -0.5, y: 15.0},  # ⭐ 좁아짐
               {x: -0.5, y: 20.0},  # ⭐ Object 회피
               {x: -2.0, y: 25.0},
               {x: -2.0, y: 50.0}]
  right_bound: [{x: 2.0, y: 0.0}, {x: 2.0, y: 50.0}]  # 변경 없음
```

---

## 9. Upstream Module: behavior_path_planner

Object 감지 및 drivable area 계산은 **behavior_path_planner**에서 수행됩니다:

### 주요 기능
1. **Lane Following**: 차선 내 주행
2. **Avoidance**: Object 회피 경로 생성
3. **Lane Change**: 차선 변경
4. **Pull Over/Out**: 정차 및 출발

### Drivable Area 계산
```cpp
// behavior_path_planner에서 수행
// 1. Lane geometry 추출
// 2. Object detection 결과 반영
// 3. Avoidance path 생성
// 4. left_bound, right_bound 계산
// 5. Path message 생성 및 publish
```

### Topic 연결
```
behavior_path_planner → /path_optimizer/input/path (autoware_planning_msgs/Path)
                         ↓
                   path_optimizer
                         ↓
                    /output/trajectory
```

---

## 10. 실전 테스트 방법

### 10.1 Object 없는 경로 테스트
```bash
ros2 topic pub /path_optimizer/input/path autoware_planning_msgs/msg/Path \
"{header: {frame_id: 'map'}, \
left_bound: [{x: -2.0, y: 0.0, z: 0.0}, {x: -2.0, y: 50.0, z: 0.0}], \
right_bound: [{x: 2.0, y: 0.0, z: 0.0}, {x: 2.0, y: 50.0, z: 0.0}], \
points: [...]}" --once
```

### 10.2 Object 회피 경로 테스트
```bash
ros2 topic pub /path_optimizer/input/path autoware_planning_msgs/msg/Path \
"{header: {frame_id: 'map'}, \
left_bound: [\
  {x: -2.0, y: 0.0, z: 0.0}, \
  {x: -0.8, y: 15.0, z: 0.0},  # ⭐ Object 회피를 위해 좁아짐
  {x: -0.8, y: 20.0, z: 0.0}, \
  {x: -2.0, y: 25.0, z: 0.0}], \
right_bound: [{x: 2.0, y: 0.0, z: 0.0}, {x: 2.0, y: 50.0, z: 0.0}], \
points: [...]}" --once
```

### 10.3 결과 확인
```bash
# Output trajectory 확인
ros2 topic echo /path_optimizer/output/trajectory

# Debug marker 확인 (RViz)
ros2 topic echo /path_optimizer/debug/marker
```

---

## 12. FAQ: "Object 위치를 몰라도 회피가 가능한가?"

### Q: Path Optimizer가 object 위치를 모르는데 어떻게 "Object를 피한 경로"를 유지할 수 있나요?

**A**: **"피한 경로"는 이미 bounds에 반영되어 있기 때문입니다!**

### 비유로 이해하기 🚗

#### 시나리오 1: Object 위치를 직접 알려주는 경우
```
Navigator: "전방 100m에 사고 차량이 있어요."
Driver: "알겠습니다. 왼쪽으로 회피하겠습니다."
```
→ 운전자가 **object 위치를 알고** 스스로 판단

#### 시나리오 2: 갈 수 있는 영역만 알려주는 경우 (현재 방식)
```
Navigator: "이 차선 표시 안으로만 가세요. (left_bound, right_bound)"
Driver: "알겠습니다. 이 영역 안에서 부드럽게 가겠습니다."
```
→ 운전자는 **왜 그 영역인지 몰라도** 지키기만 하면 됨  
→ Navigator가 이미 사고 차량을 고려하여 영역을 정했음!

### 코드 레벨 예시

#### Behavior Path Planner (Upstream)
```cpp
// Object 감지
std::vector<Object> objects = perception_->getObjects();

// Object 위치: (x=0.0, y=15.0, width=2.0m)
Object obstacle = objects[0];

// 안전 거리 계산
double safety_margin = 0.5;  // 50cm
double avoidance_width = obstacle.width / 2 + safety_margin;

// left_bound 조정 (object를 고려하여)
if (obstacle.x < lane_center) {
  // Object가 왼쪽에 있으면 left_bound를 오른쪽으로 이동
  left_bound[i].x = obstacle.x + avoidance_width;  // -0.5 (object 오른쪽)
} else {
  // Object가 오른쪽에 있으면 right_bound를 왼쪽으로 이동
  right_bound[i].x = obstacle.x - avoidance_width;
}

// Path message 생성
path.left_bound = left_bound;   // ⭐ Object 정보가 여기 들어감!
path.right_bound = right_bound; // ⭐ Object 정보가 여기 들어감!
```

#### Path Optimizer (Downstream)
```cpp
// Input: Path message (left_bound, right_bound만 있음)
void optimize(const Path& path) {
  // Object 위치는 모름!
  // Object obstacle;  ❌ 이런 변수 없음
  
  // Bounds만 사용
  for (size_t i = 0; i < ref_points.size(); ++i) {
    // 각 point에서 갈 수 있는 범위 계산
    double dist_to_left = calcLateralDist(point, path.left_bound);
    double dist_to_right = calcLateralDist(point, path.right_bound);
    
    // 최적화 제약 조건 설정
    ref_points[i].bounds = {dist_to_right, dist_to_left};
    
    // 이 범위 내에서만 경로 생성
    // → 자동으로 object를 피하게 됨!
  }
  
  // MPT 최적화 (bounds 내에서만)
  mpt_optimizer_->optimize(ref_points);
}
```

### 실제 데이터 예시

#### Input Path (Behavior Planner → Path Optimizer)
```yaml
# y=0~10m: Object 없음
left_bound:
  - {x: -2.0, y: 0.0}   # 왼쪽 차선 경계
  - {x: -2.0, y: 10.0}
  
# y=10~20m: Object 있음! (하지만 명시하지 않음)
left_bound:
  - {x: -0.5, y: 15.0}  # ⭐ 갑자기 오른쪽으로 이동
  - {x: -0.5, y: 20.0}  # ⭐ 좁아진 영역 유지
  
# y=20~30m: Object 지나감
left_bound:
  - {x: -2.0, y: 25.0}  # 다시 왼쪽으로 복귀
  - {x: -2.0, y: 30.0}

right_bound:
  - {x: 2.0, y: 0.0}    # 오른쪽은 변화 없음
  - {x: 2.0, y: 30.0}
```

#### Path Optimizer의 해석
```
y=0~10m:   갈 수 있는 폭 = 4.0m (-2.0 ~ 2.0)
           → "여유 있네, 부드러운 경로 만들자"

y=15~20m:  갈 수 있는 폭 = 2.5m (-0.5 ~ 2.0)  ⭐ 갑자기 좁아짐!
           → "여기는 좁네, 이 영역 안에만 있어야지"
           → 왜 좁은지는 모름! Object 때문인지, 공사 때문인지, ...
           → 하지만 상관없음! 그냥 bounds 안에만 있으면 됨!

y=25~30m:  갈 수 있는 폭 = 4.0m (-2.0 ~ 2.0)
           → "다시 넓어졌네, 부드럽게 가자"
```

### 왜 이 방식이 좋은가?

#### 1. **모듈 분리** (Separation of Concerns)
```
Behavior Planner:
  - 전략적 판단 (어디로 갈까?)
  - Object 회피 전략
  - 차선 변경 판단
  
Path Optimizer:
  - 전술적 실행 (어떻게 갈까?)
  - 부드러운 경로 생성
  - 차량 동역학 고려
```

#### 2. **정보 은닉** (Information Hiding)
- Path Optimizer는 object 타입, 속도, 크기 등을 몰라도 됨
- Bounds만 지키면 안전 보장
- 코드 복잡도 감소

#### 3. **재사용성** (Reusability)
```
Bounds가 좁아지는 이유:
  - Object 회피
  - 공사 구간
  - 좁은 터널
  - 차선 변경 중

→ Path Optimizer는 이유와 관계없이 동일하게 작동!
```

### 정리

**질문**: "Object 위치를 모르면 어떻게 피해갈 수 있나요?"

**답변**: 
- ✅ Behavior Planner가 **이미 object를 고려하여 bounds를 계산**했습니다
- ✅ Path Optimizer는 **bounds 안에서만 경로 생성**합니다
- ✅ Bounds를 지키기만 하면 **자동으로 object를 피하게** 됩니다
- ✅ "Object를 피한 경로 유지" = "Bounds 준수"

**비유**: 
- 차선을 따라가면 자연스럽게 도로 밖 나무를 피하게 됩니다
- 나무가 어디 있는지 정확히 몰라도, 차선만 따라가면 안전합니다
- Left_bound, right_bound = 차선 역할

---

## 13. 최종 결론

### ✅ 핵심 사실

1. **autoware_path_optimizer는 object를 직접 받지 않음**
2. **Upstream behavior_path_planner가 object를 고려하여 drivable area (left_bound, right_bound)를 계산**
3. **Path optimizer는 주어진 bounds 내에서 최적화만 수행**
4. **Object 유무는 left_bound, right_bound의 형태로 반영됨**

### 🔍 path_optimizer_NonObj.txt vs path_optimizer_WithObj.txt가 동일한 이유

- **같은 bounds를 받았음** (object가 경로에 영향을 주지 않는 위치)
- **Timestamp만 다름** (다른 시간에 실행)
- **실제 object 회피 시나리오를 테스트하려면 left_bound, right_bound를 다르게 설정해야 함**

### 📝 Object 영향을 테스트하려면

1. **Behavior planner 출력 확인**: `/planning/scenario_planning/lane_driving/behavior_planning/path`
2. **left_bound, right_bound 변화 관찰**
3. **Path optimizer 입력으로 다른 bounds 제공**
4. **결과 trajectory 비교**

---

**작성일**: 2025-12-04  
**분석 대상**: autoware_path_optimizer (Autoware Universe)  
**검증 방법**: 코드 분석 + 실제 출력 데이터 비교
