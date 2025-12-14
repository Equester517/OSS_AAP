# ObstacleCruisePlanner (OCP) - Scenario Handler 구현 가이드

## 개요

ServiceCreator로부터 scenario ID를 받아 해당 JSONL 파일을 읽고, trajectory 데이터를 추출하여 AUTOSAR 형식으로 변환한 후 OCP(ObstacleCruisePlanner)에 전송하는 기능을 구현합니다.

## 구현 흐름

### 1단계: 시나리오 이벤트 수신 (COMPLETED)

**파일**: `swc_obstaclecruiseplanner.h`, `swc_obstaclecruiseplanner.cpp`

```cpp
// Start() 메서드에서 콜백 등록
m_RPort_SCr2OCP->RegistEventHandlerscenario(
    [this](const std::uint8_t& scenario_id) {
        this->onScenarioTriggered(scenario_id);
    }
);
```

### 2단계: 콜백 함수 구현 (COMPLETED)

**함수**: `onScenarioTriggered(std::uint8_t scenario_id)`

```cpp
void Swc_obstaclecruiseplanner::onScenarioTriggered(std::uint8_t scenario_id)
{
    // 시나리오 ID 저장 (thread-safe)
    {
        std::lock_guard<std::mutex> lock(m_scenario_mutex);
        m_current_scenario_id = scenario_id;
    }
    
    // OCP에 시나리오 변경 알림
    m_PPort_OCP2SCr->SendEventscenarioTriggered(scenario_id);
    
    // 파일 로딩은 별도 스레드에서 처리 (non-blocking)
    std::thread([this, scenario_id]() {
        this->loadAndProcessScenario(scenario_id);
    }).detach();
}
```

### 3단계: 시나리오 파일 경로 결정 (COMPLETED)

**함수**: `getScenarioFilePath(std::uint8_t scenario_id)`

```cpp
std::string getScenarioFilePath(std::uint8_t scenario_id)
{
    // scenario_1 → "reference_output/scenario_1/planning__scenario_planning__trajectory.jsonl"
    // scenario_2 → "reference_output/scenario_2/planning__scenario_planning__trajectory.jsonl"
    // scenario_3 → "reference_output/scenario_3/planning__scenario_planning__trajectory.jsonl"
    return "reference_output/scenario_" + std::to_string(scenario_id) + 
           "/planning__scenario_planning__trajectory.jsonl";
}
```

### 4단계: JSONL 파일 읽기 및 파싱 (TODO - 다음 단계)

**함수**: `loadAndProcessScenario(std::uint8_t scenario_id)`

#### 현재 상태
- 파일 존재 여부 확인 ✓
- 로깅 구조 ✓

#### TODO: OSS_TEST_HELPER 통합
```cpp
// 1. JsonlReader를 사용하여 JSONL 파일 읽기
oss_test_helper::JsonlReader reader;
auto records = reader.read(scenario_file);

// 2. 각 라인(record)을 trajectory로 파싱
for (const auto& record : records) {
    // record.trajectory는 oss_test_helper::Trajectory 타입
    
    // 3. AUTOSAR 형식으로 변환
    auto autosar_traj = TrajectoryAdapter::convertTrajectory<
        oss::type::autoware_planning_msgs::msg::Trajectory>(
        record.trajectory);
    
    // 4. 내부 버퍼에 저장 (thread-safe)
    {
        std::lock_guard<std::mutex> lock(m_trajectory_buffer->mutex);
        m_trajectory_buffer->data = autosar_traj;
        m_trajectory_buffer->is_valid = true;
    }
    
    // 5. trajectory 이벤트 전송
    m_PPort_OCP2SCr->SendEventtrajectoryTriggered(autosar_traj);
}
```

## 타입 매핑

### 입력 타입 (OSS_TEST_HELPER)
```cpp
namespace oss_test_helper {
    struct Trajectory {
        Header header;
        std::vector<TrajectoryPoint> points;
    };
    
    struct TrajectoryPoint {
        Duration time_from_start;
        Pose pose;
        double longitudinal_velocity_mps;
        double lateral_velocity_mps;
        double acceleration_mps2;
        double heading_rate_rps;
        double front_wheel_angle_rad;
        double rear_wheel_angle_rad;
    };
    
    struct Pose {
        Position position;
        Orientation orientation;
    };
    
    struct Header {
        uint32_t seq;
        TimeStamp stamp;
        std::string frame_id;
    };
    // ... 기타 타입들
}
```

### 출력 타입 (AUTOSAR)
```cpp
// OCP2SCr 인터페이스
namespace oss::srv::OCP2SCr::skeleton::events::trajectory {
    using SampleType = oss::type::autoware_planning_msgs::msg::Trajectory;
}
```

## 멤버 변수 (추가됨)

```cpp
private:
    // Scenario handling members
    std::mutex m_scenario_mutex;
    std::uint8_t m_current_scenario_id{0};
    
    // Trajectory buffer for thread-safe passing
    struct TrajectoryBuffer {
        std::mutex mutex;
        oss::srv::OCP2SCr::skeleton::events::trajectory::SampleType data;
        bool is_valid{false};
    };
    std::unique_ptr<TrajectoryBuffer> m_trajectory_buffer;

    // Callback handlers
    void onScenarioTriggered(std::uint8_t scenario_id);
    
    // JSONL reading and conversion
    std::string getScenarioFilePath(std::uint8_t scenario_id);
    void loadAndProcessScenario(std::uint8_t scenario_id);
```

## 어댑터 (Trajectory Adapter)

**파일**: `trajectory_adapter.hpp`

```cpp
class TrajectoryAdapter {
    template<typename AutosarTrajectoryType>
    static AutosarTrajectoryType convertTrajectory(
        const oss_test_helper::Trajectory& internal_trajectory);
    
    // 개별 필드 변환 함수들 (구현 필요)
    // - convertTrajectoryPoint()
    // - convertHeader()
    // - convertPose()
    // - convertPosition()
    // - convertOrientation()
    // - convertDuration()
    // - convertTimestamp()
};
```

## 스레드 안전성

1. **시나리오 ID**: `m_scenario_mutex`로 보호
2. **Trajectory 버퍼**: `TrajectoryBuffer::mutex`로 보호
3. **파일 로딩**: 별도 스레드에서 처리 (메인 루프 블로킹 없음)
4. **이벤트 전송**: AUTOSAR ara::com의 내재적 thread-safety 활용

## 에러 처리

- 파일 없음: 로그 출력 후 조용히 반환
- 파싱 실패: 예외 캐치 및 에러 로그
- 변환 실패: 예외 캐치 및 에러 로그

## 빌드 구성

### CMakeLists.txt 업데이트 필요사항

```cmake
# OSS_TEST_HELPER 라이브러리 링크 (향후)
# target_link_libraries(${PARA_APP_NAME}
#                       PRIVATE
#                       oss_test_helper)
```

## 다음 단계

1. **OSS_TEST_HELPER 통합** ✓ 준비 완료
   - TrajectoryAdapter 헤더 및 구현 준비 완료
   - CMakeLists.txt trajectory_adapter.cpp 추가 완료
   - JsonlReader 라이브러리 링크 추가 필요

2. **TrajectoryAdapter 구현** ✓ COMPLETED
   - convertTrajectory() - 전체 trajectory 변환 ✓
   - convertTrajectoryPoint() - 개별 점 변환 ✓
   - convertHeader() - 헤더 정보 변환 ✓
   - convertPose() - Pose (position + orientation) 변환 ✓
   - convertPosition() - 3D 위치 변환 ✓
   - convertOrientation() - 사원수 변환 ✓
   - convertDuration() - 시간 간격 변환 ✓
   - convertTimestamp() - 타임스탬프 변환 ✓

3. **테스트 및 검증** - 다음 단계
   - 시나리오 파일 읽기 확인
   - 데이터 타입 변환 검증
   - 이벤트 전송 확인

## 참고 파일

- Header: `/home/katech/AAP/only_missionplanner/Exe_obstaclecruiseplanner/include/exe_obstaclecruiseplanner/aa/swc_obstaclecruiseplanner.h`
- Implementation: `/home/katech/AAP/only_missionplanner/Exe_obstaclecruiseplanner/src/exe_obstaclecruiseplanner/aa/swc_obstaclecruiseplanner.cpp`
- Adapter Header: `/home/katech/AAP/only_missionplanner/Exe_obstaclecruiseplanner/include/exe_obstaclecruiseplanner/aa/trajectory_adapter.hpp`
- Adapter Implementation: `/home/katech/AAP/only_missionplanner/Exe_obstaclecruiseplanner/src/exe_obstaclecruiseplanner/aa/trajectory_adapter.cpp`
- OSS_TEST_HELPER: `/home/katech/AAP/only_missionplanner/Exe_obstaclecruiseplanner/OSS_TEST_HELPER/`
