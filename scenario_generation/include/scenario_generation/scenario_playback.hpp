#ifndef SIMULATOR_SCENARIO_PLAYBACK_HPP
#define SIMULATOR_SCENARIO_PLAYBACK_HPP

#include "scenario_generation/scenario_data.hpp"

#include <crp_msgs/msg/scenario.hpp>

namespace crp
{
namespace apl
{

struct PlaybackSettings
{
    double scenario_distance{75.0};
    double scenario_lookahead_time{6.0};
    double lane_width{3.85};
    double target_path_width{3.85};
    double crossing_width{3.0};
    double prediction_horizon{4.0};
    double object_distance{150.0};
    bool enable_occlusion{true};
    bool generate_virtual_objects{true};
    double crossing_pedestrian_probability{0.5};
    double roadside_pedestrian_probability{0.02};
    double virtual_vehicle_probability{0.5};
    double pedestrian_walking_speed{0.8};
    double pedestrian_running_speed{3.0};
    double pedestrian_walking_confidence{0.9};
    double virtual_vehicle_fast_speed{15.0};
    double virtual_vehicle_slow_speed{8.0};
    double virtual_vehicle_fast_confidence{0.7};
    std::size_t maximum_virtual_objects{64};
};

struct RoadState
{
    double elevation{0.0};
    double pitch{0.0};
};

struct ScenarioEgoState
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double pitch{0.0};
};

class ScenarioPlayback
{
public:
    ScenarioPlayback(ScenarioData data, PlaybackSettings settings);
    std::size_t nearestTarget(const ScenarioEgoState & ego) const;
    double speedLimit(const ScenarioEgoState & ego) const;
    double scenarioDistance(const ScenarioEgoState & ego) const;
    RoadState roadState(const ScenarioEgoState & ego) const;
    crp_msgs::msg::Scenario message(const ScenarioEgoState & ego, double elapsed) const;
    const ScenarioData & data() const;

private:
    void applyOcclusion(crp_msgs::msg::Scenario & scenario) const;
    ScenarioData m_data;
    PlaybackSettings m_settings;
};

}
}

#endif