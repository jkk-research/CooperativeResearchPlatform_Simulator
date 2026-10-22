#ifndef SIMULATOR_SCENARIO_PLAYBACK_HPP
#define SIMULATOR_SCENARIO_PLAYBACK_HPP

#include "simulator/scenario_data.hpp"
#include "simulator/vehicle_model.hpp"

#include <crp_msgs/msg/scenario.hpp>

namespace crp
{
namespace apl
{

struct PlaybackSettings
{
    double scenario_distance{75.0};
    double lane_width{3.85};
    double target_path_width{3.85};
    double crossing_width{3.0};
    double prediction_horizon{4.0};
    double object_distance{150.0};
};

class ScenarioPlayback
{
public:
    ScenarioPlayback(ScenarioData data, PlaybackSettings settings);
    std::size_t nearestTarget(const VehicleState & ego) const;
    double speedLimit(const VehicleState & ego) const;
    crp_msgs::msg::Scenario message(const VehicleState & ego, double elapsed) const;
    const ScenarioData & data() const;

private:
    ScenarioData m_data;
    PlaybackSettings m_settings;
};

}
}

#endif