#ifndef SIMULATOR_VEHICLE_MODEL_HPP
#define SIMULATOR_VEHICLE_MODEL_HPP

#include "simulator/scenario_data.hpp"

namespace crp
{
namespace apl
{

struct VehicleState
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
    double acceleration{0.0};
    double steering{0.0};
    double yaw_rate{0.0};
};

struct VehicleSettings
{
    double wheelbase{2.7};
    double maximum_acceleration{9.81};
    double maximum_deceleration{9.81};
    double maximum_steering{0.6};
    double speed_gain{6.0};
    double lookahead_time{1.5};
    double minimum_lookahead{2.0};
};

class VehicleModel
{
public:
    double followPath(const std::vector<MapPoint> & path, std::size_t nearest) const;
    void step(double speed_target, double steering_target, double time_step);

    VehicleState state;
    VehicleSettings settings;
};

}
}

#endif