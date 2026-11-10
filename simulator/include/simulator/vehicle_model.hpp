#ifndef SIMULATOR_VEHICLE_MODEL_HPP
#define SIMULATOR_VEHICLE_MODEL_HPP

#include "scenario_generation/scenario_data.hpp"

namespace crp
{
namespace apl
{

enum class VehicleModelType
{
    Kinematic,
    Dynamic
};

struct VehicleState
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double pitch{0.0};
    double speed{0.0};
    double acceleration{0.0};
    double steering{0.0};
    double yaw_rate{0.0};
    double lateral_speed{0.0};
    double lateral_acceleration{0.0};
    double yaw_acceleration{0.0};
};

struct VehicleSettings
{
    VehicleModelType model{VehicleModelType::Kinematic};
    double wheelbase{2.7};
    double maximum_acceleration{9.81};
    double maximum_deceleration{9.81};
    double maximum_steering{0.6};
    double speed_gain{6.0};
    double lookahead_time{1.5};
    double minimum_lookahead{2.0};
    double mass{1500.0};
    double yaw_inertia{2500.0};
    double front_axle_distance{1.2};
    double front_cornering_stiffness{80000.0};
    double rear_cornering_stiffness{80000.0};
    double air_density{1.225};
    double drag_coefficient{0.3};
    double frontal_area{2.2};
    double rolling_resistance_coefficient{0.012};
    double viscous_resistance{5.0};
    double kinematic_blend_speed{0.5};
    double dynamic_blend_speed{2.0};
    double integration_step{0.005};
};

class VehicleModel
{
public:
    double followPath(const std::vector<MapPoint> & path, std::size_t nearest) const;
    void validateSettings() const;
    double accelerationTarget(double speed_target) const;
    void step(double speed_target, double steering_target, double time_step, double road_pitch = 0.0);
    void stepAcceleration(double acceleration_target, double steering_target,
        double time_step, double road_pitch = 0.0);
    void stepWheelForce(double wheel_force, double steering_target,
        double time_step, double road_pitch = 0.0);

    VehicleState state;
    VehicleSettings settings;
};

}
}

#endif