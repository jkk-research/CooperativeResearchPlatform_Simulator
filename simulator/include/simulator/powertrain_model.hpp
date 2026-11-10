#ifndef SIMULATOR_POWERTRAIN_MODEL_HPP
#define SIMULATOR_POWERTRAIN_MODEL_HPP

#include "simulator/vehicle_model.hpp"

#include <string>
#include <vector>

namespace crp
{
namespace apl
{

struct PowertrainSettings
{
    double motor_gear_ratio{9.0};
    double wheel_gear_ratio{1.0};
    double wheel_radius{0.32};
    double motor_inertia{0.02};
    double wheel_inertia{1.2};
    double motor_viscous_friction{0.001};
    double wheel_viscous_friction{0.002};
    double shaft_stiffness{18000.0};
    double shaft_damping{120.0};
    double maximum_motor_torque{200.0};
    double maximum_regenerative_torque{120.0};
    double motor_torque_rate_limit{1000.0};
    double integration_step{0.001};
};

struct PowertrainState
{
    double motor_speed{0.0};
    double vehicle_speed{0.0};
    double shaft_twist{0.0};
    double motor_torque{0.0};
    double efficiency{0.0};
    double electrical_power{0.0};
    double energy_consumed{0.0};
    double energy_regenerated{0.0};
};

class PowertrainModel
{
public:
    explicit PowertrainModel(const std::string & efficiency_map_path);

    void validateSettings() const;
    void initialize(double vehicle_speed);
    double step(double acceleration_target, double vehicle_speed, double time_step,
        const VehicleSettings & vehicle);
    double efficiency(double motor_speed, double motor_torque, bool regeneration) const;

    PowertrainSettings settings;
    PowertrainState state;

private:
    std::vector<double> m_speed_grid;
    std::vector<double> m_torque_grid;
    std::vector<std::vector<double>> m_motoring_efficiency;
    std::vector<std::vector<double>> m_regeneration_efficiency;
};

}
}

#endif