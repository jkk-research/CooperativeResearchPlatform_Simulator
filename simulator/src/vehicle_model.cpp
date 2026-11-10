#include "simulator/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace crp
{
namespace apl
{

double VehicleModel::followPath(const std::vector<MapPoint> & path, std::size_t nearest) const
{
    if (path.empty()) {return 0.0;}
    const double lookahead = std::max(settings.minimum_lookahead, settings.lookahead_time * state.speed);
    auto target = std::min(nearest, path.size() - 1);
    double distance = 0.0;
    while (target + 1 < path.size() && distance < lookahead)
    {
        distance += std::hypot(path[target + 1].x - path[target].x,
            path[target + 1].y - path[target].y, path[target + 1].z - path[target].z);
        ++target;
    }
    const double delta_x = path[target].x - state.x;
    const double delta_y = path[target].y - state.y;
    const double local_y = -delta_x * std::sin(state.yaw) + delta_y * std::cos(state.yaw);
    const double local_x = std::cos(state.pitch) * (delta_x * std::cos(state.yaw) +
        delta_y * std::sin(state.yaw)) + std::sin(state.pitch) * (path[target].z - state.z);
    const double squared_distance = local_x * local_x + local_y * local_y;
    if (squared_distance < 1e-8) {return 0.0;}
    return std::clamp(std::atan2(2.0 * settings.wheelbase * local_y, squared_distance),
        -settings.maximum_steering, settings.maximum_steering);
}

void VehicleModel::validateSettings() const
{
    for (double value : {settings.wheelbase, settings.maximum_acceleration,
        settings.maximum_deceleration, settings.maximum_steering, settings.speed_gain,
        settings.lookahead_time, settings.minimum_lookahead, settings.mass, settings.yaw_inertia,
        settings.front_axle_distance, settings.front_cornering_stiffness,
        settings.rear_cornering_stiffness, settings.frontal_area, settings.kinematic_blend_speed,
        settings.dynamic_blend_speed, settings.integration_step})
    {
        if (!std::isfinite(value) || value <= 0.0)
        {
            throw std::invalid_argument("Vehicle parameters must be finite and positive");
        }
    }
    for (double value : {settings.air_density, settings.drag_coefficient,
        settings.rolling_resistance_coefficient, settings.viscous_resistance})
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            throw std::invalid_argument("Vehicle resistance parameters must be finite and nonnegative");
        }
    }
    if (settings.front_axle_distance >= settings.wheelbase ||
        settings.maximum_steering >= 0.5 * std::acos(-1.0) ||
        settings.dynamic_blend_speed <= settings.kinematic_blend_speed)
    {
        throw std::invalid_argument("Invalid axle geometry, steering limit or low-speed blend interval");
    }
}

double VehicleModel::accelerationTarget(double speed_target) const
{
    validateSettings();
    if (!std::isfinite(speed_target))
    {
        throw std::invalid_argument("Invalid speed-controller input");
    }
    if (speed_target <= 1e-6) {return -settings.maximum_deceleration;}
    return std::clamp(settings.speed_gain * (speed_target - state.speed),
        -settings.maximum_deceleration, settings.maximum_acceleration);
}

void VehicleModel::step(double speed_target, double steering_target, double time_step, double road_pitch)
{
    validateSettings();
    if (!std::isfinite(time_step) || time_step <= 0.0)
    {
        throw std::invalid_argument("Vehicle time step must be finite and positive");
    }
    const double substeps = std::ceil(time_step / settings.integration_step);
    if (substeps > 100000.0) {throw std::invalid_argument("Vehicle update requires too many substeps");}
    const auto count = static_cast<std::size_t>(substeps);
    const double duration = time_step / static_cast<double>(count);
    const double initial_yaw_rate = state.yaw_rate;
    double longitudinal = 0.0;
    double lateral = 0.0;
    for (std::size_t iteration = 0; iteration < count; ++iteration)
    {
        stepAcceleration(accelerationTarget(speed_target),
            steering_target, duration, road_pitch);
        longitudinal += state.acceleration * duration;
        lateral += state.lateral_acceleration * duration;
    }
    state.acceleration = longitudinal / time_step;
    state.lateral_acceleration = lateral / time_step;
    state.yaw_acceleration = (state.yaw_rate - initial_yaw_rate) / time_step;
}

void VehicleModel::stepWheelForce(double wheel_force, double steering_target,
    double time_step, double road_pitch)
{
    validateSettings();
    if (!std::isfinite(wheel_force)) {throw std::invalid_argument("Wheel force must be finite");}
    double drive_acceleration = wheel_force / settings.mass;
    if (settings.model == VehicleModelType::Kinematic)
    {
        const double aerodynamic = 0.5 * settings.air_density * settings.drag_coefficient *
            settings.frontal_area * state.speed * state.speed;
        const double rolling = settings.rolling_resistance_coefficient * settings.mass * 9.81 *
            std::cos(road_pitch);
        const double grade = settings.mass * 9.81 * std::sin(road_pitch);
        drive_acceleration -= (aerodynamic + rolling + grade +
            settings.viscous_resistance * state.speed) / settings.mass;
    }
    stepAcceleration(drive_acceleration, steering_target, time_step, road_pitch);
}

void VehicleModel::stepAcceleration(double acceleration_target, double steering_target,
    double time_step, double road_pitch)
{
    validateSettings();
    for (double value : {acceleration_target, steering_target, time_step, road_pitch,
        state.speed, state.lateral_speed, state.yaw_rate, state.x, state.y, state.z, state.yaw, state.pitch})
    {
        if (!std::isfinite(value)) {throw std::invalid_argument("Vehicle input and state must be finite");}
    }
    if (time_step <= 0.0 || state.speed < 0.0 || std::abs(road_pitch) >= 0.5 * std::acos(-1.0))
    {
        throw std::invalid_argument("Invalid time step, forward speed or road pitch");
    }
    state.steering = std::clamp(steering_target, -settings.maximum_steering, settings.maximum_steering);
    const double drive_acceleration = std::clamp(acceleration_target,
        -settings.maximum_deceleration, settings.maximum_acceleration);
    const double initial_yaw_rate = state.yaw_rate;
    if (settings.model == VehicleModelType::Kinematic)
    {
        const double previous_speed = state.speed;
        state.speed = std::max(0.0, state.speed + time_step * drive_acceleration);
        state.acceleration = (state.speed - previous_speed) / time_step;
        state.lateral_speed = 0.0;
        state.yaw_rate = std::tan(state.steering) * state.speed / settings.wheelbase;
        state.yaw_acceleration = (state.yaw_rate - initial_yaw_rate) / time_step;
        state.lateral_acceleration = state.speed * state.yaw_rate;
        const double midpoint_yaw = state.yaw + 0.5 * state.yaw_rate * time_step;
        state.x += time_step * state.speed * std::cos(state.pitch) * std::cos(midpoint_yaw);
        state.y += time_step * state.speed * std::cos(state.pitch) * std::sin(midpoint_yaw);
        state.z += time_step * state.speed * std::sin(state.pitch);
        state.yaw = std::remainder(state.yaw + state.yaw_rate * time_step, 2.0 * std::acos(-1.0));
        return;
    }

    const double substeps = std::ceil(time_step / settings.integration_step);
    if (substeps > 100000.0) {throw std::invalid_argument("Vehicle update requires too many substeps");}
    const auto count = static_cast<std::size_t>(substeps);
    const double step = time_step / static_cast<double>(count);
    const double rear_distance = settings.wheelbase - settings.front_axle_distance;
    const double front_distance = settings.front_axle_distance;
    const double front_stiffness = settings.front_cornering_stiffness;
    const double rear_stiffness = settings.rear_cornering_stiffness;
    const double steering_cosine = std::cos(state.steering);
    const auto blend = [&](double speed)
    {
        const double fraction = std::clamp((speed - settings.kinematic_blend_speed) /
            (settings.dynamic_blend_speed - settings.kinematic_blend_speed), 0.0, 1.0);
        return fraction * fraction * (3.0 - 2.0 * fraction);
    };
    double longitudinal_acceleration = 0.0;
    double lateral_acceleration = 0.0;
    for (std::size_t iteration = 0; iteration < count; ++iteration)
    {
        const double previous_speed = state.speed;
        const double previous_lateral = state.lateral_speed;
        const double previous_yaw_rate = state.yaw_rate;
        const double dynamic_weight = blend(previous_speed);
        const double slip_speed = std::max(previous_speed, settings.kinematic_blend_speed);
        const double front_force = front_stiffness * (state.steering -
            (previous_lateral + front_distance * previous_yaw_rate) / slip_speed);
        const double aerodynamic = 0.5 * settings.air_density * settings.drag_coefficient *
            settings.frontal_area * previous_speed * previous_speed / settings.mass;
        const double viscous = settings.viscous_resistance * previous_speed / settings.mass;
        const double grade = 9.81 * std::sin(road_pitch);
        const double rolling = settings.rolling_resistance_coefficient * 9.81 * std::cos(road_pitch);
        const double free_acceleration = drive_acceleration - aerodynamic - viscous - grade +
            dynamic_weight * (previous_yaw_rate * previous_lateral -
            front_force * std::sin(state.steering) / settings.mass);
        const double net_acceleration = previous_speed > 0.0 ? free_acceleration - rolling :
            std::max(0.0, free_acceleration - rolling);
        state.speed = std::max(0.0, previous_speed + step * net_acceleration);

        const double next_weight = blend(state.speed);
        const double kinematic_yaw_rate = state.speed * std::tan(state.steering) / settings.wheelbase;
        const double kinematic_lateral = rear_distance * kinematic_yaw_rate;
        if (next_weight == 0.0)
        {
            state.yaw_rate = kinematic_yaw_rate;
            state.lateral_speed = kinematic_lateral;
        }
        else
        {
            const double forward_speed = std::max(state.speed, settings.kinematic_blend_speed);
            const double front_effective = front_stiffness * steering_cosine;
            const double coupling = -front_distance * front_effective + rear_distance * rear_stiffness;
            const double matrix_11 = 1.0 + step * (front_effective + rear_stiffness) /
                (settings.mass * forward_speed);
            const double matrix_12 = -step * (coupling / (settings.mass * forward_speed) - forward_speed);
            const double matrix_21 = -step * coupling / (settings.yaw_inertia * forward_speed);
            const double matrix_22 = 1.0 + step * (front_distance * front_distance * front_effective +
                rear_distance * rear_distance * rear_stiffness) / (settings.yaw_inertia * forward_speed);
            const double rhs_lateral = previous_lateral + step * front_effective * state.steering / settings.mass;
            const double rhs_yaw = previous_yaw_rate + step * front_distance * front_effective *
                state.steering / settings.yaw_inertia;
            const double determinant = matrix_11 * matrix_22 - matrix_12 * matrix_21;
            if (!std::isfinite(determinant) || determinant <= 0.0)
            {
                throw std::runtime_error("Unstable lateral bicycle update; check vehicle parameters and speed");
            }
            const double dynamic_lateral = (rhs_lateral * matrix_22 - matrix_12 * rhs_yaw) / determinant;
            const double dynamic_yaw_rate = (matrix_11 * rhs_yaw - matrix_21 * rhs_lateral) / determinant;
            state.lateral_speed = (1.0 - next_weight) * kinematic_lateral + next_weight * dynamic_lateral;
            state.yaw_rate = (1.0 - next_weight) * kinematic_yaw_rate + next_weight * dynamic_yaw_rate;
        }
        const double midpoint_yaw_rate = 0.5 * (previous_yaw_rate + state.yaw_rate);
        const double midpoint_speed = 0.5 * (previous_speed + state.speed);
        const double midpoint_lateral = 0.5 * (previous_lateral + state.lateral_speed);
        const double midpoint_yaw = state.yaw + 0.5 * midpoint_yaw_rate * step;
        state.x += step * (midpoint_speed * std::cos(state.pitch) * std::cos(midpoint_yaw) -
            midpoint_lateral * std::sin(midpoint_yaw));
        state.y += step * (midpoint_speed * std::cos(state.pitch) * std::sin(midpoint_yaw) +
            midpoint_lateral * std::cos(midpoint_yaw));
        state.z += step * midpoint_speed * std::sin(state.pitch);
        state.yaw = std::remainder(state.yaw + midpoint_yaw_rate * step, 2.0 * std::acos(-1.0));
        longitudinal_acceleration += (state.speed - previous_speed) -
            step * midpoint_yaw_rate * midpoint_lateral;
        lateral_acceleration += (state.lateral_speed - previous_lateral) +
            step * midpoint_yaw_rate * midpoint_speed;
    }
    state.acceleration = longitudinal_acceleration / time_step;
    state.lateral_acceleration = lateral_acceleration / time_step;
    state.yaw_acceleration = (state.yaw_rate - initial_yaw_rate) / time_step;
}

}
}