#include "simulator/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

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
        distance += std::hypot(path[target + 1].x - path[target].x, path[target + 1].y - path[target].y);
        ++target;
    }
    const double delta_x = path[target].x - state.x;
    const double delta_y = path[target].y - state.y;
    const double local_y = -delta_x * std::sin(state.yaw) + delta_y * std::cos(state.yaw);
    const double squared_distance = delta_x * delta_x + delta_y * delta_y;
    if (squared_distance < 1e-8) {return 0.0;}
    return std::clamp(std::atan2(2.0 * settings.wheelbase * local_y, squared_distance),
        -settings.maximum_steering, settings.maximum_steering);
}

void VehicleModel::step(double speed_target, double steering_target, double time_step)
{
    state.steering = std::clamp(steering_target, -settings.maximum_steering, settings.maximum_steering);
    const double requested = speed_target <= 1e-6 ? -settings.maximum_deceleration :
        settings.speed_gain * (speed_target - state.speed);
    state.acceleration = std::clamp(requested,
        -settings.maximum_deceleration, settings.maximum_acceleration);
    const double previous_speed = state.speed;
    state.speed = std::max(0.0, state.speed + time_step * state.acceleration);
    state.acceleration = (state.speed - previous_speed) / time_step;
    state.yaw_rate = std::tan(state.steering) * state.speed / settings.wheelbase;
    const double midpoint_yaw = state.yaw + 0.5 * state.yaw_rate * time_step;
    state.x += time_step * state.speed * std::cos(midpoint_yaw);
    state.y += time_step * state.speed * std::sin(midpoint_yaw);
    state.yaw = std::remainder(state.yaw + state.yaw_rate * time_step, 2.0 * std::acos(-1.0));
}

}
}