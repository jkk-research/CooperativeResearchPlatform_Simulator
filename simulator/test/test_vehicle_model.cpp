#include "simulator/vehicle_model.hpp"
#include "simulator/powertrain_model.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char * message)
{
    if (!condition) {throw std::runtime_error(message);}
}

void finite(const crp::apl::VehicleState & state)
{
    for (double value : {state.x, state.y, state.yaw, state.speed, state.lateral_speed,
        state.yaw_rate, state.acceleration, state.lateral_acceleration, state.yaw_acceleration})
    {
        require(std::isfinite(value), "Finite dynamic state");
    }
    require(state.speed >= 0.0, "No reverse speed");
}

crp::apl::VehicleModel dynamicVehicle()
{
    crp::apl::VehicleModel result;
    result.settings.model = crp::apl::VehicleModelType::Dynamic;
    return result;
}

void testSpeedController()
{
    auto vehicle = dynamicVehicle();
    vehicle.state.speed = 10.0;
    crp::apl::SpeedController legacy;
    for (double target : {0.0, 9.5, 10.0, 10.5, 30.0})
    {
        require(legacy.accelerationTarget(vehicle, target, 0.01, 0.1) == vehicle.accelerationTarget(target),
            "Disabled PID additions preserve proportional controller exactly");
    }
    crp::apl::SpeedController integral;
    integral.settings.integral_gain = 0.5;
    integral.settings.integral_limit = 0.75;
    require(std::abs(integral.accelerationTarget(vehicle, 11.0, 1.0, 0.0) - 6.5) < 1e-12,
        "Integral accumulates speed error using simulation time");
    require(std::abs(integral.accelerationTarget(vehicle, 11.0, 1.0, 0.0) - 6.75) < 1e-12,
        "Integral contribution is bounded");
    integral.reset();
    for (int iteration = 0; iteration < 100; ++iteration)
    {
        integral.accelerationTarget(vehicle, 30.0, 0.01, 0.0);
    }
    require(integral.accelerationTarget(vehicle, 10.0, 0.01, 0.0) == 0.0,
        "Output saturation prevents integral windup");
    integral.accelerationTarget(vehicle, 11.0, 0.1, 0.0);
    require(integral.accelerationTarget(vehicle, 0.0, 0.1, 0.0) == -vehicle.settings.maximum_deceleration,
        "Stop command retains maximum braking");
    require(integral.accelerationTarget(vehicle, 10.0, 0.1, 0.0) == 0.0,
        "Stop command clears controller memory");
    crp::apl::SpeedController derivative;
    derivative.settings.derivative_gain = 2.0;
    derivative.settings.derivative_filter_time_constant = 0.0;
    require(derivative.accelerationTarget(vehicle, 10.0, 0.1, 0.0) == 0.0,
        "Derivative initializes without a kick");
    vehicle.state.speed = 10.1;
    require(std::abs(derivative.accelerationTarget(vehicle, 10.0, 0.1, 0.0) + 2.6) < 1e-12,
        "Derivative opposes increasing measured speed");
    require(std::abs(derivative.accelerationTarget(vehicle, 11.0, 0.1, 0.0) - 5.4) < 1e-12,
        "Target steps do not cause derivative kick");
    crp::apl::SpeedController filtered;
    filtered.settings.derivative_gain = 1.0;
    filtered.settings.derivative_filter_time_constant = 0.1;
    vehicle.state.speed = 10.0;
    filtered.accelerationTarget(vehicle, 10.0, 0.1, 0.0);
    vehicle.state.speed = 10.1;
    require(std::abs(filtered.accelerationTarget(vehicle, 10.0, 0.1, 0.0) + 0.6 +
        1.0 - std::exp(-1.0)) < 1e-12, "Measured derivative is low-pass filtered");
    crp::apl::SpeedController grade;
    grade.settings.road_gradient_feedforward_gain = 1.0;
    for (double pitch : {-0.1, 0.0, 0.1})
    {
        require(std::abs(grade.accelerationTarget(vehicle, vehicle.state.speed, 0.01, pitch) -
            9.81 * std::sin(pitch)) < 1e-12, "Grade feedforward sign and magnitude");
        auto tracking = dynamicVehicle();
        tracking.state.speed = 10.0;
        crp::apl::SpeedController controller;
        controller.settings.integral_gain = 2.0;
        controller.settings.derivative_gain = 0.2;
        controller.settings.road_gradient_feedforward_gain = 1.0;
        for (int iteration = 0; iteration < 8000; ++iteration)
        {
            tracking.stepAcceleration(controller.accelerationTarget(tracking, 10.0, 0.005, pitch),
                0.0, 0.005, pitch);
        }
        require(std::abs(tracking.state.speed - 10.0) < 1e-5,
            "PID rejects residual road loads on uphill and downhill grades");
        auto compensated = dynamicVehicle();
        compensated.state.speed = 10.0;
        compensated.settings.air_density = 0.0;
        compensated.settings.rolling_resistance_coefficient = 0.0;
        compensated.settings.viscous_resistance = 0.0;
        compensated.stepAcceleration(grade.accelerationTarget(compensated, 10.0, 0.005, pitch),
            0.0, 0.005, pitch);
        require(std::abs(compensated.state.speed - 10.0) < 1e-12,
            "Feedforward cancels physical gradient load without a speed error");
    }
}

int main(int argc, char ** argv)
{
    try
    {
    require(argc == 2, "Efficiency map path supplied to model test");
        testSpeedController();
        crp::apl::VehicleModel kinematic;
        kinematic.stepAcceleration(2.0, 0.1, 0.05);
        require(std::abs(kinematic.state.speed - 0.1) < 1e-12, "Legacy kinematic acceleration");
        require(kinematic.state.lateral_speed == 0.0, "Legacy rear-axle kinematic reference");
        require(std::abs(kinematic.state.yaw_rate - 0.1 * std::tan(0.1) / 2.7) < 1e-12,
            "Legacy kinematic yaw rate");
        crp::apl::VehicleModel path_follower;
        path_follower.step(10.0, 0.0, 0.05);
        require(path_follower.state.speed > 0.0 && path_follower.state.x > 0.0, "Longitudinal acceleration");
        path_follower.step(0.0, 0.0, 1.0);
        require(path_follower.state.speed == 0.0, "No reverse motion during braking");
        require(path_follower.followPath({{0.0, 0.0}, {0.0, 10.0}}, 0) > 0.0, "Steering at standstill");
        require(path_follower.followPath({}, 0) == 0.0, "Empty path handling");
        auto stationary = dynamicVehicle();
        for (int iteration = 0; iteration < 1000; ++iteration)
        {
            stationary.stepAcceleration(0.0, 0.6, 0.05);
            finite(stationary.state);
        }
        require(stationary.state.speed == 0.0 && stationary.state.x == 0.0 &&
            stationary.state.y == 0.0 && stationary.state.yaw_rate == 0.0,
            "Standstill remains stationary despite steering");
        auto coast = dynamicVehicle();
        coast.state.speed = 20.0;
        const double expected_resistance = (0.5 * 1.225 * 0.3 * 2.2 * 400.0 + 5.0 * 20.0) /
            1500.0 + 0.012 * 9.81;
        coast.stepAcceleration(0.0, 0.0, 0.0001);
        require(std::abs(coast.state.acceleration + expected_resistance) < 1e-8,
            "Aerodynamic, viscous and rolling resistance equation");
        auto uphill = dynamicVehicle();
        auto downhill = dynamicVehicle();
        uphill.state.speed = 10.0;
        downhill.state.speed = 10.0;
        uphill.stepAcceleration(0.0, 0.0, 0.005, 0.05);
        downhill.stepAcceleration(0.0, 0.0, 0.005, -0.05);
        require(std::abs((downhill.state.acceleration - uphill.state.acceleration) -
            2.0 * 9.81 * std::sin(0.05)) < 1e-8, "Grade-load sign and magnitude");
        for (double grade : {-0.1, 0.0, 0.1})
        {
            auto tracking = dynamicVehicle();
            tracking.state.speed = 10.0;
            tracking.state.pitch = grade;
            require(tracking.accelerationTarget(10.0) == 0.0 &&
                std::abs(tracking.accelerationTarget(9.5) + 3.0) < 1e-12,
                "Speed controller uses only speed error, not road grade or modeled loads");
            for (int iteration = 0; iteration < 80; ++iteration)
            {
                tracking.step(10.0, 0.0, 0.05, grade);
            }
            require(std::abs(tracking.state.speed - 10.0) < 0.25,
                "Proportional speed controller has bounded steady-state error under road loads");
            auto coarse = dynamicVehicle();
            coarse.state.speed = 12.0;
            auto fine = coarse;
            coarse.step(10.0, 0.0, 0.05, grade);
            for (int iteration = 0; iteration < 10; ++iteration)
            {
                fine.step(10.0, 0.0, 0.005, grade);
            }
            require(std::abs(coarse.state.speed - fine.state.speed) < 1e-10 &&
                std::abs(coarse.state.x - fine.state.x) < 1e-10,
                "Speed feedback is independent of publication interval");
            const double initial_speed = tracking.state.speed;
            tracking.step(9.5, 0.0, 0.05, grade);
            require(tracking.state.speed < initial_speed - 0.1,
                "Reduced speed target starts braking in the first tick");
        }
        auto transition = dynamicVehicle();
        for (int cycle = 0; cycle < 5; ++cycle)
        {
            for (int iteration = 0; iteration < 100; ++iteration)
            {
                transition.stepAcceleration(0.8, 0.25, 0.05);
                finite(transition.state);
                require(std::abs(transition.state.yaw_rate) < 2.0, "Bounded low-speed yaw rate");
            }
            for (int iteration = 0; iteration < 100; ++iteration)
            {
                transition.stepAcceleration(-3.0, 0.25, 0.05);
                finite(transition.state);
            }
            require(transition.state.speed == 0.0 && transition.state.lateral_speed == 0.0 &&
                transition.state.yaw_rate == 0.0, "Braking settles at rest without chatter or reverse");
        }
        auto steady = dynamicVehicle();
        steady.state.speed = 10.0;
        for (int iteration = 0; iteration < 4000; ++iteration) {steady.step(10.0, 0.04, 0.005);}
        const auto & settings = steady.settings;
        const double rear_distance = settings.wheelbase - settings.front_axle_distance;
        const double expected_yaw = 0.04 * steady.state.speed / (settings.wheelbase +
            settings.mass * steady.state.speed * steady.state.speed / settings.wheelbase *
            (rear_distance / settings.front_cornering_stiffness -
            settings.front_axle_distance / settings.rear_cornering_stiffness));
        require(std::abs(steady.state.yaw_rate - expected_yaw) < 0.02 * expected_yaw,
            "Dynamic steady turn agrees with linear bicycle solution");
        auto light = dynamicVehicle();
        auto heavy = dynamicVehicle();
        light.settings.yaw_inertia = 1200.0;
        heavy.settings.yaw_inertia = 5000.0;
        light.state.speed = heavy.state.speed = 10.0;
        light.stepAcceleration(0.0, 0.04, 0.1);
        heavy.stepAcceleration(0.0, 0.04, 0.1);
        require(light.state.yaw_rate > heavy.state.yaw_rate && heavy.state.yaw_rate > 0.0,
            "Yaw inertia controls transient turn response");
        auto large_step = dynamicVehicle();
        large_step.state.speed = 10.0;
        auto small_steps = large_step;
        large_step.stepAcceleration(0.2, 0.03, 1.0);
        for (int iteration = 0; iteration < 100; ++iteration)
        {
            small_steps.stepAcceleration(0.2, 0.03, 0.01);
        }
        require(std::abs(large_step.state.x - small_steps.state.x) < 1e-8 &&
            std::abs(large_step.state.yaw_rate - small_steps.state.yaw_rate) < 1e-8,
            "Substeps make dynamics independent of publication interval for fixed acceleration input");
        auto invalid = dynamicVehicle();
        invalid.settings.mass = 0.0;
        bool rejected = false;
        try {invalid.stepAcceleration(0.0, 0.0, 0.05);}
        catch (const std::invalid_argument &) {rejected = true;}
        require(rejected, "Invalid physical settings rejected");
        crp::apl::PowertrainModel powertrain(argv[1]);
        require(std::abs(powertrain.efficiency(4000.0 * 2.0 * std::acos(-1.0) / 60.0,
            100.0, false) - 0.95) < 1e-12, "Efficiency map exact grid lookup");
        require(std::abs(powertrain.efficiency(3000.0 * 2.0 * std::acos(-1.0) / 60.0,
            75.0, false) - 0.91) < 1e-12, "Efficiency map bilinear interpolation");
        require(powertrain.efficiency(4000.0 * 2.0 * std::acos(-1.0) / 60.0,
            100.0, true) > 0.0, "Separate regenerative efficiency map");
        powertrain.initialize(13.0);
        crp::apl::VehicleSettings powertrain_vehicle;
        double applied_force = 0.0;
        for (int iteration = 0; iteration < 100; ++iteration)
        {
            applied_force = powertrain.step(1.0, 13.0, 0.001, powertrain_vehicle);
        }
        require(applied_force > 0.0 && powertrain.state.energy_consumed > 0.0,
            "Powertrain builds wheel force and accumulates motoring energy");
        const double consumed_before_braking = powertrain.state.energy_consumed;
        for (int iteration = 0; iteration < 500; ++iteration)
        {
            powertrain.step(-3.0, 13.0, 0.001, powertrain_vehicle);
        }
        require(powertrain.state.energy_consumed >= consumed_before_braking &&
            powertrain.state.energy_regenerated >= 0.0,
            "Powertrain separates drawn and recovered energy");
        crp::apl::PowertrainModel actuated(argv[1]);
        auto actuated_vehicle = dynamicVehicle();
        actuated.initialize(0.0);
        for (int iteration = 0; iteration < 1200; ++iteration)
        {
            const double wheel_force = actuated.step(3.0, actuated_vehicle.state.speed,
                0.001, actuated_vehicle.settings);
            actuated_vehicle.stepWheelForce(wheel_force, 0.0, 0.001);
            actuated.state.vehicle_speed = actuated_vehicle.state.speed;
        }
        require(actuated_vehicle.state.speed > 2.0 && actuated.state.energy_consumed > 0.0,
            "Controller acceleration produces motion and energy through the actuator plant");
        std::cout << "Vehicle dynamics, loads, standstill, transitions, inertia and integration checks passed\n";
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}