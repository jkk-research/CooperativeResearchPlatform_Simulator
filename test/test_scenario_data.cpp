#include "simulator/scenario_data.hpp"
#include "simulator/scenario_playback.hpp"
#include "simulator/vehicle_model.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

void require(bool condition, const std::string & message)
{
    if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
    try
    {
        require(argc == 2, "Pass the scenario_files directory");
        const std::string directory = argv[1];
        auto pedestrian = crp::apl::ScenarioData::load(
            directory + "/occluded_pedestrian_costmap_data.mat", 4.0);
        require(pedestrian.actors.size() == 3, "Pedestrian scenario actor count");
        require(pedestrian.lanes.size() == 6, "Lane topology");
        require(pedestrian.pedestrian_crossings.size() == 2, "MAT pedestrian crossings loaded");
        require(pedestrian.pedestrian_crossings.front().centerline.size() == 116, "Crossing samples");
        auto crossing_settings = crp::apl::PlaybackSettings();
        crossing_settings.scenario_distance = 200.0;
        crp::apl::ScenarioPlayback crossing_playback(pedestrian, crossing_settings);
        crp::apl::VehicleState crossing_ego;
        crossing_ego.x = 50.0;
        crossing_ego.y = 2.0;
        crossing_ego.yaw = std::atan2(1.0, 0.0);
        const auto crossing_message = crossing_playback.message(crossing_ego, 0.0);
        require(!crossing_message.map_features.pedestrian_crossings.empty(), "Crossings published in local range");
        const auto & local_crossing = crossing_message.map_features.pedestrian_crossings.front();
        require(crossing_message.paths.front().traffic_rules.applicable_map_feature_ids.empty(),
            "No path association inferred from crossing geometry alone");
        require(std::abs(local_crossing.width - 3.0F) < 1e-6, "Default crossing width");
        require(std::abs(local_crossing.centerline.front().y + 50.0) < 1e-6, "Crossing frame rotation");
        require(pedestrian.timeIndex(0.0) == 0, "Playback starts at zero");
        require(pedestrian.timeIndex(5.0) == 100, "Time sample indexing");
        require(pedestrian.timeIndex(500.0) == 1800, "End-of-file indexing");
        require(std::abs(pedestrian.speed_limits.front() - 50.0 / 3.6) < 1e-8, "Speed units");
        const auto & actor = pedestrian.actors.front();
        require(actor.existence_probabilities[100] == 1.0, "Existence is independent of trajectory confidence");
        require(actor.main_confidences[100] < 1.0, "Main confidence retained");
        require(actor.main_confidences.back() == actor.main_confidences[1797], "Short confidence tail padded");
        require(actor.alternatives[100].size() == 1, "Pedestrian alternative count");
        require(actor.alternatives[100][0].points.size() == 81, "Prediction horizon");
        require(std::abs(actor.alternatives[100][0].confidence - 0.21911037) < 1e-7,
            "First future-point confidence at observation 100");
        require(std::abs(actor.alternatives[100][0].points.front().x - 100.0) < 1e-8,
            "Prediction matrix observation column");
        require(std::abs(actor.headings[100] - std::atan2(1.0, 0.0)) < 1e-6, "Pedestrian heading");
        auto crossing = crp::apl::ScenarioData::load(
            directory + "/occluded_pedestrian_crossing_costmap_data.mat", 4.0);
        require(crossing.actors.size() == 2, "Crossing scenario contains car and obstacle, no pedestrian");
        auto roundabout = crp::apl::ScenarioData::load(
            directory + "/roundabout_with_conflicting_traffic_costmap_data.mat", 4.0);
        require(roundabout.lanes.size() == 24, "Roundabout lane topology");
        require(roundabout.actors.front().classification == 0,
            "Missing vehicle classification must not invent a car label");
        require(roundabout.actors.front().dimensions[0] == 10.5 &&
            roundabout.actors.front().dimensions[1] == 2.5 &&
            roundabout.actors.front().dimensions[2] == 4.5,
            "Large source dimensions preserved independently of classification");
        require(roundabout.actors.front().alternatives[100].size() == 2, "Roundabout alternatives");
        require(std::abs(roundabout.actors.front().alternatives[100][0].confidence - 0.5) < 1e-8,
            "Branch confidence is not multiplied by existence");
        require(std::isfinite(crp::apl::pathHeading(roundabout.target_path, 0)), "Derived path heading");
        crp::apl::PlaybackSettings settings;
        settings.object_distance = 500.0;
        crp::apl::ScenarioPlayback playback(std::move(roundabout), settings);
        crp::apl::VehicleState ego;
        ego.x = 10.0;
        ego.y = 20.0;
        ego.yaw = std::atan2(1.0, 0.0);
        const auto message = playback.message(ego, 5.0);
        require(message.header.frame_id == "base_link", "Scenario frame");
        require(!message.paths.empty(), "Target path is first");
        require(std::abs(message.paths.front().traffic_rules.maximum_speed - 50.0 / 3.6) < 1e-5,
            "Published speed limit");
        require(message.local_moving_objects.objects.size() == 1, "Published actor count");
        const auto & object = message.local_moving_objects.objects.front();
        require(object.existence_probability == 1.0F, "Published detection confidence");
        require(object.kinematics.predicted_paths.size() == 3, "Main and two alternative paths");
        double sum = 0.0;
        for (const auto & prediction : object.kinematics.predicted_paths)
        {
            sum += prediction.confidence;
            require(prediction.time_step.sec == 0 && prediction.time_step.nanosec == 50000000,
                "Nonzero 50 ms prediction step");
            require(prediction.path.size() == 81, "Published prediction horizon");
            require(std::isfinite(prediction.path.front().orientation.w), "Prediction orientation");
        }
        require(std::abs(sum - 1.0) < 1e-6, "Conditional trajectory probabilities normalized");
        auto conditional_data = playback.data();
        auto & conditional_actor = conditional_data.actors.front();
        conditional_actor.existence_probabilities[100] = 0.5;
        conditional_actor.main_confidences[100] = 0.3;
        conditional_actor.alternatives[100][0].confidence = 0.6;
        conditional_actor.alternatives[100][1].confidence = 0.1;
        const auto conditional_message = crp::apl::ScenarioPlayback(
            std::move(conditional_data), settings).message(ego, 5.0);
        const auto & conditional_object = conditional_message.local_moving_objects.objects.front();
        require(conditional_object.existence_probability == 0.5F, "Independent 50 percent existence");
        const auto & conditional_paths = conditional_object.kinematics.predicted_paths;
        require(std::abs(conditional_paths[0].confidence - 0.3) < 1e-6 &&
            std::abs(conditional_paths[1].confidence - 0.6) < 1e-6 &&
            std::abs(conditional_paths[2].confidence - 0.1) < 1e-6,
            "30/60/10 branch probabilities remain conditional on existence");
        const auto & global = playback.data().actors.front().positions[100];
        const auto & local = object.kinematics.initial_pose_with_covariance.pose.position;
        require(std::abs(local.x - (global.y - ego.y)) < 1e-7, "Map-to-ego x transform");
        require(std::abs(local.y + (global.x - ego.x)) < 1e-7, "Map-to-ego y transform");
        const auto & first = message.paths.front();
        require(!first.path.points.empty(), "Local target corridor");
        const auto & left = first.path.left_bound.front();
        const auto & right = first.path.right_bound.front();
        require(std::abs(std::hypot(left.x - right.x, left.y - right.y) - settings.target_path_width) < 1e-6,
            "Rotated corridor boundary width");
        crp::apl::VehicleModel vehicle;
        vehicle.step(10.0, 0.0, 0.05);
        require(vehicle.state.speed > 0.0 && vehicle.state.x > 0.0, "Longitudinal acceleration");
        vehicle.step(0.0, 0.0, 1.0);
        require(vehicle.state.speed == 0.0, "No reverse motion during braking");
        require(vehicle.followPath({{0.0, 0.0}, {0.0, 10.0}}, 0) > 0.0, "Steering at standstill");
        require(vehicle.followPath({}, 0) == 0.0, "Empty path handling");
        std::cout << "All three MAT scenarios passed loader checks\n";
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}