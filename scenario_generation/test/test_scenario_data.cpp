#include "scenario_generation/scenario_data.hpp"
#include "scenario_generation/scenario_playback.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>

void require(bool condition, const std::string & message)
{
    if (!condition) {throw std::runtime_error(message);}
}

void testOcclusion()
{
    crp::apl::ScenarioData hidden_scene;
    hidden_scene.time = {0.0, 0.05, 0.1};
    hidden_scene.target_path = {{0.0, 0.0}, {30.0, 0.0}};
    hidden_scene.speed_limits = {10.0, 10.0};
    hidden_scene.lanes = {hidden_scene.target_path};
    crp::apl::ScenarioActor blocker;
    blocker.obstacle = true;
    blocker.positions = {{10.0, 0.0}, {10.0, 0.0}, {10.0, 0.0}};
    blocker.dimensions = {4.0, 2.0, 2.0};
    blocker.headings = {0.0, 0.0, 0.0};
    blocker.speeds = {0.0, 0.0, 0.0};
    blocker.existence_probabilities = {1.0, 1.0, 1.0};
    blocker.main_confidences = {1.0, 1.0, 1.0};
    blocker.alternatives = {{}, {}, {}};
    auto hidden_pedestrian = blocker;
    hidden_pedestrian.obstacle = false;
    hidden_pedestrian.classification = 7;
    hidden_pedestrian.dimensions = {0.6, 0.6, 1.8};
    hidden_pedestrian.positions = {{20.0, 0.0}, {20.0, 5.0}, {20.0, 5.0}};
    hidden_pedestrian.existence_probabilities = {0.2, 0.2, 0.2};
    hidden_scene.actors = {blocker, hidden_pedestrian};
    crp::apl::PlaybackSettings settings;
    settings.generate_virtual_objects = false;
    const crp::apl::ScenarioPlayback visibility(hidden_scene, settings);
    const auto hidden_message = visibility.message(crp::apl::ScenarioEgoState(), 0.0);
    require(!hidden_message.occluded_regions.empty(), "Large obstacle casts an ego-view shadow");
    require(hidden_message.local_obstacles.objects.size() == 1, "Occluder does not hide itself");
    require(hidden_message.local_moving_objects.objects.empty(), "Hidden pedestrian is not published");
    const auto emerged_message = visibility.message(crp::apl::ScenarioEgoState(), hidden_scene.time_step);
    require(emerged_message.local_moving_objects.objects.size() == 1, "Pedestrian is detected outside shadow");
    require(emerged_message.local_moving_objects.objects.front().existence_probability == 1.0F,
        "Emerged pedestrian existence is 100 percent");
    auto reversed = hidden_scene;
    for (auto & actor : reversed.actors)
    {
        for (auto & point : actor.positions) {point.x = -point.x; point.y = -point.y;}
        for (auto & heading : actor.headings) {heading += std::acos(-1.0);}
    }
    const auto rear_message = crp::apl::ScenarioPlayback(reversed, settings).message(crp::apl::ScenarioEgoState(), 0.0);
    require(rear_message.local_moving_objects.objects.empty(), "Rear-facing shadow handles angle wrap at pi");
    require(!rear_message.occluded_regions.empty(), "Rear occluder casts a shadow");
    for (const auto & region : rear_message.occluded_regions)
    {
        for (const auto & point : region.points) {require(point.x < 0.0, "Rear shadow does not span the front hemisphere");}
    }
    auto rotated = hidden_scene;
    rotated.actors.front().headings = {0.7, 0.7, 0.7};
    require(crp::apl::ScenarioPlayback(rotated, settings).message(crp::apl::ScenarioEgoState(), 0.0).
        local_moving_objects.objects.empty(), "Oriented footprint hides a pedestrian behind a rotated obstacle");
    auto overlapping = hidden_scene;
    auto hidden_blocker = blocker;
    hidden_blocker.positions = {{16.0, 0.0}, {16.0, 0.0}, {16.0, 0.0}};
    overlapping.actors.push_back(hidden_blocker);
    const auto overlap_message = crp::apl::ScenarioPlayback(overlapping, settings).message(crp::apl::ScenarioEgoState(), 0.0);
    require(overlap_message.local_obstacles.objects.size() == 1, "Occluded obstacle is filtered as well as moving actors");
    require(overlap_message.occluded_regions.size() == hidden_message.occluded_regions.size(),
        "Hidden occluders do not leak additional shadow geometry");
    auto inside_ego = crp::apl::ScenarioEgoState();
    inside_ego.x = 10.0;
    require(visibility.message(inside_ego, 0.0).occluded_regions.empty(),
        "Ego inside a footprint does not create singular shadow rays");
    settings.enable_occlusion = false;
    require(crp::apl::ScenarioPlayback(hidden_scene, settings).message(
        crp::apl::ScenarioEgoState(), 0.0).local_moving_objects.objects.size() == 1, "Occlusion can be disabled");
    settings.enable_occlusion = true;
    settings.generate_virtual_objects = true;
    hidden_scene.lanes = {{{30.0, -2.0}, {15.0, -2.0}, {0.0, -5.0}}};
    crp::apl::MapCrossing crossing;
    crossing.centerline = {{20.0, -6.0}, {20.0, 6.0}};
    crossing.width = 3.0;
    hidden_scene.pedestrian_crossings = {crossing};
    const auto virtual_scene = crp::apl::ScenarioPlayback(hidden_scene, settings).message(crp::apl::ScenarioEgoState(), 0.0);
    require(!virtual_scene.virtual_object_ids.empty(), "Occluded geometry generates hypothetical actors");
    bool has_crossing_pedestrian = false;
    bool has_lane_vehicle = false;
    for (const auto & object : virtual_scene.local_moving_objects.objects)
    {
        require(object.object_id.uuid[15] == 0xF0, "Hidden real actor is not leaked as a virtual actor");
        require(object.kinematics.predicted_paths.size() == 2, "Virtual actors have two conditional branches");
        const auto & branches = object.kinematics.predicted_paths;
        require(std::abs(branches[0].confidence + branches[1].confidence - 1.0) < 1e-6,
            "Virtual branch probabilities sum to one independently of existence");
        if (object.classification.front().label == 7 && object.object_id.uuid[9] == 2)
        {
            has_crossing_pedestrian = true;
            require(object.existence_probability == 0.5F, "Crossing pedestrian existence follows MATLAB prior");
            require(std::abs(branches[0].confidence - 0.9F) < 1e-6, "Walking is more likely than running");
            require(std::abs(branches[0].path.front().position.x - 20.0) < 1e-6,
                "Pedestrian predictions follow crossing geometry");
        }
        if (object.classification.front().label == 1)
        {
            has_lane_vehicle = true;
            require(std::abs(branches[0].confidence - 0.7F) < 1e-6, "Vehicle speed mixture follows MATLAB");
            require(branches[0].path.back().position.x < branches[0].path.front().position.x,
                "Vehicle travels in lane direction, not ego direction");
            require(std::abs(branches[0].path.back().position.y - branches[0].path.front().position.y) > 1.0,
                "Vehicle follows curved lane geometry instead of a straight heading extrapolation");
        }
    }
    require(has_crossing_pedestrian, "Hidden crossing creates a possible pedestrian");
    require(has_lane_vehicle, "Hidden lane creates a possible vehicle");
    hidden_scene.actors.pop_back();
    const auto without_hidden_actor = crp::apl::ScenarioPlayback(hidden_scene, settings).message(
        crp::apl::ScenarioEgoState(), 0.0);
    require(without_hidden_actor.virtual_object_ids == virtual_scene.virtual_object_ids,
        "Hypotheses do not depend on hidden ground truth");
    hidden_scene.pedestrian_crossings.clear();
    const auto roadside_scene = crp::apl::ScenarioPlayback(hidden_scene, settings).message(crp::apl::ScenarioEgoState(), 0.0);
    bool has_roadside_pedestrian = false;
    for (const auto & object : roadside_scene.local_moving_objects.objects)
    {
        if (object.classification.front().label == 7)
        {
            has_roadside_pedestrian = true;
            require(std::abs(object.existence_probability - 0.02F) < 1e-6, "Roadside pedestrian has a low prior");
        }
    }
    require(has_roadside_pedestrian, "Occluded roadside generates a low-probability pedestrian");
    settings.maximum_virtual_objects = 1;
    require(crp::apl::ScenarioPlayback(hidden_scene, settings).message(crp::apl::ScenarioEgoState(), 0.0).
        virtual_object_ids.size() == 1, "Virtual hypothesis count is bounded");
    settings.crossing_pedestrian_probability = 0.0;
    settings.roadside_pedestrian_probability = 0.0;
    settings.virtual_vehicle_probability = 0.0;
    require(crp::apl::ScenarioPlayback(hidden_scene, settings).message(crp::apl::ScenarioEgoState(), 0.0).
        virtual_object_ids.empty(), "Zero priors suppress hypothetical actors");
    settings.virtual_vehicle_probability = 1.1;
    bool rejected_probability = false;
    try {crp::apl::ScenarioPlayback invalid(hidden_scene, settings);}
    catch (const std::runtime_error &) {rejected_probability = true;}
    require(rejected_probability, "Invalid virtual probabilities are rejected");
}

void testScenarioRange()
{
    crp::apl::ScenarioData data;
    data.time = {0.0, 0.05};
    for (int position = 0; position <= 300; position += 25)
    {
        data.target_path.push_back({static_cast<double>(position), 0.0, 0.0});
        data.speed_limits.push_back(position < 150 ? 130.0 / 3.6 : 10.0);
    }
    data.lanes = {data.target_path};
    auto settings = crp::apl::PlaybackSettings();
    settings.scenario_distance = 40.0;
    settings.scenario_lookahead_time = 6.0;
    settings.enable_occlusion = false;
    crp::apl::ScenarioPlayback ranged(data, settings);
    crp::apl::ScenarioEgoState ego;
    require(std::abs(ranged.scenarioDistance(ego) - 130.0 / 3.6 * 6.0) < 1e-10,
        "Scenario range follows MAT target speed, not current standstill speed");
    require(ranged.message(ego, 0.0).paths.front().path.points.back().point.pose.position.x >= 200.0,
        "Highway road publication covers six seconds of target travel");
    ego.x = 200.0;
    require(std::abs(ranged.scenarioDistance(ego) - 60.0) < 1e-10,
        "Scenario range changes with the local MAT speed limit");
    settings.scenario_lookahead_time = 0.0;
    require(std::abs(crp::apl::ScenarioPlayback(data, settings).scenarioDistance(ego) - 40.0) < 1e-10,
        "Zero lookahead retains fixed-distance behavior");
    settings.scenario_lookahead_time = 6.0;
    settings.scenario_distance = 100.0;
    require(std::abs(crp::apl::ScenarioPlayback(data, settings).scenarioDistance(ego) - 100.0) < 1e-10,
        "Minimum scenario distance is retained at lower target speed");
}

void testRouteDisplacement()
{
    const std::vector<crp::apl::MapPoint> route = {
        {0.0, 0.0, 0.0}, {3.0, 0.0, 0.3}, {3.0, 0.0, 0.3}, {3.0, 4.0, 0.7}};
    const auto start = crp::apl::pathPoseAtDistance(route, 0.0);
    require(std::abs(start.point.x) < 1e-12 && std::abs(start.heading) < 1e-12,
        "Zero displacement starts at route origin and tangent");
    const auto on_first = crp::apl::pathPoseAtDistance(route, 1.5);
    require(std::abs(on_first.point.x - 1.5) < 1e-12 && std::abs(on_first.point.z - 0.15) < 1e-12 &&
        std::abs(on_first.heading) < 1e-12, "Displacement interpolates XYZ and first-segment heading");
    const auto on_turn = crp::apl::pathPoseAtDistance(route, 5.0);
    require(std::abs(on_turn.point.x - 3.0) < 1e-12 && std::abs(on_turn.point.y - 2.0) < 1e-12 &&
        std::abs(on_turn.point.z - 0.5) < 1e-12 &&
        std::abs(on_turn.heading - 0.5 * std::acos(-1.0)) < 1e-12,
        "Displacement follows route order through bend and duplicate waypoint");
    const auto end = crp::apl::pathPoseAtDistance(route, 7.0);
    require(std::abs(end.point.y - 4.0) < 1e-12, "Displacement reaches route endpoint");
    bool rejected = false;
    try {static_cast<void>(crp::apl::pathPoseAtDistance(route, 7.1));}
    catch (const std::out_of_range &) {rejected = true;}
    require(rejected, "Displacement beyond target path is rejected");
}

int main(int argc, char ** argv)
{
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--range")
        {
            testScenarioRange();
            testRouteDisplacement();
            std::cout << "Scenario range and route displacement checks passed\n";
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--occlusion")
        {
            testOcclusion();
            std::cout << "Occlusion checks passed\n";
            return 0;
        }
        if (argc == 3 && std::string(argv[1]) == "--load")
        {
            const auto data = crp::apl::ScenarioData::load(argv[2], 4.0);
            require(data.speed_limits.size() == data.target_path.size(),
                "Trailing repeated speed values are trimmed to target-path length");
            std::cout << "Scenario load checks passed\n";
            return 0;
        }
        require(argc == 2, "Pass the scenario_files directory");
        const std::string directory = argv[1];
        if (std::ifstream(directory + "/highway_entry_costmap_data.mat").good())
        {
            const auto elevated = crp::apl::ScenarioData::load(directory + "/highway_entry_costmap_data.mat", 4.0);
            require(elevated.has_elevation, "Nx3 highway recognized");
            require(elevated.target_path.size() == 3002 && elevated.lanes.size() == 7, "Elevated highway topology");
            require(std::abs(elevated.target_path.back().z - 250.0) < 1e-6, "Highway target z retained");
            require(elevated.actors.size() == 7, "Elevated actor count");
            require(elevated.actors.front().positions.front().z > 0.0, "Actor z retained");
        }
        auto pedestrian = crp::apl::ScenarioData::load(
            directory + "/occluded_pedestrian_costmap_data.mat", 4.0);
        auto ramp = pedestrian;
        ramp.target_path = {{0.0, 0.0, 10.0}, {10.0, 0.0, 11.0}, {20.0, 0.0, 12.0}};
        ramp.speed_limits = {10.0, 10.0, 10.0};
        ramp.lanes = {ramp.target_path};
        ramp.has_elevation = true;
        ramp.actors.clear();
        ramp.pedestrian_crossings.clear();
        testScenarioRange();
        auto ramp_actor = pedestrian.actors.front();
        ramp_actor.positions = {{5.0, 0.0, 10.5}, {6.0, 0.0, 10.6}, {7.0, 0.0, 10.7}};
        ramp_actor.headings = {0.0, 0.0, 0.0};
        ramp_actor.main_confidences = {0.7, 0.7, 0.7};
        ramp_actor.alternatives.clear();
        ramp_actor.alternatives.resize(3);
        crp::apl::TrajectorySample elevated_branch;
        elevated_branch.confidence = 0.3;
        elevated_branch.points = {{5.0, 0.0, 11.5}, {6.0, 0.0, 11.6}};
        ramp_actor.alternatives[0].push_back(elevated_branch);
        ramp.actors.push_back(ramp_actor);
        crp::apl::MapCrossing ramp_crossing;
        ramp_crossing.centerline = {{5.0, -2.0, 10.5}, {5.0, 2.0, 10.5}};
        ramp.pedestrian_crossings.push_back(ramp_crossing);
        crp::apl::ScenarioPlayback ramp_playback(ramp, crp::apl::PlaybackSettings());
        crp::apl::ScenarioEgoState ramp_ego;
        ramp_ego.x = 5.0;
        const auto road = ramp_playback.roadState(ramp_ego);
        require(std::abs(road.elevation - 10.5) < 1e-10, "Road height interpolated between waypoints");
        require(std::abs(road.pitch - std::atan(0.1)) < 1e-10, "Gradient from road elevation");
        ramp_ego.z = road.elevation;
        ramp_ego.pitch = road.pitch;
        const auto local_ramp = ramp_playback.message(ramp_ego, 0.0);
        require(std::abs(local_ramp.paths.front().path.points.front().point.pose.position.z) < 1e-9,
            "Full pitched ego transform makes straight ramp locally level");
        require(std::abs(local_ramp.paths.front().path.left_bound.front().z) < 1e-9,
            "Lane boundary elevation retained through 3D transform");
        const auto & local_actor = local_ramp.local_moving_objects.objects.front();
        require(std::abs(local_actor.kinematics.initial_pose_with_covariance.pose.position.z) < 1e-9,
            "Elevated actor converted into ego ground plane");
        const auto & branches = local_actor.kinematics.predicted_paths;
        require(branches.size() == 2, "Main and elevated alternative predictions retained");
        require(std::abs(branches[0].path.back().position.z) < 1e-9, "Main prediction follows elevated road");
        require(std::abs(branches[1].path.front().position.z - std::cos(road.pitch)) < 1e-9,
            "Alternative prediction retains its independent height");
        require(std::abs(local_ramp.map_features.pedestrian_crossings.front().centerline.front().z) < 1e-9,
            "Crossing height retained in pitched frame");
        ramp_ego.yaw = std::acos(-1.0);
        require(std::abs(ramp_playback.roadState(ramp_ego).pitch + road.pitch) < 1e-10,
            "Grade sign follows ego travel heading");
        require(pedestrian.actors.size() == 3, "Pedestrian scenario actor count");
        require(pedestrian.lanes.size() == 6, "Lane topology");
        require(pedestrian.pedestrian_crossings.size() == 2, "MAT pedestrian crossings loaded");
        require(pedestrian.pedestrian_crossings.front().centerline.size() == 116, "Crossing samples");
        auto crossing_settings = crp::apl::PlaybackSettings();
        crossing_settings.scenario_distance = 200.0;
        crp::apl::ScenarioPlayback crossing_playback(pedestrian, crossing_settings);
        crp::apl::ScenarioEgoState crossing_ego;
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
        crp::apl::ScenarioEgoState ego;
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
        require(conditional_object.existence_probability == 1.0F, "Visible ground truth is a certain detection");
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
        testOcclusion();
        std::cout << "All three MAT scenarios passed loader checks\n";
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}