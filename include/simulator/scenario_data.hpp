#ifndef SIMULATOR_SCENARIO_DATA_HPP
#define SIMULATOR_SCENARIO_DATA_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace crp
{
namespace apl
{

struct MapPoint
{
    double x{0.0};
    double y{0.0};
};

struct TrajectorySample
{
    std::vector<MapPoint> points;
    double confidence{1.0};
};

struct MapCrossing
{
    std::vector<MapPoint> centerline;
    double width{0.0};
};

struct ScenarioActor
{
    unsigned int classification{0};
    bool obstacle{false};
    std::vector<MapPoint> positions;
    std::vector<double> speeds;
    std::vector<double> headings;
    std::vector<double> existence_probabilities;
    std::vector<double> main_confidences;
    std::vector<std::vector<TrajectorySample>> alternatives;
    std::vector<double> dimensions;
};

class ScenarioData
{
public:
    static ScenarioData load(const std::string & filename, double prediction_horizon,
        bool trim_zero_padding = true);
    std::size_t timeIndex(double elapsed) const;
    double duration() const;

    std::vector<double> time;
    double time_step{0.05};
    std::vector<MapPoint> target_path;
    std::vector<std::vector<MapPoint>> lanes;
    std::vector<double> speed_limits;
    std::vector<ScenarioActor> actors;
    std::vector<MapCrossing> pedestrian_crossings;
};

double pathHeading(const std::vector<MapPoint> & points, std::size_t index);

}
}

#endif