#include "potential_field_planning.hpp"

using namespace ROAR::global_planning;

PotentialFieldPlanning::PotentialFieldPlanning(uint64_t nx, uint64_t ny)
{
    this->nx = nx;
    this->ny = ny;
    this->max_iter = nx * ny;
    this->map_ = std::make_shared<std::vector<uint8_t>>(nx * ny);
}

PotentialFieldPlanning::~PotentialFieldPlanning()
{
}

void PotentialFieldPlanning::setObstacleCoords(std::vector<std::tuple<uint64_t, uint64_t>> obstacle_map, uint64_t bounds, float gradient)
{
    // for every x, y, calculate its position on the map, and set the value to 1
    for (auto &point : obstacle_map)
    {
        uint64_t x = std::get<0>(point);
        uint64_t y = std::get<1>(point);
        uint64_t index = x + y * nx;
        // check if index is within bound
        if (index < 0 || index >= nx * ny)
        {
            continue;
        }
        (*map_)[index] = 100;
    }
}

void PotentialFieldPlanning::setObstacles(std::vector<uint8_t> obstacle_map, uint64_t bounds, float gradient)
{
    // check if obstacle_map is same size as map
    if (obstacle_map.size() != nx * ny)
    {
        throw std::invalid_argument("obstacle_map size does not match map size");
    }

    // set map
    map_ = std::make_shared<std::vector<uint8_t>>(obstacle_map);
}

bool PotentialFieldPlanning::setStart(std::tuple<uint64_t, uint64_t> start)
{
    // check if start is valid
    // check if start is within bounds
    uint64_t x = std::get<0>(start);
    uint64_t y = std::get<1>(start);

    if (x < 0 || x >= nx || y < 0 || y >= ny)
    {
        return false;
    }
    this->start = std::make_shared<std::tuple<uint64_t, uint64_t>>(start);

    return true;
}

ROAR::global_planning::PotentialFieldPlanResult PotentialFieldPlanning::plan(std::tuple<uint64_t, uint64_t> goal, uint64_t max_iter)
{
    PotentialFieldPlanResult result;
    result.status = false;

    // check if goal is within bound
    uint64_t x = std::get<0>(goal);
    uint64_t y = std::get<1>(goal);
    if (x < 0 || x >= nx || y < 0 || y >= ny)
    {
        return result;
    }

    // check if map exist
    if (map_ == nullptr)
    {
        return result;
    }

    // check if start exist
    if (start == nullptr)
    {
        return result;
    }

    // iteratively find path from start to goal while avoiding obstacles
    std::vector<std::tuple<uint64_t, uint64_t>> path;
    x = std::get<0>(*start);
    y = std::get<1>(*start);
    path.push_back(*start);

    uint64_t iter = 0;
    while (iter < max_iter) {
        // check if goal is reached
        if (x == std::get<0>(goal) && y == std::get<1>(goal))
        {
            result.status = true;
            result.path = path;
            return result;
        }

        // check if x, y is within bound
        if (x < 0 || x >= nx || y < 0 || y >= ny)
        {
            return result;
        }

        // do greedy search toward goal while avoiding obstacles
    }


    result.status = true;
    return result;
}
