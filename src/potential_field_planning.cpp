#include "potential_field_planning.hpp"
#include <iostream>
#include <stack>
#include <map>
#include <algorithm>

using namespace ROAR::global_planning;

PotentialFieldPlanning::PotentialFieldPlanning(uint64_t nx, uint64_t ny)
{
    this->nx = nx;
    this->ny = ny;
    this->max_iter = nx * ny;
    this->map_ = std::make_shared<std::vector<float>>(nx * ny);
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
    map_ = std::make_shared<std::vector<float>>(std::vector<float>(obstacle_map.begin(), obstacle_map.end()));
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

PotentialFieldPlanning::PotentialFieldPlanningResult PotentialFieldPlanning::plan(const PotentialFieldPlanning::PotentialFieldPlanningInput::SharedPtr input)
{
    PotentialFieldPlanningResult result;
    result.status = false;

    // check if goal is within bound
    uint64_t gx = std::get<0>(input->goal);
    uint64_t gy = std::get<1>(input->goal);
    if (gx < 0 || gx >= nx || gy < 0 || gy >= ny)
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

    // generate costmap
    std::shared_ptr<std::vector<float>> costmap = p_generateCostMapFromGoal(input->goal);
    result.costmap = costmap;

    // greedy search
    std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path = std::make_shared<std::vector<std::tuple<uint64_t, uint64_t>>>();

    p_greedySearch(path, *start, input->goal, max_iter, nx, ny, costmap);

    result.path = path;
    result.status = true;
    return result;
}

bool PotentialFieldPlanning::p_greedySearch(std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path,
                                            std::tuple<uint64_t, uint64_t> start, std::tuple<uint64_t, uint64_t> goal,
                                            uint64_t max_iter, uint64_t nx, uint64_t ny,
                                            const std::shared_ptr<std::vector<float>> costmap)
{
    // check if start is within bounds
    uint64_t x = std::get<0>(start);
    uint64_t y = std::get<1>(start);
    if (x < 0 || x >= nx || y < 0 || y >= ny)
    {
        return false;
    }

    // check if goal is within bounds
    uint64_t gx = std::get<0>(goal);
    uint64_t gy = std::get<1>(goal);
    if (gx < 0 || gx >= nx || gy < 0 || gy >= ny)
    {
        return false;
    }

    // check if costmap is valid
    if (costmap == nullptr)
    {
        return false;
    }

    // check if nx*ny == length of costmap
    if (nx * ny != costmap->size())
    {
        return false;
    }

    // check if path is valid
    if (path == nullptr)
    {
        return false;
    }

    if (start == goal)
    {
        path->push_back(start);
        return true;
    }

    // clear path
    path->clear();

    // dfs to goal
    std::stack<uint64_t> stack; // to visit stack
    stack.push(this->get_index(start));

    // visited map
    std::vector<bool> visited = std::vector<bool>(nx * ny, false);

    // parent mapping
    std::map<uint64_t, uint64_t> parent;

    uint64_t iter = 0;
    while (!stack.empty())
    {
        if (iter >= max_iter)
        {
            return false;
        }

        int64_t index = stack.top();
        stack.pop();

        uint64_t x = index % nx;
        uint64_t y = index / nx;

        if (std::make_tuple(x, y) == goal)
        {
            // Reconstruct the path by backtracking through the parent mapping.
            std::tuple<uint64_t, uint64_t> coord = std::make_tuple(x, y);
            while (coord != start)
            {
                path->push_back(coord);
                uint64_t parent_index = parent[index];
                coord = getCoordFromIndex(parent_index);
                index = parent_index;
            }
            path->push_back(start);
            std::reverse(path->begin(), path->end()); // Reverse the path to start from the start point.
            return true;                              // Goal found.
        }

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                {
                    continue; // Skip the current cell.
                }

                uint64_t new_x = x + dx;
                uint64_t new_y = y + dy;

                if (new_x >= 0 && new_x < nx && new_y >= 0 && new_y < ny)
                {
                    uint64_t new_index = get_index(std::make_tuple(new_x, new_y));

                    // if not visited and have lower cost than now
                    if (!visited[new_index] && (*costmap)[new_index] < (*costmap)[index])
                    {
                        stack.push(new_index);
                        parent[new_index] = index;
                    }
                }
            }
        }

        iter++;
    }

    return false;
}