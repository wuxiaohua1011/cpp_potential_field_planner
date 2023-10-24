#include <iostream>
#include "src/potential_field_planning.hpp"

void p_printMap(std::vector<float> map, uint64_t nx, uint64_t ny)
{
    for (int i = 0; i < map.size(); i++)
    {
        // print with 2 decimal points
        std::cout << std::fixed;
        std::cout.precision(2);

        std::cout << map[i] << " ";
        if ((i + 1) % nx == 0)
        {
            std::cout << std::endl;
        }
    }
    std::cout << "-------" << std::endl;
}

int main(int, char **)
{
    uint64_t nx = 10;
    uint64_t ny = 10;
    ROAR::global_planning::PotentialFieldPlanning planner(10, 10);

    // print map before
    std::cout << "Map BEFORE: " << std::endl;
    auto map = planner.get_map();
    p_printMap(map, nx, ny);

    planner.setStart(std::make_tuple(0, 0));

    // set obstacles
    std::vector<std::tuple<uint64_t, uint64_t>> obstacle_map = {
        std::make_tuple(1, 1),
        std::make_tuple(2, 2),
        std::make_tuple(3, 3),
        std::make_tuple(4, 4),
        std::make_tuple(5, 5)};
    planner.setObstacleCoords(obstacle_map, 100, 1);

    // print map after
    std::cout << "Map With Obstacle: " << std::endl;
    map = planner.get_map();
    p_printMap(map, nx, ny);

    ROAR::global_planning::PotentialFieldPlanning::PotentialFieldPlanningInput input;
    input.goal = std::make_tuple(9, 9);
    input.max_iter = nx * ny;
    ROAR::global_planning::PotentialFieldPlanning::PotentialFieldPlanningResult result = planner.plan(
        std::make_shared<ROAR::global_planning::PotentialFieldPlanning::PotentialFieldPlanningInput>(input));

    // print map after
    if (result.costmap != nullptr)
    {
        std::cout << "Map With Distance to Goal: " << std::endl;
        map = planner.get_map();
        p_printMap(*result.costmap, nx, ny);
    }

    // print result
    if (result.status)
    {
        if (result.path == nullptr)
        {
            std::cout << "Path is null!" << std::endl;
            return 0;
        }
        std::cout << "Path found!" << std::endl;
        std::cout << "Path: ";

        for (auto &point : *result.path)
        {
            std::cout << "(" << std::get<0>(point) << ", " << std::get<1>(point) << ") ";
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "Path not found!" << std::endl;
    }
}
