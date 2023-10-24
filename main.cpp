#include <iostream>
#include "src/potential_field_planning.hpp"

void p_printMap(std::vector<uint8_t> map, uint64_t nx, uint64_t ny)
{
    for (int i = 0; i < map.size(); i++)
    {
        std::cout << (int)map[i] << " ";
        if ((i + 1) % nx == 0)
        {
            std::cout << std::endl;
        }
    }
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
    
    ROAR::global_planning::PotentialFieldPlanResult result = planner.plan(std::make_tuple(9, 9));

    // print result
    if (result.status)
    {
        std::cout << "Path found!" << std::endl;
        std::cout << "Path: ";
        for (auto &point : result.path)
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
