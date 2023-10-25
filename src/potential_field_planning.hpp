#ifndef ROAR__GLOBAL_PLANNING__POTENTIAL_FIELD_HPP
#define ROAR__GLOBAL_PLANNING__POTENTIAL_FIELD_HPP

#include <vector>
#include <cmath>
#include <math.h>
#include <memory>
#include <iostream>

namespace ROAR
{
    namespace global_planning
    {

        class PotentialFieldPlanning
        {

        public:
            struct PotentialFieldPlanningResult
            {
                bool status;
                std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path = nullptr;
                std::shared_ptr<std::vector<float>> costmap = nullptr;
            };
            struct PotentialFieldPlanningInput
            {
                typedef std::shared_ptr<PotentialFieldPlanningInput> SharedPtr;
                std::tuple<uint64_t, uint64_t> goal;
                uint64_t max_iter = 1000;
                uint64_t goal_threshold = 0; // distance to goal to consider goal reached
            };
            /**
             * @brief nx, ny: size of the map
             */
            PotentialFieldPlanning(uint64_t nx, uint64_t ny);

            ~PotentialFieldPlanning();
            /**
             * @brief obstacle_map: a list of tuples of (x, y) coordinates of obstacles
             * @return void
             */
            void setObstacleCoords(std::vector<std::tuple<uint64_t, uint64_t>> obstacle_map, uint64_t bounds, float gradient);

            void setObstacles(std::vector<uint8_t> obstacle_map, uint64_t bounds, float gradient);

            /**
             * @brief start: a tuple of (x, y) coordinates of the start point
             * @return bool: true if the start point is valid, false otherwise
             */
            bool setStart(std::tuple<uint64_t, uint64_t> start);

            /**
             * @brief goal: a tuple of (x, y) coordinates of the goal point
             * @return PotentialFieldPlanResult: a struct containing a boolean status and a vector of tuples representing the path
             */
            PotentialFieldPlanningResult plan(const PotentialFieldPlanningInput::SharedPtr input);

            std::shared_ptr<std::vector<float>> p_generateCostMapFromGoal(std::tuple<uint64_t, uint64_t> goal)
            {
                int64_t gx = static_cast<int64_t>(std::get<0>(goal));
                int64_t gy = static_cast<int64_t>(std::get<1>(goal));

                if (gx == -1 || gy == -1)
                {
                    // goal is too big
                    return nullptr;
                }

                // get copy of existing map
                std::shared_ptr<std::vector<float>> costmap = std::make_shared<std::vector<float>>(*map_);

                // map goal cost to all points
                for (uint64_t i = 0; i < nx * ny; i++)
                {
                    std::tuple<uint64_t, uint64_t> coord = getCoordFromIndex(i);
                    // distance between x, y and gx, gy
                    int64_t x = static_cast<int64_t>(std::get<0>(coord));
                    int64_t y = static_cast<int64_t>(std::get<1>(coord));

                    float dx = x - gx;
                    float dy = y - gy;

                    float dist = sqrt(dx * dx + dy * dy);

                    // std::cout << "coord: " << std::get<0>(coord) << ", " << std::get<1>(coord)
                    //           << " gx: " << gx << " gy: " << gy
                    //           << " dx: " << dx << " dy: " << dy << " dist raw: " << dx * dx + dy * dy << " dist: " << dist << std::endl;

                    (*costmap)[i] += dist;
                }
                return costmap;
            }

            bool p_greedySearch(
                std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path,
                std::tuple<uint64_t, uint64_t> start, std::tuple<uint64_t, uint64_t> goal,
                uint64_t max_iter, uint64_t nx, uint64_t ny, uint64_t goal_threshold,
                const std::shared_ptr<std::vector<float>> costmap);

            std::vector<float> get_map()
            {
                return *map_;
            }

            void reset()
            {
                map_ = std::make_shared<std::vector<float>>(nx * ny);
                start = nullptr;
            }

            uint64_t get_index(std::tuple<uint64_t, uint64_t> coord)
            {
                uint64_t x = std::get<0>(coord);
                uint64_t y = std::get<1>(coord);
                return y * nx + x;
            }

            std::tuple<uint64_t, uint64_t> getCoordFromIndex(uint64_t index)
            {
                uint64_t x = index % nx;
                uint64_t y = index / nx;
                return std::make_tuple(x, y);
            }

            bool isWithinGoal(std::tuple<uint64_t, uint64_t> coord, std::tuple<uint64_t, uint64_t> goal, uint64_t goal_threshold)
            {

                int64_t x = static_cast<int64_t>(std::get<0>(coord));
                int64_t y = static_cast<int64_t>(std::get<1>(coord));
                int64_t gx = static_cast<int64_t>(std::get<0>(goal));
                int64_t gy = static_cast<int64_t>(std::get<1>(goal));
                return (std::abs(x - gx) <= goal_threshold && std::abs(y - gy) <= goal_threshold);
            }

        private:
            uint64_t nx, ny, max_iter;
            std::shared_ptr<std::tuple<uint64_t, uint64_t>> start;
            std::shared_ptr<std::vector<float>> map_;
        };
    }
}

#endif // ROAR__GLOBAL_PLANNING__POTENTIAL_FIELD_HPP