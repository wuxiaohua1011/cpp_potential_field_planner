#ifndef ROAR__GLOBAL_PLANNING__POTENTIAL_FIELD_HPP
#define ROAR__GLOBAL_PLANNING__POTENTIAL_FIELD_HPP

#include <vector>
#include <cmath>
#include <math.h>
#include <memory>

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
                uint64_t gx = std::get<0>(goal);
                uint64_t gy = std::get<1>(goal);

                // get copy of existing map
                std::shared_ptr<std::vector<float>> costmap = std::make_shared<std::vector<float>>(*map_);

                // map goal cost to all points
                for (uint64_t i = 0; i < nx * ny; i++)
                {
                    uint64_t x = i % nx;
                    uint64_t y = i / nx;
                    // distance between x, y and gx, gy
                    float dx = gx - x;
                    float dy = gy - y;
                    float dist = sqrt(dx * dx + dy * dy);

                    (*costmap)[i] += dist;
                }
                return costmap;
            }

            bool p_greedySearch(
                std::shared_ptr<std::vector<std::tuple<uint64_t, uint64_t>>> path,
                std::tuple<uint64_t, uint64_t> start, std::tuple<uint64_t, uint64_t> goal,
                uint64_t max_iter, uint64_t nx, uint64_t ny,
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

        private:
            uint64_t nx, ny, max_iter;
            std::shared_ptr<std::tuple<uint64_t, uint64_t>> start;
            std::shared_ptr<std::vector<float>> map_;
        };
    }
}

#endif