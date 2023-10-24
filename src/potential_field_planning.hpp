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
        struct PotentialFieldPlanResult
        {
            bool status;
            std::vector<std::tuple<uint64_t, uint64_t>> path;
        };

        class PotentialFieldPlanning
        {

        public:
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
            ROAR::global_planning::PotentialFieldPlanResult plan(std::tuple<uint64_t, uint64_t> goal, uint64_t max_iter);

            std::vector<uint8_t> get_map()
            {
                return *map_;
            }

        private:
            uint64_t nx, ny, max_iter;
            std::shared_ptr<std::tuple<uint64_t, uint64_t>> start;
            std::shared_ptr<std::vector<uint8_t>> map_;
        };
    }
}

#endif