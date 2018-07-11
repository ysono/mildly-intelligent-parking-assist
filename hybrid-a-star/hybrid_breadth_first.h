#ifndef HYBRID_A_STAR_HYBRID_BREADTH_FIRST_H
#define HYBRID_A_STAR_HYBRID_BREADTH_FIRST_H


#include <vector>

class HBF {
public:

    struct CarState {
        double x;
        double y;
        double theta;

        size_t x_idx;
        size_t y_idx;
        size_t theta_idx;

        unsigned int iteration_cnt;
        double cost;

        CarState(double x, double y, double theta);
        CarState(double x, double y, double theta, unsigned int iteration, CarState const & goal);
    };

    struct CarInfo {
        double wheelbase;
        double turning_radius;
        double length;
        double width;
    };

    HBF(std::vector<std::vector<bool>> const & obst_grid, double expansion_dist, CarInfo const & car_info);

    std::vector<CarState> search(CarState const & start, CarState const & goal);

private:

    static const size_t NUM_ORIENTATION_BUCKETS = 90;

    static std::vector<double> create_expansion_thetas(double expansion_dist, CarInfo const & car_info);

    static size_t theta_to_idx(double theta);

    static size_t dist_to_idx(double dist);

    static double heuristic(CarState const & state, CarState const & goal);

    static bool are_car_states_in_same_bucket(CarState const & left, CarState const & right);

    const double expansion_dist; // In the unit of the discretized distance.
    const std::vector<double> expansion_thetas;

    const double ego_diagonal_dist; // In the unit of the discretized distance.
    const double ego_diagonal_angle;

    const std::vector<std::vector<bool>> obstacles_grid;
    const size_t grid_w, grid_h;

    bool is_within_bounds(CarState const & state);

    bool does_car_collide_w_obstacle(CarState const &state);

    std::vector<CarState> expand(CarState const & state, CarState const & goal);

};


#endif //HYBRID_A_STAR_HYBRID_BREADTH_FIRST_H
