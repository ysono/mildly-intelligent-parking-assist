#include <cmath>
#include <tuple>
#include <vector>
#include "hybrid_breadth_first.h"


HBF::CarState::CarState(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    this->x_idx = dist_to_idx(x);
    this->y_idx = dist_to_idx(y);
    this->theta_idx = theta_to_idx(theta);
}

HBF::CarState::CarState(double x, double y, double theta, unsigned int iteration, CarState const & goal) :
    CarState(x, y, theta)
{
    this->iteration_cnt = iteration;
    this->cost = iteration + heuristic(*this, goal);
}

std::vector<double> HBF::create_expansion_thetas(double const expansion_dist, CarInfo const & car_info) {
    std::vector<double> expansion_thetas{0};

    size_t num_more_points = 7;
    double max_steering = atan2(car_info.wheelbase, car_info.turning_radius); // bicycle model
    double steering_interval = max_steering / num_more_points;

    for (int i = 1; i <= num_more_points; i++) {
        double abs_steering = steering_interval * i;
        for (int direction : {1, -1}) {
            double steering = abs_steering * direction;
            double change_in_theta = expansion_dist / car_info.wheelbase * tan(steering);
            expansion_thetas.push_back(change_in_theta);
        }
    }

    return expansion_thetas;
}

size_t HBF::theta_to_idx(double theta) {
    double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
    return (int)(round(new_theta * NUM_ORIENTATION_BUCKETS / (2*M_PI))) % NUM_ORIENTATION_BUCKETS;
}

size_t HBF::dist_to_idx(double dist) {
    return size_t(floor(dist));
}

double HBF::heuristic(CarState const & state, CarState const & goal) {
    // TODO maybe penalize switching between forward/backward.
    return pow(state.x - goal.x, 2) +
           pow(state.y - goal.y, 2) +
           pow(state.theta - goal.theta, 2);
}

bool HBF::are_car_states_in_same_bucket(CarState const & left, CarState const & right) {
    return left.x_idx == right.x_idx &&
           left.y_idx == right.y_idx &&
           left.theta_idx == right.theta_idx;
}

bool HBF::is_within_bounds(CarState const & state) {
    return state.x_idx < grid_w &&
           state.y_idx < grid_h &&
           state.theta_idx < NUM_ORIENTATION_BUCKETS;
}

bool HBF::does_car_collide_w_obstacle(CarState const &state) {
    auto eval_one_corner = [&state, this](double const & angle) {
        double x = state.x + ego_diagonal_dist * cos(angle);
        double y = state.y + ego_diagonal_dist * sin(angle);

        double x_idx = dist_to_idx(x);
        double y_idx = dist_to_idx(y);

        bool is_corner_inside_grid =
                x_idx >= 0 && x_idx < grid_w &&
                y_idx >= 0 && y_idx < grid_h;

        // If out of bounds, then consider clear, b/c boundaries are not considered obstacles;
        // otherwise, algo could have problem initiating the traversal from the starting point,
        // e.g. if ego's length is > 1.
        // Also note, the obstacles grid uses (y, x) indexing.
        return is_corner_inside_grid && obstacles_grid[y_idx][x_idx];
    };
    return eval_one_corner(state.theta - ego_diagonal_angle) ||
            eval_one_corner(state.theta + ego_diagonal_angle) ||
            eval_one_corner(-state.theta - ego_diagonal_angle) ||
            eval_one_corner(-state.theta + ego_diagonal_angle);
}

HBF::HBF(std::vector<std::vector<bool>> const & obst_grid, double expansion_dist, CarInfo const & car_info) :
        expansion_dist(expansion_dist),
        expansion_thetas(create_expansion_thetas(expansion_dist, car_info)),
        ego_diagonal_dist(sqrt(pow(car_info.length / 2, 2) + pow(car_info.width / 2, 2))),
        ego_diagonal_angle(atan2(car_info.width, car_info.length)),
        obstacles_grid(obst_grid),
        grid_w(obst_grid[0].size()),
        grid_h(obst_grid.size())
{}

std::vector<HBF::CarState> HBF::expand(CarState const & state, CarState const & goal) {
    std::vector<CarState> next_states;
    for (double delta_theta : expansion_thetas) {
        double next_theta = state.theta + delta_theta;
        while (next_theta < 0) { next_theta += 2 * M_PI; }

        for (int delta_dist_multiplier : {1, -1}) {
            double delta_dist = expansion_dist * delta_dist_multiplier;

            // Origin is at top left; x increases towards right; y increases towards bottom;
            // theta starts with 0 pointing right and increasing clockwise.
            double next_x = state.x + delta_dist * cos(next_theta);
            double next_y = state.y + delta_dist * sin(next_theta);

            CarState next_state(next_x, next_y, next_theta, state.iteration_cnt + 1, goal);
            next_states.push_back(next_state);
        }
    }
    return next_states;
}

std::vector<HBF::CarState> HBF::search(CarState const & start, CarState const & goal) {

    assert(is_within_bounds(start));
    assert(is_within_bounds(goal));

    // Remembers the best continuous state from which a given discrete state can be directly reached.
    // I.e. the preceding continuous state is outside the destination discrete bucket (except the starting discrete state).
    // Non-nullptr value means the discrete state was traversed (aka "closed") already.
    std::vector<std::vector<std::vector< std::shared_ptr<CarState> >>> discrete_state_came_from(
            grid_w,
            std::vector<std::vector< std::shared_ptr<CarState> >>(
                    grid_h,
                    std::vector< std::shared_ptr<CarState> >(
                            NUM_ORIENTATION_BUCKETS)));

    discrete_state_came_from[start.x_idx][start.y_idx][start.theta_idx] = std::make_shared<CarState>(start);

    // These are the leaf nodes of the breath-first traversal.
    std::vector<CarState> open_continuous_states{start};

    while (! open_continuous_states.empty()) {

        // Sort from the worst cost to the best cost.
        sort(open_continuous_states.begin(), open_continuous_states.end(),
             [](CarState const & left, CarState const & right) {
                 return left.cost > right.cost;
             });
        CarState curr_st = open_continuous_states.back();
        open_continuous_states.pop_back();

        if (are_car_states_in_same_bucket(curr_st, goal)) {
            // Found solution.
            std::vector<CarState> soln_path;
            while (! are_car_states_in_same_bucket(curr_st, start)) {
                soln_path.push_back(curr_st);
                curr_st = *(discrete_state_came_from[curr_st.x_idx][curr_st.y_idx][curr_st.theta_idx]);
            }
            soln_path.push_back(curr_st);
            return soln_path;
        }

        std::vector<CarState> next_states = expand(curr_st, goal);
        for (CarState next_st : next_states) {
            if (next_st.y < 0 ||
                next_st.x < 0 ||
                next_st.y >= grid_h ||
                next_st.x >= grid_w) {
                // The center point of ego is out of bounds.
                continue;
            }

            if (does_car_collide_w_obstacle(next_st)) {
                // At least one corner of ego collides with an obstacle.
                continue;
            }

            // Note, the obstacles grid uses (y, x) indexing.
            if (obstacles_grid[next_st.y_idx][next_st.x_idx] ||
                discrete_state_came_from[next_st.x_idx][next_st.y_idx][next_st.theta_idx] != nullptr) {
                // Blocked, or traversed already.
                continue;
            }

            open_continuous_states.push_back(next_st);
            discrete_state_came_from[next_st.x_idx][next_st.y_idx][next_st.theta_idx] = std::make_shared<CarState>(curr_st);
        }
    }

    throw std::runtime_error("No solution");
}

