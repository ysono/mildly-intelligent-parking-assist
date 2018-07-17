#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include "hybrid_breadth_first.h"

using std::cerr;
using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    if (argc <= 11) {
        cerr << "Args:" << endl
             << "path/to/grid.txt" << endl
             << "start_x start_y start_theta" << endl
             << "goal_x goal_y goal_theta" << endl
             << "ego_wheelbase ego_turning_radius ego_length ego_width" << endl;
        return 1;
    }

    char* grid_file_name = argv[1];

    double start_x              = std::strtod(argv[2], nullptr);
    double start_y              = std::strtod(argv[3], nullptr);
    double start_theta          = std::strtod(argv[4], nullptr);
    HBF::CarState start(start_x, start_y, start_theta);

    double goal_x               = std::strtod(argv[5], nullptr);
    double goal_y               = std::strtod(argv[6], nullptr);
    double goal_theta           = std::strtod(argv[7], nullptr);
    HBF::CarState goal(goal_x, goal_y, goal_theta);

    HBF::CarInfo car_info{};
    car_info.wheelbase          = std::strtod(argv[8], nullptr);
    car_info.turning_radius     = std::strtod(argv[9], nullptr);
    car_info.length             = std::strtod(argv[10], nullptr);
    car_info.width              = std::strtod(argv[11], nullptr);

    std::vector<std::vector<bool>> grid;
    std::ifstream grid_file(grid_file_name);
    for (std::string line; std::getline(grid_file, line); ) {
        if (line.empty()) {
            continue;
        }

        std::vector<bool> bool_line;
        bool_line.reserve(line.length());
        for (char c : line) {
            bool_line.push_back(c == '1');
        }
        grid.push_back(bool_line);
    }

    // The distance to move during expansion is hard coded to be this multiple of grid cell.
    // While this is not adjustable, the resolution at which the input grid is generated can be adjusted.
    double expansion_dist = 1.5;

    HBF hbf(grid, expansion_dist, car_info);

    std::vector<HBF::CarState> path = hbf.search(start, goal);

    for (auto iter = path.cend() - 1; iter >= path.cbegin(); iter--) {
        HBF::CarState st = *iter;
        cout << st.x << ',' << st.y << ',' << st.theta << endl;
    }

    return 0;
}