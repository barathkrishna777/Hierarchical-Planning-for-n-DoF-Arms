#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "utils.h"

struct dijkstraNode {
    int idx;
    double g = std::numeric_limits<double>::infinity();
    bool closed = false;

    dijkstraNode() : idx(-1) {}
    dijkstraNode(int _idx) : idx(_idx) {}
};

std::vector<dijkstraNode> dijkstra_coarse_nodes;

struct dijkstraComparator {
    bool operator()(const int& idx_a, const int& idx_b) const {
        return dijkstra_coarse_nodes[idx_a].g > dijkstra_coarse_nodes[idx_b].g;
    }
};

inline int get_coarse_x(int coarse_index, int coarse_x_size) {
    return coarse_index % coarse_x_size;
}
inline int get_coarse_y(int coarse_index, int coarse_x_size) {
    return coarse_index / coarse_x_size;
}

void dijkstra_search_coarse(double* fine_map, int fine_x_size, int fine_y_size,
                            int start_fine_x, int start_fine_y, int goal_fine_x, int goal_fine_y,
                            int coarse_factor, const std::string& output_filename = "map_low_cost.txt") {

    int coarse_x_size = static_cast<int>(std::ceil(static_cast<double>(fine_x_size) / coarse_factor));
    int coarse_y_size = static_cast<int>(std::ceil(static_cast<double>(fine_y_size) / coarse_factor));
    int coarse_map_size = coarse_x_size * coarse_y_size;
    dijkstra_coarse_nodes.assign(coarse_map_size, dijkstraNode());

    for (int i = 0; i < coarse_map_size; ++i) {
        dijkstra_coarse_nodes[i].idx = i;
    }
    double g_max = 0;

    std::priority_queue<int, std::vector<int>, dijkstraComparator> open_set;

    int start_coarse_x = start_fine_x / coarse_factor;
    int start_coarse_y = start_fine_y / coarse_factor;
    int idx_start_coarse = GETMAPINDEX(start_coarse_x, start_coarse_y, coarse_x_size, coarse_y_size);

    dijkstra_coarse_nodes[idx_start_coarse].g = 0.0;
    open_set.push(idx_start_coarse);

    std::cout << "Starting A* loop..." << std::endl;

    while(!open_set.empty()) {
        int idx_current_coarse = open_set.top();
        open_set.pop();

        int current_x = get_coarse_x(idx_current_coarse, coarse_x_size);
        int current_y = get_coarse_y(idx_current_coarse, coarse_x_size);

        if (dijkstra_coarse_nodes[idx_current_coarse].closed) {
            continue;
        }
        dijkstra_coarse_nodes[idx_current_coarse].closed = true;


        for (int i = 0; i < numDirections; ++i) {
            int next_coarse_x = current_x + dx[i];
            int next_coarse_y = current_y + dy[i];

            if (next_coarse_x < 0 || next_coarse_x >= coarse_x_size || next_coarse_y < 0 || next_coarse_y >= coarse_y_size) {
                continue;
            }

            int idx_next_coarse = GETMAPINDEX(next_coarse_x, next_coarse_y, coarse_x_size, coarse_y_size);

            if (isCoarseCellOccupied(next_coarse_x, next_coarse_y, coarse_factor, fine_map, fine_x_size, fine_y_size)) {
                continue;
            }

            if (dijkstra_coarse_nodes[idx_next_coarse].closed) {
                 continue;
             }

            double tentative_g = dijkstra_coarse_nodes[idx_current_coarse].g + move_cost[i];

            if (tentative_g < dijkstra_coarse_nodes[idx_next_coarse].g) {
                dijkstra_coarse_nodes[idx_next_coarse].g = tentative_g;
                open_set.push(idx_next_coarse);
                if (tentative_g > g_max) {
                    g_max = tentative_g;
                }
            }
        }
    }

    std::cout << "Writing output map..." << std::endl;
    std::ofstream output_file(output_filename);
    if (!output_file.is_open()) {
        std::cerr << "Error: Cannot open output file map_low_cost.txt" << std::endl;
        return;
    }
    output_file << "height " << fine_y_size << std::endl;
    output_file << "width " << fine_x_size << std::endl;

    for (int fine_y = 0; fine_y < fine_y_size; ++fine_y) {
        for (int fine_x = 0; fine_x < fine_x_size; ++fine_x) {
            int coarse_x = fine_x / coarse_factor;
            int coarse_y = fine_y / coarse_factor;
            int idx_coarse = GETMAPINDEX(coarse_x, coarse_y, coarse_x_size, coarse_y_size);

            double cost = 1.0;

             if (idx_coarse >= 0 && idx_coarse < coarse_map_size &&
                !isCoarseCellOccupied(coarse_x, coarse_y, coarse_factor, fine_map, fine_x_size, fine_y_size) &&
                dijkstra_coarse_nodes[idx_coarse].g != std::numeric_limits<double>::infinity())
            {
                 cost = (g_max > 0) ? (dijkstra_coarse_nodes[idx_coarse].g / g_max) : 0.0;
             } else if (fine_map[GETMAPINDEX(fine_x, fine_y, fine_x_size, fine_y_size)] == 1.0) {
                 cost = 1.0;
             }

            if (cost > 0.25) {
                 output_file << "1 ";
             } else {
                 output_file << "0 ";
            }
        }
        output_file << std::endl;
    }
    output_file.close();
    std::cout << "Output map written to " << output_filename << std::endl;
}

void get_low_cost_regions(double *map, int x_size, int y_size, double* armstart_anglesV_rad,
                              double* armgoal_anglesV_rad, int planning_coarse_factor, int numofDOFs) {
    // std::ofstream m_log_fstream;
    // m_log_fstream.open("map_low_cost.txt", std::ios::trunc);

    int start_x, start_y;
    int goal_x, goal_y;

    // get start and goal coordinates of end effector from the configurations
    int x0, y0;
    int x1 = ((double)x_size)/2.0;
    int y1 = 0;
    for (int i = 0; i < numofDOFs; i++) {
        x0 = x1;
        y0 = y1;
        x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-armstart_anglesV_rad[i]);
        y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-armstart_anglesV_rad[i]);
    }
    start_x = (int)x1;
    start_y = (int)y1;

    x1 = ((double)x_size)/2.0;
    y1 = 0;
    for (int i = 0; i < numofDOFs; i++) {
        x0 = x1;
        y0 = y1;
        x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-armgoal_anglesV_rad[i]);
        y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-armgoal_anglesV_rad[i]);
    }
    goal_x = (int)x1;
    goal_y = (int)y1;

    dijkstra_search_coarse(map, x_size, y_size, start_x, start_y, goal_x, goal_y, planning_coarse_factor);
    tie(map, x_size, y_size) = loadMap("map_low_cost.txt");
}

#endif // DIJKSTRA_HPP