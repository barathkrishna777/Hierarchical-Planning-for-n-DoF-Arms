#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "utils.h"

struct dijkstraNode {
    int idx;
    double g = std::numeric_limits<double>::infinity();
    double h = std::numeric_limits<double>::infinity();
    bool closed = false;

    dijkstraNode() : idx(-1) {}
    dijkstraNode(int _idx) : idx(_idx) {}
};

std::vector<dijkstraNode> dijkstra_nodes;

struct dijkstraComparator {
    bool operator()(const int& idx_a, const int& idx_b) const {
        return dijkstra_nodes[idx_a].g + dijkstra_nodes[idx_a].h > dijkstra_nodes[idx_b].g + dijkstra_nodes[idx_a].h;
    }
};

inline double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

inline int get_x(int index, int x_size) {
    return index % x_size;
}
inline int get_y(int index, int x_size) {
    return index / x_size;
}


void astar_search(double* map, int x_size, int y_size, int start_x, int start_y, int goal_x, int goal_y) {

    int map_size = x_size * y_size;
    dijkstra_nodes.resize(map_size);
    for (int i = 0; i < map_size; ++i) {
        dijkstra_nodes[i].idx = i;
        dijkstra_nodes[i].g = std::numeric_limits<double>::infinity();
        dijkstra_nodes[i].closed = false;
    }
    double g_max = 0;

    std::priority_queue<int, std::vector<int>, dijkstraComparator> open_set;

    int idx_start = GETMAPINDEX(start_x, start_y, x_size, y_size);
    int idx_goal = GETMAPINDEX(goal_x, goal_y, x_size, y_size);

    dijkstra_nodes[idx_start].g = 0.0;
    open_set.push(idx_start);

    std::cout << "Starting A* loop..." << std::endl;

    while(!open_set.empty()) {
        int idx_current = open_set.top();
        open_set.pop();

        int current_x = get_x(idx_current, x_size);
        int current_y = get_y(idx_current, x_size);

        if (dijkstra_nodes[idx_current].closed) {
            continue;
        }
        dijkstra_nodes[idx_current].closed = true;


        for (int i = 0; i < numDirections; ++i) {
            int next_x = current_x + dx[i];
            int next_y = current_y + dy[i];

            if (next_x < 0 || next_x >= x_size || next_y < 0 || next_y >= y_size) {
                continue;
            }

            int idx_next = GETMAPINDEX(next_x, next_y, x_size, y_size);

            if (map[idx_next] == 1.0) {
                continue;
            }

            if (dijkstra_nodes[idx_next].closed) {
                 continue;
             }

            double tentative_g = dijkstra_nodes[idx_current].g + move_cost[i];
            double h_next = heuristic(next_x, next_y, goal_x, goal_y);
            dijkstra_nodes[idx_next].h = h_next;

            if (tentative_g < dijkstra_nodes[idx_next].g) {
                dijkstra_nodes[idx_next].g = tentative_g;
                open_set.push(idx_next);
                if (tentative_g > g_max) {
                    g_max = tentative_g;
                }
            }
        }
    }

    // Normalize the g values (only for reachable nodes) and write map
    std::cout << "Writing output map..." << std::endl;
    std::ofstream output_file("map_low_cost.txt");
    if (!output_file.is_open()) {
        std::cerr << "Error: Cannot open output file map_low_cost.txt" << std::endl;
        return;
    }
    output_file << "height " << y_size << std::endl;
    output_file << "width " << x_size << std::endl;

    for (int y = 0; y < y_size; ++y) {
        for (int x = 0; x < x_size; ++x) {
            int idx = GETMAPINDEX(x, y, x_size, y_size);
            double normalized_g = 1.0;

            if (dijkstra_nodes[idx].g != std::numeric_limits<double>::infinity()) {
                 normalized_g = (g_max > 0) ? (dijkstra_nodes[idx].g / g_max) : 0.0;
            } else if (map[idx] == 1.0) {
            }

            if (normalized_g > 0.25) {
                output_file << "1 ";
            } else {
                output_file << "0 ";
            }
        }
        output_file << std::endl;
    }
    output_file.close();
    std::cout << "Output map written to map_low_cost.txt" << std::endl;
}

#endif