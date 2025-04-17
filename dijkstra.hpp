#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "utils.h"

class low_cost {

private:

    double *map;
    int x_size;
    int y_size;
    double *armstart_anglesV_rad;
    double *armgoal_anglesV_rad;
    int planning_coarse_factor;
    int numofDOFs;
    int goal_coarse_x, goal_coarse_y;
    int coarse_x_size, coarse_y_size, coarse_map_size;

    struct dijkstraNode {
        int idx;
        double g = std::numeric_limits<double>::infinity();
        double h = std::numeric_limits<double>::infinity();
        bool closed = false;
        int idx_parent = -1;
        bool expanded = false;
        dijkstraNode() : idx(-1) {}
        dijkstraNode(int _idx) : idx(_idx) {}
    };

    struct dijkstraComparator {
        bool operator()(const dijkstraNode& a, const dijkstraNode& b) const {
            return a.g + a.h > b.g + b.h;
        }
    };

    std::vector<dijkstraNode> dijkstra_coarse_nodes;
    
    inline int get_coarse_x(int coarse_index, int coarse_x_size) {
        return coarse_index % coarse_x_size;
    }
    inline int get_coarse_y(int coarse_index, int coarse_x_size) {
        return coarse_index / coarse_x_size;
    }

    double heuristic(int coarse_x, int coarse_y, int goal_coarse_x, int goal_coarse_y) {
        return std::sqrt(std::pow(coarse_x - goal_coarse_x, 2) + std::pow(coarse_y - goal_coarse_y, 2));
    }

    inline int get_idx(int x, int y, int x_size, int y_size) {
        return GETMAPINDEX(x, y, x_size, y_size);
    }

    inline int get_coarse_idx(int x, int y) const {
        // Bounds check might be good here if needed, but A* loop does it too
        return GETMAPINDEX(x, y, coarse_x_size, coarse_y_size);
    }

    // Get fine index from fine coordinates
    inline int get_fine_idx(int x, int y) const {
         return GETMAPINDEX(x, y, x_size, y_size);
    }

    inline std::vector<int> get_neighbor_ids(int idx, int coarse_x_size, int coarse_y_size) {
        std::vector<int> neighbor_ids;
        for (int i = 0; i < numDirections; ++i) {
            int next_x = get_coarse_x(idx, coarse_x_size) + dx[i];
            int next_y = get_coarse_y(idx, coarse_x_size) + dy[i];
            if (next_x >= 0 && next_x < coarse_x_size && next_y >= 0 && next_y < coarse_y_size) {
                neighbor_ids.push_back(get_idx(next_x, next_y, coarse_x_size, coarse_y_size));
            }
        }
        return neighbor_ids;
    }

    void dijkstra_search_coarse(int start_coarse_x, int start_coarse_y, int goal_coarse_x, int goal_coarse_y, int planning_coarse_factor) {
        dijkstra_coarse_nodes.resize(coarse_map_size);

        for (int i = 0; i < coarse_map_size; ++i) {
            dijkstra_coarse_nodes[i].idx = i;
        }
        double g_max = 0;

        std::cout << __LINE__ << std::endl;

        std::priority_queue<dijkstraNode, std::vector<dijkstraNode>, dijkstraComparator> open_set;
        int idx_start_coarse = get_idx(start_coarse_x, start_coarse_y, coarse_x_size, coarse_y_size);;

        int idx_goal_coarse = get_idx(goal_coarse_x, goal_coarse_y, coarse_x_size, coarse_y_size);
        dijkstra_coarse_nodes[idx_start_coarse].g = 0.0;
        dijkstra_coarse_nodes[idx_start_coarse].h = heuristic(start_coarse_x, start_coarse_y, goal_coarse_x, goal_coarse_y);
        open_set.push(dijkstra_coarse_nodes[idx_start_coarse]);

        std::cout << __LINE__ << std::endl;

        while(!open_set.empty()) {
            dijkstraNode current_node = open_set.top();
            open_set.pop();

            if (current_node.idx == idx_goal_coarse) {
                break;
            }
            int idx_current_coarse = current_node.idx;

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

                if (isCoarseCellOccupied(next_coarse_x, next_coarse_y, planning_coarse_factor, map, x_size, y_size)) {
                    continue;
                }

                if (dijkstra_coarse_nodes[idx_next_coarse].closed) {
                    continue;
                }

                double tentative_g = dijkstra_coarse_nodes[idx_current_coarse].g + move_cost[i];
                dijkstra_coarse_nodes[idx_next_coarse].h = heuristic(next_coarse_x, next_coarse_y, goal_coarse_x, goal_coarse_y);

                if (tentative_g < dijkstra_coarse_nodes[idx_next_coarse].g) {
                    dijkstra_coarse_nodes[idx_next_coarse].g = tentative_g;
                    dijkstra_coarse_nodes[idx_next_coarse].idx_parent = idx_current_coarse;
                    open_set.push(idx_next_coarse);
                    if (tentative_g > g_max) {
                        g_max = tentative_g;
                    }
                }
            }
        }
    }

    std::vector<int> reconstruct_path() {
        std::vector<int> path;
        int idx = get_idx(goal_coarse_x / planning_coarse_factor, goal_coarse_y / planning_coarse_factor, x_size, y_size);

        while (idx != -1) {
            path.push_back(idx);
            idx = dijkstra_coarse_nodes[idx].idx_parent;
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    void save_low_cost_map(const std::vector<int>& path) {
        std::string filename = "low_cost_map.txt";
        std::cout << "Saving low-cost map (fine resolution) to " << filename << "..." << std::endl;

        // 1. Create a set of coarse indices representing the corridor
        std::set<int> corridor_coarse_indices;
        int corridor_radius = 2; // How many neighbors out from path to include (0=path only, 1=path+immediate, 2=path+neighbors+neighbors-of-neighbors)

        std::vector<int> current_layer = path; // Start with path nodes
        for (int r = 0; r <= corridor_radius; ++r) {
            std::vector<int> next_layer;
            for (int idx : current_layer) {
                // Add current node to corridor if not already present
                if (corridor_coarse_indices.find(idx) == corridor_coarse_indices.end()) {
                    corridor_coarse_indices.insert(idx);

                    // If exploring neighbors (r < radius), find and add them for next layer
                    if (r < corridor_radius && corridor_coarse_indices.find(idx) == corridor_coarse_indices.end()) {
                        corridor_coarse_indices.insert(idx); // Mark as expanded
    
                        // Use the helper function to get neighbors
                        std::vector<int> neighbors = get_neighbor_ids(idx, coarse_x_size, coarse_y_size);
                        for (int neighbor_idx : neighbors) {
                             // Check validity and if not already in corridor (optional optimization)
                             if (neighbor_idx != -1 && corridor_coarse_indices.find(neighbor_idx) == corridor_coarse_indices.end()) {
                                next_layer.push_back(neighbor_idx);
                             }
                        }
                    }
                }
            }
            current_layer = std::move(next_layer); // Move to next layer of neighbors
             if (current_layer.empty()) break; // Stop if no new neighbors found
        }
         std::cout << "Corridor defined with " << corridor_coarse_indices.size() << " coarse cells." << std::endl;


        // 2. Open output file
        std::ofstream low_cost_map_file(filename); // Use ofstream for writing
        if (!low_cost_map_file.is_open()) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        // 3. Write header (using fine map dimensions)
        low_cost_map_file << "height " << y_size << "\n";
        low_cost_map_file << "width " << x_size << "\n";

        // 4. Iterate through FINE map cells
        for (int fy = 0; fy < y_size; ++fy) {
            for (int fx = 0; fx < x_size; ++fx) {
                int fine_idx = get_fine_idx(fx, fy);
                int output_value = 1; // Default to high-cost/obstacle

                // Check if original fine cell is an obstacle
                if (map[fine_idx] == 1.0) {
                    output_value = 1;
                } else {
                    // Find corresponding coarse cell index
                    int cx = fx / planning_coarse_factor;
                    int cy = fy / planning_coarse_factor;
                    int coarse_idx = get_coarse_idx(cx, cy);

                    // Check if the coarse cell is in the corridor set
                    if (corridor_coarse_indices.count(coarse_idx)) {
                        // Cell is free and its coarse representation is in the corridor
                        output_value = 0;
                    } else {
                        // Cell is free but its coarse representation is outside the corridor
                        output_value = 1;
                    }
                }
                low_cost_map_file << output_value << (fx == x_size - 1 ? "" : " "); // Add space except for last element
            }
            low_cost_map_file << "\n"; // Newline after each row
        }

        low_cost_map_file.close();
        std::cout << "Low-cost map saved successfully." << std::endl;
    }

public:

    std::tuple<double*, int, int> get_low_cost_regions() {
        int start_x, start_y;
        int goal_x, goal_y;

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

        std::cout << __LINE__ << std::endl;

        dijkstra_search_coarse(start_x, start_y, goal_x, goal_y, planning_coarse_factor);

        std::cout << __LINE__ << std::endl;

        std::vector<int> path = reconstruct_path();

        std::cout << __LINE__ << std::endl;

        save_low_cost_map(path);

        std::cout << __LINE__ << std::endl;

        auto loaded_map_data = loadMap("low_cost_map.txt");

        std::cout << __LINE__ << std::endl;

        return loaded_map_data;
    }

    low_cost(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, 
             int planning_coarse_factor, int numofDOFs) {
        this->map = map;
        this->x_size = x_size;
        this->y_size = y_size;
        this->armstart_anglesV_rad = armstart_anglesV_rad;
        this->armgoal_anglesV_rad = armgoal_anglesV_rad;
        this->planning_coarse_factor = planning_coarse_factor;
        this->numofDOFs = numofDOFs;
        this->coarse_x_size = static_cast<int>(std::ceil(static_cast<double>(x_size) / planning_coarse_factor));
        this->coarse_y_size = static_cast<int>(std::ceil(static_cast<double>(y_size) / planning_coarse_factor));
        this->coarse_map_size = coarse_x_size * coarse_y_size;
    }
};

#endif // DIJKSTRA_HPP