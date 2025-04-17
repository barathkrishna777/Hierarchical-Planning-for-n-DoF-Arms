#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "utils.h"

class low_cost {

    private:
        double *map;
        double *armstart_anglesV_rad;
        double *armgoal_anglesV_rad;
    
        int x_size;
        int y_size;
        int planning_coarse_factor;
        int numofDOFs;
        int coarse_x_size;
        int coarse_y_size;
        int coarse_map_size;
    
        int start_coarse_x, start_fine_x;
        int start_coarse_y, start_fine_y;
        int goal_coarse_x, goal_fine_x;
        int goal_coarse_y, goal_fine_y;

        struct DijkstraNode {
            int idx = -1;
            double g = std::numeric_limits<double>::infinity();
            bool closed = false;
    
            DijkstraNode() = default;
            DijkstraNode(int _idx) : idx(_idx) {}
        };
    
        std::vector<DijkstraNode> dijkstra_coarse_nodes;
    
        struct DijkstraComparator {
            const std::vector<DijkstraNode>& nodes;
            DijkstraComparator(const std::vector<DijkstraNode>& node_vec) : nodes(node_vec) {}
            
            bool operator()(const int& idx_a, const int& idx_b) const {
                return nodes[idx_a].g > nodes[idx_b].g;
            }
        };
    
        inline int get_coarse_x(int coarse_index) const {
            if (coarse_x_size <= 0) return -1;
            return coarse_index % coarse_x_size;
        }
        inline int get_coarse_y(int coarse_index) const {
            if (coarse_x_size <= 0) return -1;
            return coarse_index / coarse_x_size;
        }
    
        inline int get_coarse_idx(int x, int y) const {
            if (coarse_x_size <= 0 || coarse_y_size <= 0) return -1;
            if (x < 0 || x >= coarse_x_size || y < 0 || y >= coarse_y_size) return -1;
            return GETMAPINDEX(x, y, coarse_x_size, coarse_y_size);
        }
    
        inline int get_fine_idx(int x, int y) const {
             if (x_size <= 0 || y_size <= 0) return -1;
             if (x < 0 || x >= x_size || y < 0 || y >= y_size) return -1;
             return GETMAPINDEX(x, y, x_size, y_size);
        }
    
        inline int get_coarse_idx_from_fine(int fine_idx) const {
            if (x_size <= 0 || coarse_x_size <= 0 || coarse_y_size <= 0 || planning_coarse_factor <= 0) {
                return -1;
            }
            if (fine_idx < 0 || fine_idx >= (x_size * y_size)) {
                return -1;
            }
            int fine_x = fine_idx % x_size;
            int fine_y = fine_idx / x_size;
            int coarse_x = fine_x / planning_coarse_factor;
            int coarse_y = fine_y / planning_coarse_factor;

            return get_coarse_idx(coarse_x, coarse_y);
        }
    
         inline std::vector<int> get_neighbor_ids(int idx) const {
            std::vector<int> neighbor_ids;
            if (idx < 0 || idx >= coarse_map_size) return neighbor_ids;
    
            int current_x = get_coarse_x(idx);
            int current_y = get_coarse_y(idx);
            if (current_x == -1 || current_y == -1) return neighbor_ids;
    
            for (int i = 0; i < numDirections; ++i) {
                int next_x = current_x + dx[i];
                int next_y = current_y + dy[i];
                int neighbor_idx = get_coarse_idx(next_x, next_y);
                if (neighbor_idx != -1) {
                    neighbor_ids.push_back(neighbor_idx);
                }
            }
            return neighbor_ids;
        }
    
        std::vector<double> dijkstra_search_coarse(int idx_start_coarse, int idx_goal_coarse) {
            dijkstra_coarse_nodes.assign(coarse_map_size, DijkstraNode());
            for (int i = 0; i < coarse_map_size; ++i) {
                dijkstra_coarse_nodes[i].idx = i;
            }
    
            DijkstraComparator comparator(dijkstra_coarse_nodes);
            std::priority_queue<int, std::vector<int>, DijkstraComparator> open_set(comparator);
    
            int start_cx = get_coarse_x(idx_start_coarse);
            int start_cy = get_coarse_y(idx_start_coarse);
    
            dijkstra_coarse_nodes[idx_start_coarse].g = 0.0;
            open_set.push(idx_start_coarse);
    
            while(!open_set.empty()) {
                int idx_current_coarse = open_set.top();
                open_set.pop();

                if (idx_current_coarse < 0 || static_cast<size_t>(idx_current_coarse) >= dijkstra_coarse_nodes.size()) continue;
                DijkstraNode& current_node = dijkstra_coarse_nodes[idx_current_coarse];
                if (current_node.closed) {
                    continue;
                }
                current_node.closed = true;
    
                for (int idx_next_coarse : get_neighbor_ids(idx_current_coarse)) {
                    if (idx_next_coarse < 0 || static_cast<size_t>(idx_next_coarse) >= dijkstra_coarse_nodes.size()) continue;
                    DijkstraNode& neighbor_node = dijkstra_coarse_nodes[idx_next_coarse];
    
                    if (neighbor_node.closed) {
                        continue;
                    }
    
                    int next_coarse_x = get_coarse_x(idx_next_coarse);
                    int next_coarse_y = get_coarse_y(idx_next_coarse);
                    if (next_coarse_x == -1 || next_coarse_y == -1 || isCoarseCellOccupied(next_coarse_x, next_coarse_y, planning_coarse_factor, map, x_size, y_size)) {
                        continue;
                    }
    
                    int current_coarse_x = get_coarse_x(idx_current_coarse);
                    int current_coarse_y = get_coarse_y(idx_current_coarse);
                    double dx_move = next_coarse_x - current_coarse_x;
                    double dy_move = next_coarse_y - current_coarse_y;
                    double move_dist = std::sqrt(dx_move*dx_move + dy_move*dy_move);
                    double edge_cost = move_dist * planning_coarse_factor;
    
                    double tentative_g = current_node.g + edge_cost;
    
                    if (tentative_g < neighbor_node.g) {
                        neighbor_node.g = tentative_g;
                        open_set.push(idx_next_coarse);
                    }
                }
            }
    
            std::vector<double> final_g_costs(coarse_map_size);
            for(int i = 0; i < coarse_map_size; ++i) {
                final_g_costs[i] = dijkstra_coarse_nodes[i].g;
            }
            return final_g_costs;
        }
    
        void save_low_cost_map(const std::string& filename,
                               const std::vector<double>& forward_costs,
                               const std::vector<double>& backward_costs,
                               int idx_goal_coarse)
    {
        std::ofstream low_cost_map_file(filename);
        if (!low_cost_map_file.is_open()) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        double optimal_cost_L = std::numeric_limits<double>::infinity();
        if (idx_goal_coarse >= 0 && static_cast<size_t>(idx_goal_coarse) < forward_costs.size()) {
            optimal_cost_L = forward_costs[idx_goal_coarse];
        }

        double eps = 0.5;
        double cost_threshold = optimal_cost_L * (1.0 + eps);

        int idx_start_fine = get_fine_idx(start_fine_x, start_fine_y);
        int idx_goal_fine = get_fine_idx(goal_fine_x, goal_fine_y);

        low_cost_map_file << "height " << y_size << std::endl;
        low_cost_map_file << "width " << x_size << std::endl;

        int map_size = x_size * y_size;
        for (int i = 0; i < map_size; ++i) {
            int output_value = 1;

            if (map[i] == 1.0) {
                output_value = 1;
            } else {
                int coarse_idx = get_coarse_idx_from_fine(i);

                bool is_low_cost = false;
                if (coarse_idx != -1 &&
                    static_cast<size_t>(coarse_idx) < forward_costs.size() &&
                    static_cast<size_t>(coarse_idx) < backward_costs.size())
                {
                    double g_start = forward_costs[coarse_idx];
                    double g_goal = backward_costs[coarse_idx];

                    if (g_start != std::numeric_limits<double>::infinity() &&
                        g_goal != std::numeric_limits<double>::infinity() &&
                        (g_start + g_goal <= cost_threshold))
                    {
                        is_low_cost = true;
                    }
                }
                output_value = is_low_cost ? 0 : 1;

                if (i == idx_start_fine || i == idx_goal_fine) {
                    if (map[i] != 1.0) {
                        output_value = 0;
                    }
                }
            }

            low_cost_map_file << output_value << ((i + 1) % x_size == 0 ? "" : " ");
            if ((i + 1) % x_size == 0) {
                low_cost_map_file << std::endl;
            }
        }

        low_cost_map_file.close();
        std::cout << "Low-cost map saved successfully." << std::endl;
    }
    
    public:
        low_cost(double* map_ptr, int fine_x, int fine_y,
                 double* start_angles, double* goal_angles,
                 int coarse_factor, int dofs)
            : map(map_ptr), x_size(fine_x), y_size(fine_y),
              armstart_anglesV_rad(start_angles), armgoal_anglesV_rad(goal_angles),
              planning_coarse_factor(coarse_factor), numofDOFs(dofs)
        {

            if (!map_ptr || fine_x <= 0 || fine_y <= 0 || coarse_factor <= 0 || dofs <= 0) {
                 throw std::invalid_argument("Invalid arguments provided to low_cost constructor.");
            }
    
            this->coarse_x_size = static_cast<int>(std::ceil(static_cast<double>(x_size) / planning_coarse_factor));
            this->coarse_y_size = static_cast<int>(std::ceil(static_cast<double>(y_size) / planning_coarse_factor));
            this->coarse_map_size = coarse_x_size * coarse_y_size;
    
            std::vector<std::pair<double, double>> joint_positions(numofDOFs + 1);

            double x0,y0,x1,y1;
                
            x1 = ((double)x_size)/2.0;
            y1 = 0;
            for(int i = 0; i < numofDOFs; i++){
                x0 = x1;
                y0 = y1;
                x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-armgoal_anglesV_rad[i]);
                y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-armgoal_anglesV_rad[i]);
            }
            this->goal_fine_x = (int)(x1);
            this->goal_fine_y = (int)(y1);

            x1 = ((double)x_size)/2.0;
            y1 = 0;
            for(int i = 0; i < numofDOFs; i++){
                x0 = x1;
                y0 = y1;
                x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-armstart_anglesV_rad[i]);
                y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-armstart_anglesV_rad[i]);
            }
            this->start_fine_x = (int)(x1);
            this->start_fine_y = (int)(y1);
    
            this->start_coarse_x = std::max(0, std::min(start_fine_x / planning_coarse_factor, this->coarse_x_size - 1));
            this->start_coarse_y = std::max(0, std::min(start_fine_y / planning_coarse_factor, this->coarse_y_size - 1));
            this->goal_coarse_x = std::max(0, std::min(goal_fine_x / planning_coarse_factor, this->coarse_x_size - 1));
            this->goal_coarse_y = std::max(0, std::min(goal_fine_y / planning_coarse_factor, this->coarse_y_size - 1));
        }
    
        std::tuple<double*, int, int> generate_and_load_guidance_map(const std::string& output_filename) {
    
            int idx_start_coarse = get_coarse_idx(start_coarse_x, start_coarse_y);
            int idx_goal_coarse = get_coarse_idx(goal_coarse_x, goal_coarse_y);
    
            std::vector<double> forward_costs = dijkstra_search_coarse(idx_start_coarse, idx_goal_coarse);
            std::vector<double> backward_costs = dijkstra_search_coarse(idx_goal_coarse, idx_start_coarse);
            save_low_cost_map(output_filename, forward_costs, backward_costs, idx_goal_coarse);
            auto loaded_map_data = loadMap(output_filename);

            return loaded_map_data;
        }
};    

#endif // DIJKSTRA_HPP