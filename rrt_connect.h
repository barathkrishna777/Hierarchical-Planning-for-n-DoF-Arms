#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <queue>

#include "utils.h"
#include "astar.hpp"

class RRT_Connect_Planner {
public:
    int x_size, y_size;
    int numofDOFs;
    double *map;
    double eps;
    std::mt19937 generator;
    int goal_id = -1;

    RRT_Connect_Planner(int x_size, int y_size, int numofDOFs, double *map, double eps) {
        this->x_size = x_size;
        this->y_size = y_size;
        this->numofDOFs = numofDOFs;
        this->map = map;
        this->eps = eps;
        std::random_device rd;
        generator = std::mt19937(rd());
    }

    void get_low_cost_regions(double *map, int x_size, int y_size, double* armstart_anglesV_rad,
                              double* armgoal_anglesV_rad) {
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

        astar_search(map, x_size, y_size, start_x, start_y, goal_x, goal_y);
        tie(map, x_size, y_size) = loadMap("map_low_cost.txt");
    }

    node new_node(int numofDOFs) {
        std::uniform_real_distribution<double> distribution(0.0, 2 * PI);
    
        node n;
        int max_attempts = 1000;
        int attempts = 0;
    
        while (attempts < max_attempts) {
            attempts++;
            n.angles.clear();
            for (int i = 0; i < numofDOFs; ++i) {
                n.angles.push_back(distribution(generator));
            }
            
            if (IsValidArmConfiguration(n.angles.data(), numofDOFs, map, x_size, y_size)) {
                return n;
            }
        }
        throw std::runtime_error("Failed to find a valid random node.");
    }


    void build_tree(std::vector<node>& tree_A, std::vector<node>& tree_B, 
                    double* armstart_anglesV_rad, double* armgoal_anglesV_rad, const int& max_nodes) {
            int n = 0;
            tree_A.clear();
            tree_B.clear();

            // Ensure IDs are unique
            node q_init;
            q_init.id = 0;
            q_init.angles.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
            q_init.g = 0;
            tree_A.push_back(q_init);

            node q_goal;
            q_goal.id = 0;
            q_goal.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
            q_goal.g = 0;
            tree_B.push_back(q_goal);

            while (n < max_nodes) {
            node q_rand = new_node(numofDOFs);

            if (n % 2 == 0) {
                int status_A = extend(tree_A, q_rand);
                if (status_A == 0) continue; // Trapped, retry

                ++n;
                int status_B = connect(tree_B, tree_A.back());
                if (status_B == 2) {
                    std::cout << "Goal node connected!" << std::endl;
                    return;
                }
                if (status_B != 0) ++n;
            } 
            else {
                int status_B = extend(tree_B, q_rand);
                if (status_B == 0) continue; // Trapped, retry

                ++n;
                int status_A = connect(tree_A, tree_B.back());
                if (status_A == 2) {
                    std::cout << "Goal node connected!" << std::endl;
                    return;
                }
                if (status_A != 0) 
                ++n;
            }
        }
    }


    int extend(std::vector<node>& tree, node& q_rand) {
        int nearest_node_id = nearest_neighbor(tree, q_rand);
        node q_extended = interpolate_eps(tree, nearest_node_id, q_rand, eps);
    
        if (q_extended.id == -1) {  
            return 0; // Trapped
        }
    
        double reached_distance = distance(q_extended, q_rand);
    
        q_extended.id = tree.size();  // Ensure unique ID
        tree.push_back(q_extended);
    
        double edge_cost = distance(tree[nearest_node_id], q_extended);
        tree[nearest_node_id].neighbors.emplace_back(q_extended.id, edge_cost);
        tree[q_extended.id].neighbors.emplace_back(nearest_node_id, edge_cost);
    
        return (reached_distance < 1e-3) ? 2 : 1; // Reached or Advanced
    }

    int connect(std::vector<node>& tree, node& q_new) {
        int status;
        node last_valid_node = q_new;
    
        do {
            status = extend(tree, last_valid_node);
            if (status == 1) { // Advanced
                last_valid_node = tree.back();
            }
        } while (status == 1);
    
        return status;
    }

    node interpolate_eps(std::vector<node>& tree, int id, node n, double eps) {
        double dist = distance(tree[id], n);
        dist = std::min(dist, eps);
        int numofsamples = std::max(1, (int)(dist / (PI / 20)));
    
        std::vector<double> config(numofDOFs);
        std::vector<double> prev_config = tree[id].angles;
    
        for (int i = 0; i < numofsamples; i++) {
            for (int j = 0; j < numofDOFs; j++) {
                config[j] = tree[id].angles[j] + ((double)(i) / (numofsamples - 1)) * (n.angles[j] - tree[id].angles[j]);
            }
    
            if (!IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size)) {
                if (i == 0) {
                    node invalid_node;
                    invalid_node.id = -1;
                    return invalid_node;
                } 
                else {
                    config = prev_config;
                    break;
                }
            }
            prev_config = config;
        }
        
        node interpolated_node;
        interpolated_node.id = tree.size();
        interpolated_node.angles = config;
    
        return interpolated_node;
    }

    int nearest_neighbor(std::vector<node>& tree, node& q_rand) {
        double min_dist = std::numeric_limits<double>::infinity();
        int nearest_node_id = -1;

        for (int i = 0; i < tree.size(); ++i) {
            double dist = distance(tree[i], q_rand);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }
        return nearest_node_id;
    }

    std::vector<int> dijkstra(std::vector<node>& tree) {
        if (tree.empty()) {
            std::cerr << "Error: Tree is empty!" << std::endl;
            return {};  // Return an empty path or handle the error accordingly
        }
        int q_goal_ID = tree.back().id;
        
        using NodePair = std::pair<double, int>;
        std::priority_queue<NodePair, std::vector<NodePair>, std::greater<NodePair>> open;
    
        tree[0].g = 0;
        open.push({0.0, 0});
    
        while (!open.empty()) {
            int current_id = open.top().second;
            open.pop();
    
            if (tree[current_id].closed)
                continue;
            tree[current_id].closed = true;
    
            if (current_id == q_goal_ID) {
                std::cout << "Goal node connected to the tree!" << std::endl;
                break;
            }
    
            for (auto& [next_id, edge_cost] : tree[current_id].neighbors) {
                if (tree[next_id].closed)
                    continue;
    
                double tentative_dist = tree[current_id].g + edge_cost;
                if (tentative_dist < tree[next_id].g) {
                    tree[next_id].g = tentative_dist;
                    tree[next_id].parent = current_id;
                    open.push({tentative_dist, next_id});
                }
            }
        }
    
        if (tree[q_goal_ID].g == std::numeric_limits<double>::infinity()) {
            std::cout << "No path found to the goal node." << std::endl;
            return {};
        }

        std::vector<int> path;
        for (int at = q_goal_ID; at >= 0; at = tree[at].parent) {
            path.push_back(at);
        }

        std::reverse(path.begin(), path.end());
        
        return path;
    }

    void save_to_file(std::vector<node>& tree_A, std::vector<node>& tree_B, 
                      const std::vector<int>& shortestPath) {
        std::ofstream m_log_fstream;
        m_log_fstream.open("rrt_connect.txt", std::ios::trunc); // Creates new or replaces existing file
        if (!m_log_fstream.is_open()) {
            throw std::runtime_error("Cannot open file");
        }
        // loop through the tree and write out all the joint angles and neighbors
        m_log_fstream << "tree_A" << std::endl;
        for (const auto& node : tree_A) {
            m_log_fstream << "Node ID: " << node.id << ", Angles: ";
            for (int k = 0; k < numofDOFs; ++k) {
                m_log_fstream << node.angles[k] << ",";
            }
            
            m_log_fstream << "Neighbors: ";
            for (const auto& neighbor : node.neighbors) {
                m_log_fstream << neighbor.first << " ";
            }
            m_log_fstream << std::endl;
        }

        m_log_fstream << "tree_B" << std::endl;
        for (const auto& node : tree_B) {
            m_log_fstream << "Node ID: " << node.id << ", Angles: ";
            for (int k = 0; k < numofDOFs; ++k) {
                m_log_fstream << node.angles[k] << ",";
            }
            
            m_log_fstream << "Neighbors: ";
            for (const auto& neighbor : node.neighbors) {
                m_log_fstream << neighbor.first << " ";
            }
            m_log_fstream << std::endl;
        }

        m_log_fstream << "path:" << std::endl;
        for (const auto& node_id : shortestPath) {
            m_log_fstream << node_id << ", ";
        }

        m_log_fstream.close();
    }
};