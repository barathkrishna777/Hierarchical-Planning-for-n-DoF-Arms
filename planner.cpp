/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "rrt.h"
#include "rrt_star.h"

/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTBIAS 1
#define RRTSTAR 2

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

// RRT Planner
void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
    int &vertices)
{
    const int num_nodes = 2000;
    double eps = 0.5;
    std::vector<node> tree;

    RRT_Planner rrt(x_size, y_size, numofDOFs, map, map, eps);

    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Building tree" << std::endl;
    rrt.build_tree(tree, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree.size();

    std::cout << "Running Dijkstra on the tree" << std::endl;
    std::vector<int> shortestPath = rrt.dijkstra(tree, armgoal_anglesV_rad);

    // Check if a valid path was found
    if (shortestPath.empty())
    {
        std::cout << "No valid path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    *planlength = static_cast<int>(shortestPath.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken to build tree and find path: " << elapsed.count() << " seconds" << std::endl;

    // Free old plan memory before allocating a new one
    if (*plan != nullptr)
    {
        for (int i = 0; i < *planlength; ++i)
        {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double **)malloc(*planlength * sizeof(double *));
    if (!*plan)
    {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i)
    {
        (*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i])
        {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j)
        {
            (*plan)[i][j] = tree[shortestPath[i]].angles[j];
        }
    }

    // write out the vertices to a file
    std::ofstream m_vertices_fstream;
    rrt.save_to_file(tree, shortestPath);

}

// RRT Planner with task biasing
void plannerRRT_Bias(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
    int &vertices)
{
    const int num_nodes = 500;
    double eps = 0.5;
    std::vector<node> tree;

    int planner_coarse_factor = 5;
    std::cout << "Identifying low cost regions" << std::endl;

    low_cost l(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, planner_coarse_factor, numofDOFs);
    double *low_cost_map_data = nullptr;
    tie(low_cost_map_data, x_size, y_size) = l.generate_and_load_guidance_map("low_cost_map.txt");

    RRT_Planner rrt(x_size, y_size, numofDOFs, map, low_cost_map_data, eps);

    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Building tree" << std::endl;
    rrt.build_tree(tree, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree.size();

    std::cout << "Running Dijkstra on the tree" << std::endl;
    std::vector<int> shortestPath = rrt.dijkstra(tree, armgoal_anglesV_rad);

    // Check if a valid path was found
    if (shortestPath.empty())
    {
        std::cout << "No valid path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    *planlength = static_cast<int>(shortestPath.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken to build tree and find path: " << elapsed.count() << " seconds" << std::endl;

    // Free old plan memory before allocating a new one
    if (*plan != nullptr)
    {
        for (int i = 0; i < *planlength; ++i)
        {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double **)malloc(*planlength * sizeof(double *));
    if (!*plan)
    {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i)
    {
        (*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i])
        {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j)
        {
            (*plan)[i][j] = tree[shortestPath[i]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    std::ofstream m_vertices_fstream;
    rrt.save_to_file(tree, shortestPath);

    std::cout << "Vertices saved to file!" << std::endl;
}

// RRT* Planner
void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
    int &vertices)
{
    const int num_nodes = 1000;
    double eps = 0.5;
    std::vector<node> tree;

    RRT_Star_Planner rrt_star(x_size, y_size, numofDOFs, map, eps, armgoal_anglesV_rad);

    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Building tree" << std::endl;
    rrt_star.build_tree(tree, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree.size();
    std::cout << "Reconstructing path from the tree" << std::endl;
    std::vector<int> shortestPath = rrt_star.reconstruct_path(tree);
    // Check if a valid path was found
    if (shortestPath.empty())
    {
        std::cout << "No valid path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    *planlength = static_cast<int>(shortestPath.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken to build tree and find path: " << elapsed.count() << " seconds" << std::endl;

    // Free old plan memory before allocating a new one
    if (*plan != nullptr)
    {
        for (int i = 0; i < *planlength; ++i)
        {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double **)malloc(*planlength * sizeof(double *));
    if (!*plan)
    {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i)
    {
        (*plan)[i] = (double *)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i])
        {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j)
        {
            (*plan)[i][j] = tree[shortestPath[i]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    std::ofstream m_vertices_fstream;
    rrt_star.save_to_file(tree, shortestPath);

    std::cout << "Vertices saved to file!" << std::endl;
}

int main(int argc, char **argv)
{
    double *map;
    int x_size, y_size, vertices = 0;

    tie(map, x_size, y_size) = loadMap(argv[1]);
    const int numOfDOFs = std::stoi(argv[2]);
    double *startPos = doubleArrayFromString(argv[3]);
    double *goalPos = doubleArrayFromString(argv[4]);
    int whichPlanner = std::stoi(argv[5]);
    string outputFile = argv[6];

    std::cout << "Map file: " << argv[1] << std::endl;

    if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) ||
        !IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size))
    {
        throw runtime_error("Invalid start or goal configuration!\n");
    }

    double **plan = NULL;
    int planlength = 0;

    if (whichPlanner == RRT)
    {
        std::cout << "Using RRT" << std::endl;
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

    else if (whichPlanner == RRTBIAS)
    {
        std::cout << "Using RRT with task biasing" << std::endl;
        plannerRRT_Bias(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

    else if (whichPlanner == RRTSTAR)
    {
        std::cout << "Using RRT*" << std::endl;
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

    else
    {
        std::cerr << "Invalid planner ID!" << std::endl;
        return 1;
    }

    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) ||
        !equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs))
    {
        throw std::runtime_error("Start or goal position not matching");
    }

    std::ofstream m_log_fstream;
    m_log_fstream.open(outputFile, std::ios::trunc);
    if (!m_log_fstream.is_open())
    {
        throw std::runtime_error("Cannot open file");
    }
    m_log_fstream << argv[1] << endl;

    for (int i = 0; i < planlength; ++i)
    {
        for (int k = 0; k < numOfDOFs; ++k)
        {
            m_log_fstream << plan[i][k] << ",";
        }
        m_log_fstream << endl;
    }
    std::ofstream m_vertices_fstream;
    m_vertices_fstream.open("vertices.txt", std::ios::out | std::ios::app);
    m_vertices_fstream << vertices << endl;
}
