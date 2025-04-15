/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "prm.h"
#include "rrt.h"
#include "rrt_connect.h"
#include "rrt_star.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

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
    const int num_nodes = 1000;
	double eps = 1.0;
    std::vector<node> tree;
    RRT_Planner rrt(x_size, y_size, numofDOFs, map, eps);

    std::cout << "Identifying low cost regions" << std::endl;
    rrt.get_low_cost_regions(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    auto start = std::chrono::high_resolution_clock::now();    

    std::cout << "Building tree" << std::endl;
    rrt.build_tree(tree, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree.size();

    std::cout << "Running Dijkstra on the tree" << std::endl;
    std::vector<int> shortestPath = rrt.dijkstra(tree, armgoal_anglesV_rad);

    // Check if a valid path was found
    if (shortestPath.empty()) {
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
    if (*plan != nullptr) {
        for (int i = 0; i < *planlength; ++i) {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double**)malloc(*planlength * sizeof(double*));
    if (!*plan) {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i) {
        (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i]) {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j) {
            (*plan)[i][j] = tree[shortestPath[i]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    std::ofstream m_vertices_fstream;
    rrt.save_to_file(tree, shortestPath);
    
    std::cout << "Vertices saved to file!" << std::endl;
}


// RRT-Connect Planner
void plannerRRTConnect(
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
	double eps = 1.0;
    std::vector<node> tree_A, tree_B;
    RRT_Connect_Planner rrt_connect(x_size, y_size, numofDOFs, map, eps);

    std::cout << "Identifying low cost regions" << std::endl;
    rrt_connect.get_low_cost_regions(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    auto start = std::chrono::high_resolution_clock::now();    

    std::cout << "Building tree" << std::endl;
    rrt_connect.build_tree(tree_A, tree_B, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree_A.size() + tree_B.size();

    std::cout << "Running Dijkstra on the two trees" << std::endl;
    std::vector<int> shortestPath_A = rrt_connect.dijkstra(tree_A);
    std::vector<int> shortestPath_B = rrt_connect.dijkstra(tree_B);

    if (shortestPath_A.empty() || shortestPath_B.empty()) {
        std::cerr << "Error: One of the trees failed to find a valid path!" << std::endl;
        return;
    }

    int connection_node = shortestPath_A.back();

    // Remove the duplicate connection node from shortestPath_B (if it exists)
    if (!shortestPath_B.empty() && shortestPath_B.front() == connection_node) {
        shortestPath_B.erase(shortestPath_B.begin());
    }
    std::reverse(shortestPath_B.begin(), shortestPath_B.end());

    std::vector<int> shortestPath;
    shortestPath.reserve(shortestPath_A.size() + shortestPath_B.size());
    shortestPath.insert(shortestPath.end(), shortestPath_A.begin(), shortestPath_A.end());
    shortestPath.insert(shortestPath.end(), shortestPath_B.begin(), shortestPath_B.end());

    // Check if a valid path was found
    if (shortestPath.empty()) {
        std::cout << "No valid path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    *planlength = static_cast<int>(shortestPath.size());

    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    std::cout << "Time taken to build trees and find path: " << elapsed.count() << " seconds" << std::endl;

    // Free old plan memory before allocating a new one
    if (*plan != nullptr) {
        for (int i = 0; i < *planlength; ++i) {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double**)malloc(*planlength * sizeof(double*));
    if (!*plan) {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i) {
        (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i]) {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j) {
            if(i < shortestPath_A.size())
                (*plan)[i][j] = tree_A[shortestPath_A[i]].angles[j];
            else
                (*plan)[i][j] = tree_B[shortestPath_B[i - shortestPath_A.size()]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    rrt_connect.save_to_file(tree_A, tree_B, shortestPath);
    
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
	double eps = 1.0;
    std::vector<node> tree;
    RRT_Star_Planner rrt_star(x_size, y_size, numofDOFs, map, eps, armgoal_anglesV_rad);

    std::cout << "Identifying low cost regions" << std::endl;
    rrt_star.get_low_cost_regions(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    auto start = std::chrono::high_resolution_clock::now();    

    std::cout << "Building tree" << std::endl;
    rrt_star.build_tree(tree, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = tree.size();

    std::cout << "Reconstructing path from the tree" << std::endl;
    std::vector<int> shortestPath = rrt_star.reconstruct_path(tree);

    // Check if a valid path was found
    if (shortestPath.empty()) {
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
    if (*plan != nullptr) {
        for (int i = 0; i < *planlength; ++i) {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double**)malloc(*planlength * sizeof(double*));
    if (!*plan) {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i) {
        (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i]) {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j) {
            (*plan)[i][j] = tree[shortestPath[i]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    std::ofstream m_vertices_fstream;
    rrt_star.save_to_file(tree, shortestPath);
    
    std::cout << "Vertices saved to file!" << std::endl;
}

// PRM Planner
void plannerPRM(
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
    const int num_nodes = 200;
    std::vector<node> graph;
    PRM_Planner prm(x_size, y_size, numofDOFs, map);

    std::cout << "Identifying low cost regions" << std::endl;
    prm.get_low_cost_regions(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    auto start = std::chrono::high_resolution_clock::now();    

    std::cout << "Building roadmap" << std::endl;
    prm.build_roadmap(graph, num_nodes);

    std::cout << "Querying roadmap with the start and goal nodes" << std::endl;
    prm.query(graph, armstart_anglesV_rad, armgoal_anglesV_rad, num_nodes);

    vertices = graph.size();

    // Ensure start and goal nodes have neighbors
    if (graph[num_nodes].neighbors.empty() || graph[num_nodes + 1].neighbors.empty()) {
        std::cout << "Start or goal node has no valid connections!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    std::cout << "Running Dijkstra on the roadmap" << std::endl;
    std::vector<int> shortestPath = prm.dijkstra(graph, num_nodes, num_nodes + 1);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken to build roadmap and find path: " << elapsed.count() << " seconds" << std::endl;

    // Check if a valid path was found
    if (shortestPath.empty()) {
        std::cout << "No valid path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    *planlength = static_cast<int>(shortestPath.size());

    // Free old plan memory before allocating a new one
    if (*plan != nullptr) {
        for (int i = 0; i < *planlength; ++i) {
            free((*plan)[i]);
        }
        free(*plan);
    }

    *plan = (double**)malloc(*planlength * sizeof(double*));
    if (!*plan) {
        std::cerr << "Memory allocation failed for plan!" << std::endl;
        *planlength = 0;
        return;
    }

    for (int i = 0; i < *planlength; ++i) {
        (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
        if (!(*plan)[i]) {
            std::cerr << "Memory allocation failed for plan[" << i << "]!" << std::endl;
            *planlength = 0;
            return;
        }

        for (int j = 0; j < numofDOFs; ++j) {
            (*plan)[i][j] = graph[shortestPath[i]].angles[j];
        }
    }

    std::cout << "Path successfully extracted!" << std::endl;

    // write out the vertices to a file
    prm.save_to_file(graph, shortestPath);

    std::cout << "Vertices saved to file!" << std::endl;
}

int main(int argc, char** argv) {
	double* map;
	int x_size, y_size, vertices = 0;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	// planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as the 
	//// grading script will not work.
	///////////////////////////////////////

	if (whichPlanner == PRM) {
		std::cout << "Using PRM" << std::endl;
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

	else if (whichPlanner == RRT) {
		std::cout << "Using RRT" << std::endl;
		plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
	}

    else if (whichPlanner == RRTCONNECT) {
        std::cout << "Using RRT-Connect" << std::endl;
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

    else if (whichPlanner == RRTSTAR) {
        std::cout << "Using RRT*" << std::endl;
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, vertices);
    }

	else {
		std::cerr << "Invalid planner ID!" << std::endl;
		return 1;
	}

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
    std::ofstream m_vertices_fstream;
	m_vertices_fstream.open("vertices.txt", std::ios::out | std::ios::app);
    m_vertices_fstream << vertices << endl;

}
