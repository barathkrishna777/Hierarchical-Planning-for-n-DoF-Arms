#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h>
#include <queue>
#include <chrono>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

// define the directions vector for the 8-connected grid
#define numDirections 8
const int dx[numDirections] = { 1, 0, -1, 0, 1, -1, -1, 1 };
const int dy[numDirections] = { 0, 1, 0, -1, 1, -1, 1, -1 };
const double move_cost[numDirections] = { 1.0, 1.0, 1.0, 1.0, sqrt(2), sqrt(2), sqrt(2), sqrt(2) };

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 100

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

struct node {
    int id;
    std::vector<double> angles;
    std::vector<std::pair<int, double>> neighbors;
    double g;
    bool closed;
    int parent;

    node () {
        this->id = -1;
        g = std::numeric_limits<double>::infinity();
        closed = false;
        parent = -1;
    }
};

inline double distance(node n1, node n2) {
	double dist = 0;
	for (int i = 0; i < n1.angles.size(); ++i) {
		dist += pow(n1.angles[i] - n2.angles[i], 2);
	}
	return sqrt(dist);
}

bool obstacle_free(node n1, node n2, int numofDOFs, int x_size, int y_size, double* map) {
	double dist = distance(n1, n2);
	int numofsamples = std::max(1, (int)(dist / (PI / 100)));

	std::vector<double> config(numofDOFs);
	for (int i = 0; i < numofsamples; i++) {
		for (int j = 0; j < numofDOFs; j++)
			config[j] = n1.angles[j] + ((double)(i) / (numofsamples - 1)) * (n2.angles[j] - n1.angles[j]);

		if (!IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size))
			return false;
	}
	
	return true;
}

bool isCoarseCellOccupied(int coarse_x, int coarse_y, int coarse_factor, 
                          double* fine_map, int fine_x_size, int fine_y_size) {
    int start_fine_x = coarse_x * coarse_factor;
    int start_fine_y = coarse_y * coarse_factor;
    int end_fine_x = std::min(start_fine_x + coarse_factor, fine_x_size);
    int end_fine_y = std::min(start_fine_y + coarse_factor, fine_y_size);

    for (int fine_y = start_fine_y; fine_y < end_fine_y; ++fine_y) {
        for (int fine_x = start_fine_x; fine_x < end_fine_x; ++fine_x) {
            if (fine_map[GETMAPINDEX(fine_x, fine_y, fine_x_size, fine_y_size)] == 1.0) {
                return true;
            }
        }
    }
    return false;
}

inline double wrap2pi(double angle) {
	angle = fmod(angle, 2.0 * PI);
	if (angle < 0.0) {
		angle += 2.0 * PI;
	}
	return angle;
}

bool forwardKinematics(const double* angles, int numofDOFs, int x_size,
                       std::vector<std::pair<double, double>>& joint_positions_out) {

    if (numofDOFs <= 0) 
		return false;

    if (joint_positions_out.size() != static_cast<size_t>(numofDOFs + 1)) {
        joint_positions_out.resize(numofDOFs + 1);
    }

    double current_x = static_cast<double>(x_size) / 2.0;
    double current_y = 0.0;
    joint_positions_out[0] = {current_x, current_y};

    double accumulated_angle = 0.0;

    for(int i = 0; i < numofDOFs; i++){
        double link_angle_rad = angles[i];

        current_x += LINKLENGTH_CELLS * cos(link_angle_rad);
        current_y += LINKLENGTH_CELLS * sin(link_angle_rad);

        joint_positions_out[i + 1] = {current_x, current_y};
    }

    return true;
}

double* inverseKinematics(double target_x, double target_y, int numofDOFs, int x_size,
                           double initial_guess[] = nullptr,
                           int max_iterations = 100, double tolerance = 1.0) {

    if (numofDOFs <= 0) return nullptr;

    double base_x = static_cast<double>(x_size) / 2.0;
    double base_y = 0.0;

    double target_dist_sq = pow(target_x - base_x, 2) + pow(target_y - base_y, 2);
    double max_reach = numofDOFs * LINKLENGTH_CELLS;
    if (target_dist_sq > pow(max_reach, 2) + tolerance) {
        return nullptr;
    }
     if (target_dist_sq < 1e-6 && numofDOFs > 0) {
         // Handle singularity at the base: return a default config (e.g., all zeros)
         // if the target is exactly the base.
         // std::cerr << "IK Warning: Target is very close to the base." << std::endl;
         // For simplicity, returning nullptr, but a specific configuration might be better.
         // return nullptr;
     }


    double* current_angles = new double[numofDOFs];
    if (initial_guess != nullptr) {
        for (int i = 0; i < numofDOFs; ++i) {
            current_angles[i] = wrap2pi(initial_guess[i]);
        }
    } else {
        for (int i = 0; i < numofDOFs; ++i) {
            current_angles[i] = 0.0;
        }
    }

    std::vector<std::pair<double, double>> joint_positions(numofDOFs + 1);


    // --- CCD Iteration Loop ---
    for (int iter = 0; iter < max_iterations; ++iter) {

        forwardKinematics(current_angles, numofDOFs, x_size, joint_positions);
        double current_ee_x = joint_positions[numofDOFs].first;
        double current_ee_y = joint_positions[numofDOFs].second;

        double error_sq = pow(current_ee_x - target_x, 2) + pow(current_ee_y - target_y, 2);
        if (error_sq < tolerance * tolerance) {
             for (int i = 0; i < numofDOFs; ++i) {
                 current_angles[i] = wrap2pi(current_angles[i]);
             }
            return current_angles;
        }

        for (int j = numofDOFs - 1; j >= 0; --j) {
            forwardKinematics(current_angles, numofDOFs, x_size, joint_positions);
            double joint_j_x = joint_positions[j].first;
            double joint_j_y = joint_positions[j].second;
            double current_ee_x_loop = joint_positions[numofDOFs].first;
            double current_ee_y_loop = joint_positions[numofDOFs].second;

            double vec_cur_x = current_ee_x_loop - joint_j_x;
            double vec_cur_y = current_ee_y_loop - joint_j_y;
            double vec_tar_x = target_x - joint_j_x;
            double vec_tar_y = target_y - joint_j_y;

            double angle_cur = atan2(vec_cur_y, vec_cur_x);
            double angle_tar = atan2(vec_tar_y, vec_tar_x);
            double delta_angle = angle_tar - angle_cur;

            current_angles[j] = wrap2pi(current_angles[j] + delta_angle);
        }
    }

    forwardKinematics(current_angles, numofDOFs, x_size, joint_positions);
    double final_error = sqrt(pow(joint_positions[numofDOFs].first - target_x, 2) + pow(joint_positions[numofDOFs].second - target_y, 2));

    delete[] current_angles;
    return nullptr;
}

#endif // UTILS_H