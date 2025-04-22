/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex>    // For regex and split logic
#include <iostream> // cout, endl
#include <fstream>  // For reading/writing files
#include <eigen3/Eigen/Dense>

/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y * XSIZE + X)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 250

using Eigen::Matrix4d;
using Eigen::Vector3d;
using std::cout;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

/// @brief
/// @param filepath
/// @return map, x_size, y_size
tuple<double *, int, int> loadMap(string filepath)
{
  std::FILE *f = fopen(filepath.c_str(), "r");
  if (f)
  {
  }
  else
  {
    printf("Opening file failed! \n");
    throw runtime_error("Opening map file failed!");
  }
  int height, width;
  if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2)
  {
    throw runtime_error("Invalid loadMap parsing map metadata");
  }

  ////// Go through file and add to m_occupancy
  // m_occupancy.resize(height, std::vector<bool>(width, false)); // (H,W)
  double *map = new double[height * width];

  double cx, cy, cz;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      char c;
      do
      {
        if (fscanf(f, "%c", &c) != 1)
        {
          throw runtime_error("Invalid parsing individual map data");
        }
      } while (isspace(c));
      if (!(c == '0'))
      {
        map[y + x * width] = 1;
      }
      else
      {
        map[y + x * width] = 0;
      }
    }
  }
  fclose(f);
  return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string &str, const string &delim)
{
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
  const std::regex ws_re(delim);
  return {std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator()};
}

double *doubleArrayFromString(string str)
{
  vector<string> vals = split(str, ","); // Don't need to worry about trailing comma like in python
  double *ans = new double[vals.size()];
  for (int i = 0; i < vals.size(); ++i)
  {
    ans[i] = std::stod(vals[i]);
  }
  return ans;
}

typedef struct
{
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

void ContXY2Cell(double x, double y, short unsigned int *pX, short unsigned int *pY, int x_size, int y_size)
{
  double cellsize = 1.0;
  // take the nearest cell
  *pX = (int)(x / (double)(cellsize));
  if (x < 0)
    *pX = 0;
  if (*pX >= x_size)
    *pX = x_size - 1;

  *pY = (int)(y / (double)(cellsize));
  if (y < 0)
    *pY = 0;
  if (*pY >= y_size)
    *pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
    params->Y1 = p1x;
    params->X1 = p1y;
    params->Y2 = p2x;
    params->X2 = p2y;
  }
  else
  {
    params->X1 = p1x;
    params->Y1 = p1y;
    params->X2 = p2x;
    params->Y2 = p2y;
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

  params->DeltaX = params->X2 - params->X1;
  params->DeltaY = params->Y2 - params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  }
  else
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

bool equalDoubleArrays(double *v1, double *v2, int size)
{
  for (int i = 0; i < size; ++i)
  {
    if (abs(v1[i] - v2[i]) > 1e-3)
    {
      cout << endl;
      return false;
    }
  }
  return true;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double *map,
                       int x_size, int y_size)

{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  // printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  // make sure the line segment is inside the environment
  if (x0 < 0 || x0 >= x_size ||
      x1 < 0 || x1 >= x_size ||
      y0 < 0 || y0 >= y_size ||
      y1 < 0 || y1 >= y_size)
    return 0;

  ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
  ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

  // printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  // iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do
  {
    get_current_point(&params, &nX, &nY);
    if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1)
      return 0;
  } while (get_next_point(&params));

  return 1;
}

inline Matrix4d getTransformationMatrix(double theta, double d, double a, double alpha)
{
  Matrix4d T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
      sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
      0, sin(alpha), cos(alpha), d,
      0, 0, 0, 1;
  return T;
}

// New function to check a 3D line segment for collision with 2D obstacles
bool Check3DLinkCollision2DObstacles(double x0, double y0, double z0, double x1, double y1, double z1,
                                     double *map, int x_size, int y_size)
{
  // Determine the bounding box in the XY-plane of the 3D link
  double min_x = std::min(x0, x1);
  double max_x = std::max(x0, x1);
  double min_y = std::min(y0, y1);
  double max_y = std::max(y0, y1);

  short unsigned int start_x_cell, start_y_cell, end_x_cell, end_y_cell;

  ContXY2Cell(min_x, min_y, &start_x_cell, &start_y_cell, x_size, y_size);
  ContXY2Cell(max_x, max_y, &end_x_cell, &end_y_cell, x_size, y_size);

  // Iterate through the grid cells within the bounding box
  for (int y = start_y_cell; y <= end_y_cell; ++y)
  {
    for (int x = start_x_cell; x <= end_x_cell; ++x)
    {
      if (map[GETMAPINDEX(x, y, x_size, y_size)] == 1)
      {
        // Check if the line segment intersects the plane z=0 at this (x, y)
        // We can parameterize the line segment:
        // x(t) = x0 + t(x1 - x0)
        // y(t) = y0 + t(y1 - y0)
        // z(t) = z0 + t(z1 - z0)
        // where 0 <= t <= 1

        // If the z-component changes sign or is zero at some point t,
        // and the corresponding x(t) and y(t) fall within the current cell,
        // then there's a potential collision.

        if ((z0 <= 0 && z1 >= 0) || (z0 >= 0 && z1 <= 0))
        {
          double t;
          if (z1 != z0)
          {
            t = -z0 / (z1 - z0);
            if (t >= 0 && t <= 1)
            {
              double intersection_x = x0 + t * (x1 - x0);
              double intersection_y = y0 + t * (y1 - y0);

              short unsigned int cell_x, cell_y;
              ContXY2Cell(intersection_x, intersection_y, &cell_x, &cell_y, x_size, y_size);

              if (cell_x == x && cell_y == y)
              {
                return false; // Collision detected
              }
            }
          }
          else if (z0 == 0)
          {
            // If the entire segment has z=0, we need to check if any part of the
            // 2D projection overlaps the obstacle. We can use Bresenham's algorithm
            // on the 2D projection.
            bresenham_param_t params;
            short unsigned int nX0, nY0, nX1, nY1;
            ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
            ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);
            get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
            do
            {
              int current_x, current_y;
              get_current_point(&params, &current_x, &current_y);
              if (current_x == x && current_y == y)
              {
                return false; // Collision detected
              }
            } while (get_next_point(&params));
          }
        }
      }
    }
  }
  return true; // No collision detected within the bounding box
}

int IsValidArmConfiguration(double *angles, int numofDOFs,
                            double *map, int x_size, int y_size)
{
    if (numofDOFs != 6)
        throw std::runtime_error("This function is designed for 6-DoF arms.");

    double link_length = LINKLENGTH_CELLS;
    double d[6]     = {1.0, 0.0, 0.0, 1.0, 0.0, 1.0};
    double alpha[6] = {M_PI_2, 0.0, M_PI_2, -M_PI_2, M_PI_2, 0.0};
    double a[6]     = {0.0, link_length, link_length, 0.0, 0.0, 0.0};

    Matrix4d T_cumulative = Matrix4d::Identity();
    Vector3d prev_joint_pos(static_cast<double>(x_size) / 2.0, 0.0, 1.0);

    for (int i = 0; i < numofDOFs; ++i)
    {
        Matrix4d T_i = getTransformationMatrix(angles[i], d[i], a[i], alpha[i]);
        Matrix4d T_current = T_cumulative * T_i;
        Vector3d current_joint_pos = T_current.block<3, 1>(0, 3);

        if (i > 0)
        {
            if (prev_joint_pos.x() < 0 || prev_joint_pos.x() >= x_size ||
                prev_joint_pos.y() < 0 || prev_joint_pos.y() >= y_size ||
                current_joint_pos.x() < 0 || current_joint_pos.x() >= x_size ||
                current_joint_pos.y() < 0 || current_joint_pos.y() >= y_size)
            {
                return 0;  // Outside map bounds
            }

            if (!Check3DLinkCollision2DObstacles(prev_joint_pos.x(), prev_joint_pos.y(), prev_joint_pos.z(),
                                                 current_joint_pos.x(), current_joint_pos.y(), current_joint_pos.z(),
                                                 map, x_size, y_size))
            {
                return 0;  // Collision with map
            }
        }

        prev_joint_pos = current_joint_pos;
        T_cumulative = T_current;
    }

    return 1;
}

int IsValidStartGoalConfig(double *angles, int numofDOFs,
                           double *map, int x_size, int y_size)
{
    if (numofDOFs != 6)
        throw std::runtime_error("This function is designed for 6-DoF arms.");

    // first, check base cell
    int base_x = x_size / 2;
    int base_y = 0;
    if (map[GETMAPINDEX(base_x, base_y, x_size, y_size)] == 1)
        return 0;  // base is in an obstacle

    // DH parameters
    double link_length = LINKLENGTH_CELLS;
    double d[6]     = {1.0, 0.0, 0.0, 1.0, 0.0, 1.0};
    double alpha[6] = {M_PI_2, 0.0, M_PI_2, -M_PI_2, M_PI_2, 0.0};
    double a[6]     = {0.0, link_length, link_length, 0.0, 0.0, 0.0};

    // forward kinematics, checking each joint cell
    Matrix4d T_cumulative = Matrix4d::Identity();
    Vector3d current_pos, prev_pos;
    prev_pos << static_cast<double>(base_x), static_cast<double>(base_y), 1.0;

    for (int i = 0; i < numofDOFs; ++i)
    {
        // compute next transform
        Matrix4d T_i = getTransformationMatrix(angles[i], d[i], a[i], alpha[i]);
        T_cumulative = T_cumulative * T_i;
        current_pos = T_cumulative.block<3,1>(0,3);

        // project into map cells
        unsigned short cx, cy;
        ContXY2Cell(current_pos.x(), current_pos.y(), &cx, &cy, x_size, y_size);

        // check bounds
        if ((int)cx < 0 || cx >= x_size || (int)cy < 0 || cy >= y_size)
            return 0;  // joint lies outside map

        // check occupancy
        if (map[GETMAPINDEX(cx, cy, x_size, y_size)] == 1)
            return 0;  // joint is inside an obstacle

        // advance
        prev_pos = current_pos;
    }

    return 1;  // all joints (and base) are in free space
}

/** Your final solution will be verified by this script which will
 * send the default 5 arguments:
 *    map, numOfDOFs, startPos, goalPos, inputSolutionFile
 * Do NOT change this. Use this to check that your solution file output
 * is verified as valid using this script.
 * */
int main(int argc, char **argv)
{
    double *map;
    int x_size, y_size;

    // load map
    std::tie(map, x_size, y_size) = loadMap(argv[1]);
    int numOfDOFs = std::stoi(argv[2]);

    // parse start & goal
    double *startPos = doubleArrayFromString(argv[3]);
    double *goalPos  = doubleArrayFromString(argv[4]);
    std::string inputSolutionFile = argv[5];

    // 1) quick‐reject start
    if (!IsValidStartGoalConfig(startPos, numOfDOFs, map, x_size, y_size)) {
        std::cerr << "Start configuration collides with map\n";
        return -1;
    }
    // 2) quick‐reject goal
    if (!IsValidStartGoalConfig(goalPos, numOfDOFs, map, x_size, y_size)) {
        std::cerr << "Goal configuration collides with map\n";
        return -1;
    }

    // 3) now open & verify the trajectory itself
    std::ifstream infile(inputSolutionFile);
    if (!infile.is_open()) {
        std::cerr << "Failed to open solution file\n";
        return -1;
    }

    std::string curLine;
    double *curPos = nullptr, *nextPos = nullptr;

    // skip the first line (map comment)
    std::getline(infile, curLine);

    while (std::getline(infile, curLine)) {
        nextPos = doubleArrayFromString(curLine);

        // full collision check on every waypoint
        if (!IsValidArmConfiguration(nextPos, numOfDOFs, map, x_size, y_size)) {
            infile.close();
            std::cerr << "Trajectory point collides with map\n";
            return -1;
        }

        // start‐match check
        if (!curPos) {
            if (!equalDoubleArrays(nextPos, startPos, numOfDOFs)) {
                infile.close();
                std::cerr << "First waypoint does not match start\n";
                return -1;
            }
        }
        curPos = nextPos;
    }

    // final goal‐match check
    if (!equalDoubleArrays(curPos, goalPos, numOfDOFs)) {
        std::cerr << "Last waypoint does not match goal\n";
        return -1;
    }

    infile.close();
    std::cout << "Plan verified OK\n";
    return 0;
}
