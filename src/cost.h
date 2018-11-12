#ifndef COST_H
#define COST_H
#include "vehicle.h"



using namespace std;
static double horizon = 30;

double calculate_cost(Vehicle & vehicle,  map<int, vector<Vehicle>> & predictions, vector<Vehicle> & trajectory);
float goal_distance_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory,   map<int, vector<Vehicle>> & predictions, map<string, float> & data);

double inefficiency_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory, map<int, vector<Vehicle>> & predictions, map<string, float> & data);

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> & predictions, int lane);

map<string, float> get_helper_data(Vehicle & vehicle, Vehicle & trajectory, map<int, vector<Vehicle>> & predictions);

double get_lane_cost( Vehicle &trajectory,map<string,float>&data);
double collision_cost( Vehicle &trajectory, map<int, vector<Vehicle>> & predictions, map<string, float> & data);
double collision_distance( Vehicle &trajectory,map<int, vector<Vehicle>> & predictions, int lane);
#endif