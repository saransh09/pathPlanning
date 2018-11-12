#include <uWS/uWS.h>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <fstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "cost.h"

Vehicle::Vehicle(){};


Vehicle::Vehicle(double v,double s,double a,int lane,string state){
	this->v  = v;
	this->s = s;
	this->a = a;
	this->lane = lane;
	this->state = state;
}


Vehicle::~Vehicle(){};

Vehicle Vehicle:: choose_next_state(map<int,vector<Vehicle>>predictions){
	vector<string>states = get_successor_states();
	double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<Vehicle> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    	cout<<*it<<" ";
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            cout<<cost<<endl;
            costs.push_back(cost);
            final_trajectories.push_back(trajectory[0]);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    cout<<"selected state"<<" "<<final_trajectories[best_idx].state<<" "<<final_trajectories[best_idx].lane<<endl; 
    return final_trajectories[best_idx];
}


vector<string> Vehicle:: get_successor_states(){
  vector<string>states;
  states.push_back("KL");

  string current_state = this->state;

  if(current_state.compare("KL") ==0){
    states.push_back("PLCL");
    states.push_back("PLCR");
  }
  else if(current_state.compare("PLCL")==0){
    if(this->lane>0){
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  else if(current_state.compare("PLCR")==0){
    if(this->lane<2){
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,std::map<int, std::vector<Vehicle>> sensor_fusion){
	vector<Vehicle>trajectory;

	if(state.compare("KL")==0){
		trajectory = keep_lane_trajectory(sensor_fusion);
	}
	else if(state.compare("PLCL")==0 || state.compare("PLCR")==0){
		trajectory = prep_lane_change_trajectory(state,sensor_fusion);
    }
	else if(state.compare("LCL")==0 || state.compare("LCR")==0){
		trajectory = lane_change_trajectory(state,sensor_fusion);
	}
	return trajectory;
}


vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v;
        } else {
        	double s = this->s + this->v;
            double max_velocity_in_front = (vehicle_ahead.s - s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_accel = min(max(-10.0,new_accel),10.0);
    new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};    
}




vector<Vehicle> Vehicle::keep_lane_trajectory(map<int,vector<Vehicle>> predictions){
	std::vector<Vehicle> trajectory;
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(new_v, new_s, new_a, this->lane, "KL"));
    return trajectory;
} 


vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    bool is_change_possible = 1;
    Vehicle f_vehicle;
    bool found_vehicle = get_vehicle_ahead(predictions,new_lane,f_vehicle);
    if(found_vehicle==true){
    	if(f_vehicle.s-20<=this->s){
    		is_change_possible = 0;
    	}
    }
    else{
    	Vehicle b_vehicle;
	    bool found_vehicle = get_vehicle_behind(predictions,new_lane,b_vehicle);
	    if(found_vehicle==true){
	    	if(b_vehicle.s+20>=this->s){
	    		is_change_possible = 0;
	    	}
	    }
    }
    
    if(!is_change_possible){
    	return trajectory;
    }
    else{
	    vector<double> kinematics = get_kinematics(predictions, new_lane);
    	trajectory.push_back(Vehicle(kinematics[1], kinematics[0], kinematics[2], new_lane, state));
    	return trajectory;
    }
}


vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions){
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle>trajectory;

    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    } 
    else{
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(new_v, new_s, new_a, this->lane, state));
    return trajectory;
}



bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    int max_s = -10000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    int min_s = 100000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_in_lane_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & r2Vehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = 100000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            r2Vehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_in_lane_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & f2Vehicle) {
    int max_s = -10000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            f2Vehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}



vector<Vehicle> Vehicle::generate_predictions(std::vector<std::vector<double>> sensor_fusion,int prev_size,double car_s,int horizon){
	vector<Vehicle>predictions;

	for(int i=0;i<sensor_fusion.size();i++)
	{
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double s = sensor_fusion[i][5];
		float d = sensor_fusion[i][6];

		double v = sqrt(vx*vx+vy*vy);
		

		// s +=0.02*prev_size*v;
		if((car_s - s <30 && car_s>s) || (s-car_s<30 && s>car_s))
		{
			

			int lane;
			if(d<4 && d>=0)
			{
				lane = 0;
			}
			else if(d<=8 && d>4)
			{
				lane = 1;
			}
			else
			{
				lane = 2;
			}
			

			Vehicle pred = Vehicle(v,s,0,lane,"CS");
			predictions.push_back(pred);
		}
	}
	return predictions;
}

