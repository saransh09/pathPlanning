#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

float goal_distance_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory,  map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}

double inefficiency_cost(Vehicle & trajectory, map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    double added_cost = 1;
    double proposed_speed_intended = lane_speed(trajectory,predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        if((trajectory.state == "PLCL" && trajectory.lane ==0 )|| (trajectory.state =="PLCR" && trajectory.lane ==2)){
            proposed_speed_intended = 0;
            added_cost = 100;
        }
        else{
            proposed_speed_intended = trajectory.target_speed;
        }
    }

    double proposed_speed_final = lane_speed(trajectory,predictions, data["final_lane"]);
    if (proposed_speed_final < 0){

        proposed_speed_final = trajectory.target_speed;
    }

    double cost = (2.0*trajectory.target_speed - proposed_speed_intended - proposed_speed_final)/trajectory.target_speed;

    cost*=added_cost;
    return cost;
}

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> & predictions, int lane) {
   
    Vehicle front_vehicle ;
    bool found_vehicle = vehicle.get_vehicle_in_lane_ahead(predictions,lane,front_vehicle);
    if(found_vehicle)
    {
        if(front_vehicle.s > vehicle.s + vehicle.v){
            return -1;
        }
        else return front_vehicle.v;
    }
    Vehicle rear_vehicle;
    found_vehicle = vehicle.get_vehicle_in_lane_behind(predictions,lane,rear_vehicle);
    if(found_vehicle){
        if(vehicle.s>rear_vehicle.s+rear_vehicle.v){
            return -1;
        }
        else return rear_vehicle.v;
    }
    return -1.0;
}


double collision_distance(double present_car_s,Vehicle &trajectory, map<int, vector<Vehicle>> & predictions,int lane){
    std::vector<double> collision_data;
    double dist = 0;
    Vehicle front_vehicle;
    Vehicle rear_vehicle;
    Vehicle copy = trajectory;
    copy.s = present_car_s;

    bool found_vehicle_front = copy.get_vehicle_ahead(predictions,lane,front_vehicle);
    bool found_vehicle_back  = copy.get_vehicle_behind(predictions,lane,rear_vehicle);

    if(!found_vehicle_front && !found_vehicle_back){
        return 0;
    }

    else if(found_vehicle_front || found_vehicle_back){
        if(found_vehicle_front && !found_vehicle_back){
            dist+=(front_vehicle.s - copy.s);
            dist =  1 - dist/(horizon);
            return dist;
        }
        else if(!found_vehicle_front && found_vehicle_back){
            dist+=(copy.s - rear_vehicle.s);
            dist = 1 - dist/(horizon);
            return dist;
        }
        else if(found_vehicle_front && found_vehicle_back){
            dist+=(front_vehicle.s - rear_vehicle.s);
            dist = 1 - dist/(2*horizon);
            return dist;
        }    
    }
    return dist;
}


double collision_cost(double present_car_s,Vehicle &trajectory,map<int, vector<Vehicle>> & predictions, map<string, float> & data){
    double cost = collision_distance(present_car_s,trajectory,predictions,data["intended_lane"]);    
    return cost;
}


double get_lane_cost(Vehicle &trajectory,map<string,float>&data){
    return abs(data["final_lane"] - data["intended_lane"]);
}

double calculate_cost( Vehicle & vehicle,  map<int, vector<Vehicle>> & predictions,  vector<Vehicle> & trajectory) {
    double cost = 0;

    for(int i=0;i<trajectory.size();i++){
        map<string,float>trajectory_data = get_helper_data(vehicle,trajectory[i],predictions);
        
        cost+=600*inefficiency_cost(trajectory[i],predictions,trajectory_data);
        cost+=10*get_lane_cost(trajectory[i],trajectory_data);
        if(trajectory[i].state == "PLCL" || trajectory[i].state =="PLCR" || trajectory[i].state=="KL"){
            if(trajectory[i].state =="KL"){
                cost+=exp(10*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data));
            }
            else{
                cost+=exp(15*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data));
            }
        }
    }
    return cost;
}


map<string,float>get_helper_data( Vehicle & vehicle,  Vehicle & trajectory,  map<int, vector<Vehicle>> & predictions){
    map<string,float>trajectory_data;
    int intended_lane;
    if(trajectory.state.compare("PLCL")==0){
        intended_lane = trajectory.lane-1;
    }
    else if(trajectory.state.compare("PLCR")==0){
        intended_lane = trajectory.lane+1;
    }
    else intended_lane = trajectory.lane;

    int final_lane = trajectory.lane;

    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;

    return trajectory_data;
}