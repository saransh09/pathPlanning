#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "cost.h"



using namespace std;

using json = nlohmann::json;

static double max_s = 6945.554;

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef){
		frenet_d *= -1;
	}

	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++){
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}

vector<pair<double,double>> pairsort(vector<double>a, vector<double>b, int n){ 
    vector<pair<double,double>> vect; 
  
    for (int i = 0; i < n; i++)  { 
        vect.push_back(make_pair(a[i],b[i]));
    } 
  
    sort(vect.begin(), vect.end());
    
    return vect;
    }

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )){
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

tk::spline generate_spline(vector<double> x, vector<double> y){
  tk::spline x_to_y;
  x_to_y.set_points(x, y);
  return x_to_y;
}

bool sort_by_s(vector<double> i, vector<double> j){return (i[2] < j[2]);}

void sort_map_waypoints(vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s){
  vector<vector<double>> map_wps;
  for(int i = 0; i<maps_x.size(); i++){
    map_wps.push_back({maps_x[i], maps_y[i], maps_s[i]});
  }
  sort(map_wps.begin(), map_wps.end(), sort_by_s);

  maps_x.clear();
  maps_y.clear();
  maps_s.clear();

  for(int i = 0; i<map_wps.size(); i++){
    maps_x.push_back(map_wps[i][0]);
    maps_y.push_back(map_wps[i][1]);
    maps_s.push_back(map_wps[i][2]);
  }
}

vector<vector<double>> upsample_map_waypoints(vector<double> maps_x, vector<double> maps_y, vector<double> maps_s){
  sort_map_waypoints(maps_x, maps_y, maps_s);
  tk::spline s_to_x = generate_spline(maps_s, maps_x);
  tk::spline s_to_y = generate_spline(maps_s, maps_y);
  vector<double> x_upsampled, y_upsampled, s_upsampled;
  for(double i = 0; i<=max_s; i++){ 
    x_upsampled.push_back(s_to_x(i));
    y_upsampled.push_back(s_to_y(i));
    s_upsampled.push_back(double(i));
  }
  return {x_upsampled, y_upsampled, s_upsampled};
}


int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> ref_vel_list;
  Vehicle ego = Vehicle();

  double buffer_distance = 10;
  bool first_time= true;
  tk::spline s_prev;

  string map_file_ = "../data/highway_map.csv";
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&buffer_distance,&ref_vel_list,&first_time,&s_prev,&ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double dist = 0.5;
            double next_s;
            double next_d = 6;
            double lane_number = 1;

            int prev_size = previous_path_x.size();
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double ref_s;

            double ref_vel = 49.5;

            double future_car_s = end_path_s;
            
            vector<double>pts_x;
            vector<double>pts_y;

            if(prev_size<2)
            {

              cout<<"first"<<end_path_s<<" "<<car_s<<endl;
              ego = Vehicle(car_speed*0.447,car_s,0,1,"KL");
              vector<Vehicle> preds = ego.generate_predictions(sensor_fusion,prev_size,car_s,horizon);

              map<int,vector<Vehicle>> predictions;
              for(int i=0;i<preds.size();i++){
                predictions[i].push_back(preds[i]);
              }

              Vehicle future_car = ego.choose_next_state(predictions);
              ego = future_car;
              lane_number = future_car.lane;

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              double prev_car_s = getFrenet(prev_car_x,prev_car_y,car_yaw,map_waypoints_x,map_waypoints_y)[0];
              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);

              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);

              vector<double> future_car_xy = getXY(future_car.s+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              pts_x.push_back(future_car_xy[0]);
              pts_y.push_back(future_car_xy[1]);

              double vel = car_speed*0.447;
              double a = future_car.a;

              for(int i=0;i<50;i++){
                vel+=0.02*a;
                vel = min(48*0.447,vel);
                ref_vel_list.push_back(vel);
              }

              ref_s = car_s;
            }
            else{

              string present_state = ego.state;
              double present_a = ego.a;
              int present_lane = ego.lane;



              ego = Vehicle(car_speed*0.447,car_s,present_a,present_lane,present_state);

              vector<Vehicle> preds = ego.generate_predictions(sensor_fusion,prev_size,car_s,horizon);

              map<int,vector<Vehicle>> predictions;
              for(int i=0;i<preds.size();i++){
                predictions[i].push_back(preds[i]);
              }

              Vehicle future_car = ego.choose_next_state(predictions);
              ego = future_car;
              lane_number = future_car.lane;

              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              double t = ref_x - ref_x_prev;
              ref_yaw = atan2((ref_y-ref_y_prev),t);

              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);

              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);

              vector<double> future_car_xy = getXY(future_car.s+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              pts_x.push_back(future_car_xy[0]);
              pts_y.push_back(future_car_xy[1]);

              double vel = car_speed*0.447;
              double a = future_car.a;

              for(int i=0;i<prev_size;i++){
                ref_vel_list[i] = ref_vel_list[i+50-prev_size-1];
              }

              for(int i=prev_size;i<50;i++){
                vel  = ref_vel_list[i-1]+0.02*a;
                vel = min(vel,48*0.447);
                ref_vel_list[i] = vel;
              }

              ref_s = end_path_s;
            }


            vector<double>ref_waypoint_s = getFrenet(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y);
            double ref_s_ = ref_waypoint_s[0];

            double ref_s_1 = ref_s_;
            double ref_s_2 = ref_s_;
            double ref_s_3 = ref_s_;

            if(ref_s_ + 30 >max_s){
              ref_s_1  = max_s - ref_s_;
            }
            if(ref_s_ + 60 > max_s){
              ref_s_2  = max_s - ref_s_;
            }
            if(ref_s_ + 90 >max_s){
              ref_s_3  = max_s - ref_s_;
            }

            if(prev_size>0){
              car_s = end_path_s;
            }

            vector<double> next_wp0 = getXY(ref_s_1+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            vector<double> next_wp1 = getXY(ref_s_2+60,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            vector<double> next_wp2 = getXY(ref_s_3+90,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            pts_x.push_back(next_wp0[0]);
            pts_x.push_back(next_wp1[0]);
            pts_x.push_back(next_wp2[0]);

            pts_y.push_back(next_wp0[1]);
            pts_y.push_back(next_wp1[1]);
            pts_y.push_back(next_wp2[1]);

            tk::spline s;
            

            for (int i = 0; i < pts_x.size(); i++) {
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;

              pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }

            s.set_points(pts_x,pts_y);

            int start;
             if(prev_size<2){
              next_x_vals.push_back(car_x);
              next_y_vals.push_back(car_y);
              start = 1;
             }
             else{
              for(int i=0;i<prev_size;i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
              start = prev_size;
             }
          
            double x_add_on = 0;

            for(int i=start;i<50;i++){
              double x_value = next_x_vals[i-1];
              double y_value = next_y_vals[i-1];
              
              double x_point = x_add_on + 0.02*ref_vel_list[i-1];
              x_add_on = x_point;
              double y_point = s(x_point);

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;          

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            s_prev.set_points(pts_x,pts_y);
            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);            
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
