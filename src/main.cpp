#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// some constants
const double DIST_TOO_CLOSE = 30; // meters
const double MAX_VEL = 49.0; //mph
const double MAX_ACC = 0.224; // =5m/s2
const double MAX_DEC = 0.224; // =10m/s2
const double FREQ_CHANGE_COUNTER = 100;
int change_wait_timer = 100;
bool change_allowed = false;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // start in middle lane (Lane 1)
  int lane = 1;

  // set reference velocity
  double ref_vel = 0.0; //mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // size of previous path
          int prev_size = previous_path_x.size();

          if(prev_size>0)
          {
              car_s = end_path_s;
          }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          // >>>>> 1. PREDICTION
          // >>>>> use sensor fusion data to know where are the other cars
          bool front_too_close = false;
          bool left_too_close = false;
          bool right_too_close = false;

          for(int i=0; i<sensor_fusion.size(); i++){
              float d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += (double)prev_size*0.02*check_speed;

              if (d < (2+4*lane+2) && d > (2+4*lane-2)){
                  // we have decided this car is in our lane
                  // now need to see if this car is too close to ego car
                  if ((check_car_s > car_s) && (check_car_s - car_s < DIST_TOO_CLOSE)){
                      front_too_close = true;
                  }
              }
              else if (d > (2+4*lane)+2) {
                  // we have decided this car is in the lane on right
                  if ((check_car_s > car_s) && (check_car_s - car_s < 0.5*DIST_TOO_CLOSE)){
                      right_too_close = true;
                  }
                  else if ((check_car_s < car_s) && (car_s - check_car_s < 0.5*DIST_TOO_CLOSE)){
                      right_too_close = true;
                  }
              }
              else if (d < (2+4*lane)-2) {
                  // we have decided this car is in the lane on left
                  if ((check_car_s > car_s) && (check_car_s - car_s < 0.5*DIST_TOO_CLOSE)){
                      left_too_close = true;
                  }
                  else if ((check_car_s < car_s) && (car_s - check_car_s < 0.5*DIST_TOO_CLOSE)){
                      left_too_close = true;
                  }
              }
          }

          std::cout << "Left Hand Side Occupied: " << left_too_close << std::endl;
          std::cout << "Front Car Detected: " << front_too_close << std::endl;
          std::cout << "Right Hand Side Occupied: " << right_too_close << std::endl;


          // >>>>> 2. BEHAVIOR
          // >>>>> determine vehicle speed target and which lane to go next

          // A simple Adaptive-Cruise-Control which only looks at car in front
          if (front_too_close){
              ref_vel -= MAX_DEC;
          }
          else if (ref_vel < MAX_VEL){
              ref_vel += MAX_ACC;
          }

          // Decide which lane to go next.
          if (change_allowed && front_too_close && !left_too_close && lane>0){
              // Change to left lane if this lane occupied, AND left lane empty, AND not at left most lane
              lane -= 1;
              change_wait_timer = 0;
              change_allowed = false;
          }
          else if (change_allowed && front_too_close && !right_too_close && lane<2){
              // Change to right lane if no space on left, but right is open, AND not at right most lane
              lane += 1;
              change_wait_timer = 0;
              change_allowed = false;
          }

          // add a timer logic to avoid frequently changing lanes
          change_wait_timer += 1;
          if (change_wait_timer > FREQ_CHANGE_COUNTER) {
              change_allowed = true;
          }

          std::cout << "Lane Change Allowed: " << change_allowed << std::endl;
          std::cout << "Lane Number Selected: " << lane << std::endl;
          std::cout << "============================" << std::endl;



          // >>>>> 3. TRAJECTORY
          // >>>>> create a list of (x,y) points for the car to follow every 20ms

          // generate a list of widely spaced waypoints, evenly spaced at 30m
          // will use those later and fill it with more points to control speed
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if no previous path, use car localization data as starting reference
          if (prev_size < 2){
            double prev_car_x = car_x - cos(ref_yaw);
            double prev_car_y = car_y - sin(ref_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          // otherwise use previous path as starting reference
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // in Frenet add evenly 30m spaced points ahead of the starting references
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transform the points so that the first point is at origin and yaw 0
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points for the spline
          s.set_points(ptsx, ptsy);

          // create the actual (x,y) points for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i=0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that the desired refrence velocity is kept
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of the path planner after filling it with previous points, here we will always output 50 points
          for (int i=0; i <= 50-previous_path_x.size(); i++) {

            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // transfer back to normal coordinates
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
