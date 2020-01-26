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
#include <math.h>
//#include <thread>
//#include <chrono>


// for convenience
using json = nlohmann::json;

int main() 
{
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
  //  std::cout << "size of map_waypoints_x =" << map_waypoints_x.size() << std::endl;
  //  print(map_waypoints_x);

  // start in lane 1
  int lane = 1;
  // have a reference velocity to target 
  double ref_vel = 0; //mph  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event            
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(data);

      if (s != "") 
      {
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

      /*
      * TODO: define a path made up of (x,y) points that the car will visit
      *   sequentially every .02 seconds
      */
      
      // Step 2  Don't crash into the car in front of us   
      // Start of my TODO code: 
          int prev_size = previous_path_x.size(); 
      
          if (prev_size > 0){
            car_s = end_path_s;
          }

          // define  
          bool slow_down = false; 
          bool lane0_clear_front = true; 
          bool lane1_clear_front = true; 
          bool lane2_clear_front = true; 
          bool lane0_clear_rear = true; 
          bool lane1_clear_rear = true; 
          bool lane2_clear_rear = true; 
          // create empty vectors 
          vector<double> car_lane0_front;
          vector<double> car_lane1_front;
          vector<double> car_lane2_front;
          vector<double> car_lane0_rear;
          vector<double> car_lane1_rear;
          vector<double> car_lane2_rear;
      
          // This for loop is to check each car's location in lanes 
          for (int i = 1; i < sensor_fusion.size(); i++) {  
            float d = sensor_fusion[i][6]; 
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            double check_car_s1 = check_car_s + ((double)prev_size * 0.02 * check_speed);

            if ((check_car_s > car_s) && abs(check_car_s1 - car_s) < 50){
              if (d < 4 && d >= 0) {
                car_lane0_front.push_back(check_car_s);
              } else if (d < 8 && d >= 4) {
                car_lane1_front.push_back(check_car_s);
              } else if (d < 12 && d >= 8) {
                car_lane2_front.push_back(check_car_s);
              } else {
                continue;
              }
            }
            else if ((check_car_s < car_s) && abs(car_s - check_car_s1) < 30){
              if (d < 4 && d >= 0) {
                car_lane0_rear.push_back(check_car_s);
              } else if (d < 8 && d >= 4) {
                car_lane1_rear.push_back(check_car_s);
              } else if (d < 12 && d >= 8) {
                car_lane2_rear.push_back(check_car_s);
              } else {
                continue;
              }
            }
            else {
              continue;
            }
          } // end of for loop 
          
          // print out for debug
          std::cout << " car_lane0_front: " << car_lane0_front.size() << std::endl;
          std::cout << " car_lane1_front: " << car_lane1_front.size() << std::endl;
          std::cout << " car_lane2_front: " << car_lane2_front.size() << std::endl;
          std::cout << " car_lane0_rear: " << car_lane0_rear.size() << std::endl;
          std::cout << " car_lane1_rear: " << car_lane1_rear.size() << std::endl;
          std::cout << " car_lane2_rear: " << car_lane2_rear.size() << std::endl;
          std::cout << " empty:                                   " << std::endl;

          // Step 3: make decision: lane change 
          if (car_lane0_front.size() > 0){
            lane0_clear_front = false;  
          //std::cout << " lane = 0f: " << car_lane0_front[0] <<std::endl; 
          } else {
            lane0_clear_front = true;
          }
          if (car_lane1_front.size()> 0){
            lane1_clear_front = false;
          //std::cout << " lane = 1f: " << car_lane1_front[0] <<std::endl; 
          } else {
            lane1_clear_front = true;
          }
          if (car_lane2_front.size()> 0){
            lane2_clear_front = false;
            //std::cout << " lane = 2f: " << car_lane2_front[0] <<std::endl; 
          } else {
            lane2_clear_front = true;
          }
          if (car_lane0_rear.size()> 0){
            lane0_clear_rear = false;  
          //std::cout << " lane = 0r: " << car_lane0_rear[0] <<std::endl; 
          } else {
            lane0_clear_rear = true;
          }
          if (car_lane1_rear.size()> 0){
            lane1_clear_rear = false;
          //std::cout << " lane = 1r: " << car_lane1_rear[0] <<std::endl; 
          } else {
            lane1_clear_rear = true;
          }
          if (car_lane2_rear.size()> 0){
            lane2_clear_rear = false;
          //std::cout << " lane = 2r: " << car_lane2_rear[0] <<std::endl; 
          } else {
            lane2_clear_rear = true;
          }
      
          //std::cout << " car_s: " << car_s <<std::endl;
          //std::cout << " lane0_clear_front: " << lane0_clear_front <<std::endl; 
          //std::cout << " lane1_clear_front: " << lane1_clear_front <<std::endl; 
          //std::cout << " lane2_clear_front: " << lane2_clear_front <<std::endl; 
          //std::cout << " lane0_clear_rear: " << lane0_clear_rear <<std::endl; 
          //std::cout << " lane1_clear_rear: " << lane1_clear_rear <<std::endl; 
          //std::cout << " lane2_clear_rear: " << lane2_clear_rear <<std::endl; 
          //std::cout << " empty:               " <<std::endl; 

          // if the ego car is in lane = 0 

          if (lane == 0 && lane0_clear_front == false) {
            // slow_down first
            slow_down = true; 
            if (lane1_clear_front == true && lane1_clear_rear == true){
              lane = 1;  
            } 
            else if (lane1_clear_front == false && lane1_clear_rear == true) {
              ref_vel -= 0.224/4; 
            }
            else {
              //ref_vel -= 0.224;
              slow_down = true;
            }
          } 


          // if the ego car is in lane = 1 
          // only when the car just start in lane 1   
          if ((lane == 1) && (lane1_clear_rear == false) && (lane1_clear_front == true)) {
              if (ref_vel < 49.5) {
                ref_vel += 0.224;
              }
          } 
          // if there is a car in front of the ego car
          else if (lane == 1 && lane1_clear_front == false) { 
            //slow down first 
            slow_down = true; 
            // check if it is safe of make a lane chane 
            // if the left lane is clear, move to left lane
            if (lane0_clear_front == true && lane0_clear_rear == true) {
              lane = 0;
            } 
            // if left lane is not the option, then check the right lane 
            else if (lane2_clear_front == true && lane2_clear_rear ==true ){
              lane = 2; 
            }
            // if left lane is not ready, then slow down 
            else if (lane0_clear_front == false && lane0_clear_rear == true) {
              ref_vel -= 0.224/2; 
            }  
            // if right lane is not ready, then slow down 
            else if (lane2_clear_front == false && lane2_clear_rear == true) {
              ref_vel -= 0.224/2; 
            }
            else {
              //ref_vel -= 0.224;
              slow_down = true; 
            }
          } 



          // if the ego car is in lane = 2

          if (lane == 2 && lane2_clear_front == false) {
            // slow down first
            slow_down = true; 
            if (lane1_clear_front == true && lane1_clear_rear == true) {
              lane = 1;
            } 
            else if (lane1_clear_front == false && lane1_clear_rear == true) {
              ref_vel -= 0.224/4; 
            }       
            else {
              //ref_vel -= 0.224;
              slow_down = true;
            }
          } 

          if (slow_down){
            ref_vel -= 0.224;
          } else if (ref_vel < 49.5){
            ref_vel += 0.224;
          }
          
        // Step 1: Make the car stay in a specific lane 

        // Create a list of widely spaced (x,y) waypoints, evently spaced at 30m 
        // Later we will interopate these waypoints with a spline and fill it in with more points that control speed  
          vector<double> ptsx;
          vector<double> ptsy;

        // reference x,y, yaw states
        // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);  
          
        // if previous size is almost empty, use the car as starting reference 

          if (prev_size < 2){
            // use two points that make the path tangent to the car
            // use the current car position, we can back calculate the previous points to calcuate the tangent of the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2]; 
            double ref_y_prev = previous_path_y[prev_size-2]; 
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size();i++){
            //transfer the global coordinate to car coordinate
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] =(shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] =(shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));  
          }

          // create a spline 
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          //define the actual (x,y) points we will use for the planner 
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i=0; i< previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          } 

          double target_x = 30.0 ;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on = 0; 

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points.

          for (int i=1; i <= 50 - previous_path_x.size(); i++){
            double N = (target_dist/(0.02*ref_vel/2.24)); 
            double x_point = x_add_on + (target_x)/N;  
            double y_point = s(x_point);
            
            x_add_on = x_point;

            double x_ref = x_point; 
            double y_ref = y_point;
          
        // tranfer back from car coordinates to global coordinates

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y; 

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          } // end of for loop

//END of my TODO Code

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