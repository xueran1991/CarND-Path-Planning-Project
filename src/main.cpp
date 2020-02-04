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
//#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::sort;

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
  
  // ----My code beside TODO  
  double acc = 0.0;
  double ref_vel = 0.0;		//reference velocity for calculating the trajactories
  int lane = 1;		// 0:Left lane; 1:Middle lane; 2:Right lane
  
  // ----END
  
  // Add some variables I defined as parameters
  h.onMessage([&acc, &ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_speed = j[1]["speed"]; // in mph

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double max_vel = 49.0*1.61/3.6;	// Maximum velocity in m/s of the car
  		  double max_acc = 10.0;	// Maximum acceleration in m/s2 of the car
  		  double max_jerk = 10.0;	// Maximum jerk of the car in m/s3
  		  double dt = 0.02;			// Time for 1 cycle
          int previous_lane = lane;
          
          double cur_car_speed = car_speed * 1.61/3.6;	// Convert from mph to m/s 
          						// It's a little different with the ref_vel
          // FSM: KL, LCL, LCR
          bool too_close = 0;
          double target_vel = max_vel;
          
          vector<bool> LC_feasibility = checkCL_feasibility(sensor_fusion, car_s, car_d, lane, ref_vel);
          cout << "LC_0: " << LC_feasibility[0] 
               << "\tLC_1: " << LC_feasibility[1]
               << "\tLC_2: " << LC_feasibility[2] << endl;
          
          // Vector to store the car's velocity within the car's too close range
          vector<double> same_lane_cars_vel;
          
          for(auto ocar: sensor_fusion){
            //ocar: other car
            //sensor_fusion: 0:id, 1:x, 2:y, 3:vx, 4:vy, 5:s, 6:d
          	//cout << ocar << endl;
            double ocar_d = ocar[6];            
            
            if (ocar_d < (lane*4+4) && ocar_d > (lane*4)){
              //Check if the ocar in the same lane with me
              
              //cout << "Same lane; \n";
              double ocar_s = ocar[5];
              double ocar_vx = ocar[3];
              double ocar_vy = ocar[4];
              double ocar_vel = sqrt(ocar_vx*ocar_vx + ocar_vy*ocar_vy)*1.61/3.6; // in m/s
              
              if (ocar_s < end_path_s+20 && ocar_s > car_s){
                // Check my car is too close to the ocar.
                // If the car is between my car and the end of the planning path+20
                
                same_lane_cars_vel.push_back(ocar_vel);                
              }
              
            }
            
          }
          
          if(same_lane_cars_vel.size() > 0){
            cout << "Too close; cars in front: " << same_lane_cars_vel.size() << endl;
            too_close = 1;
            //sort(same_lane_cars_vel.begin(), same_lane_cars_vel.end());
            target_vel = same_lane_cars_vel[0];
            // Implentment a LC
            if(lane==0 && LC_feasibility[1]==1){
              //To the middle lane
              lane = 1;
            }
            if(lane==1){
              //
              if(LC_feasibility[2]==1){
                lane = 2;
              }
              if(LC_feasibility[0]==1){
                lane = 0;
              }
            }
            if(lane==2 && LC_feasibility[1]==1){
              //To the middle lane
              lane = 1;
            }            
            
          }else{
            too_close = 0;
            target_vel = max_vel;
          }          
          
	  // Velocity control
          if (ref_vel < target_vel){
          	ref_vel += max_acc*dt*2;
          }else if(ref_vel > target_vel){
          	ref_vel -= max_acc*dt*2;
            cout << "Speed down --- ";
          }
         
           
                 
          // Take 3 points in the future as the anchor point for spline
          // Anchor points for spline stored in anchor_points_x, anchor_points_y
          // Create a spline s
          tk::spline s;
          int anchor_point_num = 3;
          int anchor_point_interval = 50; // in meters, just for spline generation
          vector<double> anchor_points_x;
          vector<double> anchor_points_y;
          int previous_path_size = previous_path_x.size();
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          // Use previous path way points to make path smooth
          if(previous_path_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            anchor_points_x.push_back(prev_car_x);
            anchor_points_x.push_back(car_x);
            
            anchor_points_y.push_back(prev_car_y);
            anchor_points_y.push_back(car_y);
          }else{
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            
            double prev_ref_x = previous_path_x[previous_path_size-2];
            double prev_ref_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            
            anchor_points_x.push_back(prev_ref_x);
            anchor_points_x.push_back(ref_x);
            
            anchor_points_y.push_back(prev_ref_y);
            anchor_points_y.push_back(ref_y);
          }
          
          // Get anchor points
          // Consider smooth the 3 anchor when CL
          // ** Try to smooth the trajectory when implentment a LC 
          double next_anchor_d = previous_lane*4+2;
          for(int i=1; i<=anchor_point_num; ++i){
            // Take a anchor point in Frenet every 'anchor_point_interval' of s value
            double next_anchor_s = car_s + anchor_point_interval*i;
            // If car is going to implement LC, the anchor points space evenly between current lane and next lane
            //next_anchor_d += (lane - previous_lane)*4/3*i;
            next_anchor_d = lane*4+2;           
            vector<double> anchor_point = getXY(next_anchor_s, next_anchor_d, 
                                                map_waypoints_s, map_waypoints_x, map_waypoints_y);
            anchor_points_x.push_back(anchor_point[0]);
            anchor_points_y.push_back(anchor_point[1]);
          }
          
          // Shift car ref angle
          for(int i=0; i<anchor_points_x.size(); ++i){
          	double shift_x = anchor_points_x[i] - ref_x;
            double shift_y = anchor_points_y[i] - ref_y;
            
            anchor_points_x[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            anchor_points_y[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }
          
          // Generate spline s by anchor points
          s.set_points(anchor_points_x, anchor_points_y); 
          
          // Use previous path first
          for(int i=0; i<previous_path_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }          
          
          // Interpolate way points via spline
          for(int i=1; i<=50-previous_path_size; ++i){
            // Every path waypoint distance increasement = time increasement * velocity
            double x_point = ref_vel*dt*i;
            double y_point = s(x_point);
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Distense increasement in global coordinate
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
            // Get global-XY way point
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          //----------END-----------
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
