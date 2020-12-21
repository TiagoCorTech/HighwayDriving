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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          //std::cout << " Speed: "<< car_speed << " Distance: " << car_s <<  std::endl;
          
          //Model Parameters:
          double max_v = 49/2.24;  //MPH to m/s
          double dt   = 0.02; // 20 ms
          double max_a = 9; // m/s2
          int nPoints = 150;
          
          //Car's State: 
          int car_line = d2line(car_d);
          vector<double> car_frenet{car_s,car_d};
          double car_a = 5;
          
          
          //Inicio agregando datos que sobraron: 
          int path_size = previous_path_x.size();
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          std::cout<<"Ciclo : ------------------------ "<<std::endl;
          std::cout<<"path_size : "<<path_size<<std::endl;
			
          //Obtengo posicion del carro en la que va a quedar:
             
          
          
          
          double pos_x;
          double pos_y;
          double pos_yaw;
          double pos_speed;
          vector<double> pos_frenet;
          double pos_s;
          double pos_d;
          
          
          if (path_size > 2){
            
          pos_x = previous_path_x[path_size-1];
          pos_y = previous_path_y[path_size-1];

          double pos_x2 = previous_path_x[path_size-2];
          double pos_y2 = previous_path_y[path_size-2];

          pos_yaw = deg2rad(atan2(pos_y-pos_y2,pos_x-pos_x2));
          pos_speed = distance(pos_x, pos_y, pos_x2, pos_y2)/dt;

          pos_frenet = getFrenet(pos_x,pos_y,rad2deg(pos_yaw),map_waypoints_x,map_waypoints_y);
          pos_s = car_frenet[0];
          pos_d = car_frenet[1];    
            
          }else{
          
            pos_x = car_x;
            pos_y = car_y;
            pos_yaw = car_yaw;
            pos_speed = car_speed;
            pos_s = car_s;
            pos_d = car_d;
            
          }
          
          std::cout<<"pos_x : "<<pos_x<<std::endl;
          std::cout<<"pos_y : "<<pos_y<<std::endl;
          std::cout<<"pos_yaw : "<<pos_yaw<<std::endl;
          std::cout<<"pos_speed : "<<pos_speed<<std::endl;
          std::cout<<"pos_s : "<<pos_s<<std::endl;
          std::cout<<"pos_d : "<<pos_d<<std::endl;
                  
          
          
          //Verifico lineas ocupadas
          vector<int> lineStates{0,0,0};
          int sensor_fusion_size = sensor_fusion.size();
          
          for( int i = 0; i < sensor_fusion_size; i++){

          	double sf_line = d2line(sensor_fusion[i][6]);
            double sf_s = sensor_fusion[i][5];
            
            //Will we crash?
            if( ( sf_s - car_s < 50) && ( sf_s - car_s > 0) ){
              //Vas a chocar, debes reducir la velocidad o cambiar de carril, por ahora no hay cambio de carril:
				lineStates[sf_line] = 1;
                std::cout << "Linea Ocupada: " << sf_line << std::endl;
            }
 
          }
          
          std::cout<<"Linea 1: "<<lineStates[0]<<" Linea 2: "<<lineStates[1]<<" Linea 3: "<<lineStates[2]<<std::endl;
          
          int state = 0;
          // 0 -> Keep Foward, 1 -> Move Left, 2 -> move Right, 3 -> Slow Down 
            
          
          if(lineStates[car_line] == 0){
          	//Keep Foward
            state = 0; //Move foward
          }else{
          	for(int sumLine = -1; sumLine < 2 ; sumLine ++){
              int objLine = car_line + sumLine;
              if(objLine != car_line && objLine >= 0 && objLine <3){
                if(lineStates[objLine] == 0){
                  state =  sumLine == - 1 ? 1 : 2; 
                  break;
                }
              }
            }
            state = 3;            
          }
            
          std::cout<<"Estado: "<<state<<std::endl;
            
          double next_s;
          double next_d;
          double next_x; 
          double next_y;
          
          
          double fut1_s;
          double fut1_d;
          double fut2_s;
          double fut2_d;
          double fut3_s;
          double fut3_d;
          
          car_a = max_a;
          state = 0;
          //Create points
          switch (state){
            case 0: // Keep Foward
              {
                
              fut1_s = pos_s ;
              fut1_d = pos_d ;
              
              fut2_s = pos_s + 30;
              fut2_d = line2d(d2line(pos_d));
              
              fut3_s = pos_s + 60;
              fut3_d = line2d(d2line(pos_d));
              
              break;
              }
            case 1: // Move Left
              {
              int objLine = car_line - 1;
              
              fut1_s = pos_s;
              fut1_d = pos_d;
              
              fut2_s = pos_s + 30;
              fut2_d = line2d(objLine);
              
              fut3_s = pos_s + 60;
              fut3_d = line2d(objLine);
              
              
              break;
              }
            case 2: // Move Left
              {
              
              int objLine = car_line + 1;
              fut1_s = pos_s;
              fut1_d = pos_d;
              
              fut2_s = pos_s + 50;
              fut2_d = line2d(objLine);
              
              fut3_s = pos_d + 100;
              fut3_d = line2d(objLine);
              
              
              break;
              }
            case 3:
              {
              car_a = - max_a;
              fut1_s = pos_s ;
              fut1_d = pos_d;
              
              fut2_s = pos_s + 50;
              fut2_d = line2d(d2line(pos_d));
              
              fut3_s = pos_s + 100;
              fut3_d = line2d(d2line(pos_d));
              
              break;
              }
          
          }
          
          std::cout<<"fut1_s,d: "<<fut1_s<<","<<fut1_d<<std::endl;
          std::cout<<"fut2_s,d: "<<fut2_s<<","<<fut2_d<<std::endl;
          std::cout<<"fut3_s,d: "<<fut3_s<<","<<fut3_d<<std::endl;
          
          
          //Agrego trayectorias:
          vector<double> fut1_xy = getXY(fut1_s,fut1_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          double fut1_x = fut1_xy[0];
          double fut1_y = fut1_xy[1];
          vector<double> fut2_xy = getXY(fut2_s,fut2_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          double fut2_x = fut2_xy[0];
          double fut2_y = fut2_xy[1];
          vector<double> fut3_xy = getXY(fut3_s,fut3_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          double fut3_x = fut3_xy[0];
          double fut3_y = fut3_xy[1];

          std::cout<<"fut1 x,y: "<<fut1_x<<","<<fut1_y<<std::endl;
          std::cout<<"fut2 x,y: "<<fut2_x<<","<<fut2_y<<std::endl;
          std::cout<<"fut3 x,y: "<<fut3_x<<","<<fut3_y<<std::endl;

          
          vector<double>  X{fut1_x,fut2_x,fut3_x};
          vector<double>  Y{fut1_y,fut2_y,fut3_y};
          
          tk::spline spline;
          spline.set_points(X,Y);
          
          double dist;
          double dx;

          
          
          for (int i = 0; i < nPoints - path_size; ++i) { 
			
            
            car_a = (pos_speed + car_a*dt) > max_v ? 0 : car_a ;
            
            std::cout<<"Car_a: "<<car_a<< " ";
            
            pos_speed = pos_speed < 0 ? 0 : pos_speed + car_a*dt ;
            
            std::cout<<"Pos_speed: "<<pos_speed<< " ";
            
            dist = pos_speed*dt;
            
            std::cout<<"Distance: "<<dist<< " " ;

            
            double xd = sqrt(dist*dist/2);
            double new_dist = 0;
            
            while( new_dist < dist*0.9 ){
               
               xd *= 1.05;
                
               next_x = pos_x + xd;
               next_y = spline(next_x);
              
               new_dist = distance(next_x, next_y, pos_x, pos_y);
               
            }
            
            //pos_speed = distance(next_x, next_y, pos_x, pos_y)/dt;
            
            
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
            
            std::cout<<"Next X,Y: "<<next_x<<","<<next_y<<std::endl;
            
            pos_x = next_x;
            pos_y = next_y;

          }
          
          
            
          
          //END
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