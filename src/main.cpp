
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <array>


using namespace std;

#include "json.hpp"
#include "PathPlan.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

constexpr double deg2rad( double x ) {
  return x * M_PI / 180.0 ;
}

// double rad2deg(double x) { return x * 180 / pi(); }
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

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


int main() {


  uWS::Hub h, d ;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  // max-s is now calcualted

  // create my path-planner object .
  PathPlan plan ;

  // open highway data sream

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line    ;

  // load each data into plan.road
  while (getline(in_map_, line)) {
    istringstream iss(line);

    MapWayPoint wp ;

    iss >> wp.x;
    iss >> wp.y;
    iss >> wp.s;
    iss >> wp.nx ;
    iss >> wp.ny ;

    plan.road.Add( wp ) ;
  }

  // process map - compute splines etc
  plan.road.PreCalculate() ;

  // set simulation clock
  plan.t_now = 0.0 ;

  h.onMessage([&plan](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry" ) {

          // load all telemetry data into plan object.



          // get previous x-points
          auto  prev_x = j[1]["previous_path_x"] ;

          // we don't actually use this data anymore
          // as we already have it
          // we are only interseted in how many points actually got used last time
          //

          /*
          plan.prev_x = std::vector<double>(prev_x.begin() , prev_x.end() ) ;

          // get previous y-points
          auto prev_y = j[1]["previous_path_y"] ;
          plan.prev_y = std::vector<double>(prev_y.begin() , prev_y.end() ) ;

          plan.end_s  = (double) j[1]["end_path_s"] ;
          plan.end_d  = (double) j[1]["end_path_d"] ;
          */

          plan.n_prev = prev_x.size() ;

          //
          // update the internal time reference
          // if we generate n_next last time , and still have n_prev left to run
          // then we must have gone ( n_next - n_prev ) timesteps ahead
          //
          // we experimented with using real time , but this method is accurate enougth for my purposes.

          plan.t_now  += ( plan.n_next - plan.n_prev ) * 0.02 ;

          //
          VehicleData car ;


          car.t     = plan.t_now   ;
          car.x     = j[1]["x"]    ;
          car.y     = j[1]["y"]    ;
          car.s     = j[1]["s"]    ;
          car.d     = j[1]["d"]    ;
          // we could calculate these but not used
          car.vx    = 0.0 ;
          car.vy    = 0.0 ;
          // we don't need this all the time
          //car.yaw   = deg2rad( j[1]["yaw"] )  ;
          //
          car.speed = double(  j[1]["speed"] ) * 0.44704 ;

          // update self
          plan.TrackVehicle( -1, car ) ;

          auto sensor_fusion_data = j[1]["sensor_fusion"] ;

          for ( auto &sf : sensor_fusion_data  ) {
//
            car.t     = plan.t_now ;
            car.x     = sf[1]  ;
            car.y     = sf[2]  ;
            car.vx    = sf[3]  ;
            car.vy    = sf[4]  ;
            car.s     = sf[5]  ;
            car.d     = sf[6]  ;
            // not required all the time so not computed
            // car.speed = sqrt( car.vx * car.vx + car.vy * car.vy ) ;
            //car.yaw   = atan2( car.vy , car.vx ) ;

            // update the list of sensor fusion detected cars
            // we assume ids are >=0
            if ( sf[0] >= 0 )
              plan.TrackVehicle( sf[0], car ) ;

          }

          // we have the data , now execute our planner
          // to generate our x,y points

          plan.Execute()    ;

          // send the x,y points generated to simulator
          json msgJson;
          msgJson["next_x"] = plan.next_x ;
          msgJson["next_y"] = plan.next_y ;
          auto msg = "42[\"control\","+ msgJson.dump()+"]" ;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&plan,&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    //
    plan.Reset() ;
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

  h.run() ;





}
