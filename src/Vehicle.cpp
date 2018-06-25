#include <iostream>
#include "Vehicle.hpp"
#include "misc.hpp"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd ;
using Eigen::VectorXd ;


// We have a new vehicle position
void VehicleTracker::Update( MapData md, const VehicleData v ) {
  // determine which lane left / right  wheels are in
  l_lane = floor(( v.d - 1.0 ) / 4.0 ) ;
  r_lane = floor(( v.d + 1.0 ) / 4.0 ) ;
  c_lane = ( l_lane + r_lane ) / 2     ; //

  // we keep last 4 positions + latest
  // we may need them
  last4[(nu++)&3] =  latest ;
  latest = v ;

}


double VehicleTracker::EstimateSpeed() {
  // we assume s-speed is approxiametly x/y speed .
  return  sqrt( latest.vx * latest.vx + latest.vy * latest.vy ) ;

}
