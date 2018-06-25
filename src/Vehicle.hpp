#ifndef __VEHICLE_HPP_
#define __VEHICLE_HPP_ 1

#include "MapData.hpp"
#include <ostream>


// simple vehicle data class
class VehicleData {

 public:
  double t     ;
  double x  	 ;
  double y  	 ;
  double vx 	 ;
  double vy 	 ;
  double s  	 ;
  double d     ;
  double yaw   ;
  double speed ;

 public :

  VehicleData() {
    t = x = y = vx = vy = s = d = yaw = speed = 0.0 ;
  }

} ;


class VehicleTracker {

 public:
  int id 				;
  int nu         		;
  int l_lane, c_lane, r_lane ;

  double dist 	    ;

  VehicleData latest   ;
  VehicleData last4[4] ;

  VehicleTracker() {
    id = -999        ;
    nu = 0           ;

  } ;

  // get ith previous vehicle data
  void  Update( const MapData m, VehicleData v ) ;

  double EstimateSpeed() ;




} ;


#endif
