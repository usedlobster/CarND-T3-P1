#ifndef MAPDATA_H__
#define MAPDATA_H__

#include <math.h>
#include <iostream>

#include <vector>
#include "spline.h"

// holds road waypoints data.

class MapWayPoint {
	public:
		double x , y      ;   // x and y position
		double s 	   	  ;
		double nx , ny 	  ;  // normal x/y components
	public:
		MapWayPoint() {}  ;
} ;

// holds all the road points of the simulator track
//
// in reality would probably have another process
// continuously creating the data on the fly
//

 class MapData {

	public:

		std::vector<MapWayPoint> wpts ;

	public:
		double max_s ;

		double s_low , s_high ;

		MapData() ;

		void Add( MapWayPoint ) ;

        // wrap an s to [ 0 , max_s ]
		double wrap_s( double s )      ;

		// compensate for difference between s values when  subtracted
		double wrap_s_diff( double s ) ;


		void PreCalculate() ;
        // given s and d calculate the x,y values ( approx )
		void getXY( double s , double d , double &x , double &y ) ;
		// given x and y calculate the s,d values ( approx )
		void getSD( double x , double y , double &s , double &d ) ;

} ;




#endif
