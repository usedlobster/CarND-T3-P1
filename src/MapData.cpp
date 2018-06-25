
#include "MapData.hpp"

using namespace std ;

#include "spline.h"



// splines declared global , instead of in class to avoid some
// namespace warnings that may be errors depnding on version of gcc or ( other ) compiler
//

tk::spline spline_x  ;
tk::spline spline_y  ;
tk::spline spline_nx ;
tk::spline spline_ny ;

MapData::MapData()
{
	wpts.clear()  ;
}


// nothing going on here !
void MapData::Add( MapWayPoint w )
{
    wpts.push_back( w ) ;
}

// return s value in [0 , max_s ] interval
double MapData::wrap_s( double s )
{
    if ( s > max_s )
        s-=max_s ;
    else if ( s < 0 )
        s+=max_s ;

    return s ;
}

// for difference between two s value
double MapData::wrap_s_diff( double s  )
{
    if ( s < -max_s/2.0 )
        s += max_s ;
    else if ( s > max_s/2.0 )
        s = max_s - s  ;

    return s ;
}


// take all the waypoint points and compute
// splines for x,y , nx , ny
//
// we also extend outward and inward 10 waypoints
// which is approx +/- 300m
//
//
void MapData::PreCalculate()
{
	MapWayPoint w ;
	double s ;
	//
	vector<double>   pts_s ;
	vector<double>   pts_x ;
	vector<double>   pts_y ;
	vector<double>  pts_nx ;
	vector<double>  pts_ny ;


	int n = wpts.size() ;
	double dx = wpts[0].x - wpts[n-1].x ;
	double dy = wpts[0].y - wpts[n-1].y ;
	double end_seg   = sqrt( ( dx * dx ) + ( dy * dy )) ;

	max_s = wpts[n-1].s + end_seg ;

    s_low = 0      ;
    s_high = max_s ;

    // extend out 10 way points either side
	for ( int i = -10 ; i <= n+10  ; i++ )
	{
		int j = ( n + i + n ) % n  ;

		if ( i < 0 )
			s = wpts[j].s - max_s ;
		else if ( i >=n )
			s = wpts[j].s + max_s ;
		else
			s = wpts[j].s ;

        if ( s < s_low )
            s_low = s ;

        if ( s > s_high )
            s_high = s ;

		 pts_s.push_back( s ) ;
		 pts_x.push_back( wpts[j].x ) ;
		 pts_y.push_back( wpts[j].y ) ;
		pts_nx.push_back( wpts[j].nx ) ;
		pts_ny.push_back( wpts[j].ny ) ;

	}

// create a spline of x,y points and their normals at given s positions.

	 spline_x.set_points( pts_s , pts_x  ) ;
	 spline_y.set_points( pts_s , pts_y  ) ;
	spline_nx.set_points( pts_s , pts_nx ) ;
	spline_ny.set_points( pts_s , pts_ny ) ;

}

// given x & y return s,d
//

void MapData::getSD( double x , double y , double &s , double &d )
{
    int j ;
    double nearest ;


    // find nearest waypoint ( by simple euclidean distance^2 )
    j=-1 ;
    nearest = 0.0 ;
    for ( int i = 0 ; i < int(wpts.size()) ; i++ )
    {
      double xx = ( x - wpts[i].x ) * ( x - wpts[i].x ) + ( y - wpts[i].y ) * ( y - wpts[i].y ) ;
      if ( j < 0 || xx < nearest )
      {
       j = i ;
       nearest = xx ;
      }
    }

    // j is our nearest suspect

    double xx = x - wpts[j].x ;
    double yy = y - wpts[j].y ;
    double nx = wpts[j].nx ;
    double ny = wpts[j].ny ;

    // in theory this should always be 1.0 as they are nomralized already
    // - but beign paranoid calculate anyway.
    double dd = ( nx * nx + ny * ny ) ;

    // [nx ny] points along +d-axis
    d = ( xx * nx + yy * ny ) / dd ;
    // [-ny nx] is along the +s-axis
    s = wrap_s( wpts[j].s + ((  -xx * ny + yy * nx ) / dd )) ;

}

// given s & d values return x,y position
void MapData::getXY( double s , double d , double &x , double &y )
{
	// we slightly decrease the effect of d -
	// this is a bit of a hack , but otherwise seems to cause out-of-lane errors
	// when d is at 10.0 at some parts of the track.
	x  = spline_x( s ) +  spline_nx( s ) *  d*0.98 ;
    y  = spline_y( s ) +  spline_ny( s ) *  d*0.98 ;
}




