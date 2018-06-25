#ifndef __PATHPLAN_HPP__
#define __PATHPLAN_HPP__


#include "Vehicle.hpp"
#include "MapData.hpp"
#include "JMT.hpp"
#include "misc.hpp"

#include <vector>
#include <unordered_map>
#include <map>

using namespace std ;
// states ( all should be even )


struct Target {
    public:
        int mode                     ;
        int fid                      ;
        double s_target,d_target     ;
        double keep,keep_inc         ;
        double stuck                 ;
} ;


class Trajectory {
	private:

	public:
        bool    bad              ;
        int     mode             ;
        QuinticPath s , d        ;
        double score	         ;

	public:

		Trajectory() { bad = true ; mode = 0 ;  score = 0 ; }

} ;

class PathPlan {

	//
	// lazy programming - i've made everything public
	//

	public:


		double MoveForward( VehicleData & , double , double ) ;

        VehicleTracker *GetNear(  int side , int rank  ) ;

        void  JMT( double start[3] , double end[3] , double T , double result[6] ) ;

        bool MakeNewPath( Trajectory &best ) ;
        bool CheckTrajectory( Trajectory &test ) ;


        int state    		  ;
        MapData road 		  ;
        double t_now 	  	  ;

        int n_prev   		  ;
        vector<double> prev_x ;
        vector<double> prev_y ;

        int n_next   ;
        vector<double> next_x ;
        vector<double> next_y ;

        double end_s ;
        double end_d ;
        // sensor fusion vehicles by id
        unordered_map <int,VehicleTracker> vtrack ;

        // map of vehicles relative to us
        struct near_t
        {
        	int id      ;
        	double s ;
        } nearmap[3][4] ;

        // previous given locations
        VehicleTracker ego ;


        void TrackVehicle( int id , VehicleData v ) ;

        // keep last (MAX_POINTS) vehicle positions generated
        VehicleData v_prev[MAX_POINTS] ;
        // current location to generate from
        VehicleData v_now              ;

        Trajectory traj 	       ;
        Target target              ;

        int vote_lc_l   ;
        double score_l  ;
        int vote_lc_r   ;
        double score_r  ;

        int VoteLaneChange( int vote , int offset ) ;
        double ScoreLane( int offset , int current_lane_no ) ;
        double WorthLaneChange( int offset ) ;
        void SetTargetLane( int offset ) ;

        void UpdateTarget( )  ;
        void UpdatePlan()  	 ;

        void Execute() ;

        void Reset() ;

	public:

        PathPlan() ;

        void Debug() ;


} ;


#endif
