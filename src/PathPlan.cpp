#include "JMT.hpp"
#include "PathPlan.hpp"

#include <iostream>
#include <iomanip>
#include <math.h>



static	std::default_random_engine generator( time(0))  ;
static  std::normal_distribution<double> n_dist ( 0.0,1.0 );

double constexpr Logistic( double x ) {
  return ( 2.0 / ( 1.0 + exp( -x ))) - 1.0 ;
}

PathPlan::PathPlan() {
  Reset() ;
}

void PathPlan::Reset() {
  state  = S_INIT ;
  n_next = n_prev = 0 ;
  t_now  = 0 ;
  traj.bad = true ;
  target.mode= M_NONE ;
  target.d_target = 6.0 ;
  target.keep  = SAFE_DISTANCE - KEEP_CYCLE ;
  target.keep_inc = KEEP_INCREMENT ;
  vote_lc_l = 0 ;
  vote_lc_r = 0 ;
}


double Peturb( double a, double b, double lo, double hi ) {
  double v ;

  v = a + n_dist( generator ) *b  ;
  if ( v < lo )
    return lo ;
  else if ( v > hi )
    return hi ;

  return v ;
}

double PeturbDown( double a, double b, double lo, double hi ) {
  double v ;

  v = a - fabs( n_dist( generator )) *b  ;
  if ( v < lo )
    return lo ;
  else if ( v > hi )
    return hi ;

  return v ;

}

// score trajectory ,
// either a complete failure , return false
// or a value in [0,1] where 0 is best , 1 = worst
//

bool PathPlan::CheckTrajectory( Trajectory &k  ) {

  double s1 = k.s.get( k.s.t1 ) ;
  double s0 = k.s.get( k.s.t0 ) ;

  // compute distance travelled

  double s_travel = road.wrap_s_diff( s1 - s0 ) ;

  if ( s_travel < -1e-3  ||  s_travel >  SPEED_LIMIT * k.s.T )
    return false ;

  double d1 = k.d.get( k.d.t1 ) ;
  double d0 = k.d.get( k.d.t0 ) ;

  if ( fabs( d1 - d0 ) > 6.0 )
    return false ;


  // sume of s-speed/jerk/acc
  double sum_sdot = 0.0 ;
  double sum_jerk = 0.0 ;
  double sum_acc  = 0.0 ;
  double s_jerk_t, s_acc_t, sd_t ;
  int n = 0 ;

  // we should check at every point at 0.02 intervals,
  // but to speed things up use 0.11s -

  for ( double t = k.s.t0  ; t <= k.s.t1 ; t+= 0.11, n++ ) {

    s_jerk_t = fabs( k.s.get3d( t )) ;
    if (  s_jerk_t > MAX_JERK )
      return false ;

    // sum_jerk += s_jerk_t  ;

    s_acc_t = fabs( k.s.get2d( t )) ;
    if (  s_acc_t > MAX_ACCEL )
      return false ;

    // sum_acc += s_acc_t  ;

    sd_t = k.s.get1d( t ) ;
    if ( sd_t < 0 || sd_t > MAX_SPEED )
      return false ;

    // sum_sdot += ( sd_t )  ;

  }


  // how near to s_target :-
  k.score  = 10 *  fabs( target.s_target - s1 )  ;
  k.score +=       fabs( target.d_target - d1 ) ;


  return true ; // trajectory seems ok !
}

//
bool PathPlan::MakeNewPath(  Trajectory &best ) {
  Trajectory a, b   ;

  a.mode = target.mode ;
  //
  a.s.ic[0] = v_now.s ;
  a.s.ic[1] = traj.bad ? 0.0 : traj.s.get1d( v_now.t ) ;
  a.s.ic[2] = traj.bad ? 0.0 : traj.s.get2d( v_now.t ) ;

  // create a fallback - trajectory if we fail to find an optimized one
  // slow down 50%
  a.s.ec[2] = 0.0                                 ;
  a.s.ec[1] = a.s.ic[1] * 0.3                    ;
  a.s.ec[0] = v_now.s + 2.5*(a.s.ec[1]+a.s.ic[1]) ;

  a.s.JMT( v_now.t, 5.0 ) ;

  a.d.ic[0] = v_now.d ;
  a.d.ic[1] = traj.bad ? 0.0 : traj.d.get1d( v_now.t ) ;
  a.d.ic[2] = traj.bad ? 0.0 : traj.d.get2d( v_now.t ) ;

  // keep lane constant - non changing
  a.d.ec[2] = 0.0 ;
  a.d.ec[1] = 0.0 ;
  a.d.ec[0] = target.d_target ;

  a.d.JMT( v_now.t, 4.0 ) ;
  // not strictly necessary
  a.bad = !CheckTrajectory( a ) ;
  if ( !a.bad )
    b = a ;
  else {
    b = best     ;
    b.bad = true ;
  }

  // do we have a leading vehicle targeted in follow mode
  VehicleTracker *f ;
  double f_speed ;

  // our we following mode
  if ( target.fid >= 0 ) {
    f = &vtrack[ target.fid ]    ;
    f_speed = f->EstimateSpeed() ;
  } else
    f = nullptr ;

// the difference between now , and when the readings were taken.
// assumes all vehicles update same time as ego
// which in this simulation they do.

  double Tlag = ( v_now.t - ego.latest.t  ) ;

  int n_good    = 0 ;
  int n_attempt = 0 ;

  for ( n_attempt = 0 ;  n_attempt < 1000  ; n_attempt++ ) {
    double Tp,smax ;

    Tp = Peturb( 6.0, 4.0, 2.0, 10.0  )  ;
    smax = a.s.ic[0] + ( Tp )  * 0.5 * ( a.s.ic[1] + MAX_SPEED ) ;

    if ( f != nullptr  ) {
      // estimate where the car in front will be in Tp seconds from now
      double f1 = f->latest.s + ( f_speed * ( Tp  + Tlag  ))     ;
      // we need to aim for this to keep SAFE_DISTANCE
      // ratio
      double s1 = f1 - ( target.keep + 0.3 * f_speed ) ;
      target.s_target = std::min( smax, s1 ) ;
    } else
      target.s_target =  smax ;

    // peturb end conditions
    for ( int i = 0 ; i < 20 ; i++ ) {
      a.s.ec[2] = ( !(i&1) ) ? a.s.ic[2]  : Peturb( a.s.ic[2], 5.0, -10.0, 10.0 ) ;

      if ( f == nullptr  ) {
        a.s.ec[1] = (!i) ? MAX_SPEED : PeturbDown( MAX_SPEED, MAX_SPEED/2.0 , 0, MAX_SPEED ) ;
        a.s.ec[0] = (!i) ? target.s_target : Peturb( target.s_target, 210.0, a.s.ic[0], a.s.ic[0] + 405 ) ;
      } else {
        a.s.ec[1] = (!i) ? f_speed  : Peturb( f_speed , MAX_SPEED/2.0, 0, MAX_SPEED ) ;
        a.s.ec[0] = (!i) ? target.s_target : Peturb( target.s_target, 12.5, target.s_target-25.0, target.s_target+25.0 ) ;
      }


      a.s.JMT( v_now.t, Tp ) ;

      if ( CheckTrajectory( a )) {
        a.bad = false ;
        n_good++ ;
        // b.bad is true on first run
        if  ( b.bad || a.score < b.score )
          b = a ;
      }
    }

    if ( n_good > 50 )
      break ; // after 50 we must be near optimal

  }

  if  ( !b.bad )
    best = b ;
  return !best.bad ;


}


void PathPlan::Debug() {
//	VehicleTracker ego = vtrack[-1] ;

  std::cout << "\033[39m\033[2J\033[;H" ;
  std::cout << fixed << setprecision( 3 ) ;
  std::cout << std::endl << "s=" << v_now.s  << " d = " << v_now.d << " t= " << t_now << " mode = " << state << "/" << target.mode << std::endl ;
  std::cout << "[l-r] = [ " << ego.l_lane << " - " << ego.r_lane << " ] " <<  "vote : " << vote_lc_l << "," <<  vote_lc_r << " l_score : "  << score_l << "," << score_r << " keep: " << target.keep << std::endl ;
  std::cout << "path = [t0-t1] = [" << traj.s.t0 << "-" << traj.s.t1 << "] - score = " << setprecision(6) << traj.score << std::endl << std::endl ;
  for ( int ncell=3; ncell>=0; ncell-- ) {
    for ( int lside = 0; lside <3; lside++ ) {
      VehicleTracker *t = GetNear( lside, ncell ) ;
      if ( lside == 0 || lside == 2 )
        cout << "\033[33m" ;
      else
        cout << "\033[39m" ;

      cout <<  (( t == nullptr ) ? '.' : char( t->id+'a')) <<  " " << setprecision(2) << setw(12) << ((t==nullptr) ? 0.0 : t->dist ) << " " << " | " ;

    }

    std::cout << "\033[35m" << std::endl ;
  }

  std::cout << std::endl << "=" << std::endl ;

  for ( auto it : vtrack  ) {
    std::cout << char('a' + it.first )
              << ":"
              << setw( 10 ) << it.second.dist
              << std::endl  ;
  }


  std::cout << n_next << " " << n_prev << std::endl ;

  std::cout << std::flush ;

}


void PathPlan::TrackVehicle( int id, VehicleData v ) {

  // get tracker associated with id
  if ( id >= 0 ) {
    VehicleTracker &trk = vtrack[id] ;


    trk.id = id 	                 ;

    // for some reason the simulator can give out s=0,d=0 for vehicle positions
    // they are about +/- 15m's near s=0  out but that can be a problem .
    //
    if ( fabs( v.d  ) < 0.1  && fabs( v.s  ) < 0.1  )
      road.getSD( v.x, v.y, v.s, v.d ) ;

    double ds = trk.dist  = road.wrap_s_diff( v.s - ego.latest.s ) ;

    trk.Update( road, v ) ;

    int lane_side ;

    if ( trk.l_lane <= ego.r_lane && trk.r_lane >= ego.l_lane )
      lane_side = 1 ; // in same lane
    else if ( trk.r_lane == ego.l_lane-1 )
      lane_side = 0 ; // to the left of our lane
    else if ( trk.l_lane == ego.r_lane+1 )
      lane_side = 2 ; // right of our lane
    else
      return ;

    near_t *a = &nearmap[lane_side][( ds < 0.0 ) ? 1 : 2 ] ;
    near_t *b = &nearmap[lane_side][( ds < 0.0 ) ? 0 : 3 ] ;

    if ( ds < 0 )
      ds = -ds ;

    // do two item inplace sort
    if ( a->id < 0 || ds < a->s ) {
      b->id = a->id ;
      b->s  = a->s  ;
      a->id = id ;
      a->s  = ds  ;
    } else if ( b->id < 0 || ds < b->s ) {
      b->id = id ;
      b->s  = ds  ;
    }

  } else {
    //
    v.t = t_now ;
    ego.Update( road, v) ;

    // clear nearby object table
    for ( auto j = 0 ; j < 3 ; j++ )
      for ( auto i = 0 ; i < 4 ; i++ )
        nearmap[j][i].id = -1 ;
  }

}


// get a tracker from the near object map .
VehicleTracker *PathPlan::GetNear(  int side, int rank  ) {
  if ( side>=0 && side<=2 && rank>=0 && rank<=3 ) {
    near_t *n = &nearmap[side][rank] ;
    if ( n->id >= 0 ) {
      VehicleTracker *trk = &vtrack[n->id] ;
      return trk ;
    }
  }

  return nullptr ;
}

// move vehicle forward in s and d direction by amount specified
// compensating for how far along d one is .

double PathPlan::MoveForward( VehicleData &v, double s_fwd, double d_fwd  ) {
  double x2, y2 ;
  double len    ;

  // get new estimated position, at new s+s_fwd,d position .
  road.getXY( v.s + s_fwd, v.d, x2, y2 ) ;
  len = sqrt((( x2 - v.x ) * ( x2 - v.x  )  + ( y2 - v.y ) * ( y2 - v.y ))) ;

  // adjust for d-correction
  s_fwd =   s_fwd *  ( s_fwd  / len ) ;
  v.s  = road.wrap_s( v.s + s_fwd  )  ;
  v.d += d_fwd 		    		    ;
  road.getXY( v.s, v.d, v.x, v.y ) ;

  return s_fwd ;
}

int PathPlan::VoteLaneChange( int vote, int offset ) {
  int c = floor(( v_now.d ) / 4.0 ) ;

  if ( c + offset < 0 || c + offset > 2 )
    return 0 ;

  VehicleTracker *f = GetNear( 1, 2 ) ;
  if ( f !=nullptr && f->dist < SAFE_LANE_FRONT )
    return 0 ;

  VehicleTracker *k = GetNear( 1, 1 ) ;
  if ( k !=nullptr && k->dist > -SAFE_LANE_BACK )
    return 0 ;


  VehicleTracker *a, *b ;

  a  = GetNear( 1 + offset, 2 ) ;
  if ( a == nullptr  || a->dist > SAFE_LANE_ADJ_A ) {
    b = GetNear( 1 + offset, 1 ) ;
    if ( b == nullptr || b->dist < -SAFE_LANE_ADJ_B ) {
      if ( b == nullptr && a == nullptr )
        return vote+3 ;
      else if ( a == nullptr )
        return vote+2 ;
      else
        return vote+1 ;
    }
  }

  return 0 ;
}


double PathPlan::ScoreLane( int offset, int current_lane_no ) {
  double lane_w[3] = { 0.95, 1.0, 0.90 } ;

  int lane = current_lane_no + offset ;
  // check lane is in boundaries
  if ( lane < 0 || lane > 2 )
    return 0.0 ; // illegal lane - no score

  // get front vehicle f , back vehicle b
  VehicleTracker *f = GetNear( 1+offset, 2 ) ;
  VehicleTracker *b = GetNear( 1+offset, 1 ) ;

  // get distance of vehicle in front ( or assume 300m )
  double fd0 = ( f != nullptr ) ? f->dist : 300.0  ;
  if ( fd0 < SAFE_LANE_FRONT )
    return 0.0 ; // too-close to score anything

  // get distance of vehicle behind ( or assume -300m )
  double bd0 = ( b != nullptr ) ? b->dist : -300.0 ;
  if ( bd0 > - SAFE_LANE_BACK )
    return 0.0 ; // too-close to score anything

  // get speed of front vehicle ( or assume maximum )
  double fs = ( f != nullptr ) ? f->EstimateSpeed() : MAX_SPEED ;
  // get speed of front vehilce ( or assume maximum )
  double bs = ( b != nullptr ) ? b->EstimateSpeed() : MAX_SPEED ;
  //

  // a change lane manoeuvre - takes about 5 seconds so calculate estimated ( relative positions )

  double bd1 = bd0 + bs * 2.5 ;
  double fd1 = fd0 + fs * 2.5 ;
  double ed1 = ego.latest.speed * 2.5 ;

  if (( bd1 > ( ed1 - 5.0 )) || ( fd1 < ( ed1 + 5.0 )))
    return 0.0 ;

  double z =  (( fd1 + fd0 / 600.0 ))  ;


  return lane_w[ lane ] * z ;  ;





}



// determine if its worth us changing lane
// we could make more complex scorer !

double PathPlan::WorthLaneChange( int offset  ) {

  double current_lane_score  = ScoreLane( 0, ego.c_lane  ) ;
  double adj_lane_score = ScoreLane( offset, ego.c_lane ) ;

  return adj_lane_score - current_lane_score ;
}

void PathPlan::SetTargetLane( int offset ) {
  int c = floor(( v_now.d  ) / 4.0 ) ;
  target.d_target = 2 +  ( c + offset ) * 4.0 ;
  vote_lc_l = vote_lc_r = -VOTE_CHANGE ;
  target.mode = ( offset == 0 ) ? M_NONE : M_LANE_CHANGE ;
  state = S_REGEN ;

}


// generate a target ( in our case , change_lane , follow car in front or go at full speed limit )
//

void PathPlan::UpdateTarget( ) {

  if ( target.mode == M_LANE_CHANGE ) {
    if ( fabs( v_now.d - target.d_target ) < 0.125 || ( v_now.t > traj.d.t1 - 1.0  ))
      SetTargetLane( 0 ) ;
  }

  VehicleTracker *f = GetNear( 1, 2 ) ;

  int follow_id = ( f == nullptr || ( f->dist > SAFE_DISTANCE + 15.0  )) ? -1 : f->id ;

  // check vehicle in front hasn't changed  by cutting in front .
  if ( follow_id != target.fid  ) {
    target.stuck = t_now ;
    target.fid = follow_id ;
    state = S_REGEN ;
  } else if (  target.fid >=0  && t_now > target.stuck+ KEEP_TIME  ) {
    // with been stuck behind a vehicle for 3 seconds with no oppurtinty to change lane
    // lets try changing the keep distance .
    target.stuck = t_now ;
    target.keep += target.keep_inc ;
    if ( ( target.keep >= SAFE_DISTANCE + KEEP_CYCLE )  || ( target.keep <= SAFE_DISTANCE - KEEP_CYCLE ))
      target.keep_inc = -target.keep_inc ;
    state = S_REGEN ;
  }


  if ( target.mode != M_LANE_CHANGE ) {
    target.mode = target.fid < 0 ? M_MAX_V : M_FOLLOW ;

    // vote for new lane / right  change
    vote_lc_l = VoteLaneChange( vote_lc_l, -1 ) ;
    vote_lc_r = VoteLaneChange( vote_lc_r, +1 ) ;

    score_l = ( vote_lc_l > VOTE_CHANGE ) ? WorthLaneChange( -1 ) : 0.0 ;
    score_r = ( vote_lc_r > VOTE_CHANGE ) ? WorthLaneChange( +1 ) : 0.0 ;

    if ( score_l > 0.1 )
      SetTargetLane( -1 ) ;
    else if ( score_r > 0.1 )
      SetTargetLane( 1 ) ;

  }

}


// we update our plan , based on current state and trajectory mode
//

void PathPlan::UpdatePlan() {

  if ( state == S_INIT ) {
    // initial state
    traj.bad = true      ;
    target.mode = M_NONE ;
    target.d_target = 6.0 ;
    state = S_REGEN      ;
  }

  // set target to one of fore modes FOLLOW, KEEP , LANE_CHANGE , STOP
  UpdateTarget() ;

  if ( state != S_REGEN ) {
    if ( traj.bad || v_now.t < traj.s.t0 || v_now.t > traj.s.t1 - 1.0 ||
         v_now.t < traj.d.t0 || v_now.t > traj.d.t1 - 1.0  )
      state = S_REGEN ;
    else if ( target.fid > 0  && v_now.t > traj.s.t0 + 0.3 )
      state = S_REGEN ; // regenerate every 0.3 if in follow mode .

  }

  // regenerate if requested or mode changed
  if ( state == S_REGEN && MakeNewPath( traj ))
    state = S_EXECUTE ;

}

void PathPlan::Execute() {
  int i, n_seen    ;
  // clear previous points
  next_x.clear() ;
  next_y.clear() ;
  // how many of previous points got used
  n_seen = ( n_next - n_prev ) ;

  i  = 0 ;

  if ( n_seen > 0 ) {
    // push some previous points forward
    // helps continuity / smoothness
    for ( int j=0 ; j < 5 && ( n_seen < n_next ) ; j++ ) {
      v_now = v_prev[ n_seen++ ]  ;
      v_prev[i++]  = v_now        ;
      next_x.push_back( v_now.x ) ;
      next_y.push_back( v_now.y ) ;
    }
  } else
    v_now = ego.latest ;

  // v_now is current point we are generating new points from
  // update current plan and target

  UpdatePlan() ;

  // output debug information/stats
  //

  Debug() ;

  while ( i < MAX_POINTS ) {
    double s_fwd, d_fwd ;
    // the next point is +20ms
    v_now.t +=0.02      ;

    // we are following two quintic paths in traj for s and d
    // we assume are good , and inside the
    // time range it was calculated for - if not we use the end values
    //
    // we use the first order derivate to get
    // the increments we require ( which are per second )
    // so multiply by 0.02

    s_fwd = traj.s.get1d( v_now.t ) * 0.02 ;
    d_fwd = traj.d.get1d( v_now.t ) * 0.02 ;

    // apply these forward movements to get new x,y,s,d positions
    MoveForward( v_now, s_fwd, d_fwd  ) ;
    // and save for latter
    v_prev[i++] = v_now ;
    // save onto seperate x/y lists for convenience
    next_x.push_back( v_now.x ) ;
    next_y.push_back( v_now.y ) ;
  }

  n_next = next_x.size() ;

}






