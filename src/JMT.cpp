
#include "JMT.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"

using namespace std;

using Eigen::MatrixXd ;
using Eigen::VectorXd ;


void QuinticPath::JMT( double tBase , double Tperiod )
{
  T = Tperiod ;
  t0 = tBase  ;
  t1 = t0 + T ;


  double T2 = (  T * T ) ;
  double T3 = ( T2 * T ) ;
  double T4 = ( T3 * T ) ;
  double T5 = ( T4 * T ) ;

  MatrixXd A = MatrixXd(3, 3);
  //
  A << T3,    T4,    T5,
  3*T2,  4*T3,  5*T4,
  6*T, 12*T2, 20*T3 ;

  MatrixXd B = MatrixXd( 3, 1 ) ;


  B << ec[0] - ( ic[0]+ic[1]*T+0.5*ic[2]*T2 ),
  ec[1] - ( ic[1]+ic[2]*T ),
  ec[2] - ( ic[2] ) ;

  MatrixXd Ai ;

  Ai = A.inverse();

  MatrixXd C = Ai*B;

  //vector <double> result = {start[0], start[1], .5*start[2]};

  poly[0] = ic[0] ;
  poly[1] = ic[1] ;
  poly[2] = 0.5*ic[2] ;
  poly[3] = C(0);
  poly[4] = C(1);
  poly[5] = C(2);

}

double QuinticPath::get( double t )
{
    t -= t0 ; if ( t > T ) t =T ; else if ( t < 0.0 ) t = 0.0 ;
	return (((( poly[5] * t + poly[4] ) * t + poly[3] ) * t + poly[2] ) * t + poly[1] ) * t + poly[0] ;
}

double QuinticPath::get1d( double t )
{
    t -= t0 ; if ( t > T ) t =T ; else if ( t < 0.0 ) t = 0.0 ;
	return ((( 5.0 * poly[5] * t + 4.0 * poly[4] ) * t + 3.0*poly[3] ) * t + 2.0*poly[2] ) * t + poly[1]  ;
}

double QuinticPath::get2d( double t )
{
    t -= t0 ; if ( t > T ) t =T ; else if ( t < 0.0 ) t = 0.0 ;
	return (( 20.0 * poly[5] * t + 12.0 * poly[4] ) * t + 6.0*poly[3] ) * t + 2 * poly[2]   ;
}

double QuinticPath::get3d( double t )
{
    t -= t0 ; if ( t > T ) t =T ; else if ( t < 0.0 ) t = 0.0 ;
	return ( 60.0 * poly[5] * t + 24.0 * poly[4] ) * t + 6.0 *poly[3] ;
}

// return [-1,+1] => [-inf,+inf]
