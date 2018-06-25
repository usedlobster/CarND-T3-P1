#ifndef JMT_HPP_
#define JMT_HPP_


class QuinticPath {
    public:
        double ic[3]        ;
        double ec[3]        ;
        double poly[6]      ;
        double t0 , t1 , T  ;


        void JMT( double t0 , double Tperiod ) ;

        double   get( double t ) ;
        double get1d( double t ) ;
        double get2d( double t ) ;
        double get3d( double t ) ;


} ;



#endif
