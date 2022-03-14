/*
 * @Author: xindong324
 * @Date: 2022-03-03 21:57:29
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-04 09:29:34
 * @Description: file content
 */
#ifndef _PID__H_
#define _PID__H_

#define DEFAULT_PID_INTEGRATION_LIMIT 		(0.0) //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //默认pid输出限幅，0为不限幅

#define M_PI_F       3.14159265358979323846264338328      /* pi */

// ! Store the shared time constant for the derivative cutoff.
static double deriv_tau   = 7.9577e-3;

// ! Store the setpoint weight to apply for the derivative term
static double deriv_gamma = 1.0;

typedef struct pid_scaler_s {
    double p;
    double i;
    double d;
};

class PID
{
public:
	PID();
	PID(double p, double i, double d, double iLim);
	~PID(){};
    double pid_apply(const double err, double dT);
    double pid_apply_setpoint(const double setpoint, const double measured, double dT, double meas_based_d_term);
	void pid_zero();
	void pid_configure(double p, double i, double d, double iLim);
	void pid_configure_new(double p, double i, double d, double iLim);
	void pid_configure_derivative(double cutoff, double g);


    double p_;
    double i_;
    double d_;
    double iLim_;
    double iAccumulator_;
    double lastErr_;
    double lastDer_;

	pid_scaler_s pid_scaler;


    static double boundf(double val, double low_bound, double up_bound)
	{
		
		if(low_bound > up_bound){
			double temp = up_bound;
			
			up_bound = low_bound;
			low_bound = temp;
		}
		if(val > up_bound) return up_bound;
		if(val < low_bound) return low_bound;
		return val;
	}
	
};

#endif