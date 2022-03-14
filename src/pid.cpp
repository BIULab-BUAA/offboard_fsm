/*
 * @Author: xindong324
 * @Date: 2022-03-03 21:57:29
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-04 09:29:56
 * @Description: file content
 */
#include <offboard_sample/pid.h>
using namespace std;

PID::PID()
{
    pid_zero();
}

PID::PID(double p, double i, double d, double iLim)
{
    pid_zero();
    pid_configure( p, i, d, iLim);
}

double PID::pid_apply(const double err, double dT)
{
    iAccumulator_ += err * (i_ * dT * 1000.0);
    iAccumulator_ = boundf(iAccumulator_, iLim_ * -1000.0, iLim_ * 1000.0);

    double diff = (err - lastErr_);
    double dterm = 0;
    lastErr_ = err;
    if (d_ > 0.0 && dT > 0.0)
    {
        dterm = lastDer_ + dT/(dT + deriv_tau) * ((diff * d_) - lastDer_);
        lastDer_ = dterm;
    }

    return p_ * err + iAccumulator_ / 1000.0 + dterm;
    
}

void PID::pid_zero()
{
    iAccumulator_ = 0;
    lastErr_ = 0;
    lastDer_ = 0;
}

/**
 * @brief Configure the common terms that alter ther derivative
 * @param[in] cutoff The cutoff frequency (in Hz)
 * @param[in] gamma The gamma term for setpoint shaping (unsused now)
 */
void PID::pid_configure_derivative(double cutoff, double g)
{
    deriv_tau   = 1.0 / (2 * M_PI_F * cutoff);
    deriv_gamma = g;
}

/**
 * Configure the settings for a pid structure
 * @param[out] pid The PID structure to configure
 * @param[in] p The proportional term
 * @param[in] i The integral term
 * @param[in] d The derivative term
 * @param[in] iLim the limit of intger,for iLim<0,settoDefault limit
 */
void PID::pid_configure( double p, double i, double d, double iLim)
{
    p_    = p;
    i_    = i;
    d_    = d;
    iLim_ = iLim;
}

/**
 * Configure the settings for a pid structure
 * @param[in] p The proportional term
 * @param[in] i The integral term
 * @param[in] d The derivative term
 * @param[in] iLim the limit of intger,for iLim<0,settoDefault limit
 */
void PID::pid_configure_new(double p, double i, double d, double iLim)
{
	if(iLim_<0) iLim_ = DEFAULT_PID_INTEGRATION_LIMIT;
    p_    = p;
    i_    = i;
    d_    = d;
    iLim_ = iLim;
}



