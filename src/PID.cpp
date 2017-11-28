#include <cmath>
#include <iostream>
#include "PID.h"

//using namespace std;


PID::PID(double _Kp, double _Ki, double _Kd) {
    K[0] = _Kp;
    K[1] = _Ki;
    K[2] = _Kd;

    dp[0] = 0.05;
    dp[1] = 0.0000;
    dp[2] = 0.32;
}

PID::~PID() = default;


void PID::UpdateError(double cte, double speed) {
    d_error = cte - p_error;   // derivative
    i_error += cte;            // integral
    p_error = cte;             // proportional
    step++;
    s_error += fabs(cte);
    s_speed += speed;
}

double PID::TotalError() {
    return -K[0] * p_error - K[1] * i_error - K[2] * d_error;
}

uint LAP = 3000;

bool PID::shouldRestart(double cte) {
    return step>LAP || fabs(cte)>3.0;
}

float lambda = 0.2;
float lambda1 = 1+lambda;
float lambda2 = 1-lambda;


void PID::restart() {
    //auto error = TRACK * 500 - s_speed;
    if (step > 3) {
        if (state_ == inc_param) {
            save_best(s_speed);

            K[tuned] += dp[tuned];
            state_ = calc_best_err_after_inc;
        } else if (state_ == calc_best_err_after_inc) {
            if (s_speed > best_lap) {
                save_best(s_speed);

                dp[tuned] *= lambda1;
                next_param();
                K[tuned] += dp[tuned];
                state_ = calc_best_err_after_inc;
            } else {
                K[tuned] -= 2 * dp[tuned];
                state_ = calc_best_err_after_dec;
            }
        } else if (state_ == calc_best_err_after_dec) {
            if (s_speed > best_lap) {
                save_best(s_speed);

                dp[tuned] *= lambda1;
            } else {
                K[tuned] += dp[tuned];
                dp[tuned] *= lambda2;
            }
            next_param();

            K[tuned] += dp[tuned];
            state_ = calc_best_err_after_inc;
        }
    }

    std::cout << "Iteration:" << it++ << ", distance=" << s_speed << ", step=" << step;
    std::cout << " param:" << (tuned==0?"p":"d")  << " ";
    std::cout << (state_==1?"-":"+") << std::endl;
    std::cout << K[0] << "," << K[1] << "," << K[2] << "," << std::endl;
    std::cout << dp[0] << "," << dp[1] << "," << dp[2] << "," << std::endl;

    step = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;

    s_error = 0;
    s_speed = 0;
}

void PID::next_param() { // i does not improve quality
    tuned = (tuned==0)? 2:0;
}

void PID::save_best(double speed) {
    best_lap = speed;
    std::cout << "! best lap:" << best_lap << std::endl;
}
