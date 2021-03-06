#include "PID.h"
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(vector<double> p) {
    PID::p = p;

    PID::p_error = numeric_limits<double>::max(); // assume an extreme large error
    PID::i_error = 0.0;
    PID::d_error = 0.0;
}

void PID::UpdateError(double cte) {
    // The first measurement
    if(p_error == numeric_limits<double>::max()){
        p_error = cte;
    }

    // Differential error
    d_error = cte - p_error;

    // Proportional error
    p_error = cte;

    // Integral error
    i_error += cte;
}

double PID::TotalError() {
    double temp = -p[0] * p_error - p[1] * i_error - p[2] * d_error;
    // the steering value is [-1, 1].
    std::min(std::max(temp, -1.0), 1.0);
    return temp;
}


