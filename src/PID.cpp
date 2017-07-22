#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Kd = Kd;
	this->Ki = Ki;

	p_error = std::numeric_limits<double>::max();
	i_error = 0.0;
}

void PID::UpdateError(double cte) {
	if (p_error == std::numeric_limits<double>::max())
		p_error = cte;

	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;

	// std::cout << "UpdateError: " << p_error << ", " << i_error << ", " << d_error << endl;
}

double PID::TotalError() {
	double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
	// std::cout << "TotalError: " << steer << " # " << Kp << ", " << Kd << ", " << Ki << endl;
	if (steer < -1.0)
		steer = -1.0;
	if (steer > 1.0)
		steer = 1.0;
	return steer;
}

void PID::Twiddle_Init(bool tw_switch, int tw_num) {
	twiddle_switch = tw_switch;
	twiddle_num = tw_num;

	twiddle_it = 0;

	twiddle_idx = 0;
	twiddle_idx_p = 0;
	twiddle_down = false;
	twiddle_err = 0.0;

	twiddle_p[0] = 0.2;
	twiddle_p[1] = 3.0;
	twiddle_p[2] = 0.01;

	twiddle_dp[0] = 0.1;
	twiddle_dp[1] = 1.0;
	twiddle_dp[2] = 0.005;

	twiddle_err_best = std::numeric_limits<double>::max();

	Init(twiddle_p[0], twiddle_p[1], twiddle_p[2]);

}

bool PID::Twiddle(double cte) {
	twiddle_idx ++;
	twiddle_err += cte * cte;

	// off road
	if (cte < -4.0 || cte > 4.0) {
		twiddle_err = std::numeric_limits<double>::max();
		Twiddle_NextRun();
		// Need to reset simulator
		return true;
	} else if (twiddle_idx == twiddle_num) {
		Twiddle_NextRun();
		// Need to reset simulator
		return true;
	}
	return false;
}

void PID::Twiddle_NextRun() {
	twiddle_it ++;

	double err = twiddle_err == std::numeric_limits<double>::max() ?
				std::numeric_limits<double>::max() : twiddle_err / (float)twiddle_idx;
	
	// Found better p
	if (err < twiddle_err_best) {
		twiddle_err_best = err;
		twiddle_dp[twiddle_idx_p] *= 1.1;

		twiddle_p_best[0] = twiddle_p[0];
		twiddle_p_best[1] = twiddle_p[1];
		twiddle_p_best[2] = twiddle_p[2];

		std::cout << "Twiddle # " << twiddle_it << " ==> " 
				  << Kp << ", " << Kd << ", " << Ki 
				  << "  best_err = " << err << endl;

		twiddle_idx_p ++;
		if (twiddle_idx_p == 3)
			twiddle_idx_p = 0;

		twiddle_p[twiddle_idx_p] += twiddle_dp[twiddle_idx_p];
		twiddle_down = false;
	} else { // Faild to find better p
		if (twiddle_down == true) { // Now is twiddle down
			twiddle_p[twiddle_idx_p] += twiddle_dp[twiddle_idx_p];
			twiddle_dp[twiddle_idx_p] *= 0.9;

			twiddle_idx_p ++;
			if (twiddle_idx_p == 3)
				twiddle_idx_p = 0;

			twiddle_p[twiddle_idx_p] += twiddle_dp[twiddle_idx_p];
			twiddle_down = false;
		} else {
			twiddle_p[twiddle_idx_p] -= 2 * twiddle_dp[twiddle_idx_p];
			twiddle_down = true;
		}

		if (twiddle_err == std::numeric_limits<double>::max())
			std::cout << "Twiddle # " << twiddle_it << " [Off-Road]: " 
					  << Kp << ", " << Kd << ", " << Ki << endl;
		else
			std::cout << "Twiddle # " << twiddle_it << " [No-Better]: " 
					  << Kp << ", " << Kd << ", " << Ki 
					  << "  err = " << err << endl;
	}

	// If need to stop
	double sum_dp = twiddle_dp[0] + twiddle_dp[1] + twiddle_dp[2];

	if (sum_dp < 0.01) {
		std::cout << "********************************************************" << endl;
		std::cout << "Twiddle End. The best p = [" << twiddle_p_best[0] << ", "
				  << twiddle_p_best[1] << ", " << twiddle_p_best[2] << "]." << endl;
		std::cout << "The best error is: " << twiddle_err_best << endl;
		std::cout << "********************************************************" << endl;

		twiddle_switch = false;
		Init(twiddle_p_best[0], twiddle_p_best[1], twiddle_p_best[2]);
	} else {
		Init(twiddle_p[0], twiddle_p[1], twiddle_p[2]);

		twiddle_idx = 0;
		twiddle_err = 0.0;

		if (twiddle_it % 10 == 0) {
			std::cout << "=======================" << endl;
			std::cout << "Twiddle dp: " << twiddle_dp[0] << ", " << twiddle_dp[1] << ", "
						 << twiddle_dp[2] << endl;
			std::cout << "=======================" << endl;
		}
	}
}

