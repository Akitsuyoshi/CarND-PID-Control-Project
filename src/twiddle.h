#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "PID.h"

using std::cout;
using std::endl;

std::vector<double> p = {0.01, 0.001, 0.5};  // Initial pid coefficients
std::vector<double> dp = {(p[0] / 10), (p[1] / 10), (p[2] / 10)};

int twiddle_itr = 0;
int twiddle_idx = 0;
int twiddle_state = 0;
double best_err = 0;
double current_err = 0;
double total_cte = 0.0;

void twiddle(PID &pid, double cte) {
  double idx = twiddle_idx % 3;
  total_cte += (cte * cte);
  twiddle_itr += 1;
  current_err = total_cte / twiddle_itr;

  if (twiddle_itr == 1) {
    // Init twiddle
    pid.UpdateCoefficient(p[0], p[1], p[2]);
    best_err = current_err;
  }

  /*
    Twiddle has 3 states, 0, 1, and 2. Each state implies update attempt.
    Attempts is done in a order, 0, 1, 2. If the attempt reduces error, it
    goes back to 0, otherwise it goes to the next attempt.
  */
  if (twiddle_state == 0) {
    // First attempt
    p[idx] += dp[idx];
    pid.UpdateCoefficient(p[0], p[1], p[2]);
    twiddle_state = 1;
  } else if (twiddle_state == 1) {
    if (current_err < best_err) {
      // First attempt reduces cte.
      best_err = current_err;
      dp[idx] *= 1.1;
      // Go to the next element of p, and back to twiddle_state 0
      twiddle_idx += 1;
      twiddle_state = 0;
    } else {
      // Second attempt
      p[idx] -= 2 * dp[idx];
      pid.UpdateCoefficient(p[0], p[1], p[2]);
      twiddle_state = 2;
    }
  } else if (twiddle_state == 2) {
    if (current_err < best_err) {
      // Third attempt reduces cte.
      best_err = current_err;
      dp[idx] *= 1.1;
    } else {
      // Above attempts don't improve err
      p[idx] += dp[idx];
      dp[idx] *= 0.95;  // NOTE: 0.95 has better results than 0.9
    }

    // Go to the next element of p, and back to twiddle_state 0
    twiddle_state = 0;
    twiddle_idx += 1;
  }
}
