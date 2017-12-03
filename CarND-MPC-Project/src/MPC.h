#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define VARS_BOUND 1.0e19
#define DELTA_BOUND 0.436332
#define A_BOUND 1

using namespace std;

// Configuration
#define cfg_N   10
#define cfg_dt  0.1

// tuning parmeters
#define cfg_paramsSteer   10000
#define cfg_paramsAccel   10
#define cfg_paramsdSteer  1000000
#define cfg_paramsdAccel  100
#define cfg_paramsCte     600
#define cfg_paramsEpsi    300
#define cfg_paramsV       1
/*
#define cfg_paramsSteer   1000
#define cfg_paramsAccel   5
#define cfg_paramsdSteer  10000
#define cfg_paramsdAccel  50
#define cfg_paramsCte     20
#define cfg_paramsEpsi    10
#define cfg_paramsV       0.1
*/
class MPC {
private:
  // Predicted points
  std::vector<double> m_vecMpcX;
  std::vector<double> m_vecMpcY;
  double m_prevSteering = 0;
  double m_prevThrottle = 0;

 public:
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  double GetLf();
  std::vector<double> GetMpcX();
  std::vector<double> GetMpcY();
  void SetPreInput(double prevSteering, double prevThrottle);

};

#endif /* MPC_H */
