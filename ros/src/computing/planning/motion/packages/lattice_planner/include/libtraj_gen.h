/*
 *  libtraj_gen.h
 *  Trajectory Generator Library
 *
 *  Created by Matthew O'Kelly on 7/17/15.
 *  Copyright (c) 2015 Matthew O'Kelly. All rights reserved.
 *  mokelly@seas.upenn.edu
 *
 */

/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

#include <fstream>

#pragma GCC diagnostic pop

// ---------DEFINE MODE---------//
//#define GEN_PLOT_FILES
//#define DEBUG_OUTPUT
//#define DEBUG
//#define ONE_ITER
//#define STANDALONE
#define USE_HEURISTIC_INIT
#define CUBIC_STABLE

// --------UNSTABLE MODES-------//
//#define QUINTIC_STABLE
//#define FIRST ORDER

namespace autoware
{
namespace planner
{
namespace lattice
{
// --------DATA STRUCTURES-------//
union State
{
  struct
  {
    double sx;
    double sy;
    double theta;
    double kappa;
    double v;
    double vdes;
    double timestamp;
  };

  double state_value[7];
};

union Parameters
{
  struct
  {
    double a;
    double b;
    double c;
    double d;
    double s;
  };

  double param_value[5];
};

union Spline
{
  struct
  {
    // double kappa_0;
    double s;
    double kappa_1;
    double kappa_2;
    // double kappa_2;
    // double kappa_3;

    double kappa_0;
    double kappa_3;
    bool success;
  };

  double spline_value[6];
};

union Command
{
  struct
  {
    double kappa;
    double v;
  };
  double cmd_index[2];
};

struct TrajectoryGenerator
{
  // ------------CONSTANTS----------//
  // Constants for forward simulation of ego vehicle
  // Maximum curvature (radians)
  static constexpr double KMAX = 0.1900;
  // Minimum curvature (radians)
  static constexpr double KMIN = -0.1900;
  // Maximum rate of curvature (radians/second)
  static constexpr double DKMAX = 0.1021;
  // Minimum rate of curvature (radians/second)
  static constexpr double DKMIN = -0.1021;
  // Maximum acceleration (meters/second^2)
  static constexpr double DVMAX = 2.000;
  // Maximum deceleration (meters/second^2)
  static constexpr double DVMIN = -6.000;
  // Control latency (seconds)
  static constexpr double TDELAY = 0.0800;
  // static constexpr double TDELAY = 0.03;
  // Speed control logic a coefficient
  static constexpr double ASCL = 0.1681;
  // Speed control logic b coefficient
  static constexpr double BSCL = -0.0049;
  // Speed control logic threshold (meters/second)
  static constexpr double VSCL = 4.000;
  // Max curvature for speed (radians)
  static constexpr double KVMAX = 0.1485;
  // Speed control logic safety factor
  static constexpr double SF = 1.000;

  // ------------TERMINATION CRITERIA----------//
  // User defined allowable errors for goal state approximation
  // Allowable crosstrack error (meters)
  static constexpr double CROSSTRACK_E = 0.001;
  // Allowable inline error (meters)
  static constexpr double INLINE_E = 0.001;
  // Allowable heading error (radians)
  static constexpr double HEADING_E = 0.1;
  // Allowable curvature error (meters^-1)
  static constexpr double CURVATURE_E = 0.005;
  // General error, ill-defined heuristic (unitless)
  static constexpr double GENERAL_E = 0.05;

  // ------------PARAMETER PERTURBATION----------//
  // User defined perturbations for estimation of partial derivatives
  // Perturbation for a
  static constexpr double H_SX = 0.001;
  // Perturbation for b
  static constexpr double H_SY = 0.001;
  // Perturbation for d
  static constexpr double H_THETA = 0.001;
  // Perturbation for s
  static constexpr double H_K = 0.001;
  // If all parameters are perturbed equally, heuristic (unitless)
  static constexpr double H_GLOBAL = 0.001;

  // --------INTEGRATION STEP SIZE CONTROL------//
  // Set time step
  static constexpr double STEP_SIZE = 0.0001;
  // Set lightweight timestep for plotting, used in genLineStrip
  static constexpr double PLOT_STEP_SIZE = 0.1;

  // static constexpr double STEP_SIZE = 0.05;

  // ------------LOG FILES----------//
  // Open files for data logging
  std::ofstream fmm_sx_;
  std::ofstream fmm_sy_;
  std::ofstream fmm_v_;
  std::ofstream fmm_theta_;
  std::ofstream fmm_kappa_;

  // ------------FUNCTION DECLARATIONS----------//

  // initParams is used to generate the initial guess for the trajectory
  union Spline initParams(union State veh, union State goal);

  // speedControlLogic prevents the vehicle from exceeding dynamic limits
  union State speedControlLogic(union State veh_next);

  // responseToControlInputs computes the vehicles next state consider control delay
  union State responseToControlInputs(union State veh, union State veh_next, double dt);

  // getCurvatureCommand computes curvature based on the selection of trajectory parameters
  double getCurvatureCommand(union Spline curvature, double dt, double v, double t);

  // getVelocityCommand computes the next velocity command, very naieve right now.
  double getVelocityCommand(double v_goal, double v);

  // motionModel computes the vehicles next state, it calls speedControlLogic, responseToControlInputs,
  // getCurvatureCommand and and getVelocityCommand
  union State motionModel(union State veh, union State goal, union Spline curvature, double dt, double horizon,
                          int flag);

  // checkConvergence determines if the current final state is close enough to the goal state
  bool checkConvergence(union State veh_next, union State goal);

  // pDerivEstimate computes one column of the Jacobian
  union State pDerivEstimate(union State veh, union State veh_next, union State goal, union Spline curvature, int p_id,
                             double h, double dt, double horizon, int stateIndex);

  // generateCorrection inverts the Jacobian and updates the spline parameters
  union Spline generateCorrection(union State veh, union State veh_next, union State goal, union Spline curvature,
                                  double dt, double horizon);

  // nextState is used by the robot to compute commands once an adequate set of parameters has been found
  union State nextState(union State veh, union Spline curvature, double vdes, double dt, double elapsedTime);

  // trajectoryGenerator is like a "main function" used to iterate through a series of goal states
  union Spline trajectoryGenerator(double sx, double sy, double theta, double v, double kappa);

  // plotTraj is used by rViz to compute points for line strip, it is a lighter weight version of nextState
  union State genLineStrip(union State veh, union Spline curvature, double vdes, double t);
};
}  // namespace lattice
}  // namespace planner
}  // namespace autoware
#endif  // TRAJECTORYGENERATOR_H
