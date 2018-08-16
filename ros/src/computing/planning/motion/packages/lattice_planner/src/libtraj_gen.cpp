/*
 *  libtraj_gen.cpp
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

/*
 * This file contains the vehicle model for use in predicting the ego vehicle's motion
 * It also contains a heuristic for initial trajetories and
 * a gradient descent method for refining them...
 *
 * First some constants are defined, these will be specific to your vehicle
 * Nine functions are included: speedControlLogic(),
 * responsetoControlInputs(), and most
 * importantly, the vehicle's motionModel() etc...
 *
 * motionModel() calls responseToControlInputs()
 * responseToControlInputs calls speedControlLogic()
 *
 * For a standalone build use the following command: g++ trajectorygenerator.cpp -o trajectorygenerator -O2 -larmadillo
 * Dependancies: Armadillo, http://arma.sourceforge.net/
 *
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <iostream>
#include <vector>
#include <armadillo>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "libtraj_gen.h"
// #include "trajectorygenerator.h"
#pragma GCC diagnostic pop

namespace autoware
{
namespace planner
{
namespace lattice
{
//-----------------------------FUNCTIONS------------------------------------//

// ------------INITIALIZE PARAMETERS----------//
// Heuristic to compute initial paramters given
// INPUT: Initial state, and a goal state
// OUTPUT: Estimated control parameters
// Use #define USE_HEURISTIC_INIT if using Nagy 2001 method
// This is necessary for cubic splines
// Use #define FIRST_ORDER if working with first order splines

union Spline TrajectoryGenerator::initParams(union State veh, union State goal)
{
  // Local variables for init and goal:
  double sx_f = goal.sx;
  double sy_f = goal.sy;
  double theta_f = goal.theta;
  double kappa_0 = veh.kappa;
  double kappa_f = goal.kappa;

  // Initialize output
  union Spline curvature;

  // Convenience
  double d_theta = abs(theta_f);

// Heuristic for initial guess from Nagy and Kelly, 2001
// 'Trajectory Generation for Car-like Robots using
//  Cubic Curvature Polynomials'
#ifdef USE_HEURISTIC_INIT
  // Order of computation matters, need to compute d before s and so on...

  // Commented out, because unused for cubic splines
  // double c = 0.0;

  // Note we must cast to double to avoid ambiguity
  double d = sqrt((double)pow(sx_f, 2) + (double)pow(sy_f, 2));

  double s = d * ((pow(d_theta, 2)) / 5.0 + 1.0) + (2.0 / 5.0) * d_theta;

  // Commented out because unused for cubic splines
  // double a = (6* (theta_f/(pow(s,2)))) - (2*kappa_0/s) + (4*kappa_f/s);

  double b = (3 / (pow(s, 2))) * (kappa_0 + kappa_f) + (6 * theta_f / (pow(s, 3)));

  double si = 0.00;
  curvature.kappa_0 = veh.kappa;
  curvature.kappa_1 =
      (1.00 / 49.00) * (8.00 * b * si - 8.00 * b * curvature.s - 26.00 * curvature.kappa_0 - curvature.kappa_3);
  curvature.kappa_2 = 0.25 * (curvature.kappa_3 - 2.00 * curvature.kappa_0 + 5.00 * curvature.kappa_1);
  curvature.kappa_3 = goal.kappa;
  curvature.s = s;

#endif

// Alternative heuristic initial guess for first order spline
// From Thomas Howard's dissertation page 38
#ifdef FIRST_ORDER
  double d = sqrt((double)pow(sx_f, 2) + (double)pow(sy_f, 2));
  double s = d * ((pow(d_theta, 2)) / 5 + 1) + (2 / 5) * d_theta;
  union Spline curvature;
  double si = 0.00;
  curvature.kappa_0 = veh.kappa;
  curvature.kappa_1 = 0.00;
  curvature.kappa_2 = 0.00;
  curvature.kappa_3 = goal.kappa;
  curvature.s = s;
#endif

  curvature.success = true;

  // Return the initialized curvature union
  return curvature;
}

// ------------SPEED CONTROL----------//
// Function for computing safe (feasible) speed and curvature
// INPUT: Next vehicle state
// OUTPUT: Updated next vehicle state

union State TrajectoryGenerator::speedControlLogic(union State veh_next)
{
  // Calculate speed (look up from next state vector)
  double vcmd = abs(veh_next.v);
  double kappa_next = veh_next.kappa;

  // Compute safe speed
  double compare_v = (kappa_next - ASCL) / BSCL;
  double vcmd_max = std::max(VSCL, compare_v);

  // Compute safe curvature
  double compare_kappa = ASCL + (BSCL * vcmd);
  double kmax_scl = std::min(KMAX, compare_kappa);

  // Check if std::max curvature for speed is exceeded
  if (kappa_next >= kmax_scl)
  {
    // Check for safe speed
    vcmd = SF * vcmd_max;
  }

  // Update velocity command, this is not quite equivalent to Ferguson, Howard, & Liukhachev
  veh_next.v = vcmd;

  // Return vehicle state
  return veh_next;
}

// ------------RESPONSE TO CONTROL----------//
// Function for computing the vehicles response to control inputs
// Mainly deals with delays associated with controls
// Also considers upper and lower safety bounds
// INPUT: Current vehicle state, next vehicle state, sampling time
// OUTPUT: Next vehicle state

union State TrajectoryGenerator::responseToControlInputs(union State veh, union State veh_next, double dt)
{
  // Local variables:
  double kappa = veh.kappa;
  double kappa_next = veh_next.kappa;
  double v = veh.v;
  double v_next = veh_next.v;

  // Compute curvature rate command
  double kdot = (kappa_next - kappa) / dt;

  // Check against upper bound on curvature rate
  kdot = std::min(kdot, DKMAX);

  // Check against lower bound on curvature rate
  kdot = std::max(kdot, DKMIN);

  // Call the speedControlLogic function
  veh_next = speedControlLogic(veh_next);

  // Compute curvature at the next vehicle state
  kappa_next = kappa + kdot * dt;

  // Check upper bound on curvature
  kappa_next = std::min(kappa_next, KMAX);

  // Check lower bound on curvature
  kappa_next = std::max(kappa_next, KMIN);

  // Compute acceleration command
  double vdot = (v_next - v) / dt;

  // Check for upper bound on acceleration
  vdot = std::min(vdot, DVMAX);

  // Check for lower bound on acceleration
  vdot = std::max(vdot, DVMIN);

  // Compute velocity at next state
  veh_next.v = v + vdot * dt;

  // Return next vehicle state
  return veh_next;
}

// ------------COMPUTE CURVATURE----------//
// Computes the curvature at the next state
// Still messy but will clean shortly...

double TrajectoryGenerator::getCurvatureCommand(union Spline curvature, double dt, double v, double t)
{
  // Local variables for curvature constants
  double kappa_0 = curvature.kappa_0;
  double kappa_1 = curvature.kappa_1;
  double kappa_2 = curvature.kappa_2;
  double kappa_3 = curvature.kappa_3;
  double s = curvature.s;

  // Init next command
  double k_next_cmd = 0.0;

  // Assume position at t=0, si=0
  double si = 0.0;

  // Estimate distance traveled on spiral
  double st = v * t;

// These equations can be found in McNaughton, page 76
// They make the knot points equally spaced.
// They apply to stable cubic paths
#ifdef CUBIC_STABLE
  double a = 0;
  double b = 0;
  double c = 0;
  double d = 0;
  a = kappa_0;
  b = (-0.50) * (-2 * kappa_3 + 11 * kappa_0 - 18 * kappa_1 + 9 * kappa_2) / (s - si);
  c = (4.50) * (-kappa_3 + 2 * kappa_0 - 5 * kappa_1 + 4 * kappa_2) / (pow((s - si), 2));
  d = (-4.50) * (-kappa_3 + kappa_0 - 3 * kappa_1 + 3 * kappa_2) / (pow((s - si), 3));
  k_next_cmd = a + b * st + c * pow(st, 2) + d * pow(st, 3);
#endif

// These equations can be found in McNaughton, page 77
// They make the knot points equally spaced.
// They apply to stable quintic paths
#ifdef QUINTIC_STABLE
  double a = kappa_0;
  double b = kappa_1;
  double c = kappa_2 / 2.0;
        double d = -(575*kappa_0 -648*kappa_3 +81*kappa_4 -8*kappa_5 +170*kappa_1*st +22*kappa_2*pow(st,2)/ (8*pow(st,3));
        double e = 9*(37*kappa_0 -45*kappa_3 +9*kappa_4 -kappa_5 +10*kappa_1*st +kappa_2*pow(st,2))/(2*pow(st,4));
        double f = -9*(85*kappa_0 -108*kappa_3 +27*kappa_4 -4*kappa_5 +22*kappa_1*st +2*kappa_2*pow(st,2))/(8*pow(st,5));
        k_next_cmd = a + b*st + c*pow(st,2) + d*pow(st,3) + e*pow(st,4) +f*pow(st,5);
#endif

// Compute next command from Howard page 39
#ifdef FIRST_ORDER
        k_next_cmd = kappa_0 + ((kappa_1 - kappa_0)/s) * (st);
#endif

    // Return value
    return k_next_cmd;
}

// ------------COMPUTE VELOCITY----------//
// Updates the vehicles velocity based on velocity profile
// Currently assumes constant
/// Will update to include ramp...

double TrajectoryGenerator::getVelocityCommand(double v_goal, double v)
{
#ifdef DEBUG
  std::cout << "Function: getVelocityCommand()" << std::endl;
#endif
  // Some function based on parameterization of velocity profile
  // Set to constant for now
  double v_next_cmd = v_goal;
  return v_next_cmd;
}

// ------------MOTION MODEL----------//
// Computes update to vehicle state
// INPUT: Current vehicle state, parameterized control inputs, sampling time
// OUTPUT: Next vehicle state

union State TrajectoryGenerator::motionModel(union State veh, union State goal, union Spline curvature, double dt,
                                             double horizon, int flag)
{
  // Initialized the elapsed time to 0.0 s
  double t = 0.0;
  // Setup local data structures for holding
  // the previous and next vehicle states
  union State veh_next;
  union State veh_temp;
  // Set veh_temp to the current state
  veh_temp = veh;
  // Compute the stop time for the simulation
  horizon = curvature.s / goal.v;

  while (t < horizon)
  {
    // Read state vector
    double sx = veh_temp.sx;
    double sy = veh_temp.sy;
    double v = veh_temp.v;
    double theta = veh_temp.theta;
    double kappa = veh_temp.kappa;
    double v_goal = goal.v;

    // Compute change in 2D x-position (this is x_dot)
    double sx_next = sx + (v * cos(theta) * dt);

    veh_next.sx = sx_next;

    // Compute change in 2D y-position (this is y_dot)
    double sy_next = sy + (v * sin(theta) * dt);

    veh_next.sy = sy_next;

    // Compute change in 2D orientation (this is theta_dot)
    double theta_next = theta + (v * kappa * dt);

    veh_next.theta = theta_next;

    // Get curvature command
    double kappa_next = getCurvatureCommand(curvature, dt, veh.v, t);

    veh_next.kappa = kappa_next;

    // Get velocity command
    double v_next = getVelocityCommand(v_goal, v);

    veh_next.v = v_next;

    // Get acceleration command, not used yet...
    // double a_next_cmd = 0;

    // Update the next vehicle state
    veh_next = responseToControlInputs(veh, veh_next, dt);

    // Increment the timestep
    t = t + dt;

    // Update veh_temp
    veh_temp = veh_next;

    // Write to log file if we are testing
    if (flag == 1)
    {
      fmm_sx_ << veh_temp.sx << ", ";
      fmm_sy_ << veh_temp.sy << ", ";
      fmm_v_ << veh_temp.v << ", ";
      fmm_theta_ << veh_temp.theta << ", ";
      fmm_kappa_ << veh_temp.kappa << ", ";
    }
  }
// Print information to console
#ifdef DEBUG
  std::cout << "Next state sx: " << veh_next.sx << std::endl;
  std::cout << "Next state sy: " << veh_next.sy << std::endl;
  std::cout << "Next state theta: " << veh_next.theta << std::endl;
  std::cout << "Next state v: " << veh_next.v << std::endl;
  std::cout << "Next state kappa: " << veh_next.kappa << std::endl;
#endif

  // Return the state at the end of the trajectory
  return veh_next;
}

// ------------CHECK CONVERGENCE----------//
// Checks to see if we have reached target solution
// INPUT: Next vehicle state predicted by the forward motion model and goal state
// OUTPUT: Boolean
// TO DO: if theta is constrained, please check that this is an appropriate metric.
// IE consider 359 degrees vs. 1 degree
// Can use fmod (modulus) over pi or 2*pi

bool TrajectoryGenerator::checkConvergence(union State veh_next, union State goal)
{
#ifdef DEBUG
  std::cout << "Function: checkConvergence()" << std::endl;
#endif

  double sx_error = abs(veh_next.sx - goal.sx);
  double sy_error = abs(veh_next.sy - goal.sy);
  double theta_error = abs(veh_next.theta - goal.theta);

  // Commented out to suppress compiler warning about unused var
  // double v_error = abs(veh_next.v - goal.v);
  // double kappa_error = abs(veh_next.kappa - goal.kappa);

#ifdef DEBUG_OUTPUT
  std::cout << "Error sx: " << sx_error << std::endl;
  std::cout << "Error sy: " << sy_error << std::endl;
  std::cout << "Error theta: " << theta_error << std::endl;
// Commented out to suppress compiler warning about unused var
// std::cout << "Error v: " << v_error << std::endl;
// std::cout << "Error kappa: " << kappa_error << std::endl;
#endif

  // Depending on the order of the spline the full range of checks is:
  // if(sx_error<GENERAL_E && sy_error<GENERAL_E && theta_error<GENERAL_E
  // && theta_error<GENERAL_E && v_error<GENERAL_E && kappa_error<GENERAL_E)
  if (sx_error < GENERAL_E && sy_error < GENERAL_E && theta_error < GENERAL_E)
  {
#ifdef DEBUG
    std::cout << "Converged" << std::endl;
#endif
    return true;
  }

  else
  {
#ifdef DEBUG
    std::cout << "Not Converged" << std::endl;
#endif
    return false;
  }
}

// ------------ESTIMATE JACOBIAN----------//
// Forward difference method via predictive motion model
// INPUT: Initial State, Target State, Parameterized Action,
//        Predictive Motion Model, Action Parameters
// OUTPUT: Jacobian Estimate

union State TrajectoryGenerator::pDerivEstimate(union State veh, union State veh_next, union State goal,
                                                union Spline curvature, int p_id, double h, double dt, double horizon,
                                                int stateIndex)
{
#ifdef DEBUG
  std::cout << "Function: pDerivEstimate()" << std::endl;
#endif

  // Index for iterating through parameters
  int i = 0;

  // Compute difference between desired sx and actual
  // May need to update with intial state if not 0,0,0,0,0
  union State delta_state;

  // Compute the difference between the end state of the current spline and the goal
  for (i = 0; i < stateIndex; i++)
  {
    delta_state.state_value[i] = (goal.state_value[i] - veh_next.state_value[i]);
  }

  // Create parameter perturbation vector
  union Spline perturb_curve = curvature;

  // Perturb parameter
  perturb_curve.spline_value[p_id] = curvature.spline_value[p_id] + h;

  // Check forward motion model with perturbed parameter
  union State perturb_next = motionModel(veh, goal, perturb_curve, dt, horizon, 0);

  // Now create a new structure to hold the difference between goal and perturbed forward motion model
  union State delta_perturb_state;

  // Compute difference between goal sx and actual
  for (i = 0; i < stateIndex; i++)
  {
    delta_perturb_state.state_value[i] = goal.state_value[i] - perturb_next.state_value[i];
  }

  // Structure to hold the difference between the perturbed error and unperturbed erro
  union State partial_state_vector;

  // Estimate partial derivative
  for (i = 0; i < stateIndex; i++)
  {
    double num = (delta_perturb_state.state_value[i] - delta_state.state_value[i]);
    partial_state_vector.state_value[i] = num / h;
  }

  return partial_state_vector;
}

// ------------UPDATE PARAMETERS----------//
// Inverts the Jacobian and computes the parameter update

union Spline TrajectoryGenerator::generateCorrection(union State veh, union State veh_next, union State goal,
                                                     union Spline curvature, double dt, double horizon)
{
  // Compute jacobian with forward gradient
  int i;
  int stateIndex = 3;
  int j;

  // Matrix J will contain the Jacobian
  arma::mat J(stateIndex, stateIndex);

  // Initialize it to zero
  J.fill(0.0);

  // Parameter perturbation vector
  arma::vec h(stateIndex);

  // How much to perturb each parameter
  // Note h(0) is larger than others because it perturbs length ~20m
  // Rather than kappa < ~.19 rad/m
  h(0) = H_GLOBAL * 10;
  h(1) = H_GLOBAL;
  h(2) = H_GLOBAL;

  // For each parameter compute the vector associated with a small perturbation of its value
  for (i = 0; i < stateIndex; i++)
  {
    union State temp;
    temp = pDerivEstimate(veh, veh_next, goal, curvature, i, h(i), dt, horizon, stateIndex);

    // For each element of the state vector place the value in the Jacobian matrix (column-wise)
    for (j = 0; j < stateIndex; j++)
    {
      J(j, i) = temp.state_value[j];
    }
  }

  // Put parameters in a vector
  arma::vec dX(stateIndex);
  dX.fill(0.0);

  for (i = 0; i < stateIndex; i++)
  {
    dX(i) = goal.state_value[i] - veh_next.state_value[i];
  }

  // Solve for delta P
  // Note that we use a try catch routine as it is possible for J to be singular
  arma::vec dP(stateIndex);
  try
  {
    J = J.i();
  }
  catch (const std::exception& e)
  {
    curvature.success = false;
    return curvature;
    std::cout << "goal.sx: " << goal.sx << std::endl;
    std::cout << "goal.sy: " << goal.sx << std::endl;
    std::cout << "goal.theta: " << goal.sx << std::endl;
  }

  dP = J * dX;

  // Update curvature parameters
  for (i = 0; i < stateIndex; i++)
  {
    curvature.spline_value[i] = curvature.spline_value[i] - dP(i);
  }

  // Return new spline
  return curvature;
}

// ------------NEXT STATE----------//
// Computes update to vehicle state
// for the purpose of control
// Not called in local planner
// INPUT: Current vehicle state, parameterized control inputs, sampling time
// OUTPUT: Next vehicle state

union State TrajectoryGenerator::nextState(union State veh, union Spline curvature, double vdes, double dt,
                                           double elapsedTime)
{
  union State veh_next;
  union State veh_temp;
  double total_time = elapsedTime + dt;
  // std::cout<<"total_time: "<< total_time<<std::endl;
  double t = 0.00;

  veh_temp = veh;

  // Read state vector
  double sx = veh_temp.sx;
  double sy = veh_temp.sy;
  double v = veh_temp.v;
  double theta = veh_temp.theta;
  double kappa = veh_temp.kappa;
  double v_goal = vdes;

  // std::cout<<"Total sim time: " << total_time << std::endl;

  while (t <= total_time)
  {
    // Compute change in 2D x-position (this is x_dot)
    double sx_next = sx + (v * cos(theta) * dt);

    veh_next.sx = sx_next;

    // Compute change in 2D y-position (this is y_dot)
    double sy_next = sy + (v * sin(theta) * dt);

    veh_next.sy = sy_next;

    // Compute change in 2D orientation (this is theta_dot)
    double theta_next = theta + (v * kappa * dt);

    veh_next.theta = theta_next;

    // Get curvature command
    double kappa_next = getCurvatureCommand(curvature, dt, veh.v, t);

    veh_next.kappa = kappa_next;

    // Get velocity command
    double v_next = getVelocityCommand(v_goal, v);

    veh_next.v = v_next;

    // Get acceleration command
    // double a_next_cmd = 0;

    // Update the next vehicle state
    veh_next = responseToControlInputs(veh, veh_next, dt);

    // Increment the timestep
    t = t + STEP_SIZE;

    // Update veh_temp
    veh_temp = veh_next;
  }
  std::cout << "Exited the while loop t = " << t << std::endl;

  return veh_next;
}

// plotTraj is used by rViz to compute points for line strip,
// it is a lighter weight version of nextState
union State TrajectoryGenerator::genLineStrip(union State veh, union Spline curvature, double vdes, double t)
{
  union State veh_next;
  union State veh_temp;
  double dt = PLOT_STEP_SIZE;

  veh_temp = veh;

  // Read state vector
  double sx = veh_temp.sx;
  double sy = veh_temp.sy;
  double v = veh_temp.v;
  double theta = veh_temp.theta;
  double kappa = veh_temp.kappa;
  double v_goal = vdes;

  // Compute change in 2D x-position (this is x_dot)
  double sx_next = sx + (v * cos(theta) * dt);

  veh_next.sx = sx_next;

  // Compute change in 2D y-position (this is y_dot)
  double sy_next = sy + (v * sin(theta) * dt);

  veh_next.sy = sy_next;

  // Compute change in 2D orientation (this is theta_dot)
  double theta_next = theta + (v * kappa * dt);

  veh_next.theta = theta_next;

  // Get curvature command
  double kappa_next = getCurvatureCommand(curvature, dt, veh.v, t);

  veh_next.kappa = kappa_next;

  // Get velocity command
  double v_next = getVelocityCommand(v_goal, v);

  veh_next.v = v_next;

  // Get acceleration command
  // double a_next_cmd = 0;

  // Update the next vehicle state
  veh_next = responseToControlInputs(veh, veh_next, dt);

  // Update veh_temp
  veh_temp = veh_next;

  return veh_next;
}
}  // namespace lattice
}  // namespace planner
}  // namespace autoware

//------------------MAIN FUNCTION AND HELPER FOR STANDALONE OPERATION------------------------//

#ifdef STANDALONE

union Spline TrajectoryGenerator::trajectoryGenerator(double sx, double sy, double theta, double v, double kappa)
{
#ifdef DEBUG
  std::cout << "Function: main()" << std::endl;
#endif

  // Set velocity
  double v_0 = 0.10;

  // Initialize convergence criteria to false
  bool convergence = false;

  // Set goal vector
  union State goal;
  goal.sx = sx;
  goal.sy = sy;
  goal.theta = theta;
  goal.v = v;
  goal.kappa = kappa;

  // Initialize and set current state vector
  union State veh;
  veh.sx = 0.0;
  veh.sy = 0.0;
  veh.theta = 0.0;
  veh.v = v;
  veh.kappa = 0.0;

  // Initialize next state
  union State veh_next;
  veh_next.sx = 0.0;
  veh_next.sy = 0.0;
  veh_next.theta = 0.0;
  veh_next.v = 0.00;
  veh_next.kappa = 0.0;

  // Initialize parameters as a function of init and goal vectors
  union Spline curvature;
  curvature = initParams(veh, goal);

  // Initialize iteration counter
  int iteration = 0;

  // Set timestep
  double dt = step_size;

  // While loop for computing trajectory parameters
  while (convergence == false && iteration < 10)
  {
    // Set time horizon
    double horizon = curvature.s / v_0;

    // Run motion model
    veh_next = motionModel(veh, goal, curvature, dt, horizon, 0);

    // Determine convergence criteria
    convergence = checkConvergence(veh_next, goal);

    // If the motion model doesn't get us to the goal compute new parameters
    if (convergence == false)
    {
      // Update parameters
      curvature = generateCorrection(veh, veh_next, goal, curvature, dt, horizon);
      iteration++;
      if (curvature.success == false)
      {
        break;
      }

#ifdef ONE_ITER
      convergence = true;
#endif
    }
  }

  if (convergence == false)
  {
    std::cout << "Not Converged" << std::endl;
  }

  else
  {
    std::cout << "Converged in " << iteration << " iterations" << std::endl;

#ifdef LOG_OUTPUT
    // Set time horizon
    double horizon = curvature.s / v_0;
    // Run motion model and log data for plotting
    veh_next = motionModel(veh, goal, curvature, 0.1, horizon, 1);
    fmm_sx << "0.0 \n";
    fmm_sy << "0.0 \n";
#endif
  }

  return curvature;
}

int main(void)
{
  // Open files for logging
  fmm_sx.open("fmm_sx.dat");
  fmm_sy.open("fmm_sy.dat");
  fmm_v.open("fmm_v.dat");
  fmm_theta.open("fmm_theta.dat");
  fmm_kappa.open("fmm_kappa.dat");

  int i;
  int j;
  int k = 0;
  double sx = 1;
  double sy = 1;
  union Spline curvature;

#ifdef GEN_PLOT_FILES
  for (i = 5; i < 20; i++)
  {
    sx = i;
    for (j = -50; j < 50; j++)
    {
      if (abs(j) < i + 10)
      {
        std::cout << "Trajectory Number: " << k << std::endl;
        sy = j;
        double k_goal;
        if (j < 0)
        {
          k_goal = 0.15;
        }
        else
        {
          k_goal = -0.15;
        }
        curvature = trajectoryGenerator(sx, sy, 0.0, 0.1, 0.0);
        k++;
      }
    }
  }
#endif

  trajectoryGenerator(33.7553, 2.04623, 0.0726169, 11.1111, 0.0);

  // Close files
  fmm_sx.close();
  fmm_sy.close();
  fmm_v.close();
  fmm_theta.close();
  fmm_kappa.close();

  return 1;
}
#endif
