/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;

   prev_cte = 0;
   diff_cte = 0;
   int_cte = 0;

   dt = 0;

   _output_lim_maxi = output_lim_maxi;
   _output_lim_mini = output_lim_mini;
}


void PID::UpdateError(double cte, bool debug) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if (dt > 0){
      diff_cte = (cte - prev_cte)/dt;

   }
   else {
      diff_cte = 0.0;
   }
   int_cte += cte * dt;
   prev_cte = cte;

   if (debug){
      std::cout <<   "P(" << prev_cte << ")" <<
                     "I(" << int_cte  << ")" <<
                     "D(" << diff_cte << ")";
   }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = (Kp * prev_cte) + (Ki * int_cte) + (Kd * diff_cte);
    //clamp control to wihtin the output limits
    if (control <  _output_lim_mini){
       control = _output_lim_mini;
    }
    if (control > _output_lim_maxi){
       control = _output_lim_maxi;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_delta_time;

   return dt;
}