/*
PID::PID() {}

bool PID::Compute(){
    float input = *myInput;
    float dInput = input - lastInput;
    //if (action == Action::reverse) dInput = -dInput;

    error = *mySetpoint - input;
    //if (action == Action::reverse) error = -error;
    float dError = error - lastError;

    float peTerm = kp * error;
    float pmTerm = kp * dInput;

    pmTerm = 0; // force PonMeasure
    
    if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    
    pTerm = peTerm - pmTerm; // used by GetDterm()
    iTerm =  ki  * error;
    if (dmode == dMode::dOnError) dTerm = kd * dError;
    else dTerm = -kd * dInput; // dOnMeas

    //condition anti-windup (default)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
      if (iTermOut > outMax && dError > 0) aw = true;
      else if (iTermOut < outMin && dError < 0) aw = true;
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
    }

    // by default, compute output as per PID_v1
    outputSum += iTerm;                                                 // include integral amount
    if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);     // include pmTerm and clamp
    *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);  // include dTerm, clamp and drive output

    lastError = error;
    lastInput = input;
    lastTime = now;
    return true;
}

*/


#include "pid.h"

// slew_rate : 255 negates slew rate limiting
void pid_controller_init(pid_controller_t *pid, float setpoint, float k_p, float k_i, float k_d, int pid_max, float slew_rate) {
  // Clear controller variables
  pid->intergrator = 0.0f;
  pid->prevMesurement = 0.0f;
  pid->intergrator = 0.0f;
  pid->out = 0.0f;
  // Set from parameters
  pid->setpoint = setpoint;
  pid->k_p = k_p;
  pid->k_i = k_i;
  pid->k_d = k_d;
  // Set from defines
  pid->lim_max = pid_max;
  pid->lim_min = PID_LIM_MIN;

  pid->slew_rate = slew_rate;
}

void pid_controller_compute(pid_controller_t *pid, float measurement) {
  // Error Signal
  float error = pid->setpoint - measurement;
  // Proportional
  float proportial = pid->k_p * error;
  // Compute Integral
  pid->intergrator += (pid->k_i * error);
  if (pid->intergrator > pid->lim_max) {
    pid->intergrator = pid->lim_max;
  } else if (pid->intergrator < pid->lim_min) {
    pid->intergrator = pid->lim_min;
  }
  // Compute Differential on Input
  float d_input = measurement - pid->prevMesurement;
  // Compute PID Output
  float out = pid->k_p * error + pid->intergrator - pid->k_d * d_input;

  // Slew Rate Limiting
  if (out - pid->out > pid->slew_rate) {
    out = pid->out + pid->slew_rate;
  } else if (pid->out - out > pid->slew_rate) {
    out = pid->out - pid->slew_rate;
  }

  if (out > pid->lim_max) {
    out = pid->lim_max;
  } else if (out < pid->lim_min) {
    out = pid->lim_min;
  } 
  
  // Set output
  pid->out = out;
  // Keep Track of for Next Execution
  pid->prevMesurement = measurement;
}


bool compute(pid_controller_t *pid, float measurement) {
  float input = measurement;
  float dInput = input - pid->prevMesurement;
  //if (action == Action::reverse) dInput = -dInput;
  pid->error = pid->setpoint - input;
  //if (action == Action::reverse) error = -error;
  float dError = pid->error - pid->lastError;

  float peTerm = pid->k_p * pid->error;
  float pmTerm = pid->k_p * dInput;
  pmTerm = 0;
  /*
  if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    }*/
  pid->pTerm = peTerm - pmTerm;

  pid->iTerm =  pid->k_i  * pid->error;
  pid->dTerm = pid->k_d * dError;
/*
  if (dmode == dMode::dOnError) 
    else dTerm = -kd * dInput; // dOnMeas
*/

  /*
  //condition anti-windup (default)
  if (iawmode == iAwMode::iAwCondition) {
    bool aw = false;
    float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
    if (iTermOut > outMax && dError > 0) aw = true;
    else if (iTermOut < outMin && dError < 0) aw = true;
    if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
  }*/

  pid->outputSum += pid->dTerm; 
    
/*
  if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);  
*/
  pid->outputSum -= pmTerm;
  pid->out = constrain(pid->outputSum + peTerm + pid->dTerm, 0, 255);
  pid->lastError = pid->error;
  pid->lastInput = input;
  pid->lastTime = micros();
  return true;

}