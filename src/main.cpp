#include "Arduino.h"
#include "TMP23x.h"
#include "sTune.h"
#include "pid.h"

#if !( defined(MEGATINYCORE) )
  #error This is designed only fo
#endif

#ifdef LED_BUILTIN
  #undef LED_BUILTIN
#endif



#define VIN_SENSE     PIN_PA1
#define LED           PIN_PA2
#define LED1          PIN_PA3

#define SH_CTRL       PIN_PA4
#define VH_CTRL       PIN_PA5
#define SH_ADC_PIN    PIN_PA7
#define VH_ADC_PIN    PIN_PA6
// PIN_PB0  --> wired but currently not used
// PIN_PB1
#define SerialDebug         Serial


// Temperature objects, handles sensor interface
TMP23X TMP1;
TMP23X TMP2;

struct CONTROL 
{
    float setpoint;
    float input;
    float output;
    float kp;
    float ki;
    float kd;

};

/*
struct APP_DATA
{
    // structure containing application data, for passing through LOG and DEBUG interfaces
    float sample_heater_pwm_value = 0;
    float valve_heater_pwm_value = 0;
    float sample_temperature_c;
    float valve_temperature_c;
};
*/

#ifdef BOARDCONFIG_MK1_1
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    69
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  97
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255
#elif defined(BOARDCONFIG_MK2)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  101
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255
#elif defined(BOARDCONFIG_MK3)

#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  101
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255

#elif defined(BOARDCONFIG_MK4)
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   68
#define VALVE_ZONE_AMP_SOAK_TARGET_C    70
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 68
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  101
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255

#else

// DEFAULT-should work with all other env builds
#define SAMPLE_ZONE_AMP_SOAK_TARGET_C   65
#define VALVE_ZONE_AMP_SOAK_TARGET_C    65
#define SAMPLE_ZONE_VALVE_SOAK_TARGET_C 65
#define VALVE_ZONE_VALVE_SOAK_TARGET_C  90
#define HEATER_SHUTDOWN_C               0
#define SLEW_RATE_LIMIT                 255
#endif

#define numProcess                      1

CONTROL sample_amp_control[numProcess] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 2, 1, .5},

};
CONTROL valve_amp_control[numProcess] = 
{
  {HEATER_SHUTDOWN_C, 0, 0, 0, 0, 0},

};


// PID structure holder
pid_controller_t sample_zone;
pid_controller_t valve_zone;

// Reinit Controller based on STATE MACHINE
void pid_init(pid_controller_t &pid, CONTROL pid_settings){
  pid_controller_init(
    &pid, 
    pid_settings.setpoint,
    pid_settings.kp,
    pid_settings.ki,
    pid_settings.kd,
    255,
    255);
}

uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;
const uint16_t samples = 500;
const float inputSpan = 150;
const float outputSpan = 255;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 90;
float Setpoint = 60;
float Input_SH, Input_VH, Output_SH, Output_VH;
float kp,ki,kd;
sTune tuner = sTune();
sTune sample_tuner = sTune(&Input_SH, &Output_SH, tuner.ZN_PID, tuner.directIP,tuner.printALL);
//sTune sample_tuner = sTune(&sample_amp_control[0].input, &sample_amp_control[0].output, tuner.ZN_PID, tuner.directIP,tuner.printALL);
//sTune valve_tuner = sTune(&valve_amp_control[0].input,&valve_amp_control[0].output,tuner.ZN_PID, tuner.directIP,tuner.printALL);//

//sTune sample_tuner = sTune(&Input_SH, &Output_SH, tuner.ZN_PID, tuner.directIP,tuner.printALL);
//sTune valve_tuner = sTune(&Input_VH,&Output_VH,tuner.ZN_PID, tuner.directIP,tuner.printALL);//


void setup() {
  Serial.begin(9600);
  delay(10);

  //Serial.println(F("Startup"));

  //INIT peripherial I/O
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED,false);
  #ifdef LED1
    pinMode(LED1, OUTPUT);
  #endif

  digitalWrite(LED1,false);

  // INIT PWM I/O
  pinMode(SH_CTRL,OUTPUT);
  pinMode(VH_CTRL,OUTPUT);

  // INIT temperature sensor I/O
  TMP1.set_analog_pin(SH_ADC_PIN);
  TMP2.set_analog_pin(VH_ADC_PIN);
  TMP1.set_adc_reference();
  TMP2.set_adc_reference();

  sample_tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  sample_tuner.SetEmergencyStop(tempLimit);
  //valve_tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  //valve_tuner.SetEmergencyStop(tempLimit);
  
  pid_init(sample_zone,sample_amp_control[0]);
  //pid_init(valve_zone,valve_amp_control[0]);
}

void loop() {

    float optimumOutput_SH = sample_tuner.softPwm(SH_CTRL, Input_SH, Output_SH, Setpoint, 255, 0);
    //float optimumOutput_SH = sample_tuner.softPwm(SH_CTRL, sample_amp_control[0].input, sample_amp_control[0].output, Setpoint, 255, 0);
    //float optimumOutput_VH = valve_tuner.softPwm(VH_CTRL, valve_amp_control[0].input, valve_amp_control[0].output, Setpoint, 255, 0);

    switch (sample_tuner.Run()) {
      case sample_tuner.sample: // active once per sample during test
        Input_SH = TMP1.read_temperature_C();
        //sample_amp_control[0].input = TMP1.read_temperature_C();
        //valve_amp_control[0].input = TMP2.read_temperature_C();
        //Input_SH = data.sample_temperature_c;
        //Input_VH = data.valve_temperature_c;  

        sample_tuner.plotter(Input_SH, Output_SH, Setpoint, 0.5f, 3);
        //sample_tuner.plotter(sample_amp_control[0].input, sample_amp_control[0].output, Setpoint, 0.5f, 3);
        //sample_tuner.plotter(valve_amp_control[0].input, valve_amp_control[0].output, Setpoint, 0.5f, 3);
        break;

      case sample_tuner.tunings: // active just once when sTune is done

        sample_tuner.GetAutoTunings(&kp, &ki, &kd); // sketch variables updated by sTune
        sample_zone.k_p = kp;
        sample_zone.k_i = ki;
        sample_zone.k_d = kd;

        // sample_tuner.GetAutoTunings(&sample_zone.k_p,
        //                             &sample_zone.k_i,
        //                             &sample_zone.k_d);

        // sample_tuner.GetAutoTunings(&valve_zone.k_p,
        //                             &valve_zone.k_i,
        //                             &valve_zone.k_d);

        

        //valve_zone.k_p = Kp;
        //valve_zone.k_i = Ki;
        //valve_zone.k_d = Kd;

        break;

      case sample_tuner.runPid: // active once per sample after tunings
        Input_SH = TMP1.read_temperature_C();
        // sample_amp_control[0].input = TMP1.read_temperature_C();
  

        //valve_amp_control[0].input = TMP2.read_temperature_C();

        pid_controller_compute(&sample_zone,Input_SH);
        Output_SH = sample_zone.out;
        // pid_controller_compute(&sample_zone,sample_amp_control[0].input);
        //pid_controller_compute(&valve_zone,valve_amp_control[0].input);

        sample_tuner.plotter(Input_SH, optimumOutput_SH, Setpoint, 0.5f, 3);
        // sample_tuner.plotter(sample_amp_control[0].input, optimumOutput_SH, Setpoint, 0.5f, 3);
        //sample_tuner.plotter(valve_amp_control[0].input, optimumOutput_VH, Setpoint, 0.5f, 3);
        break;
    }
    
}