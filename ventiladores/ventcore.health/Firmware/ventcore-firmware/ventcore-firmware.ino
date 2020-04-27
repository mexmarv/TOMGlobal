/*
   VentCore Firmware v0.1
   www.ventcore.health
   Copyright (C)2020 Formon LLC

   This file may be redistributed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   A copy of this license has been included with this distribution in the file LICENSE.

   TOM Makers - Mexico Chapter have joined the prototype build and will be contribute to this code and hardware.
   To differantiate prototypes due to motors and 3D parts as they are being tested, we will include a model
   Marvin Nahmias - marvin@tomglobal.org


*/

// ********** BEGIN INCLUDES
// Define includes and libraries
#include <Arduino.h>
// Include the driver you are using here
// #include "A4988.h"
#include "DRV8825.h"
// No RTOS installed yet, will be using threads libraries.
#include "Thread.h"
#include "ThreadController.h"
// ********** END INCLUDES

// ********** BEGIN DEFINES
// Formon PCB pinouts
// TODO: If Ramps 1.4 or similar shields will be used for testing o prototype purposes, change here.
// MODEL can be ORIGINAL, or MEXICO in this case, DRIVER with the correct DRIVER library.
// Define DEBUG to verbose output through Serial Port.
//
#define DEBUG
#define MODEL "ORIGINAL"
#define DRIVER "D8825"
#define EN 4
#define DIR 12
#define STEP 30
#define ENDSTOP_PIN 8
#define VENTILATOR_PIN 9
#define CPAP_ENABLED_PIN 10
#define PRESSURE_SENSOR_PIN A0
#define DIFF_PRESSURE_SENSOR_PIN A1

#define MOTOR_STEPS 200
#define FULL_TD_STEPS 60  // THIS NEEDS TO BE CHANGED FOR MAXIMUM TORQUE AND TIDAL VOLUME
#define RPM 20            // TO BE CALIBRATED FOR Td/I-time
#define MICROSTEPS 32
// ********** END DEFINES

// ********** BEGIN Callbacks and Functions
// Initialize Motor Driver
// Could eventually use AcceleStep.h or similar
DRV8825 stepper(MOTOR_STEPS, DIR, STEP);

// Initialize CONTROLLER with Sensor, Parameter and Motor Threads (3) and follow ith CallBacks to be calibrated
ThreadController controller = ThreadController();
Thread* sensorThread = new Thread();
Thread* paramsThread = new Thread();
Thread* motorsThread = new Thread();

// Initialize Variables
bool cpapEnabled = false;
int tdSteps = 0;
float expirationWaitTime = 0;
bool inspiration = true;
unsigned long expirationTime;

float a0value, pressure, diffpressure, velocity, area, flow;
float td, rr, itime;

// Sensors Thread
void sensorThreadCallback() {
  // TODO: to be calibrated
  a0value = map(analogRead(PRESSURE_SENSOR_PIN), 61, 927, 0, 6000);
  pressure = a0value / 98; // cmh2o

  diffpressure = map(analogRead(DIFF_PRESSURE_SENSOR_PIN), 64, 927, 0, 6000);
  velocity = sqrt((diffpressure * 2) / 1.225);
  area = pow(0.005, 2) * 3.14159265359;
  flow = velocity * area; // cm3/s

  // Debug through Serial Port TODO: I2C Display
#ifdef DEBUG
  Serial.print("FLO: ");
  Serial.print(flow);
  Serial.print(" , PRS: ");
  Serial.println(pressure);
#endif

}

// Parameters Thread
void paramsThreadCallback() {
  if (digitalRead(CPAP_ENABLED_PIN) == LOW) {
    cpapEnabled = true;
  } else {
    cpapEnabled = false;
  }

  //TODO: HANDLE POTS VALUE ACCURACY
  td = map(analogRead(A5), 1023, 0, 250, 600);
  tdSteps = (int) ((td / 600) * FULL_TD_STEPS);

  rr = map(analogRead(A4), 1023, 0, 100, 300);
  itime = map(analogRead(A3), 1023, 0, 100, 300);

  //TODO: I-time to be handled when RR > 15 bpm
  expirationWaitTime = (60 / (rr / 10)) - 1;  //(itime/100);
  stepper.setRPM((td / 600) * RPM * 1);       //(1/(itime/100)));
}

// Motors Thread
void motorsThreadCallback() {
  if (digitalRead(ENDSTOP_PIN) == HIGH) {
    stepper.stop();
    stepper.move(5 * MICROSTEPS);
  }

  if (cpapEnabled && pressure < 0) {
    inspiration = false;
    //TODO: IMPLEMENT CHECK IF SET PRESSURE REACHED THEN STOP THE INSPIRATION
    stepper.startMove(abs(tdSteps)*MICROSTEPS);
  }

  if (inspiration && !cpapEnabled) {
    if (millis() - expirationTime > expirationWaitTime * 1000) {
      inspiration = false;
      stepper.startMove(abs(tdSteps)*MICROSTEPS);
    }
  }

  unsigned wait_time_micros = stepper.nextAction();
  if (wait_time_micros <= 0 && !inspiration) {
    expirationTime = millis();
    inspiration = true;
    stepper.startMove(-1 * tdSteps * MICROSTEPS);
  }

}

// ********** END Callbacks and Functions

// Ventcore-health Setup
void setup() {
  // Start Serial Port communication
  Serial.begin(250000);

  // Debug through Serial Port TODO: I2C Display
#ifdef DEBUG
  Serial.println("Ventcore-Health Initializing");
  Serial.println("----------------------------");
  Serial.print("Model: ");
  Serial.print(MODEL);
  Serial.print(" m-DRV: ");
  Serial.println(DRIVER);
  Serial.println("Setting INPUT Pins...");
#endif

  // Set Input Pins
  pinMode(ENDSTOP_PIN, INPUT);
  pinMode(VENTILATOR_PIN, INPUT);
  pinMode(CPAP_ENABLED_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(DIFF_PRESSURE_SENSOR_PIN, INPUT);

  // Initialize Threads with CallBacks. 100 ms for Sensors and Parameter changes, 0 ms (almost immediate) for Motor Control. TODO: Evalute RTOS
#ifdef DEBUG
  Serial.println("Initializing (3) Threads at 100/100/0ms Sensors/Params/Motors...");
#endif
  sensorThread->onRun(sensorThreadCallback);
  sensorThread->setInterval(100);

  paramsThread->onRun(paramsThreadCallback);
  paramsThread->setInterval(100);

  motorsThread->onRun(motorsThreadCallback);
  motorsThread->setInterval(0);

  // Add threads to controller
  controller.add(sensorThread);
  controller.add(paramsThread);
  controller.add(motorsThread);

  // Start Motor with RPM and u-steps
#ifdef DEBUG
  Serial.println("Starting engines...");
#endif
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();

  while (digitalRead(ENDSTOP_PIN) == LOW) {
    stepper.move(-1 * MICROSTEPS);
  }
  stepper.move(5 * MICROSTEPS);
#ifdef DEBUG
  Serial.println("Finished boot up. Running ProtoThreads");
#endif
}

// Loop is ProtoThreaded and run when Ventilator_Pin is High (ON)
void loop() {
  if (digitalRead(VENTILATOR_PIN) == HIGH) {
    controller.run();
  }
}
