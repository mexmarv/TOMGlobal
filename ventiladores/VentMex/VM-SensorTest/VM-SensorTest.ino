/*
   Demo for VentMex Motor Control and Calibration, using AccelStepper with NEMA-23, Sensors Pressure
   200 Steps, 32 Microsteps
*/

#include "VMPresion.h"
#include "VMFlujo.h"
#include <Streaming.h>

VMPresion presion(A0);  // Presion
VMFlujo flujo(A1);    // Flujo (presion diferencial)
long now;

void setup()
{
  
  Serial.begin(115200);
  
  Serial << "TOM - VentMex comenzando la calibracion ...\n";

  now = millis();
  flujo.calibrar();
  presion.calibrar();
  Serial << "Calibramos el CERO en " << _FLOAT(flujo.calibrado(),3) << " DAC | " << _FLOAT(flujo.calibradop(),3) << " cmH20\n";
  Serial << "Calibramos el CERO en " << _FLOAT(presion.calibrado(),3) << " DAC | " << _FLOAT(presion.calibradop(),3) << " cmH20\n...ambos calibrados en " << (millis() - now) << " ms. \n";

}

void loop()
{

  // Ahora probando Flujo nada mas
  now = millis();
  flujo.flujo(0.21); //Pones el valor de FiO2
  //flujo.leeRaw();
  Serial << "FLUJO: Lectura Actual en: " << (millis() - now) << "ms. \n";
  now = millis();
  presion.lee(); 
  //Serial1 << "E" << _FLOAT(flujo.flujo(0.21),2) << "," << _FLOAT(presion.get(),2)<<"\n";
  //presion.leeRaw();
  Serial << "PRESION: Lectura Actual en: " << (millis() - now) << "ms. \n";
  delay(500);
}
