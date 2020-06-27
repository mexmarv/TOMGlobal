/**
   TOM VentMex Emergency Ventilator Controller

   TOM Makers - Mexico

   Copyright (c) 2020 makersmexico.org

   Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)
   This is a human-readable summary of (and not a substitute for) the license. Disclaimer.
   You are free to:
        Share — copy and redistribute the material in any medium or format
        Adapt — remix, transform, and build upon the material
   The licensor cannot revoke these freedoms as long as you follow the license terms.
   
   Under the following terms:
   Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. 
   You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

   NonCommercial — You may not use the material for commercial purposes.

   No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
   Notices:
   You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
   No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, 
   or moral rights may limit how you use the material.

   VMPresion.h
   Calculates and stores the key diffrential pressure values of the breathing cycle with
   a 200/200/10 Sample Filter Calibration for 100/5 Raw Pressure Readings.
   
   FYI: 3 mL (177 ± 96 psi) aprox Jeringa presion de https://pubmed.ncbi.nlm.nih.gov/21469942/
**/

#ifndef VMPresion_h
#define VMPresion_h
#include <Math.h>
#include <Streaming.h>

class VMPresion {
  public:
    VMPresion(int pin):
      pin_sensor_(pin),
      calibrado_v_(0.0),
      actual_(0.0),
      pico_(0.0),
      plateau_(0.0),
      peep_(0.0) {}
   
    void leeRaw()
    {
      raw = analogRead(pin_sensor_);
      pressCM = (0.0658 * raw) - 5.19; // Calculo de ecuación calibrada VentMex en CMH2O para medidor presión. Omer Test.

      Serial << "Raw Volts: " << _FLOAT(raw * (5.0 / 1023.0), 3) << " son " << raw << " digital = " << pressCM << " cmH2O\n";
    }
    
    void calibrar()
    {

      for (byte pressStep = 0; pressStep < 200; pressStep++)
      {
        rawRead[pressStep] = analogRead(pin_sensor_);
        for (byte calibrate = 0; calibrate <= 10; calibrate++)
        {
          rawP = 0;
          for (byte x = 0; x < 200; x++) rawP += rawRead[x];
          raw = rawP / 200;
        }
        delay(20); // aguanta estabilidad de sensor
      }

      pressCM = (0.0658 * raw) - 5.19; // Calculo de ecuación calibrada VentMex en cmH2O para medidor difernecial. Omer Test.

      calibrado_v_ = raw;
      calibrado_p_ = pressCM;
    }
    
    void lee() {
      for (byte pressStep = 0; pressStep < 5; pressStep++)
      {
        rawRead[pressStep] = analogRead(pin_sensor_);
        rawP = 0;
        for (byte x = 0; x < 100; x++) rawP += rawRead[x];
        raw = rawP / 100;
      }

      pressCM = (0.0658 * raw) - 5.19; // Calculo de ecuación calibrada VentMex en cmH2O para medidor difernecial. Omer Test.

      pressCM = pressCM - calibrado_p_; // quitas la calibracion para bajar la curva

      Serial << "PRESION cmH20: \t\t\t" << _FLOAT(pressCM,2) << " cmH2O\n\n";

      actual_ = pressCM;
      pico_ = max(pico_, pressCM);
    }

    const float& get() {
      return actual_;
    }

    void borra_pico() {
      pico_ = 0.0;
    }

    void pon_plateau() {
      plateau_ = get();
    }

    void pon_peep() {
      peep_ = get();
    }

    const float& calibrado() {
      return calibrado_v_;
    }
    const float& calibradop() {
      return calibrado_p_;
    }
    const float& pico() {
      return pico_;
    }
    const float& plateau() {
      return plateau_;
    }
    const float& peep() {
      return peep_;
    }

  private:
    byte            pin_sensor_, pressStep, calibrate;
    float           actual_, flujo_, area, velocity;
    float           pico_, plateau_, peep_, calibrado_v_, calibrado_p_;
    float           pressure, pressCM;
    float           rawRead[201], raw, rawP;
    
};

#endif
