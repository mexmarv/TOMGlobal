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

   VMFlujo.h
   Calculates and stores the key diffrential pressure values of the breathing cycle with
   a 200/200/10 Sample Filter Calibration for 100/5 Raw Pressure Readings.

**/

#ifndef VMFlujo_h
#define VMFlujo_h
#include <Math.h>
#include <Streaming.h>

class VMFlujo {
  public:
    VMFlujo(int pin):
      pin_sensor_(pin),
      calibrado_v_(0.0),
      calibrado_p_(0.0),
      actual_(0.0),
      flujo_(0.0) {}

    void leeRaw()
    {
      raw = analogRead(pin_sensor_);
      pressCM = (0.0844 * raw) - 5.029; // Calculo de ecuación calibrada VentMex en CMH2O para medidor difernecial. Omer Test.

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
        Serial << ".";
      }
      Serial << "\n";
      pressCM = (0.0844 * raw) - 5.029; // Calculo de ecuación calibrada VentMex en cmH2O para medidor difernecial. Omer Test.

      calibrado_v_ = raw;
      calibrado_p_ = pressCM;
    }

    const float& flujo(const float& FiO2) {

      for (byte pressStep = 0; pressStep < 5; pressStep++)
      {
        rawRead[pressStep] = analogRead(pin_sensor_);
        rawP = 0;
        for (byte x = 0; x < 100; x++) rawP += rawRead[x];
        raw = rawP / 100;
      }

      pressCM = (0.0844 * raw) - 5.029; // Calculo de ecuación calibrada VentMex en CMH2O para medidor difernecial. Omer Test.

      pressCM = pressCM - calibrado_p_; // quitas la calibracion para bajar la curva

      //Serial << "F-zero (cmH2O): " << abs(pressCM) << "\n";

      pressCM = 0.098067 * pressCM; // CMH20 a kPA
      velocity = sqrt((2 * abs(pressCM) * 1000) / ((1.2306 + FiO2 * 5.86) / 2)); // * 1000 para PA y entre densidad del aire 1.2306 y 21% FiO2 
      area = pow(0.060, 2) * 3.14159265359; // radio de circulo del espirometro es 6 mm
      flujo_ = velocity * area * 60; // m3/s * 60 = l/m

      Serial << "FLUJO - cmH20: " << _FLOAT(pressCM,2) << "\t\t" << _FLOAT(flujo_, 2) << " l/m \n";

      actual_ = flujo_;
      return flujo_;
    }


    // Calculo de ml a Pasos a ser verificado por Omer Test.
    float volumenApasos(const float& vol_ml) {
      return ((-AMBU.ay + sqrt(sq(AMBU.ay) - 4 * AMBU.ax * (AMBU.az - vol_ml))) / (2 * AMBU.ax)) / 8.7726; // formula para llegar a 850 con 140 grados segun nuestro diseno
    }

    // Pasos a ml (fullsteps), a ser verificado por Omer Test.
    float pasosAvolumen(const float& pasos) {
      float f1, f2, f3 = 0.0;
      f1 = sq(17.5452 * AMBU.ax * pasos + AMBU.ay) - sq(AMBU.ay);
      f2 = -4 * AMBU.ax;
      f3 = (f1 / f2) - AMBU.az;
      return (f3 * -1); // formula para llegar a 140 grados con 850 ml segun nuestro diseno
    }

    const float& get() {
      return actual_;
    }

    const float& calibrado() {
      return calibrado_v_;
    }
    const float& calibradop() {
      return calibrado_p_;
    }

  private:
    byte            pin_sensor_, pressStep, calibrate;
    float           actual_, flujo_, area, velocity;
    float           calibrado_v_, calibrado_p_;
    float           pressCM, calc3, pressure;
    float           raw, rawRead[201];
    float           timer, rawP;

    // VALORES DEL SENSOR MPXV4006 0 A 6KPA 0.2 A 4.8V y AMBU
    const struct {
      float ax, ay, az;
    } AMBU{ 1.29083271e-03, 4.72985182e-01, -7.35403067e+01 }; // Rangos de AMBU bolsa
    short   pressZero   = 54.219;             // Arduino Off
    short   pressMax    = 1023.0;           // Arduino Fss
    float   resolution  = 600.52;         // Resolucion obtenida de 4.6 Full Scale / 766 mV
};

#endif
