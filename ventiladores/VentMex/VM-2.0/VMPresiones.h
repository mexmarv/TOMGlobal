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
   Calculates and stores the key pressure values of the breathing cycle with
   a 200 Sample Filter for Raw Pressure Readings.

   FYI: 3 mL (177 ± 96 psi) aprox Jeringa presion de https://pubmed.ncbi.nlm.nih.gov/21469942/
**/

#ifndef VMPresiones_h
#define VMPresiones_h
#include <Math.h>
#include <Streaming.h>

class VMPresiones {
  public:
    VMPresiones(int pin):
      pin_sensor_(pin),
      calibrado_(0.0),
      actual_(0.0),
      pico_(0.0),
      plateau_(0.0),
      peep_(0.0),
      flujo_(0.0) {}

    // Calibracion de Presion con Filtro Circular de 200 Muestras de 200 Pasos (Circular) sin nada conectado aprox 2670 ms.
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
          calc1 = raw - pressZero;
          calc2 = calc1 * pressMax;
          calc3 = resolution - float(pressZero);
          if (raw <= pressZero)
          {
            pressPsi = 0.0;
          }
          else
          {
            pressPsi = (float(calc2) / float(calc3)) / 100; // Convertir valor a PSI correcto
          }
          pressCM = pressPsi * 70.30695782; // psi a cmH20
          if (raw > pressZeroP) pressZeroP = raw;
        }
        pressZero = pressZeroP;
        delay(20); // aguanta estabilidad de sensor
      }
      calibrado_ = pressZero;
      calibrado_p_ = pressCM;
    }

    // Lectura de Presion con Filtro Circular de 50 Muestras de 50 Pasos (Circular) sin nada conectado aprox 10 ms.
    void lee() {
      for (byte pressStep = 0; pressStep < 50; pressStep++)
      {
        rawRead[pressStep] = analogRead(pin_sensor_);
        rawP = 0;
        for (byte x = 0; x < 50; x++) rawP += rawRead[x];
        raw = rawP / 50;
        calc1 = raw - pressZero;
        calc2 = calc1 * pressMax;
        float calc3 = resolution - float(pressZero);
        if (raw <= pressZero)
        {
          pressPsi = -1 * (float(calc2) / float(calc3)) / 100;
        }
        else
        {
          pressPsi = (float(calc2) / float(calc3)) / 100; // Convertir valor a PSI correcto
        }
        pressCM = pressPsi * 70.30695782; // psi a cmH20
      }

      // Serial << "Lectura: " << raw << " PSI: " << pressPsi << " cmH20: " << pressCM << ". \n";
      actual_ = pressCM;
      pico_ = max(pico_, pressCM);
    }

    const float& flujo() {

      for (byte pressStep = 0; pressStep < 50; pressStep++)
      {
        rawRead[pressStep] = analogRead(pin_sensor_);
        rawP = 0;
        for (byte x = 0; x < 50; x++) rawP += rawRead[x];
        raw = rawP / 50;
        calc1 = raw - pressZero;
        calc2 = calc1 * pressMax;
        calc3 = resolution - float(pressZero);
        if (raw <= pressZero)
        {
          pressPsi = -1 * (float(calc2) / float(calc3)) / 100;
        }
        else
        {
          pressPsi = (float(calc2) / float(calc3)) / 100; // No convertir  valor a PSI correcto para formula de flujo /100
        }
        pressCM = pressPsi * 6.89476; // psi a kPA
      }


      velocity = sqrt((2 * abs(pressCM) * 1000) / 1.28); // * 1000 para PA y entre densidad del aire 1.28
      area = pow(0.005, 2) * 3.14159265359; // radio de circulo de sensor 0.005 area = 0.000078539815
      flujo_ = velocity * area * 100 * 60; // m3/s * 1000 = l/s * 60 = l/m
      if (pressCM < 0) // no dejar valores negativos
      {
        flujo_ = flujo_ * -1.0;
      }

      //Serial << "\nVel:" << _FLOAT(velocity,6) << "s Area:" << _FLOAT(area,6) << "cm3 " << _FLOAT(flujo_,6) << "\n";
      return flujo_;
    }


    float volumenApasos(const float& vol_ml) {
      return ((-AMBU.ay + sqrt(sq(AMBU.ay) - 4 * AMBU.ax * (AMBU.az - vol_ml))) / (2 * AMBU.ax)) / 8.7726; // formula para llegar a 850 con 140 grados segun nuestro diseno
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
      return calibrado_;
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
    float           pico_, plateau_, peep_, calibrado_, calibrado_p_;
    float           pressPsi, calc3, pressCM;
    short           rawRead[201], pressZeroP, raw;
    unsigned short  calc1;
    unsigned long   calc2, timer, rawP;
    
    // VALORES DEL SENSOR MPXV4006 0 A 6KPA 0.2 A 4.8V y AMBU
    const struct {float ax, ay, az;} AMBU{ 1.29083271e-03, 4.72985182e-01, -7.35403067e+01 }; // Rangos de AMBU bolsa
    short   pressZero   = 54.219;             // Arduino Off
    short   pressMax    = 1023.0;           // Arduino Fss
    float   resolution  = 600.52;         // Resolucion obtenida de 4.6 Full Scale / 766 mV
};

#endif
