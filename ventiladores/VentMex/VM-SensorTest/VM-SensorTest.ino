
#include "VMPresiones.h"
#include <Streaming.h>

VMPresiones presion(A0); // Objeto de Presion que calcula flujo

long now;
int x;
void setup()
{

  Serial.begin(2000000);
  Serial << "\n\n\n\n\n\n\n\nTOM - VentMex comenzando la calibracion ...\n";
  now = millis();
  presion.calibrar();
  Serial << "Calibramos el CERO en " << presion.calibrado() << " digital | " << presion.calibradop() << " cmH20 en " << (millis() - now) << " ms. \n";
  //presion.borra_pico();
  delay(1000);
  x = 0;
}

void loop()
{

  now = millis();
  presion.lee(); // Leer para cada ciclo y se ajustan las presiones (PeeP, Plateau y PIp)
  Serial << "Lectura Actual (cmH20): " << presion.get()
         << " Flujo: " << _FLOAT(presion.flujo(),2) << " l/m" 
         << " | Pico: " << presion.pico()
         << " | Peep: " << presion.peep()
         << " | Plateau: " << presion.plateau()
         << " en " << (millis() - now) << "ms. \n";
  x++;
  if (x == 5) presion.pon_plateau(); // Pon Plateau demo
  if (x == 10) presion.pon_peep(); // Pon Peep demo
  if (x > 20) // Borra Pico Presion cada 5 lecturas
  {
    x = 0;
    presion.borra_pico();
    Serial << "...Pico RESET...\n"; 
  } 
  float tdSteps = presion.volumenApasos(850);
  Serial << "Prueba solo el calculo AMBU: 850ml = " <<  tdSteps << " volume2steps " << round(tdSteps * 64) << "\n";
 
  delay(500);
}
