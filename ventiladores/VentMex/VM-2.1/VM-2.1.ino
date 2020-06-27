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

   Marvin Nahmias - on UxD, systems, electronics and software, marvin@tomglobal.org
   Omer Sebastian Larranaga - on mechanotronics, electronics, 3d Design, sensors and first build tests

 *** Referenced State machine logic, constants and timing graphs taken from MIT https://e-vent.mit.edu/ and all their licencing apply accordingly.

   MIT License:

   Copyright (c) 2020 MIT

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

 *** Referenced Mechanical Original Design ONLY from Formon Ventcore-health https://www.ventcore.health/ and all their licencing apply accordingly
   @ https://www.ventcore.health/index.php/terms-of-use/

  MEXICO COFEPRIS MONITOREO Y PARAMS:

  -Volumen corriente o tidal
    2 mL a 2000 mL o mayor, para ventilador adulto-pediátrico-neonatal.
    Al menos 25 a 50 mL en el límite inferior y 2000 mL o mayor en el límite superior para el ventilador adulto-pediátrico.
    2 mL a 2000 mL para el ventilador neonatal.
  -Frecuencia respiratoria
    Rango de 0 a 150 respiraciones por minuto.
  -Presión positiva aérea continua (CPAP/PEEP)
    Rango de 0 a 20 cm H20.
  -Presión soporte (NO APLICA)
    Rango de 0 a 45 cm H20.
  -Fracción inspirada de oxígeno (FiO2) (Vendra del Oximetro)
    Entre 21 y 100%.
  -Tiempos inspiratorios y espiratorios
    Al menos 0.1 seg y 10 seg respectivamente.
  -Relación entre tiempo inspiratorio y el tiempo espiratorio
    De 1:1 a 1:3
  -Flujo inspiratorio
    Rango de 0 a 150 L/min.
  -Presión inspiratoria
    Rango de 0 a 80 cm de H20.

  MEXICO COFEPRIS ALARMAS:

  Los siguientes parámetros deben monitorearse continuamente para verificar el desempeño
  correcto del equipo y contar con alarmas audibles y visibles ambas priorizadas en tres niveles:
  ● Presión inspiratoria alta. (1 ALTA)
  ● PEEP bajo o desconexión del paciente. (2 BAJA)
  ● Apnea.
  ● Volumen minuto o corriente alto y bajo.(2 BAJA)
  ● Frecuencia respiratoria alta y baja. (2 BAJA)
  ● FiO2 alta y baja. (2 BAJO)
  ● Baja presión del suministro de gases. (1 ALTA)
  ● Batería baja. (2 BAJA)
  ● Falta de alimentación eléctrica.(1 ALTA duh apagado?)
  ● Ventilador inoperante o falla del ventilador o indicador de no usar el aparato. (1 ALTA detectar)
  ● Silencio de alarma. (BOTON DE CLEAR)

*/

// Define includes and libraries
#include <U8x8lib.h>            // u8x8 OLED Library
#include <U8g2lib.h>            // u8g2 OLED Library
#include <AccelStepper.h>       // Basic AccelStepper
#include <Streaming.h>          // Streaming Pipe Library for Serial and others
#include <Arduino.h>
// VentMex Includes
#include "VMConstantes.h"        // VentMex Constants and basic functions
#include "VMPresion.h"           // VentMex Pressure Object with 200x200x10 Circular Calibration Filter. VM Test Calibration Eq.
#include "VMFlujo.h"             // VentMex Pressure Object with 200x200x10 Circular Calibration Filter. VM Test Calibration Eq.
// External Libraries


//#define DEBUG                  // if not defined then PRODUCTION
#define PROD                     // if not defined DEBUG

// VentMex Sensor Variables
int   alarm, tipo_alarma;
float TDp, RATEp, IEp, FiO2 = 0.0;
float volume, pressure, flow, Pip, Peep, Plateau = 0.0;
int tdSteps, inRPS, esRPS = 0;
int expiracion = 0;

boolean pacienteRespiro = false;  // Bandera si respira el paciente

int AC = 1;  // Assisted Control 1 = Volume Assist SIMV , 2 = Pressure Assist AC Pressure Sensitivity
float curr_steps = 0;
float ac_value = 2.0;
int Pagina = 1;                   // Pagina 1 es todos los sensores, 2 Volumen, 3 Flujo, etc.

// Cycle parameters
float tCycleTimer;            // Absolute time (s) at start of each breathing cycle
float tIn;                    // Calculated time (s) since tCycleTimer for end of IN_STATE
float tHoldIn;                // Calculated time (s) since tCycleTimer for end of HOLD_IN_STATE
float tEx;                    // Calculated time (s) since tCycleTimer for end of EX_STATE
float tPeriod;                // Calculated time (s) since tCycleTimer for end of cycle
float tPeriodActual;          // Actual time (s) since tCycleTimer at end of cycle (for logging)
float tLoopTimer;             // Absolute time (s) at start of each control loop iteration
float tLoopBuffer;            // Amount of time (s) left at end of each loop
float tins, tesp, tNow;       // Tiempos totales de Inspiracion y Expiracion y tiempo ahora

// Graphs
int p_graph [65] = {};
int p_max = 40;               // Presion Inspiratoria de -20 a 40 cmH2O
int p_min = -20;
char data[100];
int y_offset = 42;
int x_offset = 65;
int info_y = 32;

// External Object Setup
U8G2_SSD1306_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //OLED Display I2C 0.96 no Pins because we catch user intreface messages
VMPresion presion(PRESSURE);
VMFlujo flujo(FLOW);
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR); // Steps (full), Dir, Step, Enable PINS

// Este es el Ciclo de Cálculo de Tiempos y todo con Sensores
void calculaRespiracion() {

  tPeriod = 60.0 / RATEp;
  tHoldIn = tPeriod / (1 + IEp);
  tIn = tHoldIn - HOLD_IN_DURATION;
  tEx = tPeriod - MIN_PEEP_PAUSE;

  //Calculamos Tiempos totales sin holds
  tins = tHoldIn;
  tesp = (tPeriod - tins);

  // Aqui calculamos tdSteps con algoritmo de AMBU (a calibrar)
  tdSteps = (int) flujo.volumenApasos(TDp);

  // Calc Step RPS para cada ciclo de Inspiracion y Expiracion
  inRPS = (tdSteps * 32.0) / tIn; // son RPS
  esRPS = (tdSteps * 32.0) / (tEx - tIn); // son RPS

  // Serial << "tPeriod: " << tPeriod << " tIn - tHoldIn - tEx - tEx + PeePPause= " << tIn << "-" << tHoldIn << "-" << tEx << "-" << (tEx + MIN_PEEP_PAUSE) << "\n";
  // Serial << "tins: " << tins << " tesp: " << tesp << " = " << (tins + tesp) << "\n" ;
  // Serial << "inRPS: " << inRPS << " esRPS: " << esRPS << " tdSteps:" << tdSteps*32 << "\n" ;

}

// Este es el Corazon de VentMex, el Cálculo de Ciclo de la Máquina de Estados con Asistencia Controlada VC y PC
void cicloVentMex()
{
  presion.borra_pico();
  presion.lee();
  pressure = presion.get();

  expiracion = 1; // Icono Inspiracion
  tCycleTimer = now();

  stepper.setCurrentPosition(0);

  stepper.setMaxSpeed(-inRPS);
  stepper.setSpeed(-inRPS);

  sensorPantalla();

  while (true)
  {
    stepper.runSpeed();
    //if ((now() - tCycleTimer) >= tIn) // Llegue a tIn
    if (stepper.currentPosition() <= (-tdSteps * MICROSTEPS))
    {
      presion.lee();
      Pip = presion.pico();
      pressure = presion.get();
      flow = flujo.flujo(FiO2); // Aqui va el fio2 seleccionado
      break;
    }
  }
  delay(HOLD_IN_DURATION);
  presion.lee();
  presion.pon_plateau(); // Pon Plateau (siempre como se lee presiones al principio de ciclo estamos), guarda
  flow = flujo.flujo(FiO2); // Aqui va el fio2 seleccionado
  Plateau = round(presion.plateau());

  expiracion = 0; // Icono Espiracion
  stepper.setMaxSpeed(esRPS);
  stepper.setSpeed(esRPS);

  sensorPantalla();

  while (true)
  {
    stepper.runSpeed();
    //if ((now() - tCycleTimer) >= tEx) // Llegue a Ex final
    if (stepper.currentPosition() == 0)
    { // Llegamos a tEx + MIN_PEEP_WAIT lo minimo para medir Peep
      presion.lee();
      presion.pon_peep(); // Pon Peep (siempre como se lee presiones al principio de ciclo estamos)
      Peep = round(presion.peep());
      flow = flujo.flujo(FiO2);
      break;
    }
  }
  delay(MIN_PEEP_PAUSE);
  presion.lee();
  pressure = presion.get();
  Pip = round(presion.pico());
  Plateau = round(presion.plateau());
  Peep = round(presion.peep());
  presion.borra_pico(); // pPIP se forza
  flow = flujo.flujo(FiO2);
  //Serial << presion.get() << "\n";
  //Test (delay calc 80 ms)
  //float tR = (now() - tCycleTimer) - 0.08;
  //Serial << "Tiempo Real/Calc: " << tR << "/" << tPeriod << " (" << (tR - tPeriod) << ") " << ((tR * 100 / tPeriod) - 100) << "%\n";

  /**
      pacienteRespiro = false;
      if (AC == 2)
      {
        if (pacienteRespiro)
        {
          // Si respiro
          Pip = round(presion.pico());
          Plateau = round(presion.plateau());
          Peep = round(presion.peep());
          presion.borra_pico(); // aqui no hay mas picos, ahora si poner variable
          //volume = presion.pasosAvolumen(curr_steps);
          break; // salte empieza ciclo, si respiro
        }
        if (!pacienteRespiro)
        {
          // No Respiro
          presion.pon_peep();  // no hubo respiracion pon peep otra vez
          // Poner alarma de no respiro
          //volume = presion.pasosAvolumen(curr_steps);
        }
      }
  **/

}

void homeMotor()
{
  // Serial << "Homing ...\n";
  stepper.setMaxSpeed(3200);        // 1/2 velocidad para homing
  stepper.setAcceleration(42666);   // Aceleracion rapida siempre
  stepper.moveTo(64000);            // 100 vueltas aprox max encontrar switch

  while (true)
  {
    stepper.run();
    if (digitalRead(ENDSTOP) == LOW)
    {
      //Serial << "Homed ...\n";
      stepper.setCurrentPosition(0);
      stepper.stop();
      break;
    }
    if (stepper.distanceToGo() == 0)
    {
      // Serial << "Not Homed, Error! ...\n";
      stepper.stop();
    }
  }
}

// Funcion de Lectura de Controles de Usuario y calcula periodos del motor
void leeControles()
{
  IEp   = map(analogRead(IE), 5, 1020, IE_MIN, IE_MAX);     // IE de 1:1 a 1:4 por pacientes de Covid-19
  RATEp = map(analogRead(RATE), 5, 1020, BPM_MIN, BPM_MAX); // 10 a 35 BPM vs 100 to 300
  TDp   = map(analogRead(TD), 5, 1020, VOL_MIN, VOL_MAX);   // 10 - 900 ml
  // Overrride Tests
  //TDp = 850.00;
  //RATEp = 15.00;
  //IEp = 1.00;
}

// Alarms Thread
void checaAlarmas()
{
  // Checa Alarmas
  //if (Pip >= 80) {
  //  tipo_alarma = 1;   // Pip Alta > 80ml
  //  alarma_status = ALARMA_RAPID;
  //}
  //if (Peep <= 4) { tipo_alarma = 2; alarma_status = ALARMA_LENTA; } // Peep Baja < 4ml desconectado o muy bajo
  // Apnea to do
  //if (fip <= 0.050 || fip >= 2) { tipo_alarma = 4; alarma_status = ALARMA_RAPID; } // Flujo Inspiratorio < 50 o > 2000 mL
  // if (bpm <= 5 || bpm >= 150) { tipo_alarma = 5; alarma_status = ALARMA_RAPID; } // BPM: Frequencia Respiratoria Alta 0 a 150 por minuto
  // Baja presion de gases To Do
  // Bateria Baja y AlimentacionTo Do
  // Ventilador Inoperante To Do Watch dog del VentMex

  // Display Alarmas
  switch (tipo_alarma)
  {
    case 1:
      {
        Alarmas(" ALARMA: 1-ALTA", " Presión Inspiratoria ", "  es alta > 80cmH2O ");
        break;
      }
    case 2:
      {
        Alarmas(" ALARMA: 2-BAJA", " PEEP Bajo ó paciente ", "     desconectado");
        break;
      }
    case 3:
      {
        Alarmas(" ALARMA: 3-ALTA", " Apnea ", "  Apnea ");
        break;
      }
    case 4:
      {
        Alarmas(" ALARMA: 4-BAJA", " Volumen * minuto o ", " corriente bajo/alto ");
        break;
      }
    case 5:
      {
        Alarmas(" ALARMA: 5-ALTA", " Frecuencia Resp. ", "  alta (>150) o baja (5) ");
        break;
      }
    case 6:
      {
        Alarmas(" ALARMA: 6-ALTA", " FiO2 medido ", " alta y/o baja ");
        break;
      }
    case 7:
      {
        Alarmas(" ALARMA: 7-ALTA", " Baja presión del  ", " suministro de gases ");
        break;
      }
    case 8:
      {
        Alarmas(" ALARMA: 8-ALTA", " Ventilador Inoperante ", " Falla de Ventilador ");
        break;
      }
  }

  if (alarma_status == ALARMA_OFF)
    return; //nada que sonar, no hay alarmas

  if (alarma_status == ALARMA_RAPID) {
    //pantalla.setFont(u8g2_font_open_iconic_human_1x_t);
    //pantalla.setCursor(90, 9); // RIGHT
    //pantalla.print("C"); // Alarm from Font @2x
    //pantalla.sendBuffer();
    tone(BUZZ, 1000);
    tomDelay(100);
    //pantalla.setCursor(90, 9); // RIGHT
    //pantalla.print(" ");
    //pantalla.sendBuffer();
    noTone(BUZZ);
    tomDelay(200);
  }

  if (alarma_status == ALARMA_LENTA) {
    tone(BUZZ, 1000);
    tomDelay(300);
    noTone(BUZZ);
    tomDelay(600);
  }

  if ((digitalRead(BUTTON_SELECT) == 0) && (alarma_status != ALARMA_OFF)) // READ ALARM CLEAR BUTTON
  {
    alarma_status = ALARMA_OFF; // Alarm Sound
    tipo_alarma = 0; // Alarm Display
    beep(); beep(); beep(); // 3 BEEPS CLEAR
  }
}

// Logica Pantalla : Lee Joystick, Reset de Menu si hay boton, lleva control de pantalla para pintar
void logicaPantalla()
{
  int joy = analogRead(JOYSTICK);
  int but = digitalRead(BUTTON_SELECT);

  if (joy <= 480)
  {
    Pagina++;
    if (Pagina == 6) Pagina = 1;
    beep();
  }

  if (joy >= 560)
  {
    Pagina--;
    if (Pagina == 0) Pagina = 5;
    boop();
  }

  if (Pagina == 5 && but == LOW)
  {
    pantalla.clearBuffer();
    pantalla.drawXBMP(0, 15, logo_width, logo_height, logo);
    pantalla.sendBuffer();
    sing(1);
    sing(2);
  } // TOM Easter Egg


  switch (Pagina)
  {
    case 1:
      {
        todoPantalla();
        break;
      }
    case 2:
      {
        presionGrafica();
        break;
      }
    case 3:
      {
        presionPantalla();
        break;
      }
    case 4:
      {
        flujoPantalla();
        break;
      }
    case 5:
      {
        InfoPantalla();
        break;
      }
  }
}

// Esta Pantalla es el Titulo de todas las pantallas
void sensorPantalla()
{
  char buff[6];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 0, 128, 16);
  pantalla.setDrawColor(1);

  pantalla.setFont(u8g2_font_8x13B_mr);
  pantalla.setCursor(-1, 13);
  pantalla.print("1:");
  pantalla.print(dtostrf(IEp, 1, 0, buff));
  pantalla.drawLine(26, 0, 26, 16); // Vertical Line
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.setCursor(30, 13);
  pantalla.print(dtostrf(tIn, 1, 1, buff));
  pantalla.print("/");
  pantalla.print(dtostrf((tEx - tIn), 1, 1, buff));
  pantalla.print("s");
  pantalla.setFont(u8g2_font_t0_13_te);
  pantalla.setCursor(88, 13);
  pantalla.print(dtostrf(RATEp, 3, 0, buff)); // Ritmo Respiratorio
  pantalla.drawLine(0, 16, 128, 16); // Horizontal Line

  if (expiracion == 1) //  exhala
  {
    // pantalla.setFont(u8g2_font_open_iconic_human_2x_t);
    pantalla.setFont(u8g2_font_cursor_tf);
    pantalla.setCursor(120, 8);
    pantalla.print(char(57)); // Pulmon @1n
  } else {
    // pantalla.setFont(u8g2_font_open_iconic_human_1x_t);
    pantalla.setFont(u8g2_font_cursor_tf);
    pantalla.setCursor(120, 8); // RIGHT
    pantalla.print(char(56)); // Pulmon @2b
  }
  pantalla.setFont(u8g2_font_6x12_tf); // tf tr tn te

  if (Pagina != 5) pantalla.sendBuffer(); // Todo menos en la Info escribe el Header

  p_graph[64] = (pressure) * (y_offset - 20) / p_max;

  for (int x = 0; x < 64; x++)
  {
    p_graph[x] = p_graph[x + 1];
  }

}

// Pagina 1 : Todos los Sensores y Valores
void todoPantalla() {

  char buff[6];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 17, 128, 64);
  pantalla.setDrawColor(1);

  pantalla.drawLine(62, 16, 62, 64); // Vertical Line
  pantalla.drawLine(97, 16, 97, 48); // Vertical Line
  pantalla.drawLine(0, 48, 128, 48); // Horizontal Line
  int y = 30;
  pantalla.setFont(u8g2_font_6x12_tf); // tf tr tn te
  pantalla.setCursor(0, y);
  pantalla.print(" Flujo/Vt  ");
  pantalla.print(dtostrf(flow, 3, 0, buff));
  pantalla.print("   ");
  pantalla.print(dtostrf(TDp, 3, 0, buff));
  pantalla.setCursor(0, y + 13);
  pantalla.print(" SpO²/FiO²  ");
  pantalla.print(dtostrf(21, 3, 0 , buff)); //SpO2
  pantalla.print("   ");
  pantalla.print(dtostrf(21, 3, 0 , buff)); // FiO2
  pantalla.setCursor(0, y + 30);
  pantalla.print(" P:Pep/PIn ");
  pantalla.print(dtostrf(presion.get(), 3, 0 , buff));
  pantalla.print(":");
  //pantalla.setCursor(75, y + 30);
  pantalla.print(dtostrf(Peep, 2, 0, buff));
  pantalla.print("/");
  pantalla.print(dtostrf(Pip, 2, 0 , buff));
  pantalla.sendBuffer();
}

// Pagina 2: Presiones Valores Graficas
void presionGrafica() {

  char buff[8];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 17, 128, 64);
  pantalla.setDrawColor(1);

  pantalla.drawBox(100, 51, 125, 64);
  pantalla.setCursor(100, 63);
  pantalla.setDrawColor(0);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(pressure, 3, 0, buff));
  pantalla.setDrawColor(1);

  pantalla.setCursor(0, 32);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("Ip");
  pantalla.setCursor(30, 32);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Pip, 3, 0, buff));
  pantalla.setCursor(0, 47);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("lat");
  pantalla.setCursor(30, 47);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Plateau, 3, 0, buff));
  pantalla.setCursor(0, 62);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("eep");
  pantalla.setCursor(30, 62);
  pantalla.setFont(u8g2_font_fub11_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Peep, 3, 0, buff));

  // Aqui pintamos la matriz de presiones
  for (int x = 0; x < 64; x++) // shift graph en el tiempo
  {
    pantalla.drawLine((x - 1 + x_offset), (y_offset - p_graph[x + 0]), (x + x_offset), (y_offset - p_graph[x + 1]));
  }
  pantalla.sendBuffer();
}

// Pagina 3: Presiones Valores Discretos
void presionPantalla() {

  char buff[8];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 17, 128, 64);
  pantalla.setDrawColor(1);
  int y = 36;
  // Paw PiP
  pantalla.setCursor(0, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("aw");
  pantalla.setCursor(26, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(pressure, 3, 0, buff));
  pantalla.setCursor(64, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("Ip");
  pantalla.setCursor(90, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Pip, 3, 0, buff));
  // Pplat PeeP
  y = y + 17;
  pantalla.setCursor(0, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("lt");
  pantalla.setCursor(26, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Plateau, 3, 0, buff));
  pantalla.setCursor(64, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print("P");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("ep");
  pantalla.setCursor(90, y);
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(Peep, 3, 0, buff));
  y = y + 10;
  pantalla.setCursor(50, y);
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("cmH");
  pantalla.setFont(u8g2_font_5x7_mn);
  pantalla.print("2");
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print("O");
}

// Pagina 4: Volumen Tidal y Flujo Medido
void flujoPantalla() {

  char buff[8];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 17, 128, 64);
  pantalla.setDrawColor(1);
  int y = 38;
  pantalla.setFont(u8g2_font_fub14_tr); // 20, 17 y 14 tamanos
  pantalla.setCursor(2, y);
  pantalla.print("VolT: ");
  pantalla.setCursor(70, y);
  pantalla.setFont(u8g2_font_fub17_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(TDp, 3, 0, buff));
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print(" ml");
  pantalla.setCursor(2, y + 26);
  //pantalla.drawLine(0, 48, 128, 48); // Horizontal Line
  pantalla.setFont(u8g2_font_fub14_tr);
  pantalla.print("Flow: ");
  pantalla.setCursor(70, y + 26);
  pantalla.setFont(u8g2_font_fub17_tr); // 20, 17 y 14 tamanos
  pantalla.print(dtostrf(flow, 3, 0, buff));
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.print(" l/m");

}

// Pagina 5: Info Pantalla
void InfoPantalla()
{

  pantalla.clearBuffer();                   // clear the internal memory
  pantalla.setFont(u8g2_font_helvR10_tf);
  pantalla.setFontMode(0);
  pantalla.enableUTF8Print();
  pantalla.drawStr(17, (info_y + 12), "TOM VentMex");
  pantalla.drawXBMP(0, (info_y + 15), logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
  pantalla.setFont(u8g2_font_t0_13_te);
  pantalla.drawStr(8, (info_y + 70), "makersmexico.org");
  pantalla.drawStr(17, (info_y + 100), "Marvin Nahmias");
  pantalla.drawStr(-3, (info_y + 120), "Sebastian Larranaga");
  pantalla.drawStr(33, (info_y + 140), "David Rico");
  pantalla.drawStr(17, (info_y + 160), "Manuel Victoria");
  pantalla.drawStr(5, (info_y + 180), "Claudia Dorembaum");
  pantalla.drawStr(15, (info_y + 200), "Copyright 2020");
  pantalla.sendBuffer();

  //Scrollit
  if (info_y >= -220)
  {
    info_y = info_y - 10;
  }
  else
  {
    info_y = 220;
  }
}

// Pagina Alarmas: Display de Alarmas
void Alarmas(String msg1, String msg2, String msg3)
{
  pantalla.setDrawColor(1);
  pantalla.drawBox(0, 0, 128, 64);
  pantalla.setFont(u8g2_font_8x13B_mr);
  pantalla.setCursor(0, 13);
  pantalla.setDrawColor(0);
  pantalla.print(msg1);
  pantalla.setFont(u8g2_font_6x12_tf);
  pantalla.setCursor(0, 30);
  pantalla.print(msg2);
  pantalla.setCursor(0, 40);
  pantalla.print(msg3);
  pantalla.setFont(u8g2_font_8x13B_mr);
  pantalla.setCursor(5, 63);
  pantalla.drawLine(0, 50, 128, 50); // Horizontal Line
  pantalla.drawLine(0, 49, 128, 49); // Horizontal Line
  pantalla.print(" A-T-E-N-D-E-R ");
  pantalla.setDrawColor(1);
  pantalla.sendBuffer();
  tomDelay(100);
}

// Dibuja Logo
void drawLogo() {

  int i = 0, y = 1;
  pantalla.firstPage();
  do
  {
    pantalla.setFont(u8g2_font_helvR10_tf);
    pantalla.setFontMode(0);
    pantalla.drawStr(17, 12, "TOM VentMex");
    pantalla.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
    pantalla.setFont(u8g2_font_t0_13_te);
    pantalla.drawStr(8, 60, "makersmexico.org");
    pantalla.setFont(u8g2_font_helvR10_tf);
  } while ( pantalla.nextPage() );

  tomDelay(500);

  pantalla.firstPage();
  do {
    //pantalla.setFontMode(0);
    pantalla.setFont(u8g2_font_helvR10_tf);
    pantalla.setFontMode(0);
    pantalla.drawStr(17, 12, "TOM VentMex");
    pantalla.setDrawColor(0);
    pantalla.drawXBMP(50, 15, plus_width, plus_height, plus); //X, Y, Size_X, Size_Y, data
    pantalla.setDrawColor(1);
    pantalla.drawStr(0, 60, ".>> Inicio Motor <<.");
    pantalla.setFont(u8g2_font_helvR10_tf);
  } while ( pantalla.nextPage() );
  //HOME
  homeMotor();

  pantalla.firstPage();
  do {
    //pantalla.setFontMode(0);
    pantalla.setFont(u8g2_font_helvR10_tf);
    pantalla.setFontMode(0);
    pantalla.drawStr(17, 12, "TOM VentMex");
    pantalla.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
    pantalla.drawStr(0, 60, ">Calibra Sensores<");
    pantalla.setFont(u8g2_font_helvR10_tf);
  } while ( pantalla.nextPage() );
  // PRESION
  presion.calibrar(); // Calibra Presion

  for (i; i < 50; i = i + 10) // Animate Loading...
  {

    pantalla.setDrawColor(1);
    //pantalla.drawBox(0, 1, i, 11);
    if (y == 1)
    {
      pantalla.setDrawColor(0);
      pantalla.drawXBMP(50, 15, plus_width, plus_height, plus); //X, Y, Size_X, Size_Y, data
      pantalla.setDrawColor(1);
    }
    if (y == 2)
    {
      y = 0;
      pantalla.setDrawColor(1);
      pantalla.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data

    }
    pantalla.sendBuffer();
    tomDelay(100);
    y++;
  }
  // FLUJO
  flujo.calibrar();   // Calibra Flujo

  // BEEP BEEP : LISTO!
  beep(); beep();
}

void setupTVM()
{
  beep();
  AC = 1;     // Default SIMV
  FiO2 = 21;  // Default aire normal
  alarma_status = ALARMA_OFF; // No hay alarmas - Clear All
  tipo_alarma = 0; // No hay tipo de alarma

  pantalla.setFont(u8g2_font_helvR10_tf);

  // Selecciona MODO VENTILADOR
  AC = pantalla.userInterfaceSelectionList("Modo de Control :", 1, "S.I.M.V.\nControl Asistido");
  beep();

  if (AC == 2)
  {
    // Selecciona Sencibilidad
    int acs = pantalla.userInterfaceSelectionList("Sensitividad:", 5, "0.2 cmH²O\n0.4 cmH²O\n0.6 cmH²O\n0.8 cmH²O\n1.0 cmH²O\n1.2 cmH²O\n1.4 cmH²O\n1.6 cmH²O\n1.8 cmH²O\n2.0 cmH²O");
    switch (acs)
    {
      case 0: ac_value = 0.2; break;
      case 1: ac_value = 0.4; break;
      case 2: ac_value = 0.6; break;
      case 3: ac_value = 0.8; break;
      case 4: ac_value = 1.0; break;
      case 5: ac_value = 1.2; break;
      case 6: ac_value = 1.4; break;
      case 7: ac_value = 1.6; break;
      case 8: ac_value = 1.8; break;
      case 9: ac_value = 2.0; break;
    }
    beep();
  }

  // Selecciona FiO2
  int fio = pantalla.userInterfaceSelectionList("Selección L/m:", 1, "0 - 21%\n1 - 24%\n2 - 28%\n3 - 32%\n4 - 36%\n5 - 40%\n6 - 44%\n7 - 48%\n8 - 52%\n9 - 56%\nX - 100%");

  switch (fio)
  {
    case 0: FiO2 = 21; break;
    case 1: FiO2 = 24; break;
    case 2: FiO2 = 28; break;
    case 3: FiO2 = 32; break;
    case 4: FiO2 = 36; break;
    case 5: FiO2 = 40; break;
    case 6: FiO2 = 44; break;
    case 7: FiO2 = 48; break;
    case 8: FiO2 = 52; break;
    case 9: FiO2 = 56; break;
    case 10: FiO2 = 100; break;
  }
  // BEEP Selected: FiO2 Entry
  boop();
}

// TOM Beep
void beep()
{
  tone(BUZZ, 700);
  tomDelay(100);
  noTone(BUZZ);
  tomDelay(50);
}

// TOM Boop
void boop()
{
  tone(BUZZ, 180);
  tomDelay(100);
  noTone(BUZZ);
  tomDelay(50);
}
void setup() {

  Serial.begin(2000000);
  pantalla.begin(BUTTON_SELECT, BUTTON_MOVE, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE); // Startup Logo Screen (select, next, previous, up, down, home) lo cachamos con funcipon de GetMenu
  pantalla.enableUTF8Print(); // Enable UTF8 Printing

  // Inicilaiza Pins
  pinMode(TD, INPUT);
  pinMode(IE, INPUT);
  pinMode(RATE, INPUT);
  pinMode(JOYSTICK, INPUT);
  pinMode(FLOW, INPUT);
  pinMode(ENDSTOP, INPUT);
  pinMode(PRESSURE, INPUT);

  pinMode(BUZZ , OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(ENABLE, OUTPUT);


  // DEBUG or Production
#ifdef DEBUG
  Serial << "\nTOM VentMex : Initializando\n---------------------------\nModel: " << MODEL << " Firmware: " << VERSION << "\n";
  Serial << "Initializing pins, leds, beeper and threads.\nVT,RR,I/E-1,PP,PI,TI,TE\n");
#else
  Serial << "\nModel: " << MODEL << " Firmware: " << VERSION << "\n";
#endif

  drawLogo(); // Draw Logo and reset variables, sensors and motor
  setupTVM(); // Setup de menu principal

#ifdef DEBUG
  Serial << "Habilita Motor...\n";
#endif
  Pagina = 1; // Empieza con Pagina en
}

void loop() {
  logicaPantalla();
  leeControles();
  calculaRespiracion();
  cicloVentMex();
  if (digitalRead(ENDSTOP) == LOW) setupTVM();
}
