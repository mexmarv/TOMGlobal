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
#include <BasicStepperDriver.h> // Basic Stepper Library with functions for this design V1.1.3
#include <Streaming.h>          // Streaming Pipe Library for Serial and others
#include <Arduino.h>
//#include <Math.h>

#include "VMConstantes.h"        // VentMex Constants and basic functions
#include "VMPresiones.h"        // VentMex Pressure Object with 200x200x10 Circular Calibration Filter
//#include "i2c_BMP280.h"         // BMP280 Altitude, Temp and External Pressure

//#define DEBUG               // if not defined then PRODUCTION
#define PROD                  // if not defined DEBUG

// Stepper Motor Defines
// Motor steps per revolution. e 200 steps or 1.8 degrees/step
#define MOTOR_STEPS       200
#define MICROSTEPS        32
#define RPM               120
#define FULL_TD_STEPS     78  // Usado para nuestro diseño de 38mm banda, 50mm piñon y 8.45cm2 de mano

// VentMex Sensor Variables
int alarm, tipo_alarma;
float TDp, RATEp, IEp, FiO2 = 0.0;
int   tdSteps, inRPM, exRPM = 0;
float pressure, flow, Pip, Peep, Plateau = 0.0;
boolean pacienteRespiro = false;  // Bandera si respira el paciente
int acs, AC = 1, expiracion = 0;       // Assisted Control 1 = Volume Assist , 2 = Pressure Assist
int fio;                          // de valvulas venturi o presion valvula manual en L para porcentaje FiO2
float ac_value = 2.0;
int Pagina = 1;                   // Pagina 1 es todos los sensores, 2 Volumen, 3 Flujo, etc.

// BME280 Sensor Variables
//float temperature;
//float pascal;
//static float meters;

// Cycle parameters
float tCycleTimer;            // Absolute time (s) at start of each breathing cycle
float tIn;                    // Calculated time (s) since tCycleTimer for end of IN_STATE
float tHoldIn;                // Calculated time (s) since tCycleTimer for end of HOLD_IN_STATE
float tEx;                    // Calculated time (s) since tCycleTimer for end of EX_STATE
float tPeriod;                // Calculated time (s) since tCycleTimer for end of cycle
float tPeriodActual;          // Actual time (s) since tCycleTimer at end of cycle (for logging)
float tLoopTimer;             // Absolute time (s) at start of each control loop iteration
float tLoopBuffer;            // Amount of time (s) left at end of each loop
float tins, texp, tNow;       // Tiempos totales de Inspiracion y Expiracion y tiempo ahora
unsigned long cycleCount = 0;
unsigned wait_time_micros;

// Graphs
int v_graph [150] = {};
int v_max = 800; // Volumen Tidal 0-2000 ml, no mas de 800! 
int v_min = 0;
int f_graph [150] = {};
int f_max = 40; // Flujo Inspiratorio de 0 a 150 L/m
int f_min = -20;
int p_graph [150] = {};
int p1_graph [150] = {};
int p_max = 90; // Presion Inspiratoria de 0 a 80 cmH2O 
int p_min = -50;
char data[100];
int y_offset = 62;   // bottom axis
int y0_offset = 38;  // middle axis 16 + 44 total
int y1_offset = 48;  // middle axis 16 + more to be less negative
int x_offset = 12;
int info_y = 0;

// External Object Setup
//BMP280 bmp280;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //OLED Display I2C 0.96 no Pins because we catch user intreface messages
VMPresiones presion(PRESSURE);
VMPresiones flujo(FLOW);
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE); // Steps (full), Dir, Step, Enable PINS

// MAQUINA DE ESTADOS AC-VC y AC-PC
void cicloVentMex()
{
  //
  // Ciclo de Insipiracion con Hold In para medir PIP y Plateau Presiones
  //
  //Serial << "Ciclo de Inspiracion total: " << tins << " tdSteps: " << tdSteps << " RPM Insp: " << inRPM << "\n";
  stepper.setRPM(inRPM); // Default la inspiracion total como velocidad
  stepper.startMove(-tdSteps * MICROSTEPS); // Avanzar los STEPS * Microsteps calculado
  tCycleTimer = now();
  expiracion = 0; // Icono Inspiracion
  while (true)
  {
    flow = flujo.flujo();
    pintaPantalla();
    checaAlarmas();
    wait_time_micros = stepper.nextAction(); // Esto avanza el Loop
    // Llegue a tIn para medir PIP
    if ((now() - tCycleTimer) >= tIn) { // AQUI PARAR SI LLEGAS A flow TAMBIEN
      stepper.stop(); // Para motor para que pares a medir plateau
      presion.lee();
      //Serial << "pPIP: " << presion.pico() << "\n";
    }
    // Llegue a tHoldIn para medir Plateau y terminar ciclo de Inspiracion
    if ((now() - tCycleTimer) >= tHoldIn) {
      presion.pon_plateau(); // Pon Plateau (siempre como se lee presiones al principio de ciclo estamos), guarda
      //Serial << "(tIn     )- pPIP: " << tIn << ":" << presion.pico() << "\n";
      //Serial << "(tHoldIn )- pPlateau: " << tHoldIn << ":" << presion.plateau() << "\n";
      //Serial << "Ciclo de Inspiracion (tHoldIn o tins): " << tins << " Termine en: " << (now() - tCycleTimer) << "\n\n";
      break;
    }
  }

  // Serial << "Ciclo de Expiracion : " << texp << " tdSteps: " << tdSteps << " RPM Expi: " << exRPM << "\n";
  stepper.setRPM(exRPM); // Default la expiracion como velocidad
  stepper.startMove(tdSteps * MICROSTEPS); // - Avanzar los STEPS * Microsteps calculado
  tCycleTimer = now();
  expiracion = 1; // Icono Expiracion
  while (true)
  {
    flow = flujo.flujo();
    pintaPantalla();
    checaAlarmas();
    wait_time_micros = stepper.nextAction(); // Esto avanza el Loop
    if ((now() - tCycleTimer) >= (tEx - tHoldIn)) { // Llegamos a tEx + MIN_PEEP_WAIT lo minimo para medir Peep
      stepper.stop();
      presion.pon_peep(); // Pon Peep (siempre como se lee presiones al principio de ciclo estamos)
      //Serial << "(tEx -tHoldIn) - pPeep: " << (tEx-tHoldIn) << ":" << presion.peep() << "\n";
    }

    if ((now() - tCycleTimer) >= texp )  // || digitalRead(ENDSTOP == LOW) // aqui espero a llegar a 0 o tocar home switch
    {
      Pip = round(presion.pico());
      Plateau = round(presion.plateau());
      Peep = round(presion.peep());
      presion.borra_pico(); // pPIP se forza
      //Serial << "(tEx -tHoldIn) - pPeep: " << (tEx - tHoldIn) << ":" << presion.peep() << "\n";
      //Serial << "Ciclo de Expiracion (tHoldIn - tPeriod): " << texp << " Termine en: " << (now() - tCycleTimer) << "\n";
      break; // SALTE
    }

    // Entre tEx y tPeriod revisamos si el paciente respira; con parametros AC ahorita 3 de ejemplo
    // pacienteRespiro = presion.lee() < (presion.peep() - 3) && parametroAC > AC_MIN; // pues sino falso hasta que sirva lector presion

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
        stepper.enable();
        break; // salte empieza ciclo, si respiro
      }
      if (!pacienteRespiro)
      {
        // No Respiro
        presion.pon_peep();  // no hubo respiracion pon peep otra vez
        // Poner alarma de no respiro
        stepper.enable();
      }
    }
  }
}

void calculaRespiracion() {

  tPeriod = 60.0 / RATEp;                   // segundos en cada ciclo de respiracion
  tHoldIn = tPeriod / (1 + IEp);
  tIn = tHoldIn - HOLD_IN_DURATION;
  tEx = min(tHoldIn + MAX_EX_DURATION, tPeriod - MIN_PEEP_PAUSE); // Considera MAX EX DURACION Y MIN PEEP PAUSA

  //Calculamos Tiempos
  tins = tHoldIn;
  texp = (tPeriod - tins);

  //Serial << "Ciclo Respiracion:" << " IE: " << IEp << " RATE: " << RATEp << " TDV: " << TDp << "\n";
  //Serial << "Ciclo Respiracion:" << " tPe: " << tPeriod << " tIn: " << tIn << " tHld: " << tHoldIn << " tEx: " << tEx << "\n";
  //Serial << "Ciclo Respiracion:" << " tins: " << tins << " texp: " << texp << " tEx-HoldIn: " << (tEx - tHoldIn) << "\n";

  // Aqui calculamos tdSteps
  //tdSteps = (int)((TDp / VOL_MAX) * FULL_TD_STEPS); // Vamos a asumir que 140 grados nos da todo con el rodillo, o 100 steps a 1000
  tdSteps = (int) presion.volumenApasos(TDp);
  //Serial << " TDSteps: " << tdSteps << "\n";

  // Calc Step RPM para cada ciclo de Inspiracion y Expiracion
  inRPM = (30 / (FULL_TD_STEPS / tdSteps) / tIn); // no usar 60 por segundo sino 30 porque es mitad de ciclo
  exRPM = (30 / (FULL_TD_STEPS / tdSteps) / (tEx - tHoldIn)); // no usar 60 por segundo sino 30 porque es mitad de ciclo

  //Serial << " RPM Ins: " << inRPM << "\n";
  //Serial << " RPM Exp: " << exRPM << "\n";
}

void inicializaMotor()
{
  stepper.begin(20, MICROSTEPS);
  stepper.enable();
  stepper.startMove(-50 * MOTOR_STEPS * MICROSTEPS);  // 50 vueltas o 5 segs aprox max encontrar switch

  while (true)
  {
    if (digitalRead(9) == LOW) {
      // Serial.println("\t... llegue a Home (0).");
      stepper.stop();
      // Serial.println("\t... regresando un poquito 2 STEPS = 64 microsteps.");
      stepper.move(-2 * MICROSTEPS); // regresa 2*Microsteps a 0
      stepper.disable(); // apaga por un momento
      break;
    }

    wait_time_micros = stepper.nextAction(); // Esto avanza el Loop

    // 0 wait time indicates the motor has stopped
    if (wait_time_micros <= 0) {
      stepper.stop();
      // Serial.println("\t... Error en Home Switch !!! .STOP.");
      break;
    }
  }
}

// Funcion de Lectura de Controles de Usuario y calcula periodos del motor
void leeControles()
{
  IEp   = map(analogRead(IE), 5, 1020, IE_MIN, IE_MAX);     // IE de 1:1 a 1:4 por pacientes de Covid-19
  RATEp = map(analogRead(RATE), 5, 1020, BPM_MIN, BPM_MAX); // 10 a 35 BPM vs 100 to 300
  TDp   = map(analogRead(TD), 5, 1020, VOL_MIN, VOL_MAX);   // 100 a 1000 cmH2O - vs 250 to 600
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
    //u8g2.setFont(u8g2_font_open_iconic_human_1x_t);
    //u8g2.setCursor(90, 9); // RIGHT
    //u8g2.print("C"); // Alarm from Font @2x
    //u8g2.sendBuffer();
    tone(BUZZ, 1000);
    tomDelay(100);
    //u8g2.setCursor(90, 9); // RIGHT
    //u8g2.print(" ");
    //u8g2.sendBuffer();
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

// Display Thread
void pintaPantalla() {  
  v_graph[120] = (TDp) * (y_offset - 20) / v_max; // ahora solo TDp constante pero calcular el rise y fall de esto
  f_graph[120] = (flow) * (y0_offset - 20) / f_max;
  p_graph[120] = (pressure) * (y1_offset - 20) / p_max;
  //p1_graph[120] = (diffpressure) * (y1_offset - 20) / p_max; // ver como calculamos presion negativa ?!?

  // Lee y llena matrices de graficas
  for (int x = 0; x < 120; x++) // shift graph in time
  {
    v_graph[x] = v_graph[x + 1];
    f_graph[x] = f_graph[x + 1];
    p_graph[x] = p_graph[x + 1];
    p1_graph[x] = p1_graph[x + 1];
  }

  int joy = analogRead(JOYSTICK);
  if (joy >= 560)
  {
    Pagina++;
    if (Pagina == 7) Pagina = 1;
    beep();
    tomDelay(100);
  }

  if (joy <= 480)
  {
    Pagina--;
    if (Pagina == 0) Pagina = 6;
    boop();
    tomDelay(100);
  }

  if (digitalRead(BUTTON_SELECT) == 0 && (joy <= 480 || joy >= 560)) {
    tomDelay(100);
    setupTVM();
  }

  switch (Pagina)
  {
    case 1:
      {
        drawSensors();
        break;
      }
    case 2:
      {
        VolumenGraph(TDp);
        break;
      }
    case 3:
      {
        FlowGraph(flujo.flujo());
        break;
      }
    case 4:
      {
        presion.lee(); // Lee Presiones
        pressure = presion.get();
        PressureGraph(pressure);
        break;
      }
    case 5:
      {
        ATMGraph();
        break;
      }
    case 6:
      {
        InfoGraph();
        break;
      }
  }

}

// Page 1 Dibuja todos los Sensores Discretos
void drawSensors() {

  String vtd_s, vt_s, iev_s, rte_s, prp_s, fip_s, ti_s, te_s, peep_s, fio2_s, spo2_s, pip_s, pre_s = "";
  float ti, te, fip, bpm = 0;
  char buff[24];

  ti = tins;
  te = texp;
  fip = fio; // L x min?

  vt_s += dtostrf(flow, 3, 0, buff); // no decimals Flujo medido
  vtd_s += dtostrf(TDp, 3, 0, buff); // no decimals Flujo set pot
  iev_s += dtostrf(IEp, 1, 0, buff);
  rte_s += dtostrf(RATEp, 3, 0, buff);

  pre_s  += dtostrf(presion.get(), 3, 0 , buff);
  peep_s += dtostrf(Peep, 2, 0, buff);
  pip_s  += dtostrf(Pip, 2, 0 , buff);

  fip_s += dtostrf(fip, 4, 1, buff);
  ti_s += dtostrf(ti, 2, 1, buff);
  te_s += dtostrf(te, 2, 1, buff);
  fio2_s  += dtostrf(FiO2, 3, 0 , buff);


  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13B_mr);
    u8g2.setCursor(-1, 11);
    u8g2.print("1:" + iev_s);
    u8g2.drawLine(26, 0, 26, 16); // Vertical Line
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(30, 11);
    u8g2.print(ti_s + "/" + te_s + "s");
    u8g2.setFont(u8g2_font_t0_13_te);
    //u8g2.setFont(u8g_font_5x7);
    //u8g2.setCursor(93, 6);
    //u8g2.print("RR"); // BPM
    u8g2.setCursor(90, 11);
    u8g2.print(rte_s); // Ritmo Respiratorio
    u8g2.drawLine(0, 16, 128, 16); // Horizontal Line
    u8g2.drawLine(62, 16, 62, 64); // Vertical Line
    u8g2.drawLine(97, 16, 97, 48); // Vertical Line
    u8g2.drawLine(0, 48, 128, 48); // Horizontal Line
    u8g2.setFont(u8g2_font_6x12_tf); // tf tr tn te
    int y = 30;
    u8g2.setCursor(0, y);
    u8g2.print(" Flujo/Vt  ");
    u8g2.print(vt_s);
    u8g2.print("   ");
    u8g2.print(vtd_s);
    u8g2.setCursor(0, y + 13);
    u8g2.print(" SpO²/FiO²  ");
    u8g2.print(fio2_s);
    u8g2.print("   ");
    u8g2.print(fio2_s);
    u8g2.setCursor(0, y + 30);
    u8g2.print(" P:Pep/PIn ");
    u8g2.print(pre_s);
    u8g2.print(":");
    //u8g2.setCursor(75, y + 30);
    u8g2.print(peep_s);
    u8g2.print("/");
    u8g2.print(pip_s);


    if (expiracion == 1) //  exhala
    {
      // u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
      u8g2.setFont(u8g2_font_cursor_tf);
      u8g2.setCursor(120, 0); 
      u8g2.print(char(137)); // Pulmon @1n
    } else {
      // u8g2.setFont(u8g2_font_open_iconic_human_1x_t);
      u8g2.setFont(u8g2_font_cursor_tf);
      u8g2.setCursor(120, 0); // RIGHT
      u8g2.print(char(136)); // Pulmon @2b
    }

  } while ( u8g2.nextPage() );
}

// Page 2 Grafica de Volumen
void VolumenGraph(float vleido)
{
  if (vleido >= v_max)
  {
    v_max = vleido;
  }

  u8g2.clearBuffer();                   
  u8g2.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  u8g2.setCursor(0, 10);
  u8g2.print("VTd: ");
  u8g2.print(TDp, 0);
  u8g2.print(" ml");
  u8g2.setCursor(12, 25);
  u8g2.print(vleido, 0);

  u8g2.drawHLine(x_offset - 5, y_offset + 1, 128);
  for (int x = 0; x < 128; x = x + 10)
  {
    u8g2.drawVLine(x_offset + x, y_offset, 5);
  }

  u8g2.drawVLine(x_offset - 5, 16, y_offset);
  for (int y = 16; y < 60; y = y + 10)
  {
    u8g2.drawHLine(x_offset - 10, y, 5);
  }
  for (int x = 0; x < 120; x++) // shift graph in time
  {
    //u8g2.drawPixel(x + x_offset, y_offset - v_graph[x + 1]);
    u8g2.drawLine((x - 1 + x_offset), (y_offset - v_graph[x + 0]), (x + x_offset), (y_offset - v_graph[x + 1]));
  }

  u8g2.sendBuffer();
}

// Page 3 Grafica de Flujo
void FlowGraph(float fleido)
{
  if (fleido >= f_max)
  {
    f_max = fleido;
  }

  u8g2.clearBuffer();                   
  u8g2.setFont(u8g2_font_6x12_tf);      
  u8g2.setCursor(0, 10);
  u8g2.print("Flow Max: ");
  u8g2.print(f_max, 1);
  u8g2.print(" L/m");
  u8g2.setCursor(12, 25);
  u8g2.print(fleido, 0);

  u8g2.drawHLine(x_offset - 5, y0_offset + 1, 128);
  for (int x = 0; x < 128; x = x + 10)
  {
    u8g2.drawVLine(x_offset + x, y0_offset - 2, 4);
  }

  u8g2.drawVLine(x_offset - 5, 16, 62);
  for (int y = 16; y < 60; y = y + 8)
  {
    u8g2.drawHLine(x_offset - 10, y, 5);
  }
  for (int x = 0; x < 120; x++) // shift graph in time
  {
    u8g2.drawLine((x - 1 + x_offset), (y0_offset - f_graph[x + 0]), (x + x_offset), (y0_offset - f_graph[x + 1]));
  }

  u8g2.sendBuffer();
}

// Page 4 Grafica de Presion
void PressureGraph(float pr)
{
  Pip = presion.pico();
  Peep = presion.peep();
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  u8g2.setCursor(0, 10);
  //u8g2.print(dpr , 0);
  u8g2.print("PIn:");
  u8g2.print(Pip, 0);
  u8g2.print(" Pep:");
  u8g2.print(Peep, 0);
  u8g2.print(" cmH²O");
  u8g2.setCursor(12, 25);
  u8g2.print("Paw: ");
  u8g2.print(pr , 0);
  //u8g2.print(dpr , 0);

  u8g2.drawHLine(x_offset - 5, y1_offset + 1, 128);
  for (int x = 0; x < 128; x = x + 10)
  {
    u8g2.drawVLine(x_offset + x, y1_offset - 2, 4);
  }

  u8g2.drawVLine(x_offset - 5, 16, 62);
  for (int y = 16; y < 60; y = y + 8)
  {
    u8g2.drawHLine(x_offset - 10, y, 5);
  }
  for (int x = 0; x < 120; x++) // shift graph in time
  {
    // Draw both pressures
    u8g2.drawLine((x - 1 + x_offset), (y1_offset - p_graph[x + 0]), (x + x_offset), (y1_offset - p_graph[x + 1]));
    u8g2.drawLine((x - 1 + x_offset), (y1_offset - p1_graph[x + 0]), (x + x_offset), (y1_offset - p1_graph[x + 1]));
  }

  u8g2.sendBuffer();
}

// Page 5 ATM MeasurementsPa
void ATMGraph()
{
  //readATM();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.setFontMode(0);
  u8g2.enableUTF8Print();
  u8g2.setDrawColor(1);
  u8g2.drawRBox(0, 0, 129, 16, 5);
  u8g2.setDrawColor(0);
  u8g2.drawUTF8(20, 13, "Meteorológico");
  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 30);
  u8g2.print("Temp C: ");
  //u8g2.print(temperature, 2);
  u8g2.setCursor(0, 45);
  u8g2.print("Altura m: ");
  //u8g2.print(meters, 0);
  u8g2.setCursor(0, 60);
  u8g2.print("Pascals: ");
  //pu8g2.print(pascal, 0);
  u8g2.sendBuffer();


}

// Page 6 Info Graph
void InfoGraph()
{

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.setFontMode(0);
  u8g2.enableUTF8Print();
  u8g2.drawStr(17, (info_y + 12), "TOM VentMex");
  u8g2.drawXBMP(0, (info_y + 15), logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
  u8g2.setFont(u8g2_font_t0_13_te);
  u8g2.drawStr(8, (info_y + 70), "makersmexico.org");
  u8g2.drawStr(17, (info_y + 100), "Marvin Nahmias");
  u8g2.drawStr(-3, (info_y + 120), "Sebastian Larranaga");
  u8g2.drawStr(33, (info_y + 140), "David Rico");
  u8g2.drawStr(17, (info_y + 160), "Manuel Victoria");
  u8g2.drawStr(5, (info_y + 180), "Claudia Dorembaum");
  u8g2.drawStr(15, (info_y + 200), "Copyright 2020");
  u8g2.sendBuffer();

  if (digitalRead(BUTTON_SELECT) == 0 && analogRead(JOYSTICK) >= 560)
  {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 15, logo_width, logo_height, logo);
    u8g2.sendBuffer();
    sing(1);
    sing(2);
  } // Easter Egg

  //Scrollit
  if (info_y >= -220)
  {
    info_y = info_y - 5;
  }
  else
  {
    info_y = 220;
  }
}

// Display de Alarmas (no parte del thread)
void Alarmas(String msg1, String msg2, String msg3)
{
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_8x13B_mr);
  u8g2.setCursor(0, 13);
  u8g2.setDrawColor(0);
  u8g2.print(msg1);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(0, 30);
  u8g2.print(msg2);
  u8g2.setCursor(0, 40);
  u8g2.print(msg3);
  u8g2.setFont(u8g2_font_8x13B_mr);
  u8g2.setCursor(5, 63);
  u8g2.drawLine(0, 50, 128, 50); // Horizontal Line
  u8g2.drawLine(0, 49, 128, 49); // Horizontal Line
  u8g2.print(" A-T-E-N-D-E-R ");
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
  tomDelay(100);
}

// Dibuja Logo
void drawLogo() {

  int i = 0, y = 1;
  u8g2.firstPage();
  do {
    //u8g2.setFontMode(0);
    u8g2.setFont(u8g2_font_helvR10_tf);
    u8g2.setFontMode(0);
    u8g2.drawStr(17, 12, "TOM VentMex");
    u8g2.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
    u8g2.setFont(u8g2_font_t0_13_te);
    u8g2.drawStr(8, 60, "makersmexico.org");
    u8g2.setFont(u8g2_font_helvR10_tf);
  } while ( u8g2.nextPage() );

  tomDelay(500);

  u8g2.firstPage();
  do {
    //u8g2.setFontMode(0);
    u8g2.setFont(u8g2_font_helvR10_tf);
    u8g2.setFontMode(0);
    u8g2.drawStr(17, 12, "TOM VentMex");
    u8g2.setDrawColor(0);
    u8g2.drawXBMP(50, 15, plus_width, plus_height, plus); //X, Y, Size_X, Size_Y, data
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 60, ".>> Inicio Motor <<.");
    u8g2.setFont(u8g2_font_helvR10_tf);
  } while ( u8g2.nextPage() );
  inicializaMotor();
  setupATM(); // Setup de Presion Atmosferica Sensor
  readATM();  // Lee valor de ATM
  

  u8g2.firstPage();
  do {
    //u8g2.setFontMode(0);
    u8g2.setFont(u8g2_font_helvR10_tf);
    u8g2.setFontMode(0);
    u8g2.drawStr(17, 12, "TOM VentMex");
    u8g2.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data
    u8g2.drawStr(0, 60, ">Calibra Sensores<");
    u8g2.setFont(u8g2_font_helvR10_tf);
  } while ( u8g2.nextPage() );
  presion.calibrar(); // Calibra Presion
  flujo.calibrar(); // Calibra Flujo
  

  for (i; i < 50; i = i + 10) // Animate Loading...
  {

    u8g2.setDrawColor(1);
    //u8g2.drawBox(0, 1, i, 11);
    if (y == 1)
    {
      u8g2.setDrawColor(0);
      u8g2.drawXBMP(50, 15, plus_width, plus_height, plus); //X, Y, Size_X, Size_Y, data
      u8g2.setDrawColor(1);
    }
    if (y == 2)
    {
      y = 0;
      u8g2.setDrawColor(1);
      u8g2.drawXBMP(0, 15, logo_width, logo_height, logo); //X, Y, Size_X, Size_Y, data

    }

    u8g2.sendBuffer();
    tomDelay(500);
    y++;
  }
  // BEEP BEEP : READY
  beep(); beep();
}

// Beep
void beep()
{
  tone(BUZZ, 700);
  tomDelay(100);
  noTone(BUZZ);
  tomDelay(50);
}

// Boop
void boop()
{
  tone(BUZZ, 180);
  tomDelay(100);
  noTone(BUZZ);
  tomDelay(50);
}

// BMP280 Setup Sensor Atmosferico y Temperatura
void setupATM()
{
  //if (bmp280.initialize()) Serial.println("BMP280 Sensor encontrado.");
  //else
  //{
  //  Serial.println("Sensor BMP280 NO encontrado!");
  //}
  //bmp280.setEnabled(0);
  //bmp280.triggerMeasurement();
}

// BMP280 Lectura Sensor Atmosferico y Temperatura
void readATM()
{
  //bmp280.awaitMeasurement();
  //bmp280.getTemperature(temperature);
  //bmp280.getPressure(pascal);
  //bmp280.getAltitude(meters);
  //bmp280.triggerMeasurement();
}

void setupTVM()
{
  FiO2 = 21; // Default aire normal
  alarma_status = ALARMA_OFF; // No hay alarmas - Clear All
  tipo_alarma = 0; // No hay tipo de alarma

  u8g2.setFont(u8g2_font_helvR10_tf);
  // Selecciona MODO VENTILADOR
  AC = u8g2.userInterfaceSelectionList("Modo de Control :", 1, "Volúmen Asistido\nPresión Asistida");
  beep();

  if (AC == 2)
  {
    //u8g2.setFont(u8g2_font_t0_13_te);
    u8g2.setFont(u8g2_font_helvR10_tf);
    u8g2.userInterfaceInputValue("\n\nSensitividad :\n", "en = ", acs, 10, 50, 2, " mmH20");
    ac_value = acs / 10;
  }
  switch (fio)
  {
    case 1: FiO2 = 21; break;
    case 2: FiO2 = 24; break;
    case 3: FiO2 = 28; break;
    case 4: FiO2 = 32; break;
    case 5: FiO2 = 36; break;
    case 6: FiO2 = 40; break;
    case 7: FiO2 = 50; break;
  }
  // BEEP Selected: FiO2 Entry
  beep();
}

void setup() {

  Serial.begin(2000000);
  // Start OLED Library with buttons
  u8g2.begin(BUTTON_SELECT, BUTTON_MOVE, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE); // Startup Logo Screen (select, next, previous, up, down, home) lo cachamos con funcipon de Getmenu
  u8g2.enableUTF8Print(); // Enable UTF8 Printing

  // Set Pins
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
  Serial.println();
  Serial.println("TOM Vent-Mex : Initializing");
  Serial.println("---------------------------");
  Serial.print("Model: ");
  Serial.print(MODEL);
  Serial.print(" Software Version: ");
  Serial.println(VERSION);
  Serial.println("Initializing pins, leds, beeper and threads.");
  Serial.println();
  Serial.print("VT,RR,I/E-1,PP,PI,TI,TE");
  Serial.println();
#else
  Serial.println();
  Serial.print("Model: ");
  Serial.print(MODEL);
  Serial.print(" Firmware: ");
  Serial.println(VERSION);
#endif

  drawLogo(); // Draw Logo and reset variables, sensors and motor
  setupTVM(); // Setup de menu principal
  Serial << "Habilita Motor...\n";
  stepper.enable();
  Pagina = 1; // Empieza con Pagina Total
}

void loop() {
  Serial << "Pinta OLED...\n";
  pintaPantalla();
  Serial << "Lee Controles...\n";
  leeControles();
  Serial << "Calcula Respiracion...\n";
  calculaRespiracion();
  Serial << "Corre Ciclo...\n";
  cicloVentMex(); // Cambia Logica sin Threads, se corren dentro del Ciclo sin bloquear
}
