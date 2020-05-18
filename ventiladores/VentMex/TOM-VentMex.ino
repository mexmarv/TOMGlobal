/*
   TOM VENTMEX 2.1
   Original frame design by:VentCore www.ventcore.health Copyright (C)2020 Formon LLC

   This file may be redistributed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   A copy of this license has been included with this distribution in the file LICENSE.

   TOM Makers - Mexico Chapter have joined the prototype build and will be contribute to this code and hardware as a new remix, original design did not work.
   To differantiate prototypes due to motors and 3D parts as they are being tested, we will include a model variable.
   We found out that the original stepper motor libraries do not work unfortunately, and the designed lacked power in motor drivers and overall mechanical and system design.
   We created a new thread as TOM VentMex
   Ing. Omer Sebastian Larranaga - on mechanotronics, electronics, sensors and first build tests
   Ing. Marvin Nahmias - on systems, electronics and software, marvin@tomglobal.org
   Ing. David Rico - on overall engineering, physycs and design davidrico@hotmail.com
   Lic. Manuel Victoria - Legal and Operational Support
   TOM Makers Mexico - Claudia Dorembaum, claudia@tomglobal.org

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

  - Cuando recien encendemos no tenemos problemas por que ahi establecemos nuestro ZERO y tiene que ver con la presion atmosferica.

  Recuerda que el peep debe mandarnos una alarma si baja de 4 cmH2O,
  por lo que cualquier variacion de 1 cmH20 en la presion atmosferica pudiera dispararnos una falsa alarma.
  para evitarlo reajustaremos el valor de lectura presion zero al nuevo Sensor Offset.
  Las lecturas del arduino van de 0 a 1023 y los voltajes de 0  a 5
  pero para este sensor el voltaje minimo es de Minimo 0.152 Maximo 0.378 y Tipico de 0.265
  esto varia dependiendo de la presion atmosferica, por eso hay que ajustarlo.
  para determinar un valor use una regla de 3 simple   1023 es a 5Volts como 0.265 Volts es X dando X=54.219
  Sea Level 101.325KPa
  x=54.219 / Min=31.099 y Max=77.338
  Sube 147.51 kPa para llegar al min o al max de voltaje 0.152 y 0.378
*/

// Define includes and libraries
#include <Arduino.h>
#include <U8x8lib.h>
#include <ThreadController.h>
#include <StaticThreadController.h>
#include <Thread.h>
#include "TOM-VentMex.h"      // TOM VentMex Defines
#include "TOM-Music.h"
#include "i2c_BMP280.h"

//#define DEBUG //if not defined then PRODUCTION
#define PROD

// VentMex Sensor Variables
int steps = 500; // Default Steps
int stepDelay, stepDelayI, stepDelayE = 300; // Default Delays
int TDv, TDl, IEv, FiO2, alarm, tipo_alarma = 0;
float IEp, RATEv, pressure, diffpressure, velocity, area, flow, lxm, tinalacion, texalacion, tresp, Pip, Peep, heart = 0;
bool exhale; // controlar que ciclo exhalas
int AC = 1; // Assisted Control 1 = Volume Assist , 2 = Pressure Assist
// External Sensors and OLED
BMP280 bmp280;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //OLED Display I2C 0.96

// Sensor Variables
float temperature;
float pascal;
static float meters;
int fio; // de valvulas

// Graphs
int v_graph [150] = {};
int v_max = 600;
int v_min = 0;
int f_graph [150] = {};
int f_max = 120;
int f_min = -120;
int p_graph [150] = {};
int p1_graph [150] = {};
int p_max = 30;
int p_min = -10;
char data[100];
int y_offset = 62;  // bottom axis
int y0_offset = 38;  // middle axis 16 + 44 total
int y1_offset = 48;  // middle axis 16 + more to be less negative
int x_offset = 12;
int info_y = 0;
int b = 0;
double now = 0;

int Pagina = 1; // Pagina 1 es todos los sensores, 2 Volumen, 3 Flujo

// Initialize CONTROLLER with Sensor Calc & Display, Alarms Beeps, Main Loop is Motor
ThreadController controller = ThreadController();
Thread* sensorThread = new Thread();
Thread* alarmsThread = new Thread();
Thread* motorsThread = new Thread();

// Motors Thread
void motorsThreadCallback()
{
  // Print Millis and move heart to exhale
  //
  //Serial.println(millis());
  if (heart == 0) {
    heart = 1;
  } else {
    heart = 0;
  }
  if (digitalRead(VENTILATOR) == HIGH) // Its ON !
  {

    if (analogRead(ENDSTOP) == LOW) {
      exhale = false; // Inhalando
      PeaksPressure(); // Calcula Pip y Peep (tendra que ser en todo un ciclo)
      digitalWrite(DIR, HIGH);
      for (int x = 0; x < steps * 2; x++) {
        digitalWrite(STEP, LOW);
        delayMicroseconds(stepDelayI);
        digitalWrite(STEP, HIGH);
        delayMicroseconds(stepDelayI);
      }
    } else {
      exhale = true; // Exhalando
      PeaksPressure(); // Calcula Pip y Peep (tendra que ser en todo un ciclo)
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, LOW);
      delayMicroseconds(stepDelayE);
      digitalWrite(STEP, HIGH);
      delayMicroseconds(stepDelayE);
    }
  }
}

// Alarms Thread
void alarmsThreadCallback()
{
  // Checa Alarmas
  if (Pip >= 80) {
    tipo_alarma = 1;   // Pip Alta > 180ml
    alarma_status = ALARMA_RAPID;
  }
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
    //u8g2.setFont(u8g2_font_open_iconic_human_1x_t);
    //u8g2.setCursor(90, 9); // RIGHT
    //u8g2.print("C"); // Alarm from Font @2x
    //u8g2.sendBuffer();
    tone(BUZZ, 1000);
    tomDelay(300);
    //u8g2.setCursor(90, 9); // RIGHT
    //u8g2.print(" ");
    //u8g2.sendBuffer();
    noTone(BUZZ);
    tomDelay(600);
  }

  if (digitalRead(BUTTON_SELECT) == 0) // READ ALARM CLEAR BUTTON
  {
    alarma_status = ALARMA_OFF; // Alarm Sound
    tipo_alarma = 0; // Alarm Display
    beep(); beep(); beep(); // 3 BEEPS CLEAR
  }
}

// Sensors Thread
void sensorThreadCallback() {

  // Here the Math Starts!
  // Read Sensors, Calculate and Send through OLED & Serial

  // OLD: a0value = map(analogRead(PRESSURE), 61, 927, 0, 6000); // Map 0 a 6000
  // OLD: pressure = a0value / 98.06; // cmh2o (Pascales)
  pressure = ((analogRead(PRESSURE) - Aoff) * Popmax / 10) / (Afss - Soffset); //cmH20

  // OLD: diffpressure = map(analogRead(FLOW), 64, 927, 0, 6000);
  diffpressure = ((analogRead(FLOW) - Aoff + Amax / 2) * Popmax / 10) / (Afss - Soffset); //cmH20

  //OLD: spressure = diffpressure / 98.06; // cmh20 (Pascales)
  //OLD: velocity = sqrt((diffpressure * 2) / 1.225);
  //OLD: area = pow(RADIUS_SENS_HOSE, 2) * 3.14159265359;
  //OLD: flow = velocity * area; // cm3/s = mL
  flow = K * sqrt(diffpressure);  //ml

  TDv = map(analogRead(TD), 5, 1020, 100, 1150); // TD para steps de motor
  TDl = map(analogRead(TD), 5, 1020, 7, 700); // TD para ml p
  steps = TDv; // STEPS DEL MOTOR CON TD

  IEv = map(analogRead(IE), 5, 1020, 10, 30); // IE para Motor
  IEp = map(analogRead(IE), 5, 1020, 500, 750); // IE para tiempos

  //Ritmo Respiratorio rango 0 a 150 resp/min
  RATEv = map(analogRead(RATE), 5, 1020, 0, 150);

  //MAGIC CALCs
  lxm = (flow * RATEv) / 1000; // O2 L/min
  // FiO2 no lo calculamos porque no hay sensores .... usamos valvulas
  // entonces leeremos el valor discreto de un Pot.

  // FiO2 = 21; Read Pot before

  //Calculamos el periodo de una respiracion (TIEMPO INHALACION)
  tresp = (60 / RATEv) * 1000;
  //Calculamos ancho de pulso para 1:1 (ANCHO PULSO para 1:1)
  stepDelay = 120 * (tresp / steps);
  //calculamos tiempo de inalacion y stepsI motor
  tinalacion = (tresp * 2) * (IEp / 1000);
  stepDelayI = 120 * (tinalacion / steps);

  //calculamos tiempo de exhalacion y stepsE motor
  texalacion = (tresp * 2) - tinalacion;
  stepDelayE = 120 * (texalacion / steps);


  // Mete lecturas en graficas sin dibujar (borrar esto) llenar la matriz 120 con lectura
  b++;
  p_graph[120] = (260 * sin(0.1 * b) + 250) * (y_offset - 20) / v_max;
  p1_graph[120] = (100*pow(sin(b),63) * sin(b+1.5)*8)* (y0_offset - 20) / f_max;
  //
  // HEART LIKE pow(sin(b),63) * sin(b+1.5)*8;
  v_graph[120] = (flow) * (y_offset - 20) / v_max;
  f_graph[120] = (lxm) * (y0_offset - 20) / f_max;
  //p_graph[120] = (pressure) * (y1_offset - 20) / p_max;
  //p1_graph[120] = (diffpressure) * (y1_offset - 20) / p_max;

  // Lee y llena matrices de graficas
  for (int x = 0; x < 120; x++) // shift graph in time
  {
    v_graph[x] = v_graph[x + 1];
    f_graph[x] = f_graph[x + 1];
    p_graph[x] = p_graph[x + 1];
    p1_graph[x] = p1_graph[x + 1];
  }

  if (digitalRead(BUTTON_SELECT) == 0 && digitalRead(BUTTON_MOVE) == 0) {
    setupTVM();
  }

  if (digitalRead(BUTTON_MOVE) == 0) {

    Pagina++;
    if (Pagina == 7) Pagina = 1; // 6 para Info

    boop();
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
        //VolumenGraph(260 * sin(0.1 * b) + 250); // demo function
        VolumenGraph(flow);
        break;
      }
    case 3:
      {
        //FlowGraph(90*pow(sin(b),63) * sin(b+1.5)*8); /// demo function de corazon beat
        FlowGraph(lxm);
        break;
      }
    case 4:
      {
        PressureGraph(pressure,diffpressure);
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

  String vtd_s, vt_s, iev_s, rte_s, prp_s, fip_s, ti_s, te_s, peep_s, fio2_s, spo2_s, pip_s = "";
  float ti, te, fip, bpm = 0;
  char buff[24];

  ti = (tinalacion / 2) / 1000;
  te = (texalacion / 2) / 1000;
  fip = fio; // L x min?

  vt_s += dtostrf(flow, 3, 0, buff); // no decimals Flujo medido
  vtd_s += dtostrf(TDl, 3, 0, buff); // no decimals Flujo set pot
  iev_s += dtostrf(IEv / 10, 2, 0, buff);
  rte_s += dtostrf(RATEv, 2, 0, buff);

  peep_s += dtostrf(Peep, 3, 0, buff);
  pip_s  += dtostrf(Pip, 3, 0 , buff);

  fip_s += dtostrf(fip, 4, 1, buff);
  ti_s += dtostrf(ti, 2, 1, buff);
  te_s += dtostrf(te, 2, 1, buff);
  fio2_s  += dtostrf(FiO2, 3, 0 , buff);


  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13B_mr);
    u8g2.setCursor(-8, 11);
    u8g2.print(iev_s + ":1");
    u8g2.drawLine(26, 0, 26, 16); // Vertical Line
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(30, 11);
    u8g2.print(ti_s + "/" + te_s + "s");
    u8g2.setCursor(90, 6);
    u8g2.setFont(u8g_font_5x7);
    u8g2.print(rte_s); // BPM
    u8g2.setCursor(90, 14);
    u8g2.print(rte_s); // BPM
    u8g2.drawLine(0, 16, 128, 16); // Horizontal Line
    u8g2.drawLine(62, 16, 62, 64); // Vertical Line
    u8g2.drawLine(97, 16, 97, 64); // Vertical Line
    u8g2.drawLine(0, 48, 128, 48); // Horizontal Line
    u8g2.setFont(u8g2_font_6x12_tf); // tf tr tn te
    int y = 30;
    u8g2.setCursor(0, y);
    u8g2.print(" Vol.Tidal  ");
    u8g2.print(vt_s);
    u8g2.print("   ");
    u8g2.print(vtd_s);
    u8g2.setCursor(0, y + 13);
    u8g2.print(" SpO²/FiO²  ");
    u8g2.print(fio2_s);
    u8g2.print("   ");
    u8g2.print(fio2_s);
    u8g2.setCursor(0, y + 30);
    u8g2.print(" PeeP/PIns  ");
    u8g2.print(peep_s);
    u8g2.print("   ");
    u8g2.print(pip_s);


    if (heart == 1) //  exhale is production as cicle of heart beat
    {
      u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
      u8g2.setCursor(108, 14); // RIGHT
      u8g2.print("B"); // Heart from Font @2x
    } else {
      u8g2.setFont(u8g2_font_open_iconic_human_1x_t);
      u8g2.setCursor(112, 9); // RIGHT
      u8g2.print("B"); // Heart from Font @1x
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

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  u8g2.setCursor(0, 10);
  u8g2.print("VMax:");
  u8g2.print(v_max, 1);
  u8g2.print(" ml       ");
  u8g2.print(vleido, 1);

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

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  u8g2.setCursor(0, 10);
  u8g2.print("FMax:");
  u8g2.print(f_max, 1);
  u8g2.print(" L/min   ");
  u8g2.print(fleido, 1);

  //f_graph[120] = ( fleido * (y0_offset - 20) / f_max);

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
    //u8g2.drawPixel(x + x_offset, y0_offset - f_graph[x + 1]);
    u8g2.drawLine((x - 1 + x_offset), (y0_offset - f_graph[x + 0]), (x + x_offset), (y0_offset - f_graph[x + 1]));
  }

  u8g2.sendBuffer();
}

// Page 4 Grafica de Presion
void PressureGraph(float pr, float dpr)
{

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  u8g2.setCursor(0, 10);
  u8g2.print("PMax:");
  u8g2.print(p_max, 1);
  u8g2.print("cmH²O   ");
  u8g2.print(pr , 0);
  u8g2.print("/");
  u8g2.print(dpr , 0);

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
  readATM();

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
  u8g2.print(temperature, 2);
  u8g2.setCursor(0, 45);
  u8g2.print("Altura m: ");
  u8g2.print(meters, 0);
  u8g2.setCursor(0, 60);
  u8g2.print("Pascals: ");
  u8g2.print(pascal, 0);
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
  u8g2.setFont(u8g2_font_lastapprenticebold_tr);
  u8g2.drawStr(8, (info_y + 70), "makersmexico.org");
  u8g2.drawStr(0, (info_y + 100), "Sebastian Larranaga");
  u8g2.drawStr(17, (info_y + 120), "Marvin Nahmias");
  u8g2.drawStr(33, (info_y + 140), "David Rico");
  u8g2.drawStr(17, (info_y + 160), "Manuel Victoria");
  u8g2.drawStr(5, (info_y + 180), "Claudia Dorembaum");
  u8g2.drawStr(17, (info_y + 200), ".Copyright 2020.");
  u8g2.sendBuffer();

  if (digitalRead(BUTTON_SELECT) == 0 && digitalRead(BUTTON_MOVE) == 0)
  {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 15, logo_width, logo_height, logo);
    u8g2.sendBuffer();
    sing(1);
    sing(2);
  } // Easter Egg Play Mario Bros

  //Scrollit
  if (info_y >= -220)
  {
    info_y = info_y - 5;
  }
  else
  {
    info_y = 220;
  }
  //Serial.println(info_y);
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
  u8g2.print(" A T E N D E R ");
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
    u8g2.setFont(u8g2_font_lastapprenticebold_tr);
    u8g2.drawStr(8, 60, "makersmexico.org");
    u8g2.setFont(u8g2_font_helvR10_tf);
  } while ( u8g2.nextPage() );

  // delay(1000);
  for (i; i < 130; i = i + 10) // Animate Loading...
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
    tomDelay(50);
    y++;
  }
  // BEEP BEEP : READY
  beep(); beep();
}

// Peep y Pip se calculan Max
void PeaksPressure() // Calcula PEEP y PInspiratoria (min y max) esto en ciclo respiratorio
{
  //a0value = map(analogRead(PRESSURE), 61, 927, 0, 6000);
  //pressure = a0value / 98.06; // cmh2o (Pascales)
  pressure = ((analogRead(PRESSURE) - Aoff) * Popmax / 10) / (Afss - Soffset);
  if (pressure > Pip) Pip = pressure;
  if (pressure < Peep) Peep = pressure;
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

// Delay so many milliseconds without blocking interrupts or threads
void tomDelay(unsigned long ms)
{
  unsigned long currentMillis  = millis();
  unsigned long previousMillis = millis();

  while (currentMillis - previousMillis < ms) {
    currentMillis = millis();
  }
}

// BMP280 Setup Sensor Atmosferico y Temperatura
void setupATM()
{
  if (bmp280.initialize()) Serial.println("BMP280 Sensor encontrado.");
  else
  {
    Serial.println("Sensor BMP280 NO encontrado!");
  }
  bmp280.setEnabled(0);
  bmp280.triggerMeasurement();
}

// BMP280 Lectura Sensor Atmosferico y Temperatura
void readATM()
{
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  bmp280.getAltitude(meters);
  bmp280.triggerMeasurement();
}

void setupTVM()
{
  FiO2 = 21; // Default aire normal
  alarma_status = ALARMA_OFF; // No hay alarmas - Clear All
  tipo_alarma = 0;

  u8g2.setFont(u8g2_font_helvR10_tf);
  // Selecciona MODO VENTILADOR En Fase 2 por la complejidad de AC-PC
  // AC = u8g2.userInterfaceSelectionList("Modo de Control :", 1, "Volúmen Asistido\nPresión Asistida");
  beep();
  // Selecciona Venturi Valvula 21-50
  fio = u8g2.userInterfaceSelectionList("Selecciona :", 1, "0L - 21%\n1L - 24%\n2L - 28%\n3L - 32%\n4L - 36%\n5L - 40%\n7L - 50%");

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
  u8g2.begin(BUTTON_SELECT, BUTTON_MOVE, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE); // Startup Logo Screen (select, next, previous, up, down, home)
  u8g2.enableUTF8Print(); // Enable UTF8 Printing

  // Set Pins
  pinMode(TD, INPUT);
  pinMode(IE, INPUT);
  pinMode(RATE, INPUT);
  pinMode(OXY, INPUT);
  pinMode(FLOW, INPUT);
  pinMode(ENDSTOP, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(PRESSURE, INPUT);
  pinMode(BUZZ , OUTPUT);
  digitalWrite(EN, LOW);
  pinMode(DIR, OUTPUT);
  digitalWrite(EN, LOW);

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
  Serial.print("VT,RR,I/E-1,PP,PI,TI,TE");
  Serial.println();
#endif
  setupATM(); // Setup de Presion Atmosferica Sensor
  drawLogo(); // Logo and reset variables
  readATM(); // Lee el valor
  setupTVM(); // Setup de menu principal


  // Start ProtoThreads for different intervals
  motorsThread->onRun(motorsThreadCallback);
  motorsThread->setInterval(0);             // Motors no delay
  sensorThread->onRun(sensorThreadCallback);
  sensorThread->setInterval(SENSOR_TIMER);  // Sensors
  alarmsThread->onRun(alarmsThreadCallback);
  alarmsThread->setInterval(ALARM_TIMER);   // Alarms

  // Add threads to controller
  controller.add(motorsThread);
  controller.add(sensorThread);
  controller.add(alarmsThread);

}

void loop() {
  // Simulate Alarma
  //alarm++;
  //if (alarm == 3000) { tipo_alarma = 4; alarma_status = ALARMA_LENTA; alarm = 0;}

  controller.run(); // Run Thread Controller every time (minimum slice 6-7ms) :)
}
