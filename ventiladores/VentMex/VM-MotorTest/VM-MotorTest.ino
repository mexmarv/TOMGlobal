/*
   Demo for VentMex Motor Control and Calibration, using AccelStepper with NEMA-23
   200 Steps, 32 Microsteps
   Connect STEP, DIR as indicated
*/

#include <Arduino.h>
#include <Math.h>
#include <Streaming.h>
#include <AccelStepper.h>
#include <U8g2lib.h>
#include "VMPresion.h"
#include "VMFlujo.h"

// VARIAR ESTO PARA PRUEBAS asi no usamos pots para pruebas todavia
float RATEp           = 30.00;
float TDp             = 850.00;
float IEp             = 1.0;
//

// Motor steps per revolution. e 200 steps or 1.8 degrees/step
#define MOTOR_STEPS       200
#define FULL_TD_STEPS     77  // Cambiar a FS (Mitad de fsteps por el rodillo de Omer) 180 o 150 grados 950 ml
#define MICROSTEPS        32
#define STEP              6
#define PRESSURE          A0
#define IE                A4

// VentMex Shield Pinout
// Digital
#define ENABLE                    4 // Enable Motor PIN
#define DIR                       5 // Direction Motor PIN
#define STEP                      6 // Step Motor Pin
#define BUZZ                      7 // Buzzer PIN 
#define VENTILATOR                8 // Button On/Off Motors S4 VentMex
#define ENDSTOP                   9 // Button EndStop S3 VentMex
#define BUTTON_MOVE               10 // Button Move S2 VentMex
#define BUTTON_SELECT             11 // Button Select S1 VentMex
// Analog
#define PRESSURE                  A0
#define FLOW                      A1
#define JOYSTICK                  A2
#define RATE                      A3
#define IE                        A4
#define TD                        A5

// Graphs
int p_graph [100] = {};
int p_max = 40; // Presion Inspiratoria de 0 a 40 cmH2O
int p_min = -20;
char data[100];
int y_offset = 42;  
int x_offset = 65;


// Rangos para Potenciometros
const float BPM_MIN  = 5.0;
const float BPM_MAX  = 90.0;
const float VOL_MIN  = 20.0;
const float VOL_MAX  = 900.0;
const float IE_MIN = 1.0;
const float IE_MAX = 4.0;

// Setting de Seguridad
const float MAX_PRESSURE         = 40.0; // Trigger high pressure alarm
const float MIN_PLATEAU_PRESSURE =  5.0; // Trigger low pressure alarm
const float MAX_RESIST_PRESSURE  =  2.0; // Trigger high-resistance notification
const float MIN_TIDAL_PRESSURE   =  5.0; // Trigger no-tidal-pressure alarm


// Timing Settings
const float LOOP_PERIOD          = 0.03; // The period (s) of the control loop
const float HOLD_IN_DURATION     = 0.1;  // Duration (s) to pause after inhalation
const float MIN_PEEP_PAUSE       = 0.05; // Time (s) to pause after exhalation / before watching for an assisted inhalation
const float MAX_EX_DURATION      = 1.00; // Maximum exhale duration (s)

float tCycleTimer;            // Absolute time (s) at start of each breathing cycle
float tIn;                    // Calculated time (s) since tCycleTimer for end of IN_STATE
float tHoldIn;                // Calculated time (s) since tCycleTimer for end of HOLD_IN_STATE
float tEx;                    // Calculated time (s) since tCycleTimer for end of EX_STATE
float tPeriod;                // Calculated time (s) since tCycleTimer for end of cycle
float tPeriodActual;          // Actual time (s) since tCycleTimer at end of cycle (for logging)
float tLoopTimer;             // Absolute time (s) at start of each control loop iteration
float tLoopBuffer;            // Amount of time (s) left at end of each loop
float tins, tesp;             // Tiempos totales de Inspiracion y Espiracion

int tdSteps, inRPS, esRPS = 0;
int expiracion = 0;

float volume, pressure, flow, Pip, Peep, Plateau = 0.0;

int Pagina = 1;

VMPresion presion(A0);
VMFlujo flujo(A1);

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //OLED Display I2C 0.96

// Returns the current time in seconds
inline float now()
{
  return millis() * 1e-3;
}

void calculaRespiracion() {

  tPeriod = 60.0 / RATEp;
  tHoldIn = tPeriod / (1 + IEp);
  tIn = tHoldIn - HOLD_IN_DURATION;
  //tEx = min(tHoldIn + MAX_EX_DURATION, tPeriod - MIN_PEEP_PAUSE); // Old Code para minimo de maxima exhalacion
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
      flow = flujo.flujo(0.21);
      break;
    }
  }
  delay(HOLD_IN_DURATION);
  presion.lee();
  presion.pon_plateau(); // Pon Plateau (siempre como se lee presiones al principio de ciclo estamos), guarda
  flow = flujo.flujo(0.21);
  Plateau = round(presion.plateau());

  Serial << presion.get() << "\n";
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
      flow = flujo.flujo(0.21);
      Serial << presion.get() << "\n";
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
  flow = flujo.flujo(0.21);
  //Serial << presion.get() << "\n";
  //Test (delay calc 80 ms)
  //float tR = (now() - tCycleTimer) - 0.08;
  //Serial << "Tiempo Real/Calc: " << tR << "/" << tPeriod << " (" << (tR - tPeriod) << ") " << ((tR * 100 / tPeriod) - 100) << "%\n";
}

void zigzag() // Zig Zag Prueba Basica sin mediciones ni carga ni nada
{

  expiracion = 1; // Icono Inspiracion
  tCycleTimer = now();

  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(-inRPS);
  stepper.setSpeed(-inRPS);

  sensorPantalla();
  while (true)
  {
    stepper.runSpeed();
    if (stepper.currentPosition() <= (-tdSteps * MICROSTEPS)) break;
  }

  Serial << (now() - tCycleTimer) << ":" << tIn << "(" << ((now() - tCycleTimer) - tIn) << ")\n";

  expiracion = 0; // Icono Espiracion
  tCycleTimer = now();
  stepper.setMaxSpeed(esRPS);
  stepper.setSpeed(esRPS);

  sensorPantalla();
  while (true)
  {
    stepper.runSpeed(); // Esto avanza el Loop
    if (stepper.currentPosition() == 0) break;
  }

  Serial << (now() - tCycleTimer) << ":" << (tEx - tIn) << "(" << ((now() - tCycleTimer) - (tEx - tIn)) << ")\n\n";

}

void homeMotor()
{
  Serial << "Homing ...\n";
  stepper.setMaxSpeed(3200);  // 1/2 velocidad para homing
  stepper.setAcceleration(42666); // Aceleracion rapida siempre
  stepper.moveTo(64000);  // 100 vueltas aprox max encontrar switch

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
      //Serial << "Not Homed, Error! ...\n";
      initPantalla("ERROR NOT HOME");
      stepper.stop();
    }
  }
}

void leeControles()
{
  IEp   = map(analogRead(IE), 5, 1020, IE_MIN, IE_MAX);     // IE de 1:1 a 1:4 por pacientes de Covid-19
  RATEp = map(analogRead(RATE), 5, 1020, BPM_MIN, BPM_MAX); // 5 a 30 BPM
  TDp   = map(analogRead(TD), 5, 1020, VOL_MIN, VOL_MAX);   // 100 a 1000 cmH2O - vs 250 to 600
  // Overrride Tests
  //TDp = 850.00;
  //IEp = 1.00;
}

void logicaPantalla()
{
  int joy = analogRead(JOYSTICK);
  if (joy >= 560)
  {
    Pagina++;
    if (Pagina == 5) Pagina = 1;
  }

  if (joy <= 480)
  {
    Pagina--;
    if (Pagina == 0) Pagina = 4;
  }

  if (digitalRead(BUTTON_SELECT) == 0 && (joy <= 480 || joy >= 560)) {
    //tomDelay(100);
    //setupTVM();
  }

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
  }
}

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

  pantalla.sendBuffer();

  p_graph[64] = (pressure) * (y_offset - 20) / p_max;

  for (int x = 0; x < 64; x++) 
  {
    p_graph[x] = p_graph[x + 1];
  }

}

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

void presionGrafica() {

  char buff[8];

  pantalla.setDrawColor(0);
  pantalla.drawBox(0, 17, 128, 64);
  pantalla.setDrawColor(1);

  pantalla.drawBox(100,51,125,64);
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
 

  for (int x = 0; x < 64; x++) // shift graph in time
  {
    pantalla.drawLine((x - 1 + x_offset), (y_offset - p_graph[x + 0]), (x + x_offset), (y_offset - p_graph[x + 1]));
  }
  
  pantalla.sendBuffer();
}

void initPantalla (String msj)
{
  pantalla.firstPage();
  do
  {
    pantalla.setFont(u8g2_font_helvR10_tf);
    pantalla.setFontMode(0);
    pantalla.drawStr(17, 12, "TOM VentMex");
    pantalla.setCursor(0, 50);
    pantalla.print(msj);
  } while ( pantalla.nextPage() );
}

void setup() {
  Serial.begin(2000000);
  //Serial << "\nSerial Debug de Motor y Tiempos, VentMex Calibracion:" << "\n";
  pinMode(9, INPUT_PULLUP);
  pantalla.begin(BUTTON_SELECT, BUTTON_MOVE, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE); // Screen (select, next, previous, up, down, home)
  pantalla.enableUTF8Print(); // Enable UTF8 Printing
  initPantalla("  Comienzo Homing");
  homeMotor();
  initPantalla("  Comienzo Flujo");
  flujo.calibrar();
  initPantalla("  Comienzo Presion");
  presion.calibrar();
  initPantalla("  Terminando ...");
  Pagina = 2;
  Serial << "Presion";
}

void loop()
{
  logicaPantalla();
  leeControles();
  calculaRespiracion();
  cicloVentMex();
  //zigzag();
}
