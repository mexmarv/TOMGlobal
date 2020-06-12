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
   David Rico - on overall engineering, physycs and design davidrico@hotmail.com
   Manuel Victoria - Legal and Operational Support
   TOM Makers Mexico Representation - Claudia Dorembaum, claudia@tomglobal.org

   Defines for TOM VentMex V2.0
   Arduino Mega 2560v3 + VentMex Shield + BOM Parts

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

**/

#define MODEL "TOM VentMex Shield"
#define VERSION                   2.1

#define DISPLAY_TIMER             100
#define ALARM_TIMER               100

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

// Alarms
int  alarma_status;
#define ALARMA_OFF                0
#define ALARMA_RAPID              1
#define ALARMA_LENTA              2

// Rangos para Potenciometros
const int BPM_MIN                 = 10;
const int BPM_MAX                 = 50;
const int VOL_MIN                 = 100; // Cofepris dice 0 - 2000, 100-800 segun journals medicos
const int VOL_MAX                 = 850; // Nivel Maximo de emergencia
const float IE_MIN                = 1;
const float IE_MAX                = 4;

// Setting de Seguridad
const float MAX_PRESSURE          = 40.0; // Trigger high pressure alarm
const float MIN_PLATEAU_PRESSURE  = 5.0;  // Trigger low pressure alarm
const float MAX_RESIST_PRESSURE   = 2.0;  // Trigger high-resistance notification
const float MIN_TIDAL_PRESSURE    = 5.0;  // Trigger no-tidal-pressure alarm


// Timing Settings
const float HOLD_IN_DURATION      = 0.1;  // Duration (s) to pause after inhalation
const float MIN_PEEP_PAUSE        = 0.05; // Time (s) to pause after exhalation / before watching for an assisted inhalation
const float MAX_EX_DURATION       = 1.00; // Maximum exhale duration (s)

const int BAG_CLEAR_POS = 50;   // The goal position (Steps) to retract to clear the bag
const int BAG_CLEAR_TOL = 10;   // The tolerance (Steps) to consider clear of bag

// Logo TOM y Signo +
#define logo_width      128
#define logo_height     32
#define plus_width      31
#define plus_height     32

static const unsigned char logo[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xbf,
  0xff, 0x1f, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x80, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xff, 0xbf, 0xff, 0x1f, 0x00, 0xfe, 0x3f, 0x00, 0x00, 0xe0,
  0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xbf, 0xff, 0x0f, 0x80, 0xff,
  0xff, 0x00, 0x00, 0xf0, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xbf,
  0xff, 0x0f, 0xc0, 0xff, 0xff, 0x01, 0x00, 0xfc, 0xff, 0x3f, 0x00, 0x00,
  0x00, 0x00, 0xfc, 0xbf, 0xff, 0x07, 0xe0, 0xff, 0xff, 0x03, 0x00, 0xff,
  0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xbf, 0xff, 0x07, 0xf0, 0xff,
  0xff, 0x07, 0x80, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xbf,
  0xff, 0x03, 0xf0, 0x3f, 0xfe, 0x07, 0x00, 0xfe, 0xff, 0x7f, 0x00, 0x00,
  0x00, 0x00, 0xf0, 0xbf, 0xff, 0x01, 0xf8, 0x0f, 0xf8, 0x0f, 0xc0, 0xf8,
  0xff, 0x1f, 0x01, 0x00, 0x00, 0x00, 0xe0, 0xbf, 0xff, 0x01, 0xf8, 0xc7,
  0xf1, 0x0f, 0xc0, 0xe1, 0xff, 0xc7, 0x01, 0x00, 0x00, 0x00, 0xe0, 0xbf,
  0xff, 0x00, 0xfc, 0xf3, 0xe7, 0x1f, 0xc0, 0x87, 0xff, 0xf1, 0x01, 0x00,
  0x00, 0x00, 0xe0, 0xbf, 0xff, 0x00, 0xfc, 0xf9, 0xcf, 0x1f, 0xc0, 0x1f,
  0x7e, 0xfc, 0x01, 0x00, 0x00, 0x00, 0xc0, 0xbf, 0x7f, 0x00, 0xfc, 0xf9,
  0xcf, 0x1f, 0xc0, 0x7f, 0x18, 0xff, 0x01, 0x00, 0x00, 0x00, 0x80, 0xbf,
  0x7f, 0x00, 0xfc, 0xf9, 0xcf, 0x1f, 0xc0, 0xff, 0x81, 0xff, 0x01, 0x00,
  0x00, 0x00, 0x80, 0xbf, 0x3f, 0x00, 0xfc, 0xf9, 0xcf, 0x1f, 0xc0, 0xff,
  0xe7, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x1f, 0x00, 0xfc, 0xf9,
  0xcf, 0x1f, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xbf,
  0x1f, 0x00, 0xfc, 0xf9, 0xcf, 0x1f, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00,
  0x00, 0x00, 0x00, 0xbe, 0x0f, 0x00, 0xfc, 0xf3, 0xe7, 0x1f, 0xc0, 0xff,
  0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xbc, 0x0f, 0x00, 0xfc, 0xc7,
  0xf1, 0x1f, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xbc,
  0x07, 0x00, 0xf8, 0x0f, 0xf8, 0x0f, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00,
  0x00, 0x00, 0x00, 0xbc, 0x07, 0x00, 0xf8, 0x3f, 0xfe, 0x0f, 0xc0, 0xff,
  0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xb8, 0x03, 0x00, 0xf0, 0xff,
  0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xb0,
  0x01, 0x00, 0xe0, 0xff, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0x01, 0x00,
  0x00, 0x00, 0x00, 0xb0, 0x01, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x80, 0xff,
  0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0xc0, 0xff,
  0xff, 0x01, 0x00, 0xfe, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0,
  0x00, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0xf8, 0xff, 0x0f, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xfe, 0x1f, 0x00, 0x00, 0xe0,
  0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
  0x07, 0x00, 0x00, 0x80, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char plus[] U8X8_PROGMEM = {
  0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0x07, 0xf0, 0x7f,
  0xff, 0x01, 0xc0, 0x7f, 0x7f, 0x00, 0x00, 0x7f, 0x3f, 0x00, 0x00, 0x7e,
  0x1f, 0xf0, 0x07, 0x7c, 0x0f, 0xf0, 0x07, 0x78, 0x07, 0xf0, 0x07, 0x78,
  0x07, 0xf0, 0x07, 0x70, 0x03, 0xf0, 0x07, 0x70, 0x03, 0xf0, 0x07, 0x60,
  0x03, 0xf0, 0x07, 0x60, 0xe1, 0xff, 0xff, 0x43, 0xe1, 0xff, 0xff, 0x43,
  0xe1, 0xff, 0xff, 0x43, 0xe1, 0xff, 0xff, 0x43, 0xe1, 0xff, 0xff, 0x43,
  0xe1, 0xff, 0xff, 0x43, 0xe3, 0xff, 0xff, 0x63, 0x03, 0xf0, 0x07, 0x60,
  0x03, 0xf0, 0x07, 0x60, 0x07, 0xf0, 0x07, 0x70, 0x07, 0xf0, 0x07, 0x78,
  0x0f, 0xf0, 0x07, 0x78, 0x1f, 0xf0, 0x07, 0x7c, 0x3f, 0x00, 0x00, 0x7e,
  0x7f, 0x00, 0x00, 0x7f, 0xff, 0x00, 0xc0, 0x7f, 0xff, 0x03, 0xf0, 0x7f,
  0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f
};

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define melodypin = 7;

//Mario main theme melody
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

int underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};

int underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

int melodyPin = 7;
int song = 0;

// Joystick Catch
int buttonState;                      // the current reading from the input pin
int lastButtonState = LOW;            // the previous reading from the input pin
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long debounceDelay = 60;     // the debounce time; increase if the output flickers

// Delay so many milliseconds without blocking interrupts or threads
void tomDelay(unsigned long ms)
{
  unsigned long currentMillis  = millis();
  unsigned long previousMillis = millis();

  while (currentMillis - previousMillis < ms) {
    currentMillis = millis();
  }
}

// Cacha el tema de display por el joystick
uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8) // VentMex Captura Menu del Joystick sin calibrar
{

  int reading = digitalRead(BUTTON_SELECT);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        tomDelay(300);
        return U8X8_MSG_GPIO_MENU_SELECT;
      }
    }
  }
  lastButtonState = reading;
  int turn = analogRead(JOYSTICK);
 
  if (turn > 560)
  {
    tomDelay(300); // Regresa a Home
    return U8X8_MSG_GPIO_MENU_DOWN;
  }
  if (turn < 480) {
    tomDelay(300); // Regresa a Home
    return U8X8_MSG_GPIO_MENU_UP;
  }
}

// TOM Now Function en segundos
inline float now()
{
  return millis() * 1e-3;
}

void buzz(int targetPin, long frequency, long length) {

  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, 1); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, 0); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }

}

void thDelay(unsigned long ms)
{
  unsigned long currentMillis  = millis();
  unsigned long previousMillis = millis();

  while (currentMillis - previousMillis < ms) {
    currentMillis = millis();
  }
}

void sing(int s) {
  song = s;
  if (song == 2) {
    int size = sizeof(underworld_melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {


      int noteDuration = 1000 / underworld_tempo[thisNote];

      buzz(melodyPin, underworld_melody[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.00;
      thDelay(pauseBetweenNotes);

      // stop the tone playing:
      buzz(melodyPin, 0, noteDuration);

    }

  } else {

    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {

      int noteDuration = 1000 / tempo[thisNote];

      buzz(melodyPin, melody[thisNote], noteDuration);

      int pauseBetweenNotes = noteDuration * 1.00;
      thDelay(pauseBetweenNotes);

      buzz(melodyPin, 0, noteDuration);

    }
  }
}
