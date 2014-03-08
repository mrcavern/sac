#include "serLCD.h"

#define ARDUINO_MODE 1  /* toggle for use of simulator/actual code paths */

/* SAC - automated agriculture System http://sacultivo.com/

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General/ Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Written by Øyvind Kolås pippin@gimp.org in 2013, based around a core of
sensor initialization and reading by by Andres Orencio Ramirez Perez
andy@orencio.org

Rev 2.0
Adapted previous code with new SerLCD Library for using 20x4 displays, with newer functions to control
LCD. Added new Menu's and removed some unused code for clearer comprehension.

In future releases some new sensors will be added in addition to a hardware clock for simplifying all time
functions and readings.

Code adapted by David Cuevas <mr.cavern@gmail.com> with the support and collaboration of Victor Suarez <zerasul@gmail.com>
and Adrian from Artesanos Industriales del Sur.

*/

#define RELAY1_PIN   5
#define RELAY2_PIN   6
#define RELAY3_PIN   4

#define BUTTON_UP_PIN    9
#define BUTTON_ENTER_PIN 8
#define BUTTON_DOWN_PIN 7

#define LOOP_DELAY    500
#define MENU_SPEEDUP  3
#define MENU_TIMEOUT  120

#define SOIL_MOISTURE_POWER_PIN 3

#include "sac.h"
#include <string.h>

enum {
  S_EMPTY=0,
  S_WA,
  S_V,
  S_S,
  S_H,
  S_L,
  S_A,
  S_PU,
  S_LANGUAGE,
  S_ENGLISH,
  S_ENABLED,
  S_DISABLED,
  S_TIME,
  S_SOIL_MOISTURE,
  S_CALIBRATE_MOIST,
  S_DISCONNECTED,
  S_IRRIGATION,
  S_HEATING,
  S_COOLING,
  S_LIGHT,
  S_VENTILATION,
  S_HUMIDIFIER,
  S_ALARM,
  S_RELAY1,
  S_RELAY2,
  S_RELAY3,
  S_AIR_HUMIDITY,
  S_AIR_TEMPERATURE,
  S_IRRIGATION_CYCLE,
  S_LENGTH_SEC,
  S_RETURN_TO,
  S_MAIN_MENU,
  S_SETUP,
  S_RELAYS,
  S_RESET,
  S_ON,
  S_START,
  S_DURATION,
  S_CONFIG,
  S_RANGE,
  S_LOG,
  MIN,
};

typedef struct {
//  int temperature;
  int moisture;
//  int humidity;
} LogEntry;

/* distance between datalog entries in minutes */
#define LOG_INTERVAL 60

static LogEntry datalog[24*60/LOG_INTERVAL];

#define MAX_LANGUAGE 4
typedef struct TranslatedString {
  const char *languages[MAX_LANGUAGE];
} TranslatedString;

TranslatedString string_db[]={
  /* for languages missing translations, english will be used instead */
  {{"","",""}},
  {{"  F.C."",C.C."}},
  {{"E ","E "}},
  {{"SOIL","HSO:"}},
  {{"H ","H "," "}},
  {{"L ","L ","L"}},
  {{"A ","A ","A"}},
  {{"PU ","RIEGO"}},
  {{"LANGUAGE","IDIOMA"}},
  {{"ENGLISH",         "ESPANOL"}},
  {{"<enabled>",       "<activar>"}},
  {{"<disabled>",      "<inhabilitado>"}},
  {{"TIME",            "HORA"}},
  {{"SOIL MOISTURE",   "HUMEDAD SUELO"}},
  {{"CALIBRATE SAT.","CALIBRACION SAT."}},
  {{"OFF",             "APAGADO"}},
  {{"WATERING",      "RIEGO"}},
  {{"HEATING",         "CALEFACCION",}},
  {{"COOLING",         "REFRIGERACION"}},
  {{"LIGHT",           "ILUMINACION"}},
  {{"AIR EXTRACTION",  "EXTRAC.HUMEDAD"}},
  {{"HUMIDIFIER",      "HUMIDIFICACION"}},
  {{"alarm",           "Alarm"}},
  {{"OUTPUT 1",        "SALIDA 1"}},
  {{"OUTPUT 2",        "SALIDA 2"}},
  {{"OUTPUT 3",        "SALIDA 3"}},
  {{"AIR HUMIDITY",    "HUMEDAD AIRE"}},
  {{"TEMPERATURE",     "TEMPERATURA"}},

  {{"WATERING",      "RIEGO"}},
  {{"CICLES (sec):",  "CICLOS (seg):"}},

  {{"RETURN TO",       "VOLVER"}},
  {{"MAIN MENU",       "MENU PRINCIPAL"}},
  {{"Setup...",        "Configuracion.."}},
  {{"OUTPUTS",          "SALIDAS"}},
  {{"Reset",           "RESET"}},
  {{"ON",              "ENCENDIDO"}},
  {{"START",           "COMIENZA"}},
  {{"DURATION",        "DURACION"}},
  {{"CONFIGURATION",   "CONFIGURACION"}},
  {{"Hysteresis: ",  }},
  {{"Log"}},
  {{"MIN:"}},  {{"MIN:"}},
};

int active_language = 0;
#define CFG_MAGIC_VALUE 25

void  message(char *line1, char *line2);

/* global variables, containing this loop iterations sensor readings */
float cached_moisture    = 111;
float cached_humidity    = 11;
float cached_temperature = 23;
int   cached_water_level = 1;

#define MAX_RELAYS 4
#define MAX_LIGHTS 1

#if ARDUINO_MODE


#include <EEPROM.h>
//#include <SerialLCD.h>
#include <SoftwareSerial.h>
// Pins for input/output to lcd
serLCD seriallcd(11);
//SoftwareSerial LCD(11,12); // RX, TX

// Flag to detect when the lcd is initiazed.
int lcd_initialized = 0;

// Some special values for the lcd.
//#define SLCD_BACKLIGHT_OFF 0x80
//#define SLCD_BACKLIGHT_ON  0x81
//#define SLCD_POWER_OFF     0x82
//#define SLCD_POWER_ON      0x83



// Pin for soil moisture sensor (MOISTURE).
// http://www.seeedstudio.com/wiki/Grove_-_Moisture_Sensor
#define MOISTURE_PIN A3
// Define min and max hum for soil (MOISTURE).
#define soil_moisture_MIN 45
#define soil_moisture_MAX 80

// Pin for air humdity and temperature sensor (AHTS).
#define AHTS_PIN A0    

// Pin for water tank nivel sensor (WTS).
#define WTS_PIN A2

#endif

#if ARDUINO_MODE
/* a small subset of commands we are using for driving the LCD */

//#define LCD.setCursor(u,v)     do{x=u+1,y=v+1;LCD.setCursor(u,v);}while(0)
//#define clearScreen()     do{LCD.clear();LCD.setCursor(0,0);}while(0)
//#define LCD_X()         (x-1)
//#define LCD_Y()         (y-1)
//#define LCD.print(str)  do{seriallcd.print(str);x+=strlen(str);}while(0)
//#define LCD.print(no)  do{seriallcd.print((float)no, 0);seriallcd.print(" ");if(no<0)x++;x++;if(no>=10)x++;if(no>=100)x++;if(no>=1000)x++;if(no>=10000)x++;LCD.setCursor(LCD_X(),LCD_Y());}while(0)
//#define clearScreen()()     do{}while(0)
//
//#define LCD.print(str)  do{int i;if (ticks%2){LCD.print(str);}else{for (i=0;i<strlen(str);i++)LCD.print(" ");};}while(0)
//#define LCD.print(no)  do{if (ticks%2){LCD.print(no);}else{if(no>=10)LCD.print(" ");if (no>=100)LCD.print(" ");LCD.print(" ");};}while(0)
//
#define EEPROM_write(pos,val) EEPROM.write(pos,val)
#define EEPROM_read(pos)      EEPROM.read(pos)
//
//
//#else /* nchanterm equivalent codes to the seriallcd library */
//
//#define clearScreen()()     do{nct_clear (term);x=1;y=1;}while(0)
//#define LCD.setCursor(u,v)     do{x=u+10;y=v+5;} while(0);
//#define LCD_X()         (x-10)
//#define LCD_Y()         (y-5)
//#define LCD.print(str)  do{x+=nct_print (term, x, y, str, -1);}while(0)
//#define LCD.print(no)  do{char str[10];sprintf(str, "%.0f ", 1.0*(no));LCD.print(str);LCD.setCursor(LCD_X()-1,LCD_Y());}while(0)
//#define clearScreen()()     do{nct_flush (term);}while(0)
//
//#define LCD.print(str) do{int i;if (ticks%2){LCD.print(str);}else{for (i=0;i<strlen(str);i++)LCD.print(" ");};}while(0)
//#define LCD.print(no)  do{char str[10];sprintf(str, "%.0f ", 1.0*(no));LCD.print(str);LCD.setCursor(LCD_X()-1,LCD_Y())}while(0)

static unsigned char fakeEEPROM[512] = { 0, 0,
                                         0, 0, /* relay1 */
                                         0, 0, /* relay2 */
                                         0, 0, /* relay3 */
                                         0, 1  /* language */
};
#include <stdint.h>
#define EEPROM_write(pos,val) do{fakeEEPROM[pos]=val;}while(0)
#define EEPROM_read(pos)      fakeEEPROM[pos]



#endif


//NUEVA LIBRERIA LCD CON FUNCIONES PREDEFINIDAS
//
#define LCD_SENDCOMMAND(command){seriallcd.print(0xFE); seriallcd.print(command); }
#define LCD_SPECIALCOMMAND(scommand){seriallcd.print(0x7C); seriallcd.print(scommand); }
//-------------------------------------------------------------------------------------------
void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCD_SENDCOMMAND(0x01);

}

//-------------------------------------------------------------------------------------------
void moveCursorRightOne()
{
  //moves the cursor right one space
LCD_SENDCOMMAND(0x14);
}
//-------------------------------------------------------------------------------------------
void moveCursorLeftOne()
{
  //moves the cursor left one space
LCD_SENDCOMMAND(0x10);
}
//-------------------------------------------------------------------------------------------
void scrollRight()
{
  //same as moveCursorRightOne
LCD_SENDCOMMAND(0x1C);
}
//-------------------------------------------------------------------------------------------
void scrollLeft()
{
  //same as moveCursorLeftOne
LCD_SENDCOMMAND(0x18);
}
//-------------------------------------------------------------------------------------------
void turnDisplayOff()
{
  //this tunrs the display off, but leaves the backlight on. 
LCD_SENDCOMMAND(0x08);
}
//-------------------------------------------------------------------------------------------
void turnDisplayOn()
{
  //this turns the display back ON
LCD_SENDCOMMAND(0x0C);
}
//-------------------------------------------------------------------------------------------
void underlineCursorOn()
{
  //turns the underline cursor on
LCD_SENDCOMMAND(0x0E);
}
//-------------------------------------------------------------------------------------------
/*void underlineCursorOff()
{
  //turns the underline cursor off
  LCD.print(0xFE); //command flag
  LCD.print(12); // 0x0C
}*/
//-------------------------------------------------------------------------------------------
void boxCursorOn()
{
  //this turns the box cursor on
LCD_SENDCOMMAND(0x0D);
}
//-------------------------------------------------------------------------------------------
/*void boxCursorOff()
{
  //this turns the box cursor off
  LCD.print(0xFE); //command flag
  LCD.print(12); // 0x0C
}*/
//-------------------------------------------------------------------------------------------
void toggleSplash()
{
  //this toggles the spalsh screenif off send this to turn onif on send this to turn off
LCD_SPECIALCOMMAND(0x09);
}
//-------------------------------------------------------------------------------------------
int backlight(int brightness)// 128 = OFF, 157 = Fully ON, everything inbetween = varied brightnbess 
{
  //this function takes an int between 128-157 and turns the backlight on accordingly
LCD_SPECIALCOMMAND(brightness);
}

//-------------------------------------------------------------------------------------------
void counter()
{
  //this function prints a simple counter that counts to 10
  clearScreen();
  for(int i = 0; i <= 10; i++)
  {
    seriallcd.print("Counter = ");
    seriallcd.print(i, DEC);
    delay(500);
    clearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void tempAndHumidity()
{
  //this function shows how you could read the data from a temerature and humidity 
  //sensro and then print that data to the SerLCD.
  
  //these could be varaibles instead of static numbers 
  float tempF = 77.0; 
  float tempC = 25.0;
  float humidity = 67.0;
  
  clearScreen();
  seriallcd.setCursor(0,1);
  seriallcd.print(" Temp = ");
  seriallcd.print((long)tempF, DEC);
  seriallcd.print("F ");
  seriallcd.print((long)tempC, DEC);
  seriallcd.print("C");
  seriallcd.setCursor(1,0);
  seriallcd.print(" Humidity = ");
  seriallcd.print((long)humidity, DEC); 
  seriallcd.print("%");
  delay(2500);
}
//-------------------------------------------------------------------------------------------
void backlight()
{
  //this function shows the different brightnesses to which the backlight can be set 
  clearScreen();
  for(int i = 128; i < 158; i+=2)// 128-157 are the levels from off to full brightness
  {
    backlight(i);
    delay(100);
    seriallcd.print("Backlight = ");
    seriallcd.print(i, DEC);
    delay(500);
    clearScreen();
  }
}


/* returns a translated string; if no translation found - return the original
 * string.
 */
static const char *translate(int stringno)
{
  if (active_language < 0)
    active_language = 0;
  if (active_language >= MAX_LANGUAGE)
    active_language = MAX_LANGUAGE-1;

  if (string_db[stringno].languages[active_language])
    return string_db[stringno].languages[active_language];
  else
    return string_db[stringno].languages[0];
}

int x = 1, y = 1;


volatile int32_t ticks = 0;
volatile int32_t mticks = 0;
#define MIDNIGHT (60*24)

/* called either manually in our loop or by a timer interrup, once a second.
 *
 */
void update_time_tick (void)
{
  ticks ++;
  if (ticks % 60 == 0)
    mticks ++;
  if (1.0 * ticks >= MIDNIGHT * 60)
    ticks = 0;
  if (1.0 * mticks >= MIDNIGHT)
    mticks = 0;
}

#if ARDUINO_MODE

// Function to read AHTS.
unsigned char read_ahts_dat() {
  unsigned char i = 0;
  unsigned char result=0;
  for(i=0; i< 8; i++) {
    int j = 0;
    while(!digitalRead(AHTS_PIN) && j++ < 500);  // wait for 50us.
    delayMicroseconds(30);

    if(digitalRead(AHTS_PIN))
      result |=(1<<(7-i));
    j = 0;
    while(digitalRead(AHTS_PIN) && j++ < 500);  // wait '1' finish.
  }
  return result;
}

// Function to process AHTS data and read it.
// datas is i/o parameter with [ humdity, temperature ].
void aths_read_data(float *hum_out, float *temp_out) {
  unsigned char ahts_dat[5];
  //unsigned char ahts_in;
  unsigned char i;
  float humdity, temperature;
  // start condition
  // 1. pull-down i/o pin from 18ms
  digitalWrite(AHTS_PIN,LOW);
  delay(18);

  digitalWrite(AHTS_PIN,HIGH);
  delayMicroseconds(40);

  pinMode(AHTS_PIN,INPUT);

  while(digitalRead(AHTS_PIN)){
    delayMicroseconds(15);
    pinMode(AHTS_PIN,OUTPUT);
    digitalWrite(AHTS_PIN,HIGH);

    return;
  }
  delayMicroseconds(80);

  while(!digitalRead(AHTS_PIN)){
    delayMicroseconds(15);
    pinMode(AHTS_PIN,OUTPUT);
    digitalWrite(AHTS_PIN,HIGH);

    return;
  }
  delayMicroseconds(80);
  // now ready for data reception
  for (i=0; i<5; i++)
    ahts_dat[i] = read_ahts_dat();

  pinMode(AHTS_PIN,OUTPUT);
  digitalWrite(AHTS_PIN,HIGH);

  unsigned char ahts_check_sum = ahts_dat[0]+ahts_dat[1]+ahts_dat[2]+ahts_dat[3];
  // check check_sum

  if(ahts_dat[4]!= ahts_check_sum)
  {
    message ("AHTS", "checksum error");
    return;
  }
  humdity=((float)(ahts_dat[0]*256+ahts_dat[1]))/10.0;

  if (ahts_dat[2] & 0x80)
    temperature=-((float)((ahts_dat[2]&0x7f)*256+ahts_dat[3]))/10.0;
  else
    temperature=((float)(ahts_dat[2]*256+ahts_dat[3]))/10.0;

  if (hum_out) *hum_out = humdity;
  if (temp_out) *temp_out = temperature;
}

void setup_arduino (void)
{
  // Initialize setup for AHTS.
  pinMode(AHTS_PIN, OUTPUT);
  // Initialize setup for WTS.
  pinMode(WTS_PIN, INPUT);
  // Initialize setup for relays.
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  
  pinMode(SOIL_MOISTURE_POWER_PIN, OUTPUT);

  pinMode(BUTTON_UP_PIN, INPUT);
  pinMode(BUTTON_ENTER_PIN, INPUT);
  pinMode(BUTTON_DOWN_PIN, INPUT);
 
  seriallcd.display();
}

//#else
//#include <string.h> // strcmp
//#include <stdio.h>  /* sprintf */
//#include "nchanterm.h"
//#include <unistd.h>
//
//#define delay(foo)              usleep(foo*1000)
//#define delayMicroseconds(foo)  usleep(foo)
//
//Nchanterm *term;
//int main_quit = 0;
//#define nchant_delay   10  /* timeout for button presses */

#endif

float moisture_read ();

enum {
  DISCONNECTED,
  IRRIGATION,
  HUMIDIFIER,
  VENTILATION,
  HEATING,
  COOLING,
    LIGHT,
  ALARM,
  ON
};

int roles[] = {
  S_DISCONNECTED,
  S_IRRIGATION,
  S_HUMIDIFIER,
  S_VENTILATION,
  S_HEATING,
  S_COOLING,
  S_LIGHT,
 
   
/*  S_ALARM,*/
/*  S_ON*/
};
#define MOISTURE_SMOOTHING  85

float pump_duty_cycle      = 50;
float pump_cycle_length    = 0;
float temperature_target      = 0;
float temperature_range       = 1;
float moisture_target      = 0;
float moisture_range       = 10;
float moisture_calib       = 500;

float humidity_target = 0;
float humidity_range = 10;

int   relay1_role = 0;
int   relay2_role = 0;
int   relay3_role = 0;
int   is_editing  = 0;
int   is_print    = 0;

Relay *find_relay (int role);

Relay relay[MAX_RELAYS]={{RELAY1_PIN},{RELAY2_PIN},{RELAY3_PIN}};

int lights_start[MAX_LIGHTS] = {0,};
int lights_duration[MAX_LIGHTS]   = {0,};

/* This enum specifies storage locations in EEPROM memory of the arduino if
 * these change; existing on-board configurations are not valid; please
 * change the magic if that happens to ensure old invalid settings are not
 * used.
 */
enum {
  CFG_MAGIC = 0,
  CFG_RELAY1,
  CFG_RELAY2,
  CFG_RELAY3,
  CFG_LANGUAGE,
  CFG_MOISTURE = 10,
  CFG_MOISTURE_MOISTUREOOTH,
  CFG_MOISTURE_CALIBRATION,
  CFG_IRRIGATION_CYCLE_LENGTH,
  CFG_IRRIGATION_DUTY,
  CFG_HUMIDITY = 20,
  CFG_TEMPERATURE = 30,
  CFG_LIGHT_A1S = 40, CFG_LIGHT_A1E,
};

/* XXX: fixme: add default */
ConfigItem config[]={
  {&relay[0].role,      ROLE,           CFG_RELAY1,                  IRRIGATION},
  {&relay[1].role,      ROLE,           CFG_RELAY2,                  ALARM},
  {&relay[2].role,      ROLE,           CFG_RELAY3,                  LIGHT},
  {&moisture_target,    NUMBER,         CFG_MOISTURE,                42},
  {&humidity_target,    NUMBER,         CFG_HUMIDITY,                78},
  {&temperature_target, NUMBER,         CFG_TEMPERATURE,             23},
  {&moisture_calib,     SOIL_CALIBRATE, CFG_MOISTURE_CALIBRATION,    500.0},
  {&pump_cycle_length,  NUMBER,         CFG_IRRIGATION_CYCLE_LENGTH, 4},
  {&lights_start[0],    TIME,           CFG_LIGHT_A1S,               0},
  {&lights_duration[0], TIME,           CFG_LIGHT_A1E,               0},
  {&active_language,    LANGUAGE,       CFG_LANGUAGE,                0},
  {NULL,}
};

MenuItem relay_menu[] = {
  {S_RELAY1, S_EMPTY, ROLE, &relay[0].role},
  {S_RELAY2, S_EMPTY, ROLE, &relay[1].role},
  {S_RELAY3, S_EMPTY, ROLE, &relay[2].role},
  {S_RETURN_TO, S_MAIN_MENU,       BACK, 0},
  {0}
};

int minutes = 0;

#define MAX_MENU_DEPTH 4

MenuItem *prev_menu[MAX_MENU_DEPTH];
int       prev_menu_active[MAX_MENU_DEPTH];
int       menu_depth = 0;

MenuItem log_menu[] =
{
  {S_LOG, 0, LOG, },
  {S_RETURN_TO, S_MAIN_MENU,       BACK, 0},
  {0}
};

MenuItem main_menu[] = {
  {S_A, 0,                           STATUS,              (void*)0},
  {S_RELAYS, S_CONFIG,               SUBMENU,             &relay_menu},
  {S_SOIL_MOISTURE, 0,               NUMBER,              &moisture_target},
  {S_IRRIGATION_CYCLE, S_LENGTH_SEC, NUMBER,              &pump_cycle_length},
  {S_CALIBRATE_MOIST, 0,             SOIL_CALIBRATE,      &moisture_calib}, 
  {S_AIR_HUMIDITY, 0,                NUMBER,              &humidity_target},
  {S_AIR_TEMPERATURE, 0,             NUMBER,              &temperature_target},
  {S_LIGHT,  S_START,                TIME,                &lights_start[0]},
  {S_LIGHT,  S_DURATION,             TIME,                &lights_duration[0]}, 
  {S_TIME, 0,                        TIME,                &minutes},
  {S_LANGUAGE, 0,                    LANGUAGE,            &active_language},  
/*{S_RESET, S_CONFIG,                RESET_SETTINGS,      0},*/
/*{S_LOG, 0,                         SUBMENU,             &log_menu},*/
  {0}
};

enum {
  IDLE = 0,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_ENTER,
  TIMEOUT
};

MenuItem * menu = &main_menu[0];
int menu_active = 0;

#define N_ELEM(array) (sizeof(array)/sizeof(array[0]))
#define menu_c    N_ELEM(menu)
#define roles_c   N_ELEM(roles)

void enter_menu (MenuItem *new_menu)
{
  prev_menu[menu_depth] = menu;
  prev_menu_active[menu_depth] = menu_active;
  menu_depth++;
  menu = new_menu;
  menu_active = 0;
  clearScreen();
}

void return_home (void)
{
  if (menu == main_menu && menu_active == 0)
    return;
  clearScreen();
  menu = main_menu;
  menu_active = 0;
}

void go_back (void)
{
  menu_depth--;
  menu = prev_menu[menu_depth];
  menu_active = prev_menu_active[menu_depth];
  clearScreen();
}

void store_settings (ConfigItem *menu)
{
  int i;
  for (i = 0; menu[i].data; i++)
    {
      int ival = 0;
      int pos = menu[i].eeprom_pos;
      switch (menu[i].type)
        {
          case ONOFF:
          case TIME:
          case LANGUAGE:
          case ROLE:
            {
              int *val = (int*)menu[i].data;
              ival = *val;
            }
            break;
          case SOIL_CALIBRATE:
          case NUMBER:
            {
              float *sval = (float*)menu[i].data;
              ival = *sval;
            }
            break;
            break;
          default:
            break;
        }

      EEPROM_write(pos*2, ival % 256);
      EEPROM_write(pos*2+1, ival / 256);
    }
  EEPROM_write(CFG_MAGIC, CFG_MAGIC_VALUE);
}

int load_settings (ConfigItem *menu)
{
  int i;
  if (EEPROM_read(CFG_MAGIC) != CFG_MAGIC_VALUE)
    return -1;
  for (i = 0; menu[i].data; i++)
    {
        {
          int pos = menu[i].eeprom_pos;
          int ival;
          ival = EEPROM_read(pos*2) +
          EEPROM_read(pos*2+1) * 256;

          switch (menu[i].type)
            {
              case SOIL_CALIBRATE:
              case NUMBER:
                {
                  float *val = (float*)menu[i].data;
                  *val = ival;
                }
                break;
              case TIME:
              case ROLE:
              case LANGUAGE:
                {
                  int *val = (int*)menu[i].data;
                  *val = ival;
                }
                break;
            }
        }
    }
    //message ("Loaded settings.", "");
    return 0;
}

void reset_settings (ConfigItem *menu)
{
  int i;
  for (i = 0; menu[i].data; i++)
    {
       if (menu[i].type == NUMBER ||
           menu[i].type == SOIL_CALIBRATE)
         {
           float *sval = (float*)menu[i].data;
           *sval = menu[i].default_value;
         }
       else
         {
           int *ival = (int*)menu[i].data;
           *ival = menu[i].default_value;
         }
    }
}

#if ARDUINO_MODE

/* This key debouncer relies on being called once per loop iteration,
 * it takes the input pin and a pointer to an integer to keep track of state.
 */
static int debounce_key (int pin, int *state)
{
  int pressed = digitalRead(pin);
  int ret = 0;
  if (pressed && (*state) == 0)
    {
      ret = 1;
      *state += pressed;
    }
  else
    *state = 0;

  if (*state > 80) /* start repeating press once per cycle, after 5 cycles */
    ret=1;
   
  return ret;
}

int debounce_up (void)
{
  static int state = 0;
  debounce_key (BUTTON_UP_PIN, &state);
}

int debounce_down (void)
{
  static int state = 0;
  debounce_key (BUTTON_DOWN_PIN, &state);
}

int debounce_enter (void)
{
  static int state = 0;
  debounce_key (BUTTON_ENTER_PIN, &state);
}

int get_event (void)
{
  if (debounce_up ())
    return BUTTON_UP;
  if (debounce_down ())
    return BUTTON_DOWN;
  if (debounce_enter ())
    return BUTTON_ENTER;
  return IDLE;
}
#else
int get_event (void)
{
  const char *event = nct_get_event (term, nchant_delay, NULL, NULL);

  if (!strcmp (event, "a"))
    { cached_moisture +=50; return IDLE; }
  else if (!strcmp (event, "z"))
    { cached_moisture -=50; return IDLE; }
  if (!strcmp (event, "s"))
    { cached_humidity +=2; return IDLE; }
  else if (!strcmp (event, "x"))
    { cached_humidity -=2; return IDLE; }
  if (!strcmp (event, "d"))
    { cached_temperature ++; return IDLE; }
  else if (!strcmp (event, "c"))
    { cached_temperature --; return IDLE; }
  else if (!strcmp (event, "f"))
    { cached_water_level = !cached_water_level; return IDLE; }
  else if (!strcmp (event, "v"))
    { ticks += 69 * 4 * 4; return IDLE; }


  if (!strcmp (event, "control-c")||
      !strcmp (event, "esc"))
    {
      main_quit = 1;
    }
  else if (!strcmp (event, "size-changed"))
    nct_set_size (term, nct_sys_terminal_width (),
                  nct_sys_terminal_height ());
  else if (!strcmp (event, "control-l"))
    {
      event = "forced redraw";
      nct_reflush (term);
    }
  else if (!strcmp (event, "up"))
    return BUTTON_UP;
  else if (!strcmp (event, "down"))
    return BUTTON_DOWN;
  else if (!strcmp (event, "return"))
    return BUTTON_ENTER;
  return IDLE;
}
#endif

/****/

int message_ttl = 0;

void message(char *line1, char *line2)
{

clearScreen();
//  LCD.setCursor(0,0);
seriallcd.setCursor(0,1);
//  LCD.print(line1);
seriallcd.print(line1);
//  LCD.setCursor(0,1);
seriallcd.setCursor(1,0);
  if (line2)
    seriallcd.print(line2);
  clearScreen();
  message_ttl = 3;
}

void print_time (int minutes_since_midnight);

int offset = 0;
int get_minutes_since_midnight (void)
{
  int32_t minutes = mticks;
  minutes += offset;
  while (minutes < 0)
    minutes += MIDNIGHT;
  while (minutes > MIDNIGHT)
    minutes -= MIDNIGHT;
  return minutes;
}

int get_seconds_since_midnight (void)
{
  return ticks;
}

void draw_status (int time,
                  int moisture,
                  int temperature,
                  int humidity)
{
  seriallcd.setCursor(6,0);
/*LCD.print(translate(S_A));*/
  seriallcd.print(temperature);
  seriallcd.print("C");
   
//  LCD.setCursor(0, 1);
  seriallcd.print(humidity);
  seriallcd.print("% HR");
  seriallcd.setCursor(0, 1);
  seriallcd.print(translate(S_S));
  seriallcd.print(moisture_target);
  seriallcd.print("% ");
  seriallcd.setCursor(8,1);
  seriallcd.print(translate(MIN));
  seriallcd.setCursor(16,1);
  seriallcd.print("[");
  seriallcd.setCursor(18,1);
  seriallcd.print(moisture);
  seriallcd.setCursor(19,1);
  seriallcd.print("]");
   

 
 
  {
    Relay *light = find_relay (LIGHT);
    if (light)
    {
      seriallcd.setCursor(6, 1);
      if (light->state == RELAY_ON)
        seriallcd.print(translate(S_L));
    }
  }

  {
    Relay *heating = find_relay (HEATING);
    Relay *cooling = find_relay (COOLING);
    if (heating)
    {
      seriallcd.setCursor(8, 1);
      if (heating->state == RELAY_ON)
        seriallcd.print("+");
      else
        {
          if (cooling && cooling->state == RELAY_ON)
            seriallcd.print("-");
          else
            seriallcd.print(" ");
        }
    }
  }

  {
    Relay *ventilation = find_relay (VENTILATION);
    Relay *humidifier = find_relay (HUMIDIFIER);
    if (ventilation && ventilation->state == RELAY_ON)
      {
        seriallcd.setCursor(3, 1);
        seriallcd.print(translate(S_V));
      }
    else if (humidifier && humidifier->state == RELAY_ON)
      {
        seriallcd.setCursor(3, 1);
        seriallcd.print(translate(S_H));
      }
  }
 


  {
    Relay *water = find_relay (IRRIGATION);
    if (water)
    {
      seriallcd.setCursor(0,1);
      if (cached_water_level)
        seriallcd.print("     "); /* LCD.print(translate(S_WA)); */
      else
        seriallcd.print(translate(S_WA));

      seriallcd.setCursor(0,1);
      switch (water->state)
        {
          case RELAY_ON:
            seriallcd.print(translate(S_PU));
            break;
          case RELAY_OFF:
            seriallcd.print("  ");
            break;
          case RELAY_WAITING:
            seriallcd.print(translate(S_PU));
            break;
        }
    }
  }

  seriallcd.setCursor(0,0);
  {
    print_time (time);
  }
}

int logno = 0;
void draw_log (void)
{
  seriallcd.setCursor(0,0);
  print_time(logno*LOG_INTERVAL);
  seriallcd.print(" ");
  seriallcd.print(datalog[logno].moisture);
  seriallcd.print("%  ");

  seriallcd.setCursor(0,1);
  print_time((logno+1)*LOG_INTERVAL);
  seriallcd.print(" ");
  if (logno+1 >= 24 * 60 / LOG_INTERVAL)
  seriallcd.print(datalog[0].moisture);
  else
  seriallcd.print(datalog[logno+1].moisture);
  seriallcd.print("%  ");

}


float moisture_read()
{
  static float kept = 0;
  float soil_moisture = cached_moisture * 100 / moisture_calib;
  if (soil_moisture > 100)
    {
      soil_moisture = 100;
    }
  kept = (kept * MOISTURE_SMOOTHING/100.0);
  kept = kept + soil_moisture * (100-MOISTURE_SMOOTHING)/100.0;
  return kept;
}

void end_editing (void)
{
  is_editing = 0;
  store_settings (config);
}

void print_time (int minutes_since_midnight)
{ 
  while (minutes_since_midnight < 0)
    minutes_since_midnight += MIDNIGHT;
  {
    int minutes = minutes_since_midnight;
    int hours = minutes/60;
    minutes %= 60;
    while (hours > 23)
      hours -= 24;
    if (is_editing == 1)
    {
      if (hours < 10)
      seriallcd.print(" ");
      seriallcd.print(hours);
      seriallcd.print(":");
      if (minutes < 10)
      seriallcd.print("0");
      seriallcd.print(minutes);
    }
    else if (is_editing == 2)
    {
      if (hours < 10)
        seriallcd.print(" ");
      seriallcd.print(hours);
      seriallcd.print(":");
      if (minutes < 10)
        seriallcd.print("0");
      seriallcd.print(minutes);
    }
    else
    {
      if (hours < 10)
        seriallcd.print(" ");
      seriallcd.print(hours);
      if (menu == main_menu)
        seriallcd.print(":");
      else
        seriallcd.print(":");
      if (minutes < 10)
        seriallcd.print("0");
      seriallcd.print(minutes);
    }
  }
}

void draw_debug (void)
{
#if ARDUINO_MODE
#else
  int u = -8; int v = 4; int pos;

  for (pos = 0; pos < 300; pos ++)
    {
      setCursor(u,v);
      seriallcd.print(pos);
      seriallcd.print(":");
      seriallcd.print(EEPROM_read(pos));
      u += 7;
      if (u > 70)
        {
          u = -8;
          v++;
        }
    }

  setCursor(-6,-2);
  seriallcd.print("Relay 1: ");
  if (relay[0].state == RELAY_ON)
    seriallcd.print("ON  ");
  else if (relay[0].state == RELAY_WAITING)
    seriallcd.print("WAIT");
  else
    seriallcd.print("OFF ");

  setCursor(9,-2);
  seriallcd.print("Relay 2: ");
  if (relay[1].state == RELAY_ON)
    seriallcd.print("ON ");
  else
    seriallcd.print("OFF");
  setCursor(24,-2);
  seriallcd.print("Relay 3: ");
  if (relay[2].state == RELAY_ON)
    seriallcd.print("ON ");
  else
    seriallcd.print("OFF");
#endif
}

void draw_ui (void)
{
  MenuItem *mi = &menu[menu_active];

  if (message_ttl)
    {
      message_ttl--;
      if (message_ttl>0)
        return;
     clearScreen();
    }
 
#if ARDUINO_MODE
#else
  setCursor(16,0);
  seriallcd.print("]");
  setCursor(16,1);
  seriallcd.print("]");

  setCursor(-1,0);
  seriallcd.print("[");
  setCursor(-1,1);
  seriallcd.print("[");
#endif
  seriallcd.setCursor(0,0);

  switch (mi->type)
  {
    case STATUS:
      draw_status (get_minutes_since_midnight (), moisture_read(), cached_temperature, cached_humidity);
      draw_debug ();
      clearScreen();
      return;
   
    case LOG:
      draw_log ();
      draw_debug ();
      clearScreen();
      return;

    default:

      seriallcd.print (translate(mi->label));
      seriallcd.setCursor(0,1);
      if (mi->label2)
        seriallcd.print (translate(mi->label2));

      break;
  }

  if (mi->data || mi->type == UPTIME)
    switch (mi->type)
    {
      case TIME:
        /* special casing for showing the time when not
         * editing it
         */
        if (!is_editing)
          minutes = get_minutes_since_midnight ();

        {
          int *time = ((int*)(mi->data));
          int foo = *time;
          print_time (foo);
        }
        break;
      case UPTIME:
        {
          seriallcd.setCursor(0,1);
          seriallcd.print((float)ticks);
        }
        break;
      case TEXT:
        seriallcd.print((char *)mi->data);
        break;
      case NUMBER:
        if (is_editing)
        {
          int ones, tens;
          tens = (*(float*)(mi->data));
          ones = tens % 10;
          tens -= ones;
          tens /= 10;

          if (is_editing == 1)
          {
            seriallcd.print(tens);
            seriallcd.print(ones);
          }
          else if (is_editing == 2)
          {
            seriallcd.print(tens);
            seriallcd.print(ones);
          }
          else
            seriallcd.print( (*(float*)(mi->data)) );
        }
        else
          seriallcd.print( (*(float*)(mi->data)) );
        break;

      case SOIL_CALIBRATE:
        if (is_editing)
          seriallcd.print( (*(float*)(mi->data)) );
        else
          seriallcd.print( (*(float*)(mi->data)) );
        /*seriallcd.print("s ");*/
        seriallcd.print(" -ENTER- ");
        seriallcd.print(cached_moisture);
        seriallcd.print("   ");
        /*seriallcd.print("v ");*/
        /*seriallcd.print(moisture_read());*/
       
        break;

      case ROLE:
        {
          int role = (*((int*)(mi->data)));
          seriallcd.print("[");
          if (is_editing)
            seriallcd.print(translate(roles[role]));
          else
            seriallcd.print(translate(roles[role]));
          seriallcd.print("]");
        }
        break;

      case LANGUAGE:
        {
          seriallcd.print("[");
          if (is_editing)
            seriallcd.print(translate(S_ENGLISH));
          else
            seriallcd.print(translate(S_ENGLISH));
          seriallcd.print("]");
        }
        break;

      case ONOFF:
        if(*((int*)(mi->data)))
          seriallcd.print(translate(S_ENABLED));
        else
          seriallcd.print(translate(S_DISABLED));
        break;

      default:
        break;
    }

  draw_debug ();

  clearScreen();
}

void reset_lights (void)
{
  int i;
  for (i = 0; i < MAX_LIGHTS; i++)
  {
    lights_start[i] = 0;
    lights_duration [i] = 0;
  }
}

void menu_enter (void)
{
  MenuItem *mi = &menu[menu_active];

  if (mi->type == STATUS)
    return;
  if (mi->type == SUBMENU)
  {
    enter_menu ((MenuItem*)mi->data);
    return;
  }
  if (mi->type == BACK)
  {
    go_back ();
    return;
  }

  if (mi->type == LIGHTS_RESET)
  {
    reset_lights ();
    message ("Reset lighting", "schedule.");
    end_editing ();
  }
  else if (mi->type == STORE_SETTINGS)
  {
    store_settings (config);
    message ("Settings stored.", "");
  }
  else if (mi->type == LOAD_SETTINGS)
  {
    load_settings (config);
  }
  else if (mi->type == RESET_SETTINGS)
  {
    reset_settings (config);
    end_editing ();
    message ("Factory defaults", "");
  }
  else if (mi->type == ONOFF)
  {
    int *val = (int*)mi->data;
    *val = !*val;
    end_editing ();
  }
  else
  {
    is_editing = 1;
  }
}

void editing_handle_events (int event)
{
  MenuItem *mi = &menu[menu_active];

  if (!(mi->type == NUMBER ||
        mi->type == ROLE ||
        mi->type == LANGUAGE ||
        mi->type == TIME ||
        mi->type == LOG ||
        mi->type == SOIL_CALIBRATE))
  {
    end_editing ();
    return;
  }

  if ((mi->type == NUMBER ||
       mi->type == TIME) && event == BUTTON_ENTER)
  {
    is_editing++;
    if (is_editing>=3)
    {
      if (mi->data == &minutes)
      {
        /* store back edited tim */
        offset = (minutes - (get_minutes_since_midnight () - offset));
      }
      end_editing();
      return;
    }
  }
  else if (event == BUTTON_ENTER)
  {
    end_editing();
  }

  if (mi->data || mi->type == LOG)
    switch (mi->type)
    {
      case ROLE:
        {
        int *val = (int*)mi->data;
        switch (event)
          {
            case BUTTON_UP:
              (*val) -= 1;
              if (*val < 0)
                *val = roles_c - 1;
              clearScreen();
              break;
            case BUTTON_DOWN:
              (*val) += 1;
              if ((*val) >= roles_c)
                *val = 0;
              clearScreen();
              break;
            case IDLE:
            case TIMEOUT:
            case BUTTON_ENTER:
              break;
          }
        }
        break;

      case LOG:
        {
        switch (event)
          {
            case BUTTON_UP:
              logno--;
              if (logno<0)
                logno = (24*60/LOG_INTERVAL)-1;
              clearScreen();
              break;
            case BUTTON_DOWN:
              logno++;
              if (logno >= 24 * 60 / LOG_INTERVAL)
                logno = 0;
              clearScreen();
              break;
            case IDLE:
            case TIMEOUT:
            case BUTTON_ENTER:
              break;
          }
        }
        break;

      case LANGUAGE:
        {
        int *val = (int*)mi->data;
        switch (event)
          {
            case BUTTON_UP:
              (*val) -= 1;
              if (*val < 0)
                *val = roles_c - 1;
              clearScreen();
              break;
            case BUTTON_DOWN:
              (*val) += 1;
              if ((*val) >= MAX_LANGUAGE)
                *val = 0;
              clearScreen();
              break;
            case IDLE:
            case TIMEOUT:
            case BUTTON_ENTER:
              break;
          }
        }
        break;

      case NUMBER:
        {
        float *val = (float*)mi->data;
        switch (event)
          {
            case BUTTON_UP:
              if (is_editing == 1)
                (*val) += 10;
              else
                (*val) += 1;
              if (*val > 100)
                *val = 100;
              break;
            case BUTTON_DOWN:
              if (is_editing == 1)
                (*val) -= 10;
              else
                (*val) -= 1;

              if (*val < 0)
                *val = 0;
              break;
            case IDLE:
            case TIMEOUT:
            case BUTTON_ENTER:
              break;
          }
        break;
        }

      case TIME:
        {
        int *val = (int*)mi->data;
        switch (event)
          {
            case BUTTON_UP:
              if (is_editing == 1)
                (*val) += 60;
              else
                (*val) += 1;
               
               if (*val > MIDNIGHT)
                 *val = *val - MIDNIGHT;
              break;
            case BUTTON_DOWN:
              if (is_editing == 1)
                (*val) -= 60;
              else
                (*val) -= 1;

              if (*val < 0)
                *val = MIDNIGHT - *val;
              break;
            case IDLE:
            case TIMEOUT:
            case BUTTON_ENTER:
              break;
          }
        break;
        }

      case SOIL_CALIBRATE:
        moisture_calib = cached_moisture;
        is_editing = 0;
      case IDLE:
        break;
      default:
        break;
    }
}

void handle_events (void)
{
  int event = get_event ();
  static int idle_count = 0;

  if (is_editing)
    {
      editing_handle_events (event);
      return;
    }

  if (event != IDLE)
    idle_count = 0;
 
  switch (event)
    {
      case BUTTON_ENTER:
#if ARDUINO_MODE
        seriallcd.print("");
#endif
        menu_enter ();
        break;
      case BUTTON_UP:
        menu_active --;
        if (menu_active < 0)
          while (menu[menu_active+1].label)
            menu_active++;
#if ARDUINO_MODE
        seriallcd.print("");
#endif
        clearScreen();
        break;
      case BUTTON_DOWN:
        menu_active ++;
        if (menu[menu_active].label==0)
          menu_active = 0;
#if ARDUINO_MODE
        seriallcd.print("");
#endif
        clearScreen();
        break;
      case IDLE:
        idle_count ++;
        if (idle_count * 1.0 < MENU_TIMEOUT)
          break;

        if (is_editing)
          break; /* do not jump back to start when editing */
        /* fallthrough */
      case TIMEOUT:
#if ARDUINO_MODE
        seriallcd.print("OFF");
#endif
        return_home ();
        break;
      default:
        break;
    }
}

Relay *find_relay (int role)
{
  Relay *ret = (Relay*)NULL;
  int i;
  for (i = 0; i < MAX_RELAYS; i++)
    if (relay[i].role == role)
      return &relay[i];
  return ret;
}

void relay_on (Relay *relay)
{
  if (relay->state == RELAY_ON)
    return;
  relay->state = RELAY_ON;

#if ARDUINO_MODE
  if (relay->gpio_pin)
    digitalWrite(relay->gpio_pin, HIGH);
#endif
}

void relay_wait (Relay *relay)
{
  if (relay->state == RELAY_WAITING)
    return;
  relay->state = RELAY_WAITING;
#if ARDUINO_MODE
  digitalWrite(relay->gpio_pin, LOW);
#endif
}

void relay_off (Relay *relay)
{
  if (relay->state == RELAY_OFF)
    return;
  relay->state = RELAY_OFF;
#if ARDUINO_MODE
  digitalWrite(relay->gpio_pin, LOW);
#endif
}

int elapsed_minutes (int time1, int time2)
{
  int32_t diff;
  if (time1>time2)
    {
      int32_t temp = time1;
      time1 = time2;
      time2 = temp;
    }
  diff = time2 - time1;
  /* we assume time differences larger than 1h.. to be due to wrap around,
   * thus we normalize the time
   */
  if (diff > 60 * 60)
    diff = MIDNIGHT*60-diff;

  return time2 - time1;
}

void update_relay_state (void)
{
  Relay *water       = find_relay (IRRIGATION);
  Relay *light       = find_relay (LIGHT);
  Relay *ventilation = find_relay (VENTILATION);
  Relay *humidifier  = find_relay (HUMIDIFIER);
  Relay *alarm       = find_relay (ALARM);
  Relay *heating     = find_relay (HEATING);
  Relay *cooling     = find_relay (COOLING);

  static int last_water_event = 0; /* time of day we last switch water
                                      on or off for cycling */
  int alarmed = 0;
  /* This is the core logic of SAC */
  int i;
  for (i = 0; i < MAX_RELAYS; i++)
    if (relay[i].role == DISCONNECTED)
      relay_off (&relay[i]);

  if (cooling)
    {
      int   is_cooling = cooling->state != RELAY_OFF;

      if (is_cooling)
        {
          if (cached_temperature < temperature_target - temperature_range/2)
            relay_off (cooling);
        }
      else
        {
          if (cached_temperature > temperature_target + temperature_range/2)
            relay_on (cooling);
        }
    }

  if (heating)
    {
      int   is_heating = heating->state != RELAY_OFF;

      if (is_heating)
        {
          if (cached_temperature > temperature_target + temperature_range/2)
            relay_off (heating);
        }
      else
        {
          if (cached_temperature < temperature_target - temperature_range/2)
            relay_on (heating);
        }
    }



  if (ventilation)
    {
      int   is_venting = ventilation->state != RELAY_OFF;

      if (is_venting)
        {
          if (cached_humidity < humidity_target - humidity_range/2)
            relay_off (ventilation);
        }
      else
        {
          if (cached_humidity > humidity_target + humidity_range/2)
            relay_on (ventilation);
        }
    }

  if (humidifier)
    {
      int   is_humidifing  = humidifier->state != RELAY_OFF;

      if (is_humidifing)
        {
          if (cached_humidity > humidity_target + humidity_range/2)
            relay_off (humidifier);
        }
      else
        {
          if (cached_humidity < humidity_target - humidity_range/2)
            relay_on (humidifier);
        }
    }

  if (light)
    {
      int i;
      int found = 0;
      int minutes = get_minutes_since_midnight ();

      for (i = 0; i < MAX_LIGHTS; i++)
        {
          int start;
          int end;
          start = lights_start[i];
          end   = start + lights_duration [i];

          while (end > 24 * 60)
            end -= 24 * 60;

          if (end < start)
            {
              /* time period crossing midnight */
              if ((minutes >= start && minutes < MIDNIGHT) ||
                  minutes < end)
                found = 1;
            }
          else
            {
              if (minutes >= start && minutes < end)
                found = 1;
            }

        }
      /* check if current time is inside
       * the timespan
       */
      if (found)
        relay_on (light);
      else
        relay_off (light);
    }

  if (water && !cached_water_level)
    {
      relay_off (water);
      alarmed ++;
    }
  else if (water)
    {
      float moisture = moisture_read ();
      int   is_watering = water->state != RELAY_OFF;

      if (is_watering)
        {
          if (water->state == RELAY_ON)
            {
                if (elapsed_minutes (get_seconds_since_midnight (),
                                  last_water_event) >
                    (pump_cycle_length * pump_duty_cycle / 100))
                  {
                    relay_wait (water);
                    last_water_event = get_seconds_since_midnight ();
                  }
            }
          else
            {
                if (elapsed_minutes (get_seconds_since_midnight (),
                                  last_water_event) >
                    (((100-pump_duty_cycle) * pump_cycle_length) / 100))
                  {
                    relay_on (water);
                    last_water_event = get_seconds_since_midnight ();
                  }
            }

          if (moisture > moisture_target + moisture_range/2)
            relay_off (water);
        }
      else
        {
          if (moisture < moisture_target - moisture_range/2)
            relay_on (water);
          last_water_event = get_minutes_since_midnight ();
        }
    }
  if (alarm)
    {
      if (alarmed)
        relay_on (alarm);
      else
        relay_off (alarm);
    }

#if ARDUINO_MODE
#else
#endif
  for (i = 0; i < MAX_RELAYS; i++)
    {
      if (relay[i].role == ON)
        relay_on (&relay[i]);
      else if (relay[i].role == DISCONNECTED)
        relay_off (&relay[i]);
    }
}

void read_sensors (void);

void store_log (void)
{
  /* we continously update the "current" entry */
  int logno = get_minutes_since_midnight () / LOG_INTERVAL;
  if ((logno < 0) ||
      (logno >= (24*60/LOG_INTERVAL)))
    logno = 0;
//  datalog[logno].temperature = cached_temperature;
//  datalog[logno].humidity = cached_humidity;
//  datalog[logno].moisture = moisture_read();//cached_moisture * 100 / moisture_calib;
}

void loop (void)
{
#if ARDUINO_MODE
#else
  update_time_tick ();
#endif
 
  read_sensors ();
  store_log ();
  update_relay_state ();

  handle_events (); /* handle user events, if any */


  draw_ui ();       /* draw ui in current state*/

  /* XXX
  if (menu == main_menu && menu_active == 0)
    delay (LOOP_DELAY);
  else
   */ {
      int i;
      for (i = 0; i < MENU_SPEEDUP; i++)
        {
          delay (LOOP_DELAY/MENU_SPEEDUP);
          handle_events (); /* handle user events, if any */
          draw_ui ();       /* draw ui in current state*/
        }
    }
}

void setup (void)
{
 
  reset_settings (config);
  if (load_settings (config) != 0)
    {
      menu_active = 10; /* XXX: should be language selection!! */
      is_editing = 1;
    }
#if ARDUINO_MODE
  setup_arduino ();
#endif
}

#if ARDUINO_MODE

int readingno=0;

void read_sensors (void)
{
  float air_data[2];

  readingno++;
 
  if (readingno % 4 == 0)
    {
      aths_read_data (&cached_humidity, &cached_temperature);
    }

  cached_water_level = (digitalRead(WTS_PIN) == LOW);

  digitalWrite(SOIL_MOISTURE_POWER_PIN, HIGH);
  cached_moisture = analogRead(MOISTURE_PIN);
  digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW);
}

#else

void read_sensors (void)
{
  Relay *water       = find_relay (IRRIGATION);
  Relay *ventilation = find_relay (VENTILATION);

  /* updates to fake environment */
  if (water && water->state == RELAY_ON)
    cached_moisture+=4;
  else
    cached_moisture-= ticks % 2;
  if (cached_moisture < 0)
    cached_moisture = 0;

  if (ventilation && ventilation->state == RELAY_ON)
    cached_humidity -=2;
  else
    cached_humidity += ticks % 2;

  if (cached_humidity > 100)
    cached_humidity = 100;
}

int main (int argc, char **argv)
{
  term = nct_new ();
  setup ();
  draw_ui ();
  while (!main_quit) loop ();
  return 0;
}

#endif 
