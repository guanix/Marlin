/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */
 
/*
  RepRapPro ammendations, deletions and additions
  
  G10, M0, M1, M112 and T commands added/modified by AB 18/7/12
  Much conditional-compiled code for old/unused hardware removed - AB 29/7/12
  
*/



#include "Marlin.h"

#include "ultralcd.h"
#include "led.h"
#include "z_probe.h"
#include "FPUTransform.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "EEPROMwrite.h"
#include "language.h"
#include "pins_arduino.h"
#include "slave_comms.h"
 
#define VERSION_STRING  "1.0.1 RRP"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - set head offset and temps
// G28 - Home all Axis
// G29 - Detailed Z-Probe (3 location test)
// G30 - Single Z Probe (probe current location)
// G31 - Report Curent Probe status
// G32 - Probe Z and calibrate with FPU
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp (deprecated)
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp. (deprecated)
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Fast SD transfer
// M31  - high speed xfer capabilities 
// M35  - Output time since last M109 or SD card start to serial

// M42  - Change pin status via gcode
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port 
// M115	- Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M208 - set axis max length
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M301 - Set PID parameters P I D and W
// M302 - S1 Allow cold extrudes, S0 cold extrues not allowed (default)
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set thermistor parameters
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M510 - FPU Enable
// M511 - FPU Reset
// M512 - FPU Disable
// M999 - Restart after being stopped by error

// M555 - Temporary: master/slave comms test

// TN - Select extruder N

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
float fast_home_feedrate[] = FAST_HOME_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
volatile int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
volatile bool feedmultiplychanged=false;
volatile int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float max_length[] = AXES_MAX_LENGTHS;
#ifdef ADVANCE
float advance_k = EXTRUDER_ADVANCE_K;
#endif
uint8_t active_extruder = 0;
float extruder_x_off[EXTRUDERS] = X_EXTRUDER_OFFSET;
float extruder_y_off[EXTRUDERS] = Y_EXTRUDER_OFFSET;
float extruder_z_off[EXTRUDERS] = Z_EXTRUDER_OFFSET;
float extruder_standby[EXTRUDERS];
float extruder_temperature[EXTRUDERS];
float x_off_d;
float y_off_d;
float z_off_d;
float temp_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
bool extruder_selected=false;


unsigned char FanSpeed=0;

float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
float offset[3] = {0.0, 0.0, 0.0};
float feedrate = 1500.0, next_feedrate, saved_feedrate;

// used by FPU transform code
float modified_destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static bool home_all_axis = true;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

static unsigned long starttime=0;
static unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_photpin()
{
  #ifdef PHOTOGRAPH_PIN
    #if (PHOTOGRAPH_PIN > -1)
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
    #endif
  #endif 
}

void setup_powerhold()
{
 #ifdef SUICIDE_PIN
   #if (SUICIDE_PIN> -1)
      SET_OUTPUT(SUICIDE_PIN);
      WRITE(SUICIDE_PIN, HIGH);
   #endif
 #endif
}

void suicide()
{
 #ifdef SUICIDE_PIN
    #if (SUICIDE_PIN> -1) 
      SET_OUTPUT(SUICIDE_PIN);
      WRITE(SUICIDE_PIN, LOW);
    #endif
  #endif
}

void setup()
{ 
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;
  
  for(int8_t i = 0; i < EXTRUDERS; i++)
  {
    extruder_standby[i] = STANDBY_TEMP;
    extruder_temperature[i] = DEFAULT_TEMP;
  }


  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }
  
  EEPROM_RetrieveSettings(); // loads data from EEPROM if available

  for(int8_t i=0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }


  tp_init();    // Initialize temperature loop 
  plan_init();  // Initialize planner;
  st_init();    // Initialize stepper;
  #if (LED_PIN > -1)
    led_init();
  #endif
  probe_init(); //Initializes probe if PROBE_PIN is defined
  FPUTransform_init(); //Initializes FPU when UMFPUSUPPORT defined
  setup_photpin();
  
#ifdef REPRAPPRO_MULTIMATERIALS
  setup_slave();
#endif

  LCD_INIT

}


void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  #ifdef SDSUPPORT
    card.checkautostart(false);
  #endif
  if(buflen)
  {
    #ifdef SDSUPPORT
      if(card.saving)
      {
	if(strstr(cmdbuffer[bufindr],"M29") == NULL)
	{
	  card.write_command(cmdbuffer[bufindr]);
	  SERIAL_PROTOCOLLNPGM(MSG_OK);
	}
	else
	{
	  card.closefile();
	  SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
	}
      }
      else
      {
	process_commands();
      }
    #else
      process_commands();
    #endif //SDSUPPORT
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity(1);
  checkHitEndstops();
  LCD_STATUS;
  LED_STATUS;
}

void get_command() 
{ 
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' || 
       serial_char == '\r' || 
       (serial_char == ':' && comment_mode == false) || 
       serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strstr(cmdbuffer[bufindw], "N") != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strstr(cmdbuffer[bufindw], "*") != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else 
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strstr(cmdbuffer[bufindw], "*") != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strstr(cmdbuffer[bufindw], "G") != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
	      #ifdef SDSUPPORT
              if(card.saving)
                break;
	      #endif //SDSUPPORT
              SERIAL_PROTOCOLLNPGM(MSG_OK); 
            }
            else {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
          }

        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #ifdef SDSUPPORT
  if(!card.sdprinting || serial_count!=0){
    return;
  }
  while( !card.eof()  && buflen < BUFSIZE) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' || 
       serial_char == '\r' || 
       (serial_char == ':' && comment_mode == false) || 
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1) 
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int sec,min;
        min=t/60;
        sec=t%60;
        sprintf(time,"%i min, %i sec",min,sec);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        LCD_MESSAGE(time);
        card.printingHasFinished();
        card.checkautostart(true);
        
      }
      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }     
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  
  #endif //SDSUPPORT

}


float code_value() 
{ 
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); 
}

long code_value_long() 
{ 
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); 
}

bool code_seen(char code_string[]) //Return True if the string was found
{ 
  return (strstr(cmdbuffer[bufindr], code_string) != NULL); 
}  

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define HOMEAXIS(LETTER) \
  if ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))\
    { \
    current_position[LETTER##_AXIS] = 0; \
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]); \
    destination[LETTER##_AXIS] = 1.1 * max_length[LETTER##_AXIS] * LETTER##_HOME_DIR; \
    feedrate = fast_home_feedrate[LETTER##_AXIS]; \
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder); \
    st_synchronize();\
    \
    current_position[LETTER##_AXIS] = 0;\
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);\
    destination[LETTER##_AXIS] = -LETTER##_HOME_RETRACT_MM * LETTER##_HOME_DIR;\
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder); \
    st_synchronize();\
    \
    destination[LETTER##_AXIS] = 2*LETTER##_HOME_RETRACT_MM * LETTER##_HOME_DIR;\
    feedrate = homing_feedrate[LETTER##_AXIS] ;  \
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder); \
    st_synchronize();\
    \
    current_position[LETTER##_AXIS] = LETTER##_HOME_POS;\
    destination[LETTER##_AXIS] = current_position[LETTER##_AXIS];\
    feedrate = 0.0;\
    endstops_hit_on_purpose();\
  }
  
void wait_for_temp(uint8_t& t_ext, unsigned long& codenum)
{
        /* See if we are heating up or cooling down */
      bool target_direction = isHeatingHotend(t_ext); // true if heating, false if cooling

      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp   
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) 
        {
      #else
        while ( target_direction ? (isHeatingHotend(t_ext)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) 
        {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL_F(degHotend(t_ext),1); 
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL( (int)t_ext ); 
            #ifdef TEMP_RESIDENCY_TIME
              SERIAL_PROTOCOLPGM(" W:");
              if(residencyStart > -1)
              {
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                 SERIAL_PROTOCOLLN( codenum );
              } else 
              {
                 SERIAL_PROTOCOLLN( "?" );
              }
            #else
              SERIAL_PROTOCOLLN("");
            #endif
            codenum = millis();
          }
          manage_heater();
          manage_inactivity(1);
          LCD_STATUS
          led_status();
        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(t_ext) >= (degTargetHotend(t_ext)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(t_ext) <= (degTargetHotend(t_ext)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(t_ext) - degTargetHotend(t_ext)) > TEMP_HYSTERESIS) ) 
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        }
        LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
        starttime=millis();
        previous_millis_cmd = millis();
}
  

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
      
      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity(1);
      }
      break;
      
      case 10: // Set offsets
      if(code_seen('P'))
      {
         tmp_extruder = code_value();
         get_coordinates();
         extruder_x_off[tmp_extruder] = destination[0]; // X
         extruder_y_off[tmp_extruder] = destination[1]; // Y
         extruder_z_off[tmp_extruder] = destination[2]; // Z
         if(code_seen('R'))
             extruder_standby[tmp_extruder] = code_value();
         if(code_seen('S'))
             extruder_temperature[tmp_extruder] = code_value();
      }
      LCD_STATUS;
      break;
  
    case 28: //G28 Home all Axis one at a time
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();
      
      enable_endstops(true);
      
      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;
      home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));
      
      if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) 
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
       HOMEAXIS(Y);
      }
      
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      
      if((home_all_axis) || code_seen(axis_codes[X_AXIS])) 
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS]=code_value();
        }
        current_position[X_AXIS]+=add_homeing[0];
      }

      if((home_all_axis) || code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS]=code_value();
        }
        current_position[Y_AXIS]+=add_homeing[1];
      }

      if((home_all_axis) || code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value();
        }
        current_position[Z_AXIS]+=add_homeing[2];
      }
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      
      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif
      
      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;
    case 29:
        probe_3points();
        break;
    case 30:
        probe_1point();
        break;
    case 31:
        probe_status();
        break;
    case 32:
    	FPUTransform_determineBedOrientation();
    	break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) { 
           current_position[i] = code_value()+add_homeing[i];  
           if(i == E_AXIS) {
             current_position[i] = code_value();  
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             current_position[i] = code_value()+add_homeing[i];  
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() ) 
    {
    case 0: // Stops - add me...
    case 1:
    case 112:
      break;

    case 17:
        LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x(); 
        enable_y(); 
        enable_z(); 
        enable_e0(); 
        enable_e1(); 
        enable_e2(); 
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      #ifdef LCD_4D
        SERIAL1_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      #endif
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      #ifdef LCD_4D
      SERIAL1_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      #endif
      break;
    case 21: // M21 - init SD card
      
      card.initsd();
      
      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File 
	if (card.cardOK){
		card.closefile();
		starpos = (strchr(strchr_pointer + 4,'*'));
                if(starpos != NULL){
                char* npos = strchr(cmdbuffer[bufindr], 'N');
                strchr_pointer = strchr(npos,' ') + 1;
                *(starpos-1) = '\0';
         }
	 card.removeFile(strchr_pointer + 4);
	}
	break;
	
    case 32: //M32 - fast SD transfer
        card.fast_xfer(strchr_pointer+4);
        break;
    case 33: //M31 - high speed xfer capabilities
        SERIAL_ECHOPGM("RAW:");
        SERIAL_ECHOLN(SD_FAST_XFER_CHUNK_SIZE);
        break;
#endif //SDSUPPORT

/* All the implementation is done in *.cpp files to get better compatibility with avr-gcc without the Arduino IDE */
/* Use this file to help the Arduino IDE find which Arduino libraries are needed and to keep documentation on GCode */

#include "Configuration.h"
#include "pins.h"

#ifdef ULTRA_LCD
#include <LiquidCrystal.h>
#endif

#if DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif
