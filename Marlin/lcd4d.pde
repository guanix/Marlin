#include "language.h"
#include "temperature.h"
#include "ultralcd.h"
#include "lcd4d.h"
#include "Marlin.h"
#include "language.h"
#include "temperature.h"
//#include "EEPROMwrite.h"
//===========================================================================
//=============================imported variables============================
//===========================================================================

extern volatile int feedmultiply;
extern volatile bool feedmultiplychanged;

extern volatile int extrudemultiply;

extern long position[4];   
#ifdef SDSUPPORT
#include "cardreader.h"
extern CardReader card;
#endif

//===========================================================================
//=============================public variables============================
//===========================================================================


//===========================================================================
//=============================private  variables============================
//===========================================================================

static char messagetext[LCD4D_WIDTH]="";

//return for string conversion routines
static char conv[8];

static unsigned long previous_millis_lcd=0;

#ifdef SDSUPPORT
static uint8_t oldpercent=101;
#endif

/*

void lcdProgMemprint(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    lcd.print(ch);
    ch=pgm_read_byte(++str);
  }
}
#define lcdprintPGM(x) lcdProgMemprint(MYPGM(x))

*/
//===========================================================================
//=============================functions         ============================
//===========================================================================

int intround(const float &x){return int(0.5+x);}


void lcd4d_init()
{
   MYSERIAL1.begin(115200);
   LCD_MESSAGEPGM(WELCOME_MSG)

}

void lcd4d_debug()
{

}

void lcd4d_status(const char* message)
{
  strncpy(messagetext,message,LCD4D_WIDTH);
  messagetext[strlen(message)]=0;
}

void lcd4d_statuspgm(const char* message)
{
  char ch=pgm_read_byte(message);
  char *target=messagetext;
  uint8_t cnt=0;
  while(ch &&cnt<LCD4D_WIDTH)
  {
    *target=ch;
    target++;
    cnt++;
    ch=pgm_read_byte(++message);
  }
  *target=0;
}

void lcd4d_status()
{
  
    if(((millis() - previous_millis_lcd) < LCD4D_UPDATE_INTERVAL)   )
      return;
    
  previous_millis_lcd=millis();
  
  //update Lcd

  lcd4d_update();
}

void lcd4d_update() {
  //LCDSERIAL_PROTOCOL(MESSAGE_ID)
  //LCDSERIAL_PROTOCOLLN(messagetext)
  lcd4d_showStatus();
}

void lcd4d_showStatus()
{ 
  static int olddegHotEnd0=-1;
  static int oldtargetHotEnd0=-1;

  //HotEnd0  
  int tHotEnd0=intround(degHotend0());
  if(tHotEnd0!=olddegHotEnd0)
  {
 
    SERIAL1_PROTOCOLPGM(HOTEND0_ID)
    SERIAL1_PROTOCOLLN(ftostr3(tHotEnd0))
    olddegHotEnd0=tHotEnd0;
  }
  
  int ttHotEnd0=intround(degTargetHotend0());
  if(ttHotEnd0!=oldtargetHotEnd0)
  {
    SERIAL1_PROTOCOLPGM(THOTEND0_ID)
    SERIAL1_PROTOCOLLN(ftostr3(ttHotEnd0))
    oldtargetHotEnd0=ttHotEnd0;
  }
  
  #if defined BED_USES_THERMISTOR || defined BED_USES_AD595 
    static int oldtBed=-1;
    static int oldtargetBed=-1; 
    int tBed=intround(degBed());
    if(tBed!=oldtBed)
    {
      SERIAL1_PROTOCOLPGM(TBED_ID)
      SERIAL1_PROTOCOLLN(ftostr3(tBed))
      oldtBed=tBed;
    }
    int targetBed=intround(degTargetBed());
    if(targetBed!=oldtargetBed)
    {
      SERIAL1_PROTOCOLPGM(TTBED_ID)
      SERIAL1_PROTOCOLLN(ftostr3(targetBed))
      oldtargetBed=targetBed;
    }
   #endif
     
  #if EXTRUDERS > 1
 
    static int olddegHotEnd1=-1;
    static int oldtargetHotEnd1=-1;
    int tHotEnd1=intround(degHotend1());
    if(tHotEnd1!=olddegHotEnd1)
    {
      SERIAL1_PROTOCOLPGM(THOTEND1_ID)
      SERIAL1_PROTOCOLLN(ftostr3(ttHotEnd1)
      olddegHotEnd1=tHotEnd1;
    }
    int ttHotEnd1=intround(degTargetHotend1());
    if(ttHotEnd1!=oldtargetHotEnd1)
    {
      SERIAL1_PROTOCOLPGM(THOTEND1_ID)
      SERIAL1_PROTOCOLLN(ftostr3(ttHotEnd1))
      oldtargetHotEnd1=ttHotEnd1;
    }
  #endif
  
  
  //starttime=2;
  static uint16_t oldtime=0;
  if(starttime!=0)
  {
    uint16_t time=millis()/60000-starttime/60000;
    
    if(starttime!=oldtime)
    {
      SERIAL1_PROTOCOLPGM(TIME_ID)
      SERIAL1_PROTOCOL(itostr2(time/60))
      SERIAL1_PROTOCOLPGM("h");
      SERIAL1_PROTOCOL(itostr2(time%60))
      SERIAL1_PROTOCOLPGM("m");
      oldtime=time;
    }
  }
  
  static int oldzpos=0;
  int currentz=current_position[2]*100;
  if(currentz!=oldzpos)
  {
    SERIAL1_PROTOCOLPGM(ZPOS_ID)
    SERIAL1_PROTOCOLLN(ftostr52(current_position[2]))
    oldzpos=currentz;
  }
  
  /*
  static int oldfeedmultiply=0;
  int curfeedmultiply=feedmultiply;
  
  if(feedmultiplychanged == true) {
    feedmultiplychanged = false;
    encoderpos = curfeedmultiply;
  }
  
  if(encoderpos!=curfeedmultiply||force_lcd_update)
  {
   curfeedmultiply=encoderpos;
   if(curfeedmultiply<10)
     curfeedmultiply=10;
   if(curfeedmultiply>999)
     curfeedmultiply=999;
   feedmultiply=curfeedmultiply;
   encoderpos=curfeedmultiply;
  }
  
  if((curfeedmultiply!=oldfeedmultiply)||force_lcd_update)
  {
   oldfeedmultiply=curfeedmultiply;
   lcd.setCursor(0,2);
   lcd.print(itostr3(curfeedmultiply));lcdprintPGM("% ");
  }
  */
  
  if(messagetext[0]!='\0')
  {
    SERIAL1_PROTOCOLPGM(MESSAGE_ID)
    SERIAL1_PROTOCOLLN(messagetext);
    messagetext[0]='\0';
  }
#ifdef SDSUPPORT
  uint8_t percent=card.percentDone();
  if(oldpercent!=percent)
  {
    SERIAL1_PROTOCOLPGM(SDPERCENT_ID)
    SERIAL1_PROTOCOLLN(itostr3((int)percent))
  }
#endif

  //force_lcd_update=false;
}
  
  
  
  //  convert float to string with +123.4 format
char *ftostr3(const float &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/100)%10+'0';
  conv[1]=(xx/10)%10+'0';
  conv[2]=(xx)%10+'0';
  conv[3]=0;
  return conv;
}

char *itostr2(const uint8_t &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/10)%10+'0';
  conv[1]=(xx)%10+'0';
  conv[2]=0;
  return conv;
}

//  convert float to string with +123.4 format
char *ftostr31(const float &x)
{
  int xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *ftostr32(const float &x)
{
  int xx=x*100;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/100)%10+'0';
  conv[2]='.';
  conv[3]=(xx/10)%10+'0';
  conv[4]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr31(const int &xx)
{
  conv[0]=(xx>=0)?'+':'-';
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr3(const int &xx)
{
  conv[0]=(xx/100)%10+'0';
  conv[1]=(xx/10)%10+'0';
  conv[2]=(xx)%10+'0';
  conv[3]=0;
  return conv;
}

char *itostr4(const int &xx)
{
  conv[0]=(xx/1000)%10+'0';
  conv[1]=(xx/100)%10+'0';
  conv[2]=(xx/10)%10+'0';
  conv[3]=(xx)%10+'0';
  conv[4]=0;
  return conv;
}

//  convert float to string with +1234.5 format
char *ftostr51(const float &x)
{
  int xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]=(xx/10)%10+'0';
  conv[5]='.';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

//  convert float to string with +123.45 format
char *ftostr52(const float &x)
{
  int xx=x*100;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]='.';
  conv[5]=(xx/10)%10+'0';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}



