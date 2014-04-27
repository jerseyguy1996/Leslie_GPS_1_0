//#include <SPI.h>
#include "U8glib.h"
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>



byte index = 0;
byte control = 2;
byte mode = 3;
byte ddrcValue = 0;
byte portcValue = 0;
unsigned long batteryTime = 0;
unsigned long GPSStatusTime = 0;
unsigned long last_data_received = 0; //is the GPS transmitting
long b = 0; //battery charge
boolean GPS_transmitting = false;
unsigned long last_valid_data = 0; //is the GPS transmitting valid data
boolean data_valid = true;  //this will be set to false on the first iteration
boolean buttonpress = true;
unsigned long last_button_press = 0;
char buffer[90];
boolean sentenceBegins = false;
boolean data_index = false; //use this to alternate between processing RMC or GGA messages
char messageID[6];
char time[11];
char satsUsed[3];
char GPSstatus[2];
char date[9];
char Dummy[12];
signed long segdist = 0;
signed long distance = 0;
signed long last_latitude = 0;
signed long last_longitude = 0;
signed long latDegrees, longDegrees, latFract, longFract;
char GPS_Status[8];  //char array for GPS On/Off display indicator
char valDataOne[20]; //char array for GPS Valid Data in Mode 1
char valDataTwo[8]; //char array for GPS Valid Data in Mode 2
char lat_fract[7];
char long_fract[7];
char segdist_fract[5];
char dist_fract[5];
char n_s[2];
char e_w[2];
const int milesPerLat = 6900; //length per degree of latitude is 69 miles
  const int milesPerLong = 5253; //length per degree of longitude at
                                  //40.68 degrees of latitude is 52.53 miles.  It would
                                  //be better if I could put this conversion
                                  //in code so that it could be dynamically
                                  //calculated based on the location data from
                                  //the GPS.

#define OLED_DC A2
#define OLED_CS A4
#define OLED_CLK A1
#define OLED_MOSI A0
#define OLED_RESET A3

U8GLIB_SSD1306_128X32 u8g(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RESET);


SoftwareSerial mySerial(3,4);

void draw()
{
  switch(mode)
  {
    case 1:
      u8g.setFont(u8g_font_5x7);
      u8g.setPrintPos(0,6);
      u8g.print(GPS_Status);
      u8g.setFont(u8g_font_helvR08);
      u8g.setPrintPos(0,16);
      u8g.print(valDataOne);
      break;
      
    case 2:
      u8g.setFont(u8g_font_5x7);
      u8g.setPrintPos(0,6);
      u8g.print(GPS_Status);
      u8g.setPrintPos(128 - u8g.getStrWidth(valDataTwo),6);
      u8g.print(valDataTwo);
      u8g.setPrintPos(128 - u8g.getStrWidth("000"), 28);
      u8g.print(b);
      u8g.setFont(u8g_font_helvB12);
      u8g.setPrintPos(64-u8g.getStrWidth("00.0000")/2,28); //center the text at the bottom
      u8g.print(distance/10000);
      u8g.print(dist_fract);
      u8g.print(distance - ((distance/10000)*10000));
      break;
      
    case 3:
      u8g.setFont(u8g_font_helvR08);
      u8g.setPrintPos(0,16);
      u8g.print("Stopping");
      break;

    case 4:  //this just clears the display before going to sleep
      break;          
  } 
}

void OLED_Update()
{
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );
}

// the setup routine runs once when you press reset:
void setup() {                

  Serial.begin(57600);
  mySerial.begin(9600);
  Serial.println("Turning on stuff");
  digitalWrite(A5, LOW);  //pin 28 on the PDIP
  pinMode(A5, OUTPUT);   //On/Off control on the peripherals
  pinMode(2, INPUT);      //pin 4 on the PDIP
  digitalWrite(2, HIGH);//button control
}

// the loop routine runs over and over again forever:
void loop() 
{


  if(mode == 3) sleep();
  
  check_for_buttonpress();
  //Serial.println("Checking for ButtonPress");
  
  if(timer(GPSStatusTime))
  {
    check_GPS_Status();
    OLED_Update();
    GPSStatusTime = millis() + 5000;
    
  }
  
  
  check_for_updated_data();
  
  if(timer(batteryTime))
  {
    check_battery();
    OLED_Update();
    batteryTime = millis() + 5000;
  }
  

}

void check_for_buttonpress()
{
  if(buttonpress)
  {
    Serial.println("Button Pressed");
    if((millis() - last_button_press) < 1000) //having trouble with bounce so setting really long
    {
      Serial.println("Ignored");
      buttonpress = false;
     //have to reenable interrupts here because the ISR disables them
      attachInterrupt(0, button_press, FALLING);
      return;  //we will let the switch debounce for 1 second
    }
    last_button_press = millis();
    mode++;
    OLED_Update();
    buttonpress = false;
    attachInterrupt(0, button_press, FALLING);
    Serial.print("Mode = ");
    Serial.println(mode);
  }

}


void sleep()
{
  Serial.println("Sleeping");
  OLED_Update();
  delay(2000);
  mode=4;
  OLED_Update();  //clear the display
  //current was sinking through the MOSI pin
  //digitalWrite(OLED_MOSI, LOW);
  //u8g.sleepOn();
  ddrcValue = 0;
  portcValue = 0;
  //store current value of DDRC and PORTC
  ddrcValue = DDRC;
  portcValue = PORTC;
  DDRC &= ~0b00011111; //clear all of the bits corresponding to SPI
  PORTC &= ~0b00011111; 
  digitalWrite(A5, HIGH);  //turns off the GPS and Display
  sleep_enable();
  attachInterrupt(0, button_press, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
  Serial.println("I'm Asleep");
  Serial.flush();
  sleep_cpu();
  
  //code starts back up here after wake up
  Serial.println("I'm awake");
  sleep_disable();
  //digitalWrite(OLED_MOSI, HIGH);
  DDRC |= ddrcValue;  //set DDRC and PORTC back to its pre-sleep state
  PORTC |= portcValue;
  delay(50);  //I added this delay because I was having trouble with brown out resets
  digitalWrite(A5, LOW);  //turns on the GPS and Display
  delay(500);  //I needed this delay because the GPS wasn't executing the next
                // command without it
  mySerial.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0*35");
  last_latitude = 0;
  last_longitude = 0;
  index = 0;
  data_index = false;
  data_valid = true;  //will be set to false on the first loop after wakeup
  distance = 0;
  //setting mode to 0 because when check_for_buttonpress runs it will increment it to 1
  mode = 0;
  batteryTime = millis() + 1000;
  GPSStatusTime = millis() + 1000;
}

//just to let the user know if the GPS is on and is transmitting valid data
//Both last_data_received and last_valid_data timeout after two seconds.
void check_GPS_Status()
{
  if(timer(last_data_received))
  {
      strcpy(GPS_Status, "GPS Off");
      GPS_transmitting = false;
  }
  
  else
  
  {
      strcpy(GPS_Status, "GPS On");
      GPS_transmitting = true;
  }
  
  
  if(timer(last_valid_data))
  {
      //display things differently depending on whether we are 
      //in startup or running mode
      switch(mode)
      {
        case 1:
          strcpy(valDataOne, "Locating Satellites");
          data_valid = false;
          break;
        
        case 2:
          strcpy(valDataTwo, "Invalid");
          data_valid = false;
          break;
      }
  
  }
  
  else
  
  {
      switch(mode)
      {
        case 1:            
          strcpy(valDataOne, "GPS Ready :-)");
          data_valid = true;
          break;
        
        case 2:
          strcpy(valDataTwo, "Valid");
          data_valid = true;
          break;
      }
  }
}

//if everything comes back kosher then we can calculate stats and update the display
void check_for_updated_data()
{
  if(checkforSentence()) 
  {
    Serial.println(buffer);
    if(Process_message() && strcmp(messageID, "GPGGA") == 0)
    {
      //
      if(mode == 2)
      {
        OLED_Update();
      }
    }
  }
}

void check_battery()
{ //282 is the difference in value between 980 (4.2V full charge)
  //and 698 (3.0V empty charge).  We display the percent remaining.
  
  b = 0;
  for(int i = 0; i<10; i++)
  {
    b+=analogRead(A6);
    //Serial.println(b);
    
  }
  b = b/10;
  Serial.println(b);
  b = (((b-698)*100)/282);
  Serial.println(b); 
}

//this just reads data in one character at a time until we have a complete sentence
boolean checkforSentence()
{
  char c;
  while(mySerial.available())
  {
    last_data_received = millis() + 2000;
    c = mySerial.read();
    //Serial.print(c);
    
    if(sentenceBegins && c == '\r') //we have a full sentence
    {
      sentenceBegins = false;
      return true;
    }
    
    if(sentenceBegins) //store characters to buffer
    {
      buffer[index] = c;
      index++;
      buffer[index] = '\0';
    }
    
    if(c == '$') //beginning of sentence...start saving to buffer
    {
      sentenceBegins = true;
      index = 0;
    }
   
  }
  return false;
}

//function to break apart the NMEA strings into pieces that I can
//handle.  Each call returns a segment
const char* mytok(char* pDst, const char* pSrc, char sep = ',')
{
    while ( *pSrc )
    {
        if ( sep == *pSrc )
        {
            *pDst = '\0';

            return ++pSrc;
        }

        *pDst++ = *pSrc++;
    }

    *pDst = '\0';

    return NULL;
}


//function that does all of the work
boolean Process_message()
{
  char latit1[5];
  char latit2[6];
  char longit1[6];
  char longit2[6];
  char NS[2];
  char EW[2];
  
  const char*     ptr;
  
  //Serial.println(buffer);
  //check message ID to see what kind of message we got
  ptr = mytok(messageID, buffer);
  //Serial.println(messageID);
  
  //if it is GGA, read in the data and write to SDCard if
  //the data is valid and an SD file has been created
  if(strcmp(messageID, "GPGGA") == 0 && data_index==true)
  {
    ptr = mytok(time, ptr); if(ptr == NULL) return false;
    ptr = mytok(latit1, ptr, '.'); if(ptr == NULL) return false; //get the first half of latitude
    ptr = mytok(latit2, ptr); if(ptr == NULL) return false;//get the second half of latitude
    ptr = mytok(NS, ptr); if(ptr == NULL) return false;
    ptr = mytok(longit1, ptr, '.'); if(ptr == NULL) return false;
    ptr = mytok(longit2, ptr); if(ptr == NULL) return false;
    ptr = mytok(EW, ptr); if(ptr == NULL) return false;
    ptr = mytok(Dummy, ptr); if(ptr == NULL) return false;
    ptr = mytok(satsUsed, ptr); if(ptr == NULL) return false;

    //I wait to do anything until we have 4 satellites in view.  3 could work as well
    if(atoi(satsUsed) < 4) return false;
    
    //Everything is done with fixed point math.  Floats didn't have the resolution
    //I wanted.
    unsigned long multiplier = 1000000UL;


    
    latDegrees = (atoi(latit1))/100;  //isolate degrees
    latFract = (atoi(latit1)) - (latDegrees * 100);  //isolate whole minutes
    latDegrees *= multiplier;    //scale for fixed point math
    latFract *= multiplier;      //scale for fixed point math
    latFract += ((atoi(latit2))*100UL);  //add the fractions of a minute
    latFract = latFract/60;  //convert minutes to fractions of a degree
    latDegrees += latFract;
    
    longDegrees = (atoi(longit1))/100;  //isolate degrees
    longFract = (atoi(longit1)) - (longDegrees * 100);  //isolate whole minutes
    longDegrees *= multiplier;    //scale for fixed point math
    longFract *= multiplier;      //scale for fixed point math
    longFract += ((atoi(longit2))*100UL);  //add the fractions of a minute
    longFract = longFract/60;  //convert minutes to fractions of a degree
    longDegrees += longFract;
    
    //figure out how many zeros we need after the decimal
    if(latFract<10) {strcpy(lat_fract, ".00000");}
    else if (latFract<100) {strcpy(lat_fract, ".0000");}
    else if (latFract<1000) {strcpy(lat_fract, ".000");}
    else if (latFract<10000) {strcpy(lat_fract, ".00");}
    else if (latFract<100000) {strcpy(lat_fract, ".0");}
    else {strcpy(lat_fract, ".");}

    if(longFract<10) {strcpy(long_fract, ".00000");}
    else if (longFract<100) {strcpy(long_fract, ".0000");}
    else if (longFract<1000) {strcpy(long_fract, ".000");}
    else if (longFract<10000) {strcpy(long_fract, ".00");}
    else if (longFract<100000) {strcpy(long_fract, ".0");}
    else {strcpy(long_fract, ".");}
    
    if(NS[0] == 'S') strcpy(n_s, "-");

    if(EW[0] == 'W') strcpy(e_w, "-");
    
    if(mode == 2)
    {
      //calculate the distance
      if(last_latitude == 0)
      {
        last_latitude = latDegrees;
        last_longitude = longDegrees;
      }
  
      if(!(last_latitude == 0)) //just check to make sure that it isn't the first reading
      //calculate distance of segment
      {
        long temp = 0;
        segdist = sqrt(pow(((latDegrees - last_latitude)*milesPerLat)/10000L,2) + pow(((longDegrees - last_longitude)*milesPerLong)/10000L,2));
        if(segdist<100)
        {
          segdist = 0;
        }
        temp = segdist - ((segdist/10000) * 10000); //isolate the fractional amount
        if(temp<10) {strcpy(segdist_fract, ".000");}
        else if (temp<100) {strcpy(segdist_fract, ".00");}
        else if (temp<1000) {strcpy(segdist_fract, ".0");}
        else {strcpy(segdist_fract, ".");}
        
        distance+=segdist;
        temp = 0;
        temp = distance - ((distance/10000)*10000);
        if(temp<10) {strcpy(dist_fract, ".000");}
        else if (temp<100) {strcpy(dist_fract, ".00");}
        else if (temp<1000) {strcpy(dist_fract, ".0");}
        else {strcpy(dist_fract, ".");}
        
        if(!(segdist<100))
        {
          last_latitude = latDegrees;
          last_longitude = longDegrees;
        }
  
  
  
      }
    }
    
    last_valid_data = millis() + 2000;
    return true;
    
    
    //add the segment distance to the total distance



  }
  
  //we only run this once to get today's date which is in the RMC string
  if(strcmp(messageID, "GPRMC") == 0 && data_index == false)
  {
    char tempdate[7];

    
    ptr = mytok(time, ptr); if(ptr == NULL) return false;
    ptr = mytok(GPSstatus, ptr); if(ptr == NULL) return false;
    ptr = mytok(latit1, ptr, '.'); if(ptr == NULL) return false;//get the first half of latitude
    ptr = mytok(latit2, ptr); if(ptr == NULL) return false;//get the second half of latitude
    ptr = mytok(NS, ptr); if(ptr == NULL) return false;
    ptr = mytok(longit1, ptr, '.'); if(ptr == NULL) return false;
    ptr = mytok(longit2, ptr); if(ptr == NULL) return false;
    ptr = mytok(EW, ptr); if(ptr == NULL) return false;
    ptr = mytok(Dummy, ptr); if(ptr == NULL) return false;
    ptr = mytok(Dummy, ptr); if(ptr == NULL) return false;
    ptr = mytok(tempdate, ptr); if(ptr == NULL) return false;
    
  
    //GPSstatus tells us if data is valid
    if(GPSstatus[0] == 'V') return false;
    
    //re-order the date so that it will be in a nicer format for
    //sorting on the sd card
    date[0] = tempdate[4];
    date[1] = tempdate[5];
    date[2] = '_';
    date[3] = tempdate[2];
    date[4] = tempdate[3];
    date[5] = '_';
    date[6] = tempdate[0];
    date[7] = tempdate[1];
    date[8] = '\0';
    
    data_index = true; //We should have a valid date.  Now begin receiving GGA data
    mySerial.println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0*35");
    last_valid_data = millis() + 2000;
    return true;
  }
  return false;
}


//This is the ISR.  It just sets buttonpress so that the main loop can handle it
void button_press()
{
      sleep_disable();
      detachInterrupt(0);
      buttonpress = true;

}


//keep track of time and handle millis() rollover
boolean timer(unsigned long timeout)
  {
    return (long)(millis() - timeout) >= 0;
  }
