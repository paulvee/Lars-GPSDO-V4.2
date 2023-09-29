/*
  
  ONLY for 328 based Arduinos!!
  With modulo 50000
  With getcommand from JimH (replaces dipswitches)
  Added timer_us to ns TIC values to get much much larger capture range
  Changed PI-loop now uses float with remainder and not DI-term for P-term
  Changed average TIC+DAC storing to instant TIC+TNCT1 in hold mode
  Still EEPROM storage of 3hour averages and dac start value
  Added EEPROM storage of time constant and gain and many more parameters(see getcommand "help" below)
  
  Please check gain and timeConst and adapt to your used VCO. Holdmode is very useful to check range of VCO
  
  -Hardware:
  This version uses input capture on D8 !!! so jumper D2-D8 if you have an old shield with 1PPS to D2
  Uses 1 ns res/ 1 us TIC and internal PWM (2 x 8bits)
  1PPS to capture interrupt (D8)and HC4046 pin 14
  1MHz from HC390 (div2*5)to HC4046 pin 3
  5MHz from HC390 (div2)to timer1 (D5)
  1N5711+3.9k in series from HC4046 pin 15 to ADC0. 1nF NPO + 10M to ground on ADC0
  16bit PWM DAC with two-pole lowpass filter from D3 (39k+4.7uF+39k+4.7uF) and D11 (10M)
  Put a LED + resistor on D13 to show Lock status
  Optional temperature sensors on ADC2 (used for temperature compensation) and ADC1 (just indication)
  ADC3 can be read and used for whatever you want
  For UNO a recommendation is to connect one jumper from reset to a 10uf capacitor connected to ground.
  With this jumper shorted the Arduino UNO will not reset every time the serial monitor is started.
  For downloading a new program the jumper need to be taken away.
  
  *** paulv modifications V3:  
  * added interrupt driven decharge of C1 capacitor instead of 10M bleed resistor
  * print **** instead of unLock to make it better standout
  * changed the temperature labels in the report to a more meaningful OCXO and ambient
  * added a function to automatically increase the TC while keeping a lock (especially for the reciprocal counter)
  * V3.70 added code to read a DS18B20 sensor to register the room temperatures. This sensor is located outside of the enclosure
  * V3.71 added averaging for the temp sensors for use in the report, fixed indentations to easier read the code
  * V3.71 changed variable "time" to "Ltime" because the new compiler has time as a reserved name
  * V3.80 added the code to obtain the qErr data from the NEO and incorporate that. More info on my Blog
  *
  * My changes start with *** paulv: in comments so they are easy to locate
  * 
  * Select: Arduino Nano - Old Bootloader
  * 
*/

String FW_VERSION = "3.8.1"; // current revision number

#include <EEPROM.h>

//*** paulv: added section

#include <OneWire.h> // *** paulv: added support for the DS18B20 temp sensor, it uses the 1-wire bus
// watch out, the following library disrupts the sketch, probably because it uses interupt and timers internally
// #include <DallasTemperature.h> // do not use this library!
// DS18B20 sensor setup
// Sensor data wire is connected to the Arduino digital pin D4 and pulled-up by a 4K7 resistor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with 1-Wire devices on the bus
byte dsAddress[8];   // holding the sensor address on the 1-wire bus
float roomTemp; // to store the DS18B20 sensor temperature

#include <Wire.h> // *** paulv: added support for obtaining the qErr data from the NEO
#define UBX_ADDR 0x42 // default DDC (is i2c) address
// connect SDA pin 18 on NEO 6/7/8 to A4 on Nano, pull-up is provided by the NEO
// connect SCL pin 19 on NEO 6/7/8 to A5 on Nano, pull-up is provided by the NEO
boolean apply_qErr = true;  
unsigned int mValue = 200;  // delay period between NEO polling and getting the response
                            // if this delay is too large, the time counter will stop to the increment
                            // UR8US used 300, but that's too high. It needs to be below 250 with this code
float qErr = 0;
boolean isSetqErr = false;

// Decharge pin for charging capacitor. Port is toggled between hi-Z input and output driven low
const int decharge = 10; 

// setup the automatic TC setting
boolean lock = false; // used for the lock/unlock status
int lockTimer = 0;  // used to set a time between timeConst increments
int maxTC = 500; // maximum timeConst for the autoAdvanceTC method
boolean autoTCrun = true; // use to either set execution to auto TC or fixed TC

// added IIR filter for the temperature sensors
// The readings fluctuate every second which is not relevant, average them out
// Lars already uses a filter for ADC1 that is used in the DAC compensation.
// I left that as is, and only use the filtered values for the report.
// The IIR filter algorithm:
//    avg = avg + ((new_value - avg) / filter_weight)
long tempADC1_IIR;  // PCB temperature
long tempADC2_IIR;  // OCXO temperature
float roomTemp_IIR;  // room temperature
int IIR_Temp_Filter_Weight = 3;  // IIC filter weight value

// *** paulv: end of my added section


int warmUptime = 60; // gives 1 minute hold during warmup. Set to 3 for VCTCXO or 500 for 3 minutes for a cold start
long dacValueOut = 32767; // 16bit PWM-DAC setvalue=startvalue Max 65535 (if nothing stored in the EEPROM)
long dacValue; // this is also same as "DACvalueOld" Note: is "0-65535" * timeconst
long dacValue2; // had to add this for calculation in PI-loop
long dacValueWithTempComp; // Note: is "0-65535" * timeconst

const int PWMhighPin = 3; // high PWM-DAC pin (connected to 39k)
const int PWMlowPin = 11; // low PWM-DAC pin (connected to 10M)
int valuePWMhigh; // high PWM-DAC byte
int valuePWMlow; // low PWM-DAC byte

volatile int TIC_Value; // analog read 0  - time value. About 1ns per bit with 3.9Kohm + 1nF
int TIC_ValueOld;//old not filtered TIC_value
int TIC_Offset = 500; // ADC value for Reference time

long TIC_ValueFiltered; // prefiltered TIC value
long TIC_ValueFilteredOld;// old filtered value
long TIC_ValueFilteredForPPS_lock; // prefiltered value just for PPS lock

volatile unsigned int timer1CounterValue; //counts 5MHz clock modulo 50000
long timer1CounterValueOld = 0 ;
unsigned int TCNT1new = 0;  //in main loop
unsigned int TCNT1old = 0;
unsigned long overflowCount = 0; // counter for timer1 overflows
long timer_us; // timer1 value in microseconds offset from 1pps
long timer_us_old; // used for diff_ns
long diff_ns; // difference between old and new TIC_Value
long diff_ns_ForPPS_lock; // prefiltered value just for PPS lock

long tempADC1; // *** paulv: changed to long, analog read 1  - for example for internal NTC if oscillator external to Arduino
long tempADC2; // analog read 2  - for oscillator temp eg LM35 or NTC - used for temperature compensation
long tempADC2_Filtered; // analog read 2  - temp filtered for temp correction
long tempCoeff = 0; // 650 = 1x10-11/°C if 1°C= +10bit temp and 65dac bits is 1x10-11 // set to -dacbits/tempbits*100?
long tempRef = 280; // offset for temp correction calculation - 280 is about 30C for LM35 (set for value at normal room temperature)
unsigned int temperature_Sensor_Type = 0; //

long timeConst = 32; // time constant in seconds
long timeConstOld = 32; // old time constant
int filterDiv = 2; // filterConst = timeConst / filterDiv
long filterConst = 16; // pre-filter time const in secs (TIC-filtering)
long filterConstOld = 16; // old Filter time constant

float I_term; //for PI-loop
float P_term;
long I_term_long;
float I_term_remain;

long gain = 12; //VCO freq DAC bits per TIC bit (65536/VCOrange in ppb (eg. with 1nS/bit and 100ppb DACrange gives gain=655))
float damping = 3.0; //Damping in loop

unsigned long Ltime; //seconds since start
unsigned long timeOld; //last seconds since start
unsigned int missedPPS; // incremented every time pps is missed
unsigned long timeSinceMissedPPS;
volatile boolean PPS_ReadFlag = false; // set true every time pps is received
int lockPPSlimit = 100; // if TIC filtered for PPS within +- this for lockPSfactor * timeConst = PPSlocked
int lockPPSfactor = 5;  // see above
unsigned long lockPPScounter; // counter for PPSlocked
boolean PPSlocked; //digital pin and prints 0 or 1
const int ppsLockedLED = 13; // LED pin for pps locked

int i; // counter for 300secs before storing temp and dac readings average
int j; // counter for stored 300sec readings
int k; // counter for stored 3hour readings
unsigned int StoreTIC_A[144]; //300sec storage
unsigned int StoreTempA[144];
unsigned int StoreDAC_A[144];
long sumTIC;
long sumTIC2;
long sumTemp;
long sumTemp2;
unsigned long sumDAC; 
unsigned long sumDAC2;

unsigned int totaltime3h; // counter for power-up time updated every third hour
unsigned int restarts; // counter for restarts/power-ups
boolean restartFlag = true;

unsigned int ID_Number;

boolean lessInfoDisplayed;
boolean nsDisplayedDecimals;

// for get command
int incomingByte;   // for incoming serial data in getCommand
enum Modes {hold, run};
Modes opMode = run;     //operating mode
Modes newMode = hold;   // used to reset timer_us when run is set and at to many missing PPS
unsigned int holdValue; //DAC value for Hold mode

// for TIC linearization
float TICmin = 12.0;
float TICmax = 1012.0;
float x3 = 0.03;
float x2 = 0.1;
float x1;
float TIC_Scaled;
float TIC_ValueCorr;
float TIC_ValueCorrOld;
float TIC_ValueCorrOffset;


////////////////////////////////////////////////////////////////////////////////////////////////
// timer1 capture interrupt routine - this runs at rising edge of 1 PPS on D8
// paulv: **** modified to add a digital decharge for the 1nF charging capacitor
ISR(TIMER1_CAPT_vect)
  {
    timer1CounterValue = ICR1;    // read the captured timer1 200ns counter value
    TIC_Value = analogRead(A0);   // Read the voltage on the charge capacitor
    pinMode(decharge,OUTPUT);     // *** paulv change port from hi-Z input to output
    digitalWrite(decharge, 0);    // *** paulv bring the output low to start the de-charging of C1
    PPS_ReadFlag = true;
    delayMicroseconds(20);        // *** paulv allow sufficient time to fully drain the capacitor
    pinMode(decharge,INPUT);      // *** paulv revert the port back to hi-Z (input)
  }


////////////////////////////////////////////////////////////////////////////////////////////////  
void calculation()
{
  // set timer1 start value in the beginning  
  if (Ltime < 2 || (Ltime > warmUptime-2 && Ltime < warmUptime))
    {
      TCNT1 = 25570;  // is a guessed value to get around 25000 next time
    }

  /*
  * TIC linearization and Square compensation
  *
  * This is based on finding the min & max of the TIC range to set the right span of the TIC
  * The procedure to obtain and set the min, max and square is described on page 12 of the PDF document.
  * x2 is the Square compensation factor.
  * x2 = 0.1 by default initially but can be set by the user; using command l
  * You input the TCmin (ranging from 1-500), TCmax (ranging from 800-1023)and the Square compensation (ranging from 1024-1200)
  * 
  * Lars provided the following as an explanation:
  * x2 = (((TICmid-TICmin)/(TICmax-TICmin)*1000) - 500.0)/250.0 - 0.05; // just for inf o
  * What I think Lars meant with TCmid is the TIC_Offset (normally 500), which can be set by command o between 200 and 1020 ns
  * You need to use an offset of 1000 to get the min and max numbers.
  * 
  */

  x1 = 1.0 - x3 - x2; // with default numbers, x1 = 0.87
  TIC_Scaled = ((float)TIC_Offset - TICmin)/(TICmax - TICmin)*1000; // Scaling for TIC_Offset
  TIC_ValueCorrOffset = TIC_Scaled * x1 + TIC_Scaled * TIC_Scaled * x2 / 1000.0 + TIC_Scaled * TIC_Scaled * TIC_Scaled * x3 /1000000.0;

  TIC_Scaled = ((float)TIC_Value - TICmin)/(TICmax - TICmin)*1000; // Scaling for TIC_Value
  TIC_ValueCorr = TIC_Scaled * x1 + TIC_Scaled * TIC_Scaled * x2 / 1000.0 + TIC_Scaled * TIC_Scaled * TIC_Scaled * x3 /1000000.0;


  TIC_ValueCorr -= qErr; // apply the qErr compensation

  /*
  * timer_us calculation
  *
  * timer1CounterValue is read by the interrupt service routine every 1PPS pulse
  *
  * Here is an explanation from contributor imo on the EEVBlog (reply 127):
  *  timer_us = timer_us + 50000 - (((timer1CounterValue - timer1CounterValueOld) * 200 + TIC_Value - TIC_ValueOld)  
  *  >>> those are in 200ns timer1CounterValue increments (OCXO/2) and you add difference between the previous and the new value 
  *  >>> TIC_Value is the voltage measured by the ADC from 4046+diode+3.9k+1nF in nanoseconds (everything is in nanoseconds)
  *  +50000500)/1000;  
  *  >> you add 50000500 because you do modulo 50.000 and you want to round properly (xxx.500) and finally you divide all by thousands to get microseconds
  * 
  *  An example:
  *  timer_us = timer_us + 50000 - (((1)*200 + 530 - 490) + 50000500)/1000 = 
  *  timer_us + 50000 - ((200+40) + 50000500)/1000 = 
  *  timer_us + 50000 - (50000740)/1000 = 
  *  timer_us + 50000 - 50000 = timer_us +  0    
  *  >>> no change as the 1PPS came within 1usec.. (or better to say: the OCXO's freq at this moment is such thus it fits into 1PSS +/-0.5us).
  *
  */

  timer_us = timer_us + 50000 - (((timer1CounterValue - timer1CounterValueOld) * 200 + TIC_Value - int(qErr) - TIC_ValueOld)+50000500)/1000;

  // reset timer_us if we transition from hold mode to run mode
  if (newMode == run)
    {
      timer_us = 0 ;
      timer_us_old = 0 ;
      TIC_ValueFilteredOld = TIC_Offset * filterConst;
      newMode = hold;
    }

  // reset timer_us at the beginning and at the end of the warmup period
  if (Ltime < 3 || (Ltime > warmUptime-1 && Ltime < warmUptime+1)) 
    {
      timer_us = 0 ;
    } 

  // When we're running and not in the warmup period and...
  // timer_us > TC * 65535 / gain / 1,000 | what error condition is this?
  if ((abs(timer_us)- 2) > timeConst * 65536 / gain / 1000 && opMode == run && Ltime > warmUptime )
    {
      timer_us = 0 ;
      timer_us_old = 0 ;
      TIC_ValueFilteredOld = TIC_Offset * filterConst;
    }

  // reset timer_us if the 10MHz is missing  
  if (TIC_ValueOld == 1023) 
    {
      timer_us = 0 ;
      timer_us_old = 0 ;
      TIC_ValueFilteredOld = TIC_Offset * filterConst;
    } 

  /*
  * Calculate diff_ns
  *
  * The difference between the new timer_us and the previous timer_us
  * include the TIC linerarization parameters
  * 
  * The Frequency is in ppb if updated every second!
  */

  if (TIC_ValueCorr > TIC_ValueCorrOld)
    {
      diff_ns = (timer_us - timer_us_old)*1000 + long (TIC_ValueCorr - TIC_ValueCorrOld + 0.5); 
    }else{
      diff_ns = (timer_us - timer_us_old)*1000 + long (TIC_ValueCorr - TIC_ValueCorrOld -0.5);
    }


  // time - is supposed to be approximately seconds since start
  Ltime = Ltime + (overflowCount + 50)/100;
  overflowCount = 0;
  
  // register the missedPPS events
  if (Ltime - timeOld > 1) 
    {
      missedPPS = missedPPS + 1;
      timeSinceMissedPPS = 0;
    }else{
      timeSinceMissedPPS = timeSinceMissedPPS + 1;
    }

 /*
  * Determine if we are locked to the 1PPS
  *
  */

  // Low Pass IIR Filter of TIC_Value for PPS lock  
  // /16 is used as 500ns error and /16 is about 30ns that seems reasonable
  TIC_ValueFilteredForPPS_lock = TIC_ValueFilteredForPPS_lock + (TIC_Value * 16 - TIC_ValueFilteredForPPS_lock) / 16;
                                                                             
  // Low Pass IIR Filter of diff_ns for PPS lock                                                                             
  diff_ns_ForPPS_lock = diff_ns_ForPPS_lock + (diff_ns * 16 - diff_ns_ForPPS_lock) / 16;
  
  lockPPScounter = lockPPScounter + 1; // update the counter for the report
  
  // reset the lock counter
  if (abs(TIC_ValueFilteredForPPS_lock / 16 - TIC_Offset) > lockPPSlimit)
    {lockPPScounter = 0;}

  // reset the lock counter if freq is more than 20ppb wrong 
  // (had to add this to avoid certain combinations not covered by above)
  if (abs(diff_ns_ForPPS_lock/16) > 20)
    {lockPPScounter = 0;}
  
  // Are we locked?
  if (lockPPScounter > timeConst * lockPPSfactor)
    {
      PPSlocked = 1;
    }else{
      PPSlocked = 0;
    }
 
  // turn on LED 13 if "locked"    
  digitalWrite(ppsLockedLED,PPSlocked);
  

  ////// Read the temperature values 
  int dummyreadADC = analogRead(A2); //without this ADC1 is influenced by ADC0
  tempADC1 = analogRead(A2);
  dummyreadADC = analogRead(A1); //without this ADC2 is influenced by ADC1
  tempADC2 = analogRead(A1);
  dummyreadADC = analogRead(A0); //without this TIC_Value (ADC0) is influenced by ADC2
  

  // set filter constant
  filterConst = timeConst / filterDiv;
  filterConst = constrain (filterConst, 1,1024);
  if (PPSlocked == 0 || opMode == hold) filterConst = 1;
  
  // recalculation of DAC value  
  if(timeConst != timeConstOld)
    {
      dacValue = dacValue / timeConstOld * timeConst;  
    }


  if(filterConst != filterConstOld)
    {
      TIC_ValueFilteredOld = TIC_ValueFilteredOld / filterConstOld * filterConst;
      TIC_ValueFiltered = TIC_ValueFiltered / filterConstOld * filterConst;
    }
     
  // Low Pass IIR Filter for the TICvalue (Phase Error)
  // Remember that TIC_ValueFiltered is multiplied by filterConst  
  // Don´t update if we have an outlier. 
  // Accepts diff_ns less than same ns as vco range in ppb + 200ns
  // First check to avoid overflow in next calculation (also max VCO range is about 6500ns/s)
  if abs(diff_ns < 6500)
    {
      if( abs(diff_ns * gain) < (65535 + 200 * gain))
        { // also apply the qErr correction
          TIC_ValueFiltered = TIC_ValueFiltered + ((timer_us*1000 + (TIC_Value - int(qErr))) * filterConst - TIC_ValueFiltered + (filterConst/2)) / filterConst; 
        }
    }

  // Don't change the DAC-value during warm-up time or when in hold mode
  if (Ltime > warmUptime && opMode == run) 
    {
      ////// PI-loop /////////////////////////////////////
      P_term = (TIC_ValueFiltered - TIC_Offset * filterConst) / float(filterConst) * gain; // remember /timeConst is done before dacValue is sent out
      I_term = P_term / damping / float(timeConst)  + I_term_remain; 
      I_term_long = long(I_term); 
      I_term_remain = I_term - I_term_long;
      dacValue += I_term_long;
      dacValue2 = dacValue + P_term;
      ///////////////////////////////////////////////////
    }else{
      (dacValue2 = dacValue); // No change
    }

  // Low Pass IIR Filter for temperature
  tempADC2_Filtered = tempADC2_Filtered + (tempADC2 * 100 - tempADC2_Filtered) / 100;
    
  // Temperature correction for DAC value
  dacValueWithTempComp = dacValue2 + ((tempRef * 100 - tempADC2_Filtered) * tempCoeff / 10000 * timeConst); 

  // Check that dacValue is within limits
  if (dacValue < 0) 
    { dacValue = 0;}
  if (dacValue > (65535 * timeConst))
    { dacValue = (65535 * timeConst);}
  
  // PWM-DAC value with temp compensation, check for limits again
  dacValueOut = dacValueWithTempComp / timeConst; 
  if (dacValueOut < 0)   
    { dacValueOut = 0;}
  if (dacValueOut > 65535)
    { dacValueOut = 65535;}

  // manual set of DACvalue if in hold and not 0, if zero hold is selected we use the old DACvalue
  if (holdValue > 0 && opMode == hold) 
    {dacValueOut = holdValue ;}

  // Set "16bit DAC" 
  valuePWMhigh = highByte(dacValueOut);
  valuePWMlow = lowByte(dacValueOut);
  analogWrite(PWMhighPin,valuePWMhigh);
  analogWrite(PWMlowPin,valuePWMlow);

  // Increment restart at time 100 (100 chosen arbitrary)
  if (Ltime > 100 && restartFlag == true)
    { 
      restarts = restarts + 1;
      EEPROM.write(991, highByte(restarts));
      EEPROM.write(992, lowByte(restarts));
      restartFlag = false;
    }
  
  ///////////////////////////////////////////////////
  //Storage of average readings that are printed later
  
  sumTIC = sumTIC + (TIC_Value * 10);
  sumTemp = sumTemp + (tempADC2 * 10);
  sumDAC = sumDAC + dacValueOut;
  i = i + 1;
  
  // readings for the 300 second section
  if (i == 300)
    {
      if (opMode == run) 
         {StoreTIC_A[j]= sumTIC / i;}
      else
         {StoreTIC_A[j]= TIC_Value;}
       
      sumTIC2 = sumTIC2 + sumTIC / i;
      sumTIC = 0;
      StoreTempA[j]= sumTemp / i;
      sumTemp2 = sumTemp2 + sumTemp / i;
      sumTemp = 0;
      if (opMode == run) 
         {StoreDAC_A[j]= sumDAC / i;}
      else
         {StoreDAC_A[j]= (49999 - timer1CounterValue);}
         
      sumDAC2 = sumDAC2 + sumDAC / i;
      sumDAC = 0;
      i = 0;
      j = j + 1;

      // readings for the 3 hour section
      if (j % 36 == 0) // store every 36 x 300sec (3 hours)
        {
          sumTIC2 = sumTIC2 / 36;
          if (opMode == run) 
            {
              EEPROM.write(k, highByte(sumTIC2));
              EEPROM.write(k+144, lowByte(sumTIC2));
            }else{
              EEPROM.write(k, highByte(TIC_Value));
              EEPROM.write(k+144, lowByte(TIC_Value));
            }
          sumTIC2 = 0;
          
          sumTemp2 = sumTemp2 / 36;
          if (opMode == run) 
            {
              sumTemp2 = sumTemp2 + 20480 ;
              if (lockPPScounter > 10800)
                {
                  sumTemp2 = sumTemp2 + 20480 ;
                }
            }
          
          if (Ltime < 20000) // first after start
            {
              sumTemp2 = sumTemp2 + 10240 ;
            }
            
          EEPROM.write(k+576, highByte(sumTemp2));
          EEPROM.write(k+720, lowByte(sumTemp2));
          sumTemp2 = 0;
          
          sumDAC2 = sumDAC2 / 36;
          
          if (opMode == run) 
            {
              EEPROM.write(k+288, highByte(sumDAC2));
              EEPROM.write(k+432, lowByte(sumDAC2));
            }else{
              EEPROM.write(k+288, highByte(49999 - timer1CounterValue));
              EEPROM.write(k+432, lowByte(49999 - timer1CounterValue));
            }
          
          if (opMode == run && lockPPScounter > 10800) 
            {
              EEPROM.write(1017, highByte(sumDAC2));
              EEPROM.write(1018, lowByte(sumDAC2));
            }
          
          sumDAC2 = 0;
          
          if (j == 144) // 144 x 300sec (12 hours)
            {
              j = 0;
            }
            
          k = k + 1;
          if (k == 144) // 144 x 10800sec (18 days)
            {
              k = 0;
            }
    
          // finally...
          EEPROM.write(1023, k); // store present k (index of 3 hour average, used in setup)
          
          totaltime3h = totaltime3h + 1;
          EEPROM.write(993, highByte(totaltime3h));
          EEPROM.write(994, lowByte(totaltime3h));
          
        } // end of 3 hr section
      } // end of 300 second section

  // storage of old parameters
  timer1CounterValueOld = timer1CounterValue;

  // apply the qErr value
  TIC_ValueOld = TIC_Value - int(qErr); // /0.926 

  TIC_ValueCorrOld = TIC_ValueCorr;  
  timer_us_old = timer_us; 
  timeConstOld = timeConst;
  filterConstOld = filterConst; 
  timeOld = Ltime;
  TIC_ValueFilteredOld = TIC_ValueFiltered;

}  // end of calculation()


////////////////////////////////////////////////////////////////////////////////////////////////
void poll_qErr()
{   
  // Request the data string we are interested in from the NEO
  // For details, look at the UBX protocol : section 32
  Wire.beginTransmission(UBX_ADDR);
  // Every Frame starts with a 2-byte Preamble consisting of two synchronization characters
  // and ends with a 16-bit checksum
  Wire.write(byte(0xB5)); // Sync character 1
  Wire.write(byte(0x62)); // Sync character 2
  // UBX Class TIM : Timing messages - Page 432 
  // TIM-TP 0x0D 0x01 16 bytes returned, Periodic/Polled, Time Pulse time data
  /*
   * This message contains information on the timing of the next pulse at the TIMEPULSE0 output.
   * 
   * The Quantization Error (qErr) is located at Byte Offset 8 has 4 bytes and value is in ps.
   */
  Wire.write(byte(0x0D)); // Message Class
  Wire.write(byte(0x01)); // Message ID
  Wire.write(byte(0x00)); // 1 byte Little Endian length
  Wire.write(byte(0x00)); // 1 byte Little Endian length 
  Wire.write(byte(0x0E)); // 1 byte standard UBX checksum CK_A
  Wire.write(byte(0x37)); // 1 byte standard UBX checksum CK_B
  Wire.endTransmission();     
}

////////////////////////////////////////////////////////////////////////////////////////////////
void get_qErr()
{
// Read the returned data from the NEO and process the qErr data
  Wire.requestFrom( UBX_ADDR, 24 );

  byte tp_resp[24];
  int idx = 0;
  isSetqErr = false;  // added to original code to eliminate the alternate 0 readings

  while(Wire.available())    
    {
      byte b = Wire.read();    
      if (idx <24)
        tp_resp[idx] = b;
      idx++;
      //Serial.print(int(b), HEX);         
      //Serial.print(' ');
    }
  
  // see if we have the right response; if so process it
  if ( (idx==24) && (tp_resp[0]==0xB5) && (tp_resp[1]==0x62) && (tp_resp[2]==0x0D) && (tp_resp[3]==0x01) )
  {
    int32_t ps;
    ps = *( (int32_t*)(tp_resp+14) ); // 4 bytes of qErr

    qErr = ((float)ps)/1000.0; // convert to nano seconds to make it the same as the NS values

    //Serial.print("qErr = ");
    //Serial.print(qErr);
    //Serial.print(" ns");

    if (fabs(qErr)>50.0) // eliminate outside boundary numbers
      qErr = 0;
    else
      isSetqErr = true;
  } 
}


////////////////////////////////////////////////////////////////////////////////////////////////
void getCommand()
{
  char ch;
  long zz; // *** paulv Lars used "z" as the name, but I wanted to use it. Sorry Lars
  
  enum Command {                      // this is the command set
    a = 'a', A = 'A',                 // set damping
    b = 'b', B = 'B',                 // set reference temperature 
    c = 'c', C = 'C',                 // set temperature coefficient
    d = 'd', D = 'D',                 // set dacvalue (followed by a value)
    e = 'e', E = 'E',                 // erase (followed by a value)
    f = 'f', F = 'F',                 // help (followed by a value)
    g = 'g', G = 'G',                 // gain (followed by new value)
    h = 'h', H = 'H',                 // hold (followed by a DAC value note: 0 will set only hold)
    i = 'i', I = 'I',                 // toggles less or more info
    j = 'j', J = 'J',                 // set temperature sensor type
    // k
    l = 'l', L = 'L',                 // set TIC linearization parameters min max square
    m = 'm', M = 'M',                 // set delay for i2c (UR8US calls this Kounter)
    n = 'n', N = 'N',                 // set ID number
    o = 'o', O = 'O',                 // TIC_Offset (followed by new value)
    p = 'p', P = 'P',                 // set prefilter div
    q = 'q', Q = 'Q',                 // apply qErr
    r = 'r', R = 'R',                 // run
    s = 's', S = 'S',                 // save (followed by a value)
    t = 't', T = 'T',                 // time const (followed by new value)
    // u
    // v
    w = 'w', W = 'W',                 // set warmup time (to allow for warm up of oscillator)
    x = 'x', X = 'X',                 // use automatic TC
    // y
    // z
  }; 
  
  if (Serial.available() > 0)       //process if something is there
    {
      ch = Serial.read();
      // process what came in      
      switch(ch) {
     	
        case a:				    // set damping command
        case A:
          zz = Serial.parseInt();  //needs new line or carriage return set in Arduino serial monitor
          if (zz >=50 && zz <= 1000)
            {
              damping = zz / 100.0; 
              Serial.print(F("Damping "));
              Serial.println(damping);
            }
            else { Serial.println(F("Not a valid damping value - Shall be between 50 and 1000"));}
   	    break;	
                         	
        case b:				    // set temperature offset command
        case B:
          zz = Serial.parseInt();
          if (zz >=1 && zz <= 1023)
            {
              tempRef = zz; 
              Serial.print(F("Temperature offset "));
              Serial.println(tempRef);
            }
            else { Serial.println(F("Not a valid temperature offset value - Shall be between 1 and 1023"));}
   	    break;	
                   	
        case c:				    // Set temperature coefficient
        case C:
          zz = Serial.parseInt();
          if (zz >=0 && zz <= 10000)
            {
              tempCoeff = zz; 
              Serial.print(F("Temperature Coefficient "));
              Serial.println(tempCoeff);
            }         
            else if (zz >=10001 && zz <= 20000)
              {
                tempCoeff = (10000-zz);
                Serial.print(F("Temperature Coefficient "));
                Serial.println(tempCoeff);
        	    }           
            else { Serial.println(F("Not a valid temperature coefficient value - Shall be between 0 and 20000"));}
   	    break;
   	        
        case d:				// set dacValue command
        case D:
          zz = Serial.parseInt();
          if (zz >=1 && zz <= 65535)
            {
              dacValue = zz * timeConst; 
              Serial.print(F("dacValue "));
              Serial.println(zz);
            }
            else { Serial.println(F("Not a valid dacValue - Shall be between 1 and 65535"));}
   	    break;
   	    
        case e:				// erase command
        case E:  
          zz = Serial.parseInt();      
          switch (zz) {
            
            case 1:
              Serial.println(F("Erase 3h storage in EEPROM "));	
              for (int i = 0; i < 864; i++)
                {EEPROM.write(i, 0);}          
              EEPROM.write(1023, 0);
              k = 0 ; //reset 3hours counter
        	  break;
            
            case 22:          
              Serial.println(F("Erase all EEPROM to zero"));	
              for (int i = 0; i < 1024; i++)
                {EEPROM.write(i, 0);}
              k = 0 ; //reset 3hours counter
            break;
            
            case 33:          
              Serial.println(F("Erase all EEPROM to -1"));	
              for (int i = 0; i < 1024; i++)
                {EEPROM.write(i, 255);}
              k = 0 ; //reset 3hours counter
            break;
                         
            default:
            Serial.println(F("Not a valid value for erase - Shall be 1 or 22"));
            }                      
        break;	
    
        case f:				// help command
        case F:      
          zz = Serial.parseInt();
          
          switch (zz) {
            
            case 1:          
              Serial.println("");
              Serial.println(F("Info and help - To get values for gain etc type f2 <enter>, f3 <enter> reads ADC3 and f4 <enter> EEPROM"));	
              printHeader1_ToSerial();
              Serial.print("\t");
              printHeader2_ToSerial();
              Serial.println("");
              Serial.println("");
              Serial.println(F("Typing a<value><enter> will set a new damping between between 0.50 and 10.00 set 50 to 1000"));
              Serial.println(F("Typing b<value><enter> will set a new tempRef between 1 and 1023"));
              Serial.println(F("Typing c<value><enter> will set a new tempCoeff set between 0 and 10000. Adding 10000 gives negative tc"));
              Serial.println(F("Typing d<value><enter> will set a new dacValue between 1 and 65535"));
              Serial.println(F("Typing e<value><enter> will erase the 3 hour storage in EEPROM if value 1 and all EEPROM if 22 (33 sets all EEPROM to FF)"));
              Serial.println(F("Typing g<value><enter> will set a new gain between 10 and 65535")); 
              Serial.println(F("  gain = (65536/settable VCOrange in ppb) (eg. 100ppb DACrange gives gain=655)"));
              Serial.println(F("Typing h<value><enter> will set hold mode and the entered dacValue if not h0 that uses the old"));
              Serial.println(F("Typing i<value><enter> with value 1 will toggle ns decimal point else will toggle amount of information "));
              Serial.println(F("Typing j<value><enter> Set temp sensor type 0=raw 1=LM35 2=10kNTC+68k 3=10kNTC+47k (second digit=adc1 eg 3x)"));
              Serial.println(F("Typing l<enter> will set TIC linearization parameters min max square"));
              Serial.println(F("  values 1-500 sets min to 0.1-50, values 800-1023 sets max, values 1024-1200 sets square to 0.024-0.200"));
              Serial.println(F("Typing n<value><enter> will set ID number 0-65535 that is displayed "));
              Serial.println(F("Typing o<value><enter> will set a new TIC_Offset between 10 and 1020 ns"));
              Serial.println(F("Typing p<value><enter> will set a new prefilter div between 2 and 4"));
              Serial.println(F("Typing q<value><enter> will appy (1) or not (0) the qErr correction"));
              Serial.println(F("Typing r<enter> will set run mode"));
              Serial.println(F("Typing s<value><enter> will save gain etc to EEPROM if value 1 and dacvalue if 2"));
              Serial.println(F("Typing t<value><enter> will set a new time constant between 4 and 32000 seconds"));       
              Serial.println(F("Typing w<value><enter> will set a new warmup time between 2 and 1000 seconds"));
              Serial.println(F("Typing x<value><enter> will set autoTC with 1, off with 0"));  // *** paulv added command
              Serial.println("");
              printHeader3_ToSerial();
            break;
           
            case 2:
              Serial.println("");
              Serial.print(F("Gain    ")); Serial.print("\t"); Serial.print(gain); Serial.print("\t");                   
              Serial.print(F("Damping ")); Serial.print("\t"); Serial.print(damping); Serial.print("\t");
              Serial.print(F("timeConst "));  Serial.print("\t"); Serial.print(timeConst); Serial.print("\t");
              Serial.print(F("FilterDiv "));  Serial.print("\t"); Serial.print(filterDiv); Serial.print("\t");
              Serial.print(F("TIC_Offset "));  Serial.print("\t"); Serial.println(TIC_Offset);
              Serial.print(F("TempRef "));  Serial.print("\t"); Serial.print(tempRef); Serial.print("\t");
              Serial.print(F("TempCoeff "));  Serial.print("\t"); Serial.print(tempCoeff); Serial.print("\t");
              Serial.print(F("TICmin  "));  Serial.print("\t"); Serial.print(TICmin,1); Serial.print("\t");
              Serial.print(F("TICmax  "));  Serial.print("\t"); Serial.print(TICmax,0); Serial.print("\t");
              Serial.print(F("Square comp "));  Serial.print("\t"); Serial.println(x2,3); 
              Serial.print(F("Warm up time "));  Serial.print("\t"); Serial.print(warmUptime); Serial.print("\t");
              Serial.print(F("LockPPScounter "));  Serial.print("\t"); Serial.print(lockPPScounter); Serial.print("\t");
              Serial.print(F("MissedPPS "));  Serial.print("\t"); Serial.print(missedPPS); Serial.print("\t");
              Serial.print(F("timeSinceMissedPPS  ")); Serial.println(timeSinceMissedPPS);           
              Serial.print(F("ID_Number  "));  Serial.print("\t"); Serial.print(ID_Number); Serial.print("\t");
              Serial.print(F("Restarts  "));  Serial.print("\t"); Serial.print(restarts); Serial.print("\t");
              Serial.print(F("Total hours"));  Serial.print("\t"); Serial.println(totaltime3h * 3);   
              Serial.print(F("autoTC  "));  Serial.print("\t"); Serial.print(autoTCrun); Serial.print("\t");     // *** paulv added command                        
              Serial.println("");    
              printHeader3_ToSerial();
            break;
            
            case 3:
              Serial.println ("");
              Serial.print (F("ADC3 = "));
              Serial.println(analogRead(A3));
              Serial.println ("");
            break;
            
            case 4:      
              Serial.println ("");
              Serial.println (F("EEPROM content: "));
              Serial.print (F("restarts = "));
              zz=(EEPROM.read(991)*256 + EEPROM.read(992)); Serial.println((unsigned int)zz); 
              Serial.print (F("totaltime3h = "));
              zz=(EEPROM.read(993)*256 + EEPROM.read(994)); Serial.println((unsigned int)zz); 
              Serial.print (F("temperature_Sensor_Type =  "));
              zz=(EEPROM.read(995)*256 + EEPROM.read(996)); Serial.println((unsigned int)zz); 
              Serial.print (F("ID_Number = "));
              zz=(EEPROM.read(997)*256 + EEPROM.read(998)); Serial.println((unsigned int)zz); 
              Serial.print (F("TICmin = "));
              zz=(EEPROM.read(999)*256 + EEPROM.read(1000)); Serial.println((unsigned int)zz); 
              Serial.print (F("TICmax = "));
              zz=(EEPROM.read(1001)*256 + EEPROM.read(1002)); Serial.println((unsigned int)zz); 
              Serial.print (F("x2 = "));
              zz=(EEPROM.read(1003)*256 + EEPROM.read(1004)); Serial.println((unsigned int)zz); 
              Serial.print (F("TIC_Offset = "));
              zz=(EEPROM.read(1005)*256 + EEPROM.read(1006)); Serial.println((unsigned int)zz); 
              Serial.print (F("filterDiv = "));
              zz=(EEPROM.read(1007)*256 + EEPROM.read(1008)); Serial.println((unsigned int)zz); 
              Serial.print (F("warmUptime = "));
              zz=(EEPROM.read(1009)*256 + EEPROM.read(1010)); Serial.println((unsigned int)zz); 
              Serial.print (F("damping = "));
              zz=(EEPROM.read(1011)*256 + EEPROM.read(1012)); Serial.println((unsigned int)zz); 
              Serial.print (F("tempRef = "));
              zz=(EEPROM.read(1013)*256 + EEPROM.read(1014)); Serial.println((unsigned int)zz); 
              Serial.print (F("tempCoeff = "));
              zz=(EEPROM.read(1015)*256 + EEPROM.read(1016)); Serial.println((unsigned int)zz);        
              Serial.print (F("dacValueOut = "));
              zz=(EEPROM.read(1017)*256 + EEPROM.read(1018)); Serial.println((unsigned int)zz); 
              Serial.print (F("gain = "));
              zz=(EEPROM.read(1019)*256 + EEPROM.read(1020)); Serial.println((unsigned int)zz); 
              Serial.print (F("timeConst = "));
              zz=(EEPROM.read(1021)*256 + EEPROM.read(1022)); Serial.println((unsigned int)zz); 
              Serial.print (F("k = "));
              Serial.println (EEPROM.read(1023));         
              Serial.println ("");                    
            break;
            
            default:
              Serial.println(F("Not a valid value for help - Shall be 1 to 4"));    
            }
        	  break;         
                
        case g:				    // gain command
        case G:
          zz = Serial.parseInt();
          if (zz >=10 && zz <= 65534)
            {
              gain = zz; 
              Serial.print(F("Gain "));
              Serial.println(zz);
            }
            else { Serial.println(F("Not a valid gain value - Shall be between 10 and 65534"));}
   	    break;	
        
        case h:             // hold command
        case H:
          zz = Serial.parseInt();
          if (zz >=0 && zz <= 65535)
            {
              opMode = hold;
              newMode = hold;
              Serial.print(F("Hold "));
              holdValue = zz ; 
              Serial.println(holdValue);
            }
            else { Serial.println(F("Not a valid holdValue - Shall be between 0 and 65535"));}                     
        break;
          
        case i:				    	// help command
        case I:      
          zz = Serial.parseInt();
          if (zz == 1)
            { nsDisplayedDecimals = !nsDisplayedDecimals ; }
            else
             { lessInfoDisplayed = !lessInfoDisplayed ; }
        break;
  
        case j:				      // temperature_Sensor_Type       
        case J:
          zz = Serial.parseInt();
          if (zz >=0 && zz <= 99)
            {
              temperature_Sensor_Type = zz; 
              Serial.print(F("temperature_Sensor_Type "));
              Serial.println(zz);
            }
            else { Serial.println(F("Not a valid temperature_Sensor_Type value - Shall be between 0 and 99"));}
   	    break;	
              
        case l:				      // set TIC linearization parameters command
        case L:
          zz = Serial.parseInt();
          if (zz >=1 && zz <= 500)
            {
              TICmin = zz / 10.0; 
              Serial.print(F("TICmin "));
              Serial.println(TICmin);
            }
              else if (zz >=800 && zz <= 1023)
            {
              TICmax = zz; 
              Serial.print(F("TICmax "));
              Serial.println(TICmax);
            }
              else if (zz >=1024 && zz <= 1200)
            {
              x2 = (zz -1000 ) / 1000.0; 
              Serial.print(F("square compensation "));
              Serial.println(x2);
            }
            else { Serial.println(F("Not a valid value"));}
   	    break;	
        
      case m:                 // UR8US calls this 'K'ounter, but is a delay function
      case M:                 // If the delay is too large, the timer (counter) will no longer increment.
        zz = Serial.parseInt();
        if (zz >=0 && zz <= 65535)
          {
          Serial.print(F("m "));
          mValue = zz ; 
          Serial.println(zz);
          }         
        break;

        case n:				      // ID_number      
        case N:
          zz = Serial.parseInt();
          if (zz >=0 && zz <= 65534)
            {
              ID_Number = zz; 
              Serial.print(F("ID_Number "));
              Serial.println(ID_Number);
            }
            else { Serial.println(F("Not a valid ID_Number value - Shall be between 0 and 65534"));}
   	    break;	
              
        case o:				      // TIC_Offset command
        case O:
          zz = Serial.parseInt();
          if (zz >=10 && zz <= 1020) // *** was 200
            {
              TIC_Offset = zz; 
              Serial.print(F("TIC_Offset "));
              Serial.println(TIC_Offset);
            }
            else { Serial.println(F("Not a valid TIC_offset - Shall be between 10 and 1020"));}
   	    break;	
          
        case p:				      // set prefilter div command
        case P:
          zz = Serial.parseInt();
          if (zz >=2 && zz <= 4)
            {
              filterDiv = zz; 
              Serial.print(F("Prefilter div "));
              Serial.println(filterDiv);
            }
            else { Serial.println(F("Not a valid prefilter value - Shall be between 2 and 4"));}
   	    break;	
        
        case r:              // run command
        case R:
          Serial.println(F("Run"));
          opMode = run;
          newMode = run;
        break;
          
        case s:				      // save command
        case S:        
          zz = Serial.parseInt();
          switch (zz) {
            
            case 1:
              Serial.print(F("Saved Gain and timeConstant etc "));//
              EEPROM.write(995, highByte(temperature_Sensor_Type));
              EEPROM.write(996, lowByte(temperature_Sensor_Type));
              EEPROM.write(997, highByte(ID_Number));
              EEPROM.write(998, lowByte(ID_Number));
              EEPROM.write(999, highByte(int(TICmin * 10.0)));
              EEPROM.write(1000, lowByte(int(TICmin * 10.0)));
              EEPROM.write(1001, highByte(int(TICmax)));
              EEPROM.write(1002, lowByte(int(TICmax)));
              EEPROM.write(1003, highByte(int(x2 * 1000.0)));
              EEPROM.write(1004, lowByte(int (x2 * 1000.0)));
              EEPROM.write(1005, highByte(TIC_Offset));
              EEPROM.write(1006, lowByte(TIC_Offset));
              EEPROM.write(1007, highByte(filterDiv));
              EEPROM.write(1008, lowByte(filterDiv));
              EEPROM.write(1009, highByte(warmUptime));
              EEPROM.write(1010, lowByte(warmUptime));
              EEPROM.write(1011, highByte(int (damping *100)));
              EEPROM.write(1012, lowByte(int (damping *100)));
              EEPROM.write(1013, highByte(tempRef));
              EEPROM.write(1014, lowByte(tempRef));
              EEPROM.write(1015, highByte(tempCoeff));
              EEPROM.write(1016, lowByte(tempCoeff));
              EEPROM.write(1019, highByte(gain));
              EEPROM.write(1020, lowByte(gain));
              EEPROM.write(1021, highByte(timeConst));
              EEPROM.write(1022, lowByte(timeConst));
              Serial.println("");
            break;
                      
            case 2: 
              Serial.print(F("Saved DacValue "));
              EEPROM.write(1017, highByte(dacValueOut));
              EEPROM.write(1018, lowByte(dacValueOut));
              Serial.println(""); 
            break;          
            
            default:
              Serial.println(F("Not a valid value for save - Shall be 1 or 2"));
            }
        	break;	  

      case q:             // *** paulv: apply qErr
      case Q:      
        zz = Serial.parseInt();
        if (zz == 1)
            apply_qErr = true;
          else
            apply_qErr = false;

        Serial.print(F("Apply qErr: "));
        Serial.println(apply_qErr ? "yes" : "no");            
        break;

        case t:				  // time constant command
        case T:	      	
          zz = Serial.parseInt();
          if (zz >=4 && zz <= 32000)
            {
              timeConst = zz; 
              Serial.print(F("time constant "));
              Serial.println(timeConst);
            }
            else { Serial.println(F("Not a valid time constant - Shall be between 4 and 32000"));}
        break;	
                 
        case w:				  // set warm up time command
        case W:
          zz = Serial.parseInt();
          if (zz >=2 && zz <= 1000)
            {
              warmUptime = zz; 
              Serial.print(F("Warmup time "));
              Serial.println(warmUptime);
            }
            else { Serial.println(F("Not a valid warmup time - Shall be between 2 and 1000"));}
   	    break;	
        
        case x:         // *** paulv: automatic TC operation, default is on 
        case X:          
          zz = Serial.parseInt();
          if (zz == 1)
            {
              autoTCrun = true; 
            }
          else if (zz == 0)
            {
              autoTCrun = false; 
            }    
          else 
            {Serial.println(F("Not a valid value - Shall be 0 or 1"));}
          Serial.print(F("autoTCrun: "));
          Serial.println(autoTCrun ? "yes" : "no");
        break;
        
        default:
          Serial.println(F("No valid command"));
        break;
      }; // end of switch()
      
      while(Serial.available() > 0) {
        ch = Serial.read();                //flush rest of line
      }
    } // end of if serial processing
} // end of getCommand()
 

////////////////////////////////////////////////////////////////////////////////////////////////
void printDataToSerial()
{
  Serial.print("\t");   // *** paulv : added a tab
  Serial.print((Ltime), DEC);  // time column
  Serial.print("\t"); 
  // print the qErr corrected and filtered TIC value 
  if (TIC_Value == 1023)  // ns column
    {
      Serial.print(F("Missing 10MHz?"));
      Serial.print("\t");              
    }
    else if (nsDisplayedDecimals == false)
    { 
      Serial.print(((float)timer_us *1000) + TIC_ValueCorr - TIC_ValueCorrOffset, 0);
      Serial.print("\t"); 
    }
      else
      { 
        Serial.print(((float)timer_us *1000) + TIC_ValueCorr - TIC_ValueCorrOffset, 1);
        Serial.print("\t"); 
      }
  
  Serial.print(qErr, nsDisplayedDecimals? 2 : 0); // *** paulv display the qErr value.
  Serial.print("\t"); 
  Serial.print(((float)timer_us *1000) + TIC_ValueCorr - TIC_ValueCorrOffset + qErr, nsDisplayedDecimals? 1 : 0); // display the Raw_NS value (add the qErr back in)
  Serial.print("\t"); 

  Serial.print(dacValueOut, DEC);   // DAC column
  Serial.print("\t"); 
  if (temperature_Sensor_Type == 0)   // temp column
    { Serial.print(tempADC2, DEC); }  // ADC2 is PCB temp
  else{
    // *** paulv: IIR Low Pass Filter for temperature sensor readings
    tempADC2_IIR = tempADC2_IIR + ((tempADC2 - tempADC2_IIR) / IIR_Temp_Filter_Weight);
    Serial.print(temperature_to_C(tempADC2_IIR,temperature_Sensor_Type%10), 1); 
  }
  
  Serial.print("\t");  // status column
  if (Ltime > warmUptime && opMode == run)
    { 
     if (PPSlocked == 0) 
       {
        Serial.print(F("****")); // paulv: changed from NoLock to better stand out
        lock = false; // *** paulv:  tracking lock for automatic tuning
       } else {
          Serial.print(F("Locked"));
          lock = true; // *** paulv
       } 
       Serial.print("\t");               
    }
  else if (Ltime > warmUptime)
    {   
      Serial.print(F("Hold"));
      Serial.print("\t");              
    }else{   
      Serial.print(F("WarmUp"));
      Serial.print("\t");              
    }
    
  if (lessInfoDisplayed == false)
    {
      Serial.print(diff_ns, DEC);     // diff_ns column
      Serial.print("\t");
      Serial.print(TIC_ValueFiltered * 10 / filterConst, DEC);  // filtx10 column
      Serial.print("\t");
      Serial.print(timeConst, DEC);   // tc column
      Serial.print("\t");
      Serial.print(filterConst, DEC); // filt column
      Serial.print("\t");  
      Serial.print(((49999-timer1CounterValue)), DEC); // timer1 column
      Serial.print("\t");
  
      // paulv: ADC1 is OCXO temp
      if (temperature_Sensor_Type/10 == 0) // temp1 column
        { Serial.print(tempADC1, DEC); }
      else{
        // *** paulv: IIR Low Pass Filter for temperature sensor readings
        tempADC1_IIR = tempADC1_IIR + ((tempADC1 - tempADC1_IIR) / IIR_Temp_Filter_Weight);
        Serial.print(temperature_to_C(tempADC1_IIR,temperature_Sensor_Type/10), 1); 
      }
      
      Serial.print("\t"); 
      // *** paulv added roomtemp to report
      // added IIR Low Pass Filter for temperature sensor readings
      roomTemp_IIR = roomTemp_IIR + ((roomTemp - roomTemp_IIR) / IIR_Temp_Filter_Weight);
      Serial.print(roomTemp_IIR, 1);
      Serial.print("\t");  

      // print the additional information    
      if (i == 1)
        {
          Serial.print(F("Five minute averages: TIC+DAC+temp"));
          Serial.print("\t");              
        }
      if (i == 2)
        {
          Serial.print(F("Now acquiring value: "));
          Serial.print(j);
          Serial.print("\t");              
        }
        if ((i >= 4) && (i <= 147))
          {
            Serial.print((i-4), DEC);
            Serial.print("\t");              
            Serial.print((StoreTIC_A[i-4]), DEC);
            Serial.print("\t");              
            Serial.print((StoreDAC_A[i-4]), DEC);
            Serial.print("\t");              
            unsigned int x = StoreTempA[i-4];
            if (temperature_Sensor_Type == 0)
              { Serial.print(x, DEC); }
            else
              { Serial.print(temperature_to_C(x/10, temperature_Sensor_Type%10), 1); }
            Serial.print("\t");               
          }
      if (i == 148)
        {
          Serial.print(F("Three hour averages: TIC+DAC+temp"));
          Serial.print("\t");              
        }
      if (i == 149)
        {
          Serial.print(F("Now acquiring value: "));
          Serial.print(k);
          Serial.print("\t");              
        }
      if ((i >= 150) && (i <=293))
        {
          Serial.print((i-150+1000), DEC);
          Serial.print("\t");              
          Serial.print((EEPROM.read(i-150+0)*256 + EEPROM.read(i-150+144)), DEC);
          Serial.print("\t");   
          unsigned int x = EEPROM.read(i-150+288)*256 + EEPROM.read(i-150+432);
          Serial.print(x, DEC);
          Serial.print("\t");                
          x = EEPROM.read(i-150+576)*256 + EEPROM.read(i-150+720);
          if ((x > 0) && (x < 65535))
            {
              if (temperature_Sensor_Type == 0)
                { Serial.print(x%10240, DEC); }
              else
                { Serial.print(temperature_to_C((x%10240)/10, temperature_Sensor_Type%10), 1); }
            } 
          else 
            { Serial.print(x, DEC); }
              
          Serial.print("\t");
          int y = x/10240;
          switch (y) {
            case 0:
              if (x > 0){Serial.print(F("Hold"));}    
            break;
            case 1:
              Serial.print(F("Restarted+hold"));    
            break;
            case 2:
              Serial.print(F("****"));  // paulv: changed from noLock to better stand out  
            break;
            case 3:
              Serial.print(F("Restarted"));    
            break;
            case 4:
              Serial.print(F("Locked"));    
            break;
          }
          Serial.print("\t");              
        } 
      if (i == 295)
        {
          Serial.print(F("timeConst = "));
          Serial.print(timeConst);
          Serial.print(F(" sec "));
          Serial.print("\t");              
        }
      if (i == 296)
        {
          Serial.print(F("Prefilter = "));
          Serial.print(filterConst);
          Serial.print(F(" sec "));
          Serial.print("\t");              
        }
      if (i == 297)
        {
          Serial.print(F("Damping = "));
          Serial.print(damping);
          Serial.print(F(" Gain = "));
          Serial.print(gain);
          Serial.print("\t");              
        } 
      if (i == 298)
        {
          Serial.print(F("Type f1<enter> to get help+info"));
          Serial.print("\t");              
        }
      if (i == 299)
        {
          printHeader2_ToSerial();
        } 
    } // end of If (lessInfoDisplayed) 
Serial.println("");     
  
}  // end of printDataToSerial()

////////////////////////////////////////////////////////////////////////////////////////////////
void printHeader1_ToSerial()
{
  Serial.print(F("\n\r\n\rArduino GPSDO with 1ns TIC by Lars Walenius")); // paulv: added two new lines in front
}

////////////////////////////////////////////////////////////////////////////////////////////////
void printHeader2_ToSerial()
{
  Serial.print(F("Rev. "));
  Serial.print(FW_VERSION);
  Serial.print(" with paulv V4.1 hardware mods"); // *** paulv: added my version and mod indication
  if ((ID_Number >= 0) && (ID_Number < 65535))
    {
      Serial.print(F("  ID:"));
      Serial.print(ID_Number);
      Serial.print("\t");   
    }    
} // end of printHeader2_ToSerial()

////////////////////////////////////////////////////////////////////////////////////////////////
void printHeader3_ToSerial()
{ 
  Serial.print("\t");       // *** paulv : added tab
  Serial.print(F("time"));
  Serial.print("\t");               
  Serial.print(F("ns"));    // ns with qErr applied
  Serial.print("\t");

  Serial.print(F("qErr"));  // *** paulv: show qErr
  Serial.print("\t");
  Serial.print(F("nsRAW"));
  Serial.print("\t");
  Serial.print(F("dac"));
  Serial.print("\t");              
  Serial.print(F("PCB"));   // *** paulv : changed from temp
  Serial.print("\t");  
  Serial.print(F("status"));
  Serial.print("\t");              
  Serial.print(F("diff_ns"));
  Serial.print("\t"); 
  Serial.print(F("filtX10"));
  Serial.print("\t");
  Serial.print(F("tc"));
  Serial.print("\t");              
  Serial.print(F("filt"));
  Serial.print("\t"); 
  Serial.print(F("timer1"));  
  Serial.print("\t");                           
  Serial.print(F("OCXO")); // *** paulv: changed name from temp1
  Serial.print("\t");  
  Serial.print(F("room")); // *** paulv: added roomTemp
  Serial.print("\n\r"); // *** paulv: added a new line  
} // end of printHeader3_ToSerial()

//////////////////////////////////////////////////////////////////////////////////////////////////////////

float temperature_to_C(int RawADC, int sensor) 
{
  float TempC; 
  float floatADC = RawADC;
  switch (sensor) {

    case 1: //LM35
      TempC = RawADC * 1100.0 / 1024.0 *0.1;
    break;
    case 2: //10k NTC beta 3950 + 68k (15-60C)
      TempC = floatADC * floatADC  * 0.0002536 - floatADC  * 0.2158 + 88.48 - floatADC  * floatADC  * floatADC  * 0.0000001179;   
    break;
    case 3: //10k NTC beta 3950 + 47k (15-65C)
      TempC = floatADC  * floatADC  * 0.0001564 - floatADC  * 0.1734 + 92.72 - floatADC  * floatADC  * floatADC  * 0.00000005572 ;  
    break;
    case 4: //10k NTC beta 3950 + 39k (25-70C)
      TempC = floatADC  * floatADC  * 0.0001667 - floatADC  * 0.181 + 99.21 - floatADC  * floatADC  * floatADC  * 0.00000006085 ;  
    break;
    case 5: //22k NTC beta 3950 + 120k (15-65C)
      TempC = floatADC  * floatADC  * 0.0001997 - floatADC  * 0.1953 + 92.11 - floatADC  * floatADC  * floatADC  * 0.00000008010 ;  
    break;
    case 8: //LM35 if Aref is low
      TempC = RawADC * 1070.0 / 1024.0 *0.1;
    break;
    case 9: //LM35 fahrenheit
      TempC = RawADC * 1100.0 / 1024.0 *0.1;
      TempC = TempC * 1.8 + 32; 
    break;
    default:
      TempC = RawADC;    
    }
    if (RawADC == 0){TempC = 0;}
  return TempC; // Return the Temperature in C or raw

} // end of temperature_to_C()

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  pinMode(ppsLockedLED,OUTPUT);
  pinMode(decharge,INPUT);  // *** paulv: setup the port that will decharge C1

  // *** paulv : setup the i2c interface for the NEO
  Wire.begin(UBX_ADDR); // join i2c bus
  Wire.setClock(100000);

  Serial.begin(9600);
  
  // *** paulv: Set up the DS18B20 Temp sensor connection 
  // We can do it the easy way since there is only one device on the 1 wire bus
  oneWire.search(dsAddress); // search for the address of the device on the bus
  oneWire.reset();
  oneWire.select(dsAddress); // establish the connection so we can start to use it

  // Print info and header in beginning
  printHeader1_ToSerial();
  Serial.print("\t");      // prints a tab
  
  // Read saved data from EEPROM to variables
  unsigned int y; 
  y = EEPROM.read(991)*256 + EEPROM.read(992);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      restarts = y ;
    }  
  y = EEPROM.read(993)*256 + EEPROM.read(994);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      totaltime3h = y ;
    }
  y = EEPROM.read(995)*256 + EEPROM.read(996);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      temperature_Sensor_Type = y ;    
    } 
  y = EEPROM.read(997)*256 + EEPROM.read(998);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      ID_Number = y ;
    }  
  y = EEPROM.read(999)*256 + EEPROM.read(1000);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      TICmin = y / 10.0 ;
    } 
  y = EEPROM.read(1001)*256 + EEPROM.read(1002);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      TICmax = y;
    } 
  y = EEPROM.read(1003)*256 + EEPROM.read(1004);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      x2 = y / 1000.0 ;
    } 
  y = EEPROM.read(1005)*256 + EEPROM.read(1006);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      TIC_Offset = y;
    } 
  y = EEPROM.read(1007)*256 + EEPROM.read(1008);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      filterDiv = y;
    } 
  y = EEPROM.read(1009)*256 + EEPROM.read(1010);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      warmUptime = y;
    } 
  y = EEPROM.read(1011)*256 + EEPROM.read(1012);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      damping = y / 100.0;
    } 
  y = EEPROM.read(1013)*256 + EEPROM.read(1014);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      tempRef = y;
    }  
  y = EEPROM.read(1015)*256 + EEPROM.read(1016);
  if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    { 
      tempCoeff = y;
    }     
  y = EEPROM.read(1017)*256 + EEPROM.read(1018);
  if ((y > 0) && (y < 65535)) // set last stored dacValueOut if not 0 or 65535
    { 
      dacValueOut = y;
    }   
  y = EEPROM.read(1019)*256 + EEPROM.read(1020);
  if ((y >= 10) && (y <= 65534)) // set last stored gain if between 10 and 65534
    { 
      gain = y;
    }      
  y = EEPROM.read(1021)*256 + EEPROM.read(1022);
  if ((y >= 4) && (y <= 32000)) // set last stored timeConst if between 4 and 32000
    { 
      timeConst = y;
    }   
  k = EEPROM.read (1023); // last index of stored 3 hour average
   if ((k > 143)|| (k < 0)) k = 0; //reset if k is invalid (eg with a new processor)
 
  // Set "16bit DAC"  
  valuePWMhigh = highByte(dacValueOut); 
  valuePWMlow = lowByte(dacValueOut);
  analogWrite(PWMhighPin,valuePWMhigh);
  analogWrite(PWMlowPin,valuePWMlow);
  
  // Set initial values  
  dacValue = dacValueOut * timeConst;
  timeConstOld = timeConst;
  filterConstOld = filterConst;
  TIC_ValueFilteredOld = TIC_Offset * filterConst;
  TIC_ValueFiltered = TIC_Offset * filterConst;
  TIC_ValueFilteredForPPS_lock = TIC_Offset * 16;
  tempADC2_Filtered = tempRef * 100;
  
  // Set analog ref to about 1.1 Volt  
  analogReference(INTERNAL);
  TIC_Value = analogRead(A0);// just a dummy read
  
  // Setup timer 1 - counts events on pin D5. Used with 5MHz external clock source (needs to be less than 8MHz). Clock on rising edge.  
  TCCR1A = 0;    // reset timer 1            
  TCCR1B = 0;  
  TCCR1B |= (1 << WGM12); //configure timer1 for CTC mode
  OCR1A = 49999; // timer1 counts 50000 counts instead of 65536 to give same TCNT1 value every second    
  TIMSK1 |= (1 << ICIE1); //interrupt on Capture (1PPS)   
  TCCR1B |= ((1 << CS10) | (1 << CS11) | (1 << CS12)| (1 << ICES1) | (1 << ICNC1)); // start timer 1 and interrupt with noise cancel
  
  // Print info and header in beginning
  printHeader2_ToSerial();
  Serial.println("");      // prints a carriage return  
  Serial.println(F("Type f1 <enter> to get help+info"));
  printHeader3_ToSerial(); 

  //clear  PPS flag and go on to main loop  
  PPS_ReadFlag = false;
} // end of setup()


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoAdvanceTC()
{
  /* *** paulv:
   * 
   * Function to gradually increase the timeConst to have a quick lock after startup and then gradually increase the TC
   * while keeping the lock. For this procedure, I have reduced the lockPPSfactor from 5 to 2 initially, and after a TC of 32,
   * we'll go to a factor of 1 to speed things up even more.
   * If the ns difference is becoming too large, we'll reduce the TC a bit to keep the lock.
   * If we loose the lock all together, we'll decrease the TC a bit more and retry.
   * Lars' original lock calculation is (ns +/- <100) for a period of 5*timeConst
   * look at the definition for lockPPSlimit and lockPPSfactor.
   * 
   * The challenge is to increase the TC while not disturbing the complex filtering and the PID loop.
   *
   * We'll start with a TC of 4 and increase that in steps to the maximum.
   * The lock LED will flash after we have a "lock" with a TC of 4, and will turn steady with a TC at 16 and higher.
   * After a cold start, we have a warm-up period of 1 min. It may take another 2 minutes to get a lock with TC=4.
   * If we can keep the lock, we can be at max TC in about 20 minutes.
  */
  if (Ltime > warmUptime){
    if (lock == true){
      lockTimer++;
      if (timeConst <32) {digitalWrite(ppsLockedLED,0); delay(100);} // flash the lock LED, we're not stable enough yet
      if (lockTimer >= lockPPSfactor*timeConst) { // LockPPSfactor=3 at startup
        // wait for lockPPSfactor*timeConst before we advance 
        if (timeConst >= 64) {lockPPSfactor = 1;} // we should be more stable now, reduce the wait a bit
        if (timeConst >= 32) {timeConst = timeConst + 32;}
        if (timeConst >= 24 && timeConst < 32) {timeConst = 32; lockPPSfactor = 2;}
        if (timeConst >= 16 && timeConst < 24) {timeConst = 24;} 
        if (timeConst == 8)  {timeConst = 16;}
        if (timeConst == 4)  {timeConst = 8;}        
        lockTimer=1; // reset the counter                
      }
      if (timeConst > maxTC){ timeConst = maxTC;} // the maximum
      // see if we can keep the lock going before it drifts away too much and we loose it completely
      if (abs((timer_us *1000) + TIC_ValueCorr-TIC_ValueCorrOffset) > 80){ // use the original formula to get the current timer_us
        // reduce the TC every second to keep the lock while giving the loop some time to recover
        // it could be just a glitch so we don't want to reduce too agressively
        if (timeConst <= 64) {
          timeConst = timeConst - 8;
          lockPPSfactor = 3;
        }else{
          timeConst = timeConst - 16;
          lockPPSfactor = 2;
        } 
      }
      if (timeConst < 4){ timeConst = 4;} // don't go below the minimum
    }else{ 
      // We don't have a lock yet or really have lost the lock, go down agressively to re-obtain a lock
      if (timeConst <= 32) {
        timeConst = timeConst - 8;
        lockPPSfactor = 3;
      }else{
        timeConst = timeConst - 32;
        lockPPSfactor = 2;
      }    
      if (timeConst < 4){ timeConst = 4;} // don't go below the minimum
    }
  }
} //  end of autoAdvanceTC()

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Get and process the room temperature from the DS18B20
 * You cannot use the DallasTemperature library with this code because I suspect
 * it uses interupts and timers internally, disrupting this sketch so you have to do it without
 * I found a solution here: 
 *  https://thecavepearlproject.org/2014/03/12/using-a-ds18b20-temp-sensor-without-a-dedicated-library/
 *  https://docs.arduino.cc/learn/communication/one-wire
*/
void getRoomTemp()
{ 
  byte dsData[12];
  byte dsAddress[8];

  //Set up the DS18B20 Temp sensor connection – there is only a single 1 wire sensor connected
  oneWire.search(dsAddress); // search for an active device on the bus
  oneWire.reset();  // needed before communicating with a device
  oneWire.select(dsAddress); // select the device after a reset
  
  oneWire.write(0x44);  // start a conversion, device reads the internal ADC and copies
                        // the results to the internal scratchpad registers
                        // the device needs a minimum of 750mS to do the conversion, but we can't wait.

  // we use a trick sequence to pick-up the data from the last conversion
  oneWire.reset(); // needed before communicating
  oneWire.select(dsAddress); // select the device after a reset
  oneWire.write(0xBE);  // Tell device to move the data to the Scratchpad with the results from the previous conversion

  // get the first two bytes from the Scratchpad that has the temperature data
  for (int i = 0; i < 2; i++) {
    dsData[i] = oneWire.read();
  }
  
  // process to get a real temperature value
  byte MSB = dsData[1];
  byte LSB = dsData[0];
  float tempRead = ((MSB << 8) | LSB); // use two’s compliment to reorder the data
  roomTemp = tempRead / 16; // convert to Celcius

} // end of getRoomtemp()


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  if (PPS_ReadFlag == true) // set by capture of PPS on D8
  {
    calculation();

    // *** paulv: added following
    if (autoTCrun == true) {
      autoAdvanceTC(); // automatically increase or decrease the timeConst while trying to keep a lock
    }

    // *** paulv: added code to read the room temp sensor
    getRoomTemp();

    getCommand();
    printDataToSerial();
    delay(100); // delay 100 milliseconds to give the PPS locked LED some time on if turned off in next step
    if ((dacValueOut < 3000 || dacValueOut > 62535) && opMode == run)
      { 
        digitalWrite(ppsLockedLED,false); // turn off (flash)LED 13 if DAC near limits  
      }
    PPS_ReadFlag = false;   

    if (apply_qErr)
    {
      isSetqErr = false;    
      poll_qErr();
      delay(mValue);   // this is a delay function for the NEO i2c. When set too high, the timer counter will stop incrementing       
    } 
  }
  
  // timer1 overflow counter // if no 10MHz, time will not increment as no overflows will be registered
  TCNT1old = TCNT1new;  
  TCNT1new = TCNT1; 
  if (TCNT1new < TCNT1old)   // if just got an overflow on TCNT1 may miss some during calc+print etc 
  {
    overflowCount++; // normally will increment every 10ms (50000x200ns) used for time since start(seconds)
    if (overflowCount > 31 && (overflowCount - 30)  % 100 == 0) // sense if more than 1sec since last pps pulse
    {
      Serial.println(F(" No PPS"));         
      digitalWrite(ppsLockedLED,false); // blink the LED two times if no PPS
      delay(50);
      digitalWrite(ppsLockedLED,true);
      delay(20);
      digitalWrite(ppsLockedLED,false);
      delay(100);
      digitalWrite(ppsLockedLED,true);
      delay(20);
      digitalWrite(ppsLockedLED,false); 
      if (overflowCount > 2000 ) newMode = run; // resets timer_us etc after 20s without PPS in calculation function
      if (overflowCount > 20000 )lockPPScounter = 0; // resets locked after about 200secs without PPS
      getCommand(); // make sure we don't miss a user input
      if (Ltime < warmUptime)// avoid incrementing time before pps comes first time
      {
        overflowCount = 0 ;
      }
    }
  }
  
  if (apply_qErr)
  {
    if (!isSetqErr)
    {
      get_qErr(); // collect the error data for the next 1PPS pulse and use it in the next go around
      get_qErr(); // Read once again to clean the pipe
    }else{
      qErr = 0;
    }
  }

}  // end of loop()
