
//------------------------------------------------------------------------------------------------------
//
// Arduino Peak Power Tracking Solar Charger  by Ram Sankar Pillai (www.aaram-engineering.com) 
// Project started Feb 19th 2015.
//  
//    This software implements my Peak Power Tracking Solar Charger using the Arduino Demilove developement
//    board. I'm releasing this software and hardware project as open source. It is free of any restiction for
//    anyone to use. All I ask is that if you use any of my hardware or software or ideas from this project
//    is that you give me credit and add a link to my website www.timnolan.com to your documentation. 
//    Thank you. by Tim Nolan (www.timnolan.com)   5/1/09
//
//    5/1/09  v1.00    First development version. Just getting something to work.
//
//    19/Feb/2014 code modified to suit Ram Sankar Pillai's hardware design.
//    06/Mar/2015 code modified to suit Ram Sankar Pillai's pro PCB, included LED indication
//                included EEPROM storage of watt hours
//    08/Mar/2015 introduced PWM indication on LCD display
//------------------------------------------------------------------------------------------------------

#include <LiquidCrystal.h>    // using the liquid crystal library
//#include <EEPROM.h>
LiquidCrystal lcd(7,8,0,1,2,3);

//------------------------------------------------------------------------------------------------------
// definitions
#define SOL_VOLTS_CHAN A0               // the adc channel to read solar volts
#define SOL_AMPS_CHAN A3                // the adc channel to read solar amps
#define BAT_VOLTS_CHAN A2               // the adc channel to read battery volts
#define FAN_PIN A1                  // LM 35 Temperature is connected to pin A3
#define AVG_NUM 10                      // number of iterations of the adc routine to average the adc readings
#define SOL_AMPS_SCALE 37              // the scaling value for raw adc reading to get solar amps scaled by 60 - (Scale by 54 for MAX4173T, 22 for MAX4173F, 11 for MAX4173H)
#define SOL_VOLTS_SCALE 28            // the scaling value for raw adc reading to get solar volts scaled by 27
#define BAT_VOLTS_SCALE 28            // the scaling value for raw adc reading to get battery volts scaled by 27
#define PWM_PIN 9                    // the output pin for the pwm
//#define PWM_ENABLE_PIN 8            // pin used to control shutoff function of the IR2104 MOSFET driver
#define PWM_FULL 1023                // the actual value used by the Timer1 routines for 100% pwm duty cycle
#define PWM_MAX 100                  // the value for pwm duty cyle 0-100%
#define PWM_MIN 60                  // the value for pwm duty cyle 0-100%
#define PWM_START 90                // the value for pwm duty cyle 0-100%
#define PWM_INC 1                    //the value the increment to the pwm value for the ppt algorithm
#define TRUE 1
#define FALSE 0
#define ON TRUE
#define OFF FALSE
//#define TURN_ON_MOSFETS digitalWrite(PWM_ENABLE_PIN, HIGH)      // enable MOSFET driver
//#define TURN_OFF_MOSFETS digitalWrite(PWM_ENABLE_PIN, LOW)      // disable MOSFET driver
#define ONE_SECOND 50000             //count for number of interrupt in 1 second on interrupt period of 20us
#define LOW_SOL_WATTS 5.00            //value of solar watts scaled by 100 so this is 5.00 watts
#define MIN_SOL_WATTS 1.00            //value of solar watts scaled by 100 so this is 1.00 watts
#define MIN_BAT_VOLTS 11.00          //value of battery voltage scaled by 100 so this is 11.00 volts          
#define MAX_BAT_VOLTS 14.10          //value of battery voltage scaled by 100 so this is 14.10 volts  
#define HIGH_BAT_VOLTS 13.60          //value of battery voltage scaled by 100 so this is 13.00 volts  
#define OFF_NUM 9                  // number of iterations of off charger state
#define yellowLed  10
#define greenLed  11
#define redLed  12
byte solar[8] = //icon for termometer
{
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b00000
};

byte battery[8]=
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};

//------------------------------------------------------------------------------------------------------
// global variables


float pwm = 0;                          //pwm duty cycle 0-100%
float sol_amps;                         // solar amps scaled by 100
float sol_volts;                        // solar volts scaled by 100
float bat_volts;                        // battery volts scaled by 100
float sol_watts;                        // solar watts scaled by 100
int delta = PWM_INC;                  // variable used to modify pwm duty cycle for the ppt algorithm
float old_sol_watts = 0;                // solar watts from previous time through ppt routine scaled by 100
float watts, wattHours;
int disp_page=1; //LCD Display page counter
unsigned long  prev_millis, pwm_disp_millis;
double msec, last_msec, elasped_msec, elasped_time, ampSecs, wattSecs;
enum charger_mode {
  off, on, bulk, bat_float } 
charger_state;    // enumerated variable that holds state for charger state machine


//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------
void setup()                            // run once, when the sketch starts
{
  //
  //  bitClear(ADCSRA,ADPS0) ; 
  //  bitClear(ADCSRA,ADPS1) ;
  //  bitSet(ADCSRA,ADPS2) ;
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  wattHours = 1000;
  analogWrite(PWM_PIN,0);  // setup pwm on PWM pin, 0% duty cycle
  pwm = PWM_START;                     //starting value for pwm  
  charger_state = on;                  // start with charger state as on
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(FAN_PIN,OUTPUT);
  digitalWrite(yellowLed,LOW);
  digitalWrite(greenLed,LOW);
  digitalWrite(redLed,LOW);
  digitalWrite(FAN_PIN,HIGH);
  lcd.begin(16, 2);                    // set up coulumns and rows for LCD. Using a 20x4 display.
  lcd.createChar(1,solar);
  lcd.createChar(2,battery);
}

//------------------------------------------------------------------------------------------------------
// This routine reads and averages the analog inputs for this system, solar volts, solar amps and 
// battery volts. It is called with the adc channel number (pin number) and returns the average adc 
// value as an integer. 
//------------------------------------------------------------------------------------------------------
int read_adc(int channel){

  int sum = 0;
  int temp;
  int i;

  for (i=0; i<AVG_NUM; i++) {            // loop through reading raw adc values AVG_NUM number of times  
    temp = analogRead(channel);          // read the input pin  
    sum += temp;                        // store sum for averaging
    delayMicroseconds(50);              // pauses for 50 microseconds  
  }
  return(sum / AVG_NUM);                // divide sum by AVG_NUM to get average and return it
}

//------------------------------------------------------------------------------------------------------
// This routine uses the Timer1.pwm function to set the pwm duty cycle. The routine takes the value in
// the variable pwm as 0-100 duty cycle and scales it to get 0-1034 for the Timer1 routine. 
// There is a special case for 100% duty cycle. Normally this would be have the top MOSFET on all the time
// but the MOSFET driver IR2104 uses a charge pump to generate the gate voltage so it has to keep running 
// all the time. So for 100% duty cycle I set the pwm value to 1023 - 1 so it is on 99.9% almost full on 
// but is switches enough to keep the charge pump on IR2104 working.
//------------------------------------------------------------------------------------------------------
void set_pwm_duty(void) {

  if (pwm > PWM_MAX) {					// check limits of PWM duty cyle and set to PWM_MAX
    pwm = PWM_MAX;		
  }
  else if (pwm < PWM_MIN) {				// if pwm is less than PWM_MIN then set it to PWM_MIN
    pwm = PWM_MIN;
  }
  if (pwm < PWM_MAX) {
    analogWrite(PWM_PIN,(PWM_FULL * (long)pwm / 100)); // use Timer1 routine to set pwm duty cycle at 20uS period

  }												
  else if (pwm == PWM_MAX) {				// if pwm set to 100% it will be on full but we have 
    analogWrite(PWM_PIN,(PWM_FULL - 1));          // keep switching so set duty cycle at 99.9% and slow down to 1000uS period 

  }												
}													

//------------------------------------------------------------------------------------------------------
//This routine prints the data to the LCD
//------------------------------------------------------------------------------------------------------
void lcd_display1(void) {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(1);
  //lcd.print(" ");
  lcd.print(sol_volts,1);
  lcd.print("V ");
  if ((sol_amps*sol_volts) < 10.0 && (sol_amps*sol_volts) > 0.0) {
    lcd.print("0");
    lcd.print(sol_amps*sol_volts,0);
  }
  else  if ((sol_amps*sol_volts) >= 10.0) {
    lcd.print(sol_amps*sol_volts,0);
  }
  else {
    lcd.print("00");
  }
  lcd.print("W ");


  if ((sol_amps) < 10.0 && sol_amps >0) {
    lcd.print("0");
    lcd.print(sol_amps,1);
  }
  else if (sol_amps >= 10){
    lcd.print(sol_amps,1);
  }
  else {
    lcd.print("00.0");
  }
  lcd.print("A");

  //lcd.print(sol_amps,1);
  //lcd.print("A");

  lcd.setCursor(0, 1);  
  lcd.write(2);
  //lcd.print(" ");
  lcd.print(bat_volts,1);
  lcd.print("V ");
  /*
  if ((sol_amps*bat_volts) < 10.0 && (sol_amps*bat_volts) > 0.0) {
    lcd.print("0");
    lcd.print(sol_amps*bat_volts,0);
  }
  else  if ((sol_amps*bat_volts) >= 10.0) {
    lcd.print(sol_amps*bat_volts,0);
  }
  else {
    lcd.print("00");
  }
  lcd.print("W ");
  */
  lcd.print(wattHours,0);
  lcd.print("Wh");
}


//------------------------------------------------------------------------------------------------------
// This routine reads all the analog input values for the system. Then it multiplies them by the scale
// factor to get actual value in volts or amps. Then it adds on a rounding value before dividing to get
// the result scaled by 100 to give a fractional value of two decimal places. It also calculates the input
// watts from the solar amps times the solar voltage and rounds and scales that by 100 (2 decimal places) also.
//------------------------------------------------------------------------------------------------------
void read_data(void) {
  //sol_volts = read_adc(SOL_VOLTS_CHAN)*0.0282; //old PCB value
  //bat_volts   = read_adc(BAT_VOLTS_CHAN)*0.0276; //old pcb value
  sol_volts = read_adc(SOL_VOLTS_CHAN)*0.02705;
  bat_volts   = read_adc(BAT_VOLTS_CHAN)*0.02613; 
 bat_volts = bat_volts - 0.4; //compensation with respect to digital multimeter

  sol_amps = (read_adc(SOL_AMPS_CHAN));
  sol_amps = (sol_amps/1023.0)*5000;
  sol_amps = 0.9*((sol_amps-2500)/66); //Sensitivity is 66mV
  sol_amps = sol_amps-1.2; // ======= ACS712 error compensation value
  sol_watts = sol_volts*sol_amps;
  //  temperature = (read_adc(TEMP_ADC)*0.488);
}
//------------------------------------------------------------------------------------------------------
// This routine calculates Watt Hours produced
//------------------------------------------------------------------------------------------------------
void power(void)

{


  msec = millis();
  elasped_msec = msec - last_msec; //Calculate how long has past since last call of this function
  elasped_time = elasped_msec / 1000.0; // 1sec=1000 msec
  watts = bat_volts*sol_amps; //Watts now
  ampSecs = (sol_amps*elasped_time); //AmpSecs since last measurement
  wattSecs = ampSecs * sol_volts; //WattSecs since last measurement
  // ampHours = ampHours + ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started
  wattHours = wattHours + wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started
  last_msec = msec; //Store 'now' for next time

}
//------------------------------------------------------------------------------------------------------
// This routine to indicate charger state using Green, Yellow, and red LEDs
//------------------------------------------------------------------------------------------------------

void ledIndication(void) {

  if (charger_state == off){
    digitalWrite(greenLed,LOW);
    digitalWrite(yellowLed,LOW);
    digitalWrite(redLed,HIGH);
  }
  else if ((charger_state == bulk) || (charger_state == on)) {
    digitalWrite(greenLed,LOW);
    digitalWrite(yellowLed,HIGH);
    digitalWrite(redLed,LOW);
  }
  else if (charger_state == bat_float) {
    digitalWrite(greenLed,HIGH);
    digitalWrite(yellowLed,LOW);
    digitalWrite(redLed,LOW);
  }
}

//------------------------------------------------------------------------------------------------------
// This routine is the charger state machine. It has four states on, off, bulk and float.
// It's called once each time through the main loop to see what state the charger should be in.
// The battery charger can be in one of the following four states:
// 
//  On State - this is charger state for MIN_SOL_WATTS < solar watts < LOW_SOL_WATTS. This state is probably
//      happening at dawn and dusk when the solar watts input is too low for the bulk charging state but not
//      low enough to go into the off state. In this state we just set the pwm = 100% to get the most of low
//      amount of power available.
//  Bulk State - this is charger state for solar watts > MIN_SOL_WATTS. This is where we do the bulk of the battery
//      charging and where we run the Peak Power Tracking alogorithm. In this state we try and run the maximum amount
//      of current that the solar panels are generating into the battery.
//  Float State - As the battery charges it's voltage rises. When it gets to the MAX_BAT_VOLTS we are done with the 
//      bulk battery charging and enter the battery float state. In this state we try and keep the battery voltage
//      at MAX_BAT_VOLTS by adjusting the pwm value. If we get to pwm = 100% it means we can't keep the battery 
//      voltage at MAX_BAT_VOLTS which probably means the battery is being drawn down by some load so we need to back
//      into the bulk charging mode.
//  Off State - This is state that the charger enters when solar watts < MIN_SOL_WATTS. The charger goes into this
//      state when it gets dark and there is no more power being generated by the solar panels. The MOSFETs are turned
//      off in this state so that power from the battery doesn't leak back into the solar panel. When the charger off
//      state is first entered all it does is decrement off_count for OFF_NUM times. This is done because if the battery
//      is disconnected (or battery fuse is blown) it takes some time before the battery voltage changes enough so we can tell
//      that the battery is no longer connected. This off_count gives some time for battery voltage to change so we can
//      tell this.
//------------------------------------------------------------------------------------------------------
void run_charger(void) {


  static int off_count = OFF_NUM;

  switch (charger_state) {
  case on:                                        
    if (sol_watts < MIN_SOL_WATTS) {              //if watts input from the solar panel is less than
      charger_state = off;                        //the minimum solar watts then it is getting dark so
      off_count = OFF_NUM;                        //go to the charger off state
      analogWrite(PWM_PIN,0); 
    }
    else if (bat_volts > MAX_BAT_VOLTS) {        // > 14.1V else if the battery voltage has gotten above the float
      charger_state = bat_float;                 //battery float voltage go to the charger battery float state
    }
    else if (sol_watts < LOW_SOL_WATTS) {        //else if the solar input watts is less than low solar watts
      pwm = PWM_MAX;                             //it means there is not much power being generated by the solar panel
      set_pwm_duty();			            //so we just set the pwm = 100% so we can get as much of this power as possible
    }                                            //and stay in the charger on state
    else {                                          
      pwm = ((bat_volts * 10) / (sol_volts / 10)) + 5;  //else if we are making more power than low solar watts figure out what the pwm
      charger_state = bulk;                              //value should be and change the charger to bulk state 
    }
    break;
  case bulk:
    if (sol_watts < MIN_SOL_WATTS) {              //if watts input from the solar panel is less than
      charger_state = off;                        //the minimum solar watts then it is getting dark so
      off_count = OFF_NUM;                        //go to the charger off state

    }
    else if (bat_volts > MAX_BAT_VOLTS) {        //else if the battery voltage has gotten above the float
      charger_state = bat_float;                //battery float voltage go to the charger battery float state
    }
    else if (sol_watts < LOW_SOL_WATTS) {      //else if the solar input watts is less than low solar watts
      charger_state = on;                      //it means there is not much power being generated by the solar panel
      //so go to charger on state
    }
    else {                                     // this is where we do the Peak Power Tracking ro Maximum Power Point algorithm
      if (old_sol_watts >= sol_watts) {        //  if previous watts are greater change the value of
        delta = -delta;			// delta to make pwm increase or decrease to maximize watts
      }
      pwm += delta;                           // add delta to change PWM duty cycle for PPT algorythm 
      old_sol_watts = sol_watts;              // load old_watts with current watts value for next time
      set_pwm_duty();				// set pwm duty cycle to pwm value
    }
    break;
  case bat_float:
    if (sol_watts < MIN_SOL_WATTS) {          //if watts input from the solar panel is less than
      charger_state = off;                    //the minimum solar watts then it is getting dark so
      off_count = OFF_NUM;                    //go to the charger off state
      set_pwm_duty();					
      analogWrite(PWM_PIN,0);
    }
    else if (bat_volts > MAX_BAT_VOLTS) {    //since we're in the battery float state if the battery voltage
      pwm -= 1;                               //is above the float voltage back off the pwm to lower it   
      set_pwm_duty();	

    }
    else if (bat_volts < MAX_BAT_VOLTS) {    //else if the battery voltage is less than the float voltage
      pwm += 1;                              //increment the pwm to get it back up to the float voltage
      set_pwm_duty();

      if (pwm >= 100) {                      //if pwm gets up to 100 it means we can't keep the battery at
        charger_state = bulk;                //float voltage so jump to charger bulk state to charge the battery
      }
    }
    break;
  case off:                                  //when we jump into the charger off state, off_count is set with OFF_NUM
    if (off_count > 0) {                     //this means that we run through the off state OFF_NUM of times with out doing
      off_count--;                           //anything, this is to allow the battery voltage to settle down to see if the  
    }                                        //battery has been disconnected
    else if ((bat_volts > HIGH_BAT_VOLTS) && (bat_volts < MAX_BAT_VOLTS) && (sol_volts > bat_volts)) {
      charger_state = bat_float;              //if battery voltage is still high and solar volts are high
      set_pwm_duty();		                //change charger state to battery float			

    }    
    else if ((bat_volts > MIN_BAT_VOLTS) && (bat_volts < MAX_BAT_VOLTS) && (sol_volts > bat_volts)) {
      pwm = PWM_START;                        //if battery volts aren't quite so high but we have solar volts
      set_pwm_duty();				//greater than battery volts showing it is day light then	
      charger_state = on;                     //change charger state to on so we start charging

    }                                          //else stay in the off state
    break;
  default:
    analogWrite(PWM_PIN,0);
    break;
  }
}
//------------------------------------------------------------------------------------------------------
// Main loop.
// Right now the number of times per second that this main loop runs is set by how long the printing to 
// the serial port takes. You can speed that up by speeding up the baud rate.
// You can also run the commented out code and the charger routines will run once a second.
//------------------------------------------------------------------------------------------------------
void loop()                          // run over and over again
{
  read_data();                         //read data from inputs
  power();
  run_charger();                      //run the charger state machine
  //  delay(50);
  //  read_data();  

  if ((millis() > (prev_millis+ 1500))) { // do this stuff every five seconds 
    prev_millis = millis();		
    read_data(); 
    ledIndication();                        
    lcd_display1();
    if (sol_amps > 2) { 		
      digitalWrite(FAN_PIN,HIGH);
    }
    else{
      digitalWrite(FAN_PIN,LOW);
    }
    //   if (disp_page==1) disp_page=2;
    //    else disp_page=1;
  }

}








