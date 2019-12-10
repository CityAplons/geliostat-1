#define temp_bus  4
#define IRpin     5
#define LED       6
#define BUZZER    7
#define v_pin     A8
#define SD_CS     23
#define use_debug 0
#define HeaderControl 8

#include <SPI.h>
#include <SD.h>
#include <Streaming.h>
#include <TinyGPS.h> 
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "skms5611.h"

File telemetry_file;
TinyGPS gps;
MPU6050 mpu;
SKMS5611 bmp(&Wire);
OneWire oneWire(temp_bus);
DallasTemperature sensors(&oneWire);

void makelog(String message);
String check_file(String filename);
void tone(int32_t duration);

uint8_t mode;
volatile float external_temp;
volatile float internal_temp;
float latitude;
float longitude;
float gps_alt;
volatile float bar_alt;
volatile float bar;
float init_bar;
volatile float accX, accY, accZ;
volatile float voltage;
volatile int charge;
volatile bool gps_fix;
const int toneFreq = 4000;
int rc;
unsigned long t;

void setup() {
  external_temp = 0.;
  internal_temp = 0.;
  latitude = 0.;
  longitude = 0.;
  gps_alt = 0.;
  bar_alt = 0.;
  bar = 0;
  accX = 0.;
  accY = 0.;
  accZ = 0.;
  voltage = 0.;
  charge = 0;
  mode = 0;

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(IRpin, OUTPUT);
  pinMode(HeaderControl, OUTPUT);
  
  //Temp
  sensors.begin(); 
  //sensors.setResolution(10);
  
  if(use_debug) Serial.begin(115200);
  else Serial1.begin(9600);
  
  //SD card init
  if (SD.begin(SD_CS)){
    makelog("SD card initialized.");
  } else {
    makelog("SD card initialization error, check wiring.");
    while (1) {}
  }
  String filename = "flight";
  filename = check_file(filename);
  telemetry_file = SD.open(filename + ".csv", FILE_WRITE);
  if (telemetry_file) {
    makelog("Writing header to file...");
    telemetry_file.println("CHRG,Temp.1,Temp.2,GPS Alt,Bar. Alt,Air Pressure,Latitude,Longitude,AX,AY,AZ");
    telemetry_file.flush();
    makelog("Done");
  } else {
    makelog("Error opening logfile");
  }
    
  //GPS init
  Serial2.begin(9600);
  makelog("GPS initialized.");
     
  //IMU init
  Wire.begin();
  mpu.initialize();
  bmp.connect();
  if(!mpu.testConnection()){
    if(use_debug) Serial.println("IMU initialization error, check wiring.");
    else Serial1.println("IMU initialization error, check wiring."); 
    while (1) {}
  }

  bmp.ReadProm();
  bmp.Readout();
  init_bar = bmp.GetPres();
  
  makelog("IMU initialized.\nExecuting...\n");

  tone(50);
  digitalWrite(LED, HIGH);
  
  delay(2000);
  
  //Turn off led to ensure that this function is executing
  digitalWrite(LED, LOW);
}

void loop() {
  //Radio control
  while (Serial1.available()) {
      Serial.println(Serial1.read());
  }
  if(rc == 2){
    mode == 2;
  }
  else if (rc == 1){ 
    mode = 1;
  }

  //Menu loop
  switch(mode) {
    case 0:
      //System testing
      init_test();
      break;
    case 1:
      //Normal mode
      telemetry();
      break;
    case 2:
      //Power safe mode 
      ps();
      break;
    default:
      telemetry();
      break;
  }

  //Photo
  if ( (millis()-t) > 1500) {
     takePhoto();
     t = millis();
  }

  //Therma-control
  TermaControl(internal_temp);

  //Voltage
  voltage = analogRead(v_pin)/204.6;
  charge = map(voltage, 3.57, 5, 0, 100);
}

void init_test()
{
  //Initialization test
  delay(100);
  //Test
  //GPS
  while(Serial2.available()){ // check for gps data 
    if(gps.encode(Serial2.read()))// encode gps data 
    {
      unsigned long fix_age;
      gps.f_get_position(&latitude, &longitude, &fix_age); 
      if (fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > 5000){
        makelog("No fix detected");
        latitude = 0.;
        longitude = 0.;
        gps_fix = false;
      } else {
        gps_alt = gps.f_altitude();
        if (gps_alt > 50000) gps_alt = 0.;
        gps_fix = true;
      }
    }
  }
  //Temperature
  sensors.requestTemperatures();
  external_temp = sensors.getTempCByIndex(1);
  internal_temp = sensors.getTempCByIndex(0);
  //Barometer
  bmp.ReadProm();
  bmp.Readout();
  bar = bmp.GetPres();
  //Accelerometer
  int16_t X,Y,Z;
  mpu.getAcceleration(&X, &Y, &Z);
  accX = X/16384.;
  accY = Y/16384.;
  accZ = Z/16384.;
  
  printinserial(Serial1);
  delay(500);
}

void ps()
{
  //Power safe mode and beaconing
  digitalWrite(LED, HIGH);
  tone(200);
  delay(100);
  digitalWrite(LED, LOW);
}

void telemetry(){
  //Telemetry
  
  //GPS
  while(Serial2.available()){ // check for gps data 
    if(gps.encode(Serial2.read()))// encode gps data 
    {
      unsigned long fix_age;
      gps.f_get_position(&latitude, &longitude, &fix_age); 
      if (fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > 5000){
        makelog("No fix detected");
        latitude = 0.;
        longitude = 0.;
        gps_fix = false;
      } else {
        gps_alt = gps.f_altitude();
        if (gps_alt > 50000) gps_alt = 0.;
        gps_fix = true;
      }
    }
  }
  
  //Temperature
  sensors.requestTemperatures();
  external_temp = sensors.getTempCByIndex(1);
  internal_temp = sensors.getTempCByIndex(0);
  
  //Barometer
  bmp.ReadProm();
  bmp.Readout();
  bar = bmp.GetPres();
  bar_alt = getAlt(bar,init_bar,bmp.GetTemp()/100.);
  
  //Accelerometer
  int16_t X,Y,Z;
  mpu.getAcceleration(&X, &Y, &Z);
  accX = X/16384.;
  accY = Y/16384.;
  accZ = Z/16384.;
  
  //Writing to file
  telemetry_file.print(charge);
  telemetry_file.print(',');
  telemetry_file.print(internal_temp);
  telemetry_file.print(',');
  telemetry_file.print(external_temp);
  telemetry_file.print(',');
  telemetry_file.print(gps_alt);
  telemetry_file.print(',');
  telemetry_file.print(bar_alt);
  telemetry_file.print(',');
  telemetry_file.print(bar);
  telemetry_file.print(',');
  telemetry_file.print(String(latitude,6));
  telemetry_file.print(',');
  telemetry_file.print(String(longitude,6));
  telemetry_file.print(',');
  telemetry_file.print(accX);
  telemetry_file.print(",");
  telemetry_file.print(accY);
  telemetry_file.print(",");
  telemetry_file.println(accZ);
  telemetry_file.flush();
}

double getAlt(float pressure, float init_p, float temp)
{
  return 8.3145*(temp+273.151)/(0.029*9.807)*log(init_p/pressure);
}

void SendPulse(int pulseWidth)
{ 
  int reps = pulseWidth/23.6;
 
  for(int i=0;i<=reps;i++)
  {
    digitalWrite(IRpin,HIGH);
    delayMicroseconds(11);
    digitalWrite(IRpin,LOW);
    delayMicroseconds(5);
  }
 
}

void takePhoto(void) {
  for(int i=0;i<2;i++)
  {
    //pulse for 2.0 millis
    SendPulse(2000);
    //delay for 27.8 millis
    //using a combination of delay() and delayMicroseconds()
    delay(27);
    delayMicroseconds(800);
    //on pulse for 0.5 millis
    SendPulse(500);
    //delay for 1.5 millis
    delayMicroseconds(1500);
    //on pulse for 0.5 millis
    SendPulse(500);
    //delay for 3.5 millis
    delayMicroseconds(3500);
    //send pulse for 0.5 millis
    SendPulse(500);
    if(i<1)
    {
      delay(63);
    }
  }
}

void TermaControl(float temp){                  
  if (temp>20){    
    digitalWrite(HeaderControl, LOW);               
  }                 
  if (temp<=20){
    digitalWrite(HeaderControl, HIGH);             
  }
}

String check_file(String filename)
{
  int i = 0;
  while(SD.exists(filename + ".csv")) {
    i++;
    if( i == 1 )filename = filename + i;
    else if( i <= 10 ){
      int l = filename.length() - 1;
      filename.remove(l);
      filename = filename + i;
    } else {
      int l = filename.length() - 2;
      filename.remove(l);
      filename = filename + i;
    }
  }
  return filename;
}

void makelog(String message)
{
  if(use_debug) Serial.println(message);
  else Serial1.println(message);
}

void printinserial(UARTClass &s)
{
  s.print("Bat.");
  s.print(charge);
  s.print('\t');
  s.print("Int.t: ");
  s.print(internal_temp);
  s.print('\t');
  s.print("Ext.t: ");
  s.print(external_temp);
  s.print('\t');
  s.print("Alt: ");
  s.print(gps_alt);
  s.print('\t');
  s.print("P: ");
  s.print(bar);
  s.print('\t');
  s.print("Lat: ");
  s.print(String(latitude,6));
  s.print('\t');
  s.print("Long: ");
  s.print(String(longitude,6));
  s.print('\t');
  s.print("Acc. XYZ: ");
  s.print(accX);
  s.print(", ");
  s.print(accY);
  s.print(", ");
  s.print(accZ);
  s.print('\n');
  return;
}

volatile static int32_t toggles;                    // number of ups/downs in a tone burst

void tone(int32_t duration){                        // duration in ms
  toggles = 2*toneFreq*duration/1000;               // calculate no of waveform edges (rises/falls) for the tone burst
  setupTC(TC1,0,TC3_IRQn,toneFreq);                 // Start Timer/Counter 1, channel 0, interrupt, frequency
  }
  
void setupTC(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq){
  pmc_set_writeprotect(false);                       // disable write protection of timer registers
  pmc_enable_periph_clk((uint32_t) irq);             // enable clock / interrupt
  TC_Configure(tc, channel,            
               TC_CMR_TCCLKS_TIMER_CLOCK4 |          // TIMER_CLOCK4: MCK/128=656,250Hz. 16 bits so 656,250/65,536=~10Hz/bit
               TC_CMR_WAVE |                         // Waveform mode
               TC_CMR_WAVSEL_UP_RC );                // Counter running up and reset when = Register C value (rcVal)
  const uint32_t rcVal = VARIANT_MCK/256/freq;       // target value for counter, before it resets
  TC_SetRC(tc, channel, rcVal);
  TC_Start(tc, channel);
  (*tc).TC_CHANNEL[channel].TC_IER =  TC_IER_CPCS;   // IER: CPCS bit enables RC compare interrupt when set
  (*tc).TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;   // IDR: clear CPCS bit = don't disable RC compare interrupt
  NVIC_EnableIRQ(irq);                               // Enable TC3_IRQn in the Nested Vector Interrupt Controller)
  }

void TC3_Handler(void){                              // timer ISR  TC1 ch 0
  TC_GetStatus(TC1,0);
  if (toggles != 0){
    digitalWrite(BUZZER,!digitalRead(BUZZER));     // invert the pin state (i.e toggle)
    if (toggles > 0) toggles--;
    }
  }
