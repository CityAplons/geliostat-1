#define LED 2
#define BUZZER 3
#define SD_CS 23
#define temp_bus 22
#define use_debug 1

#include <SPI.h>
#include <SD.h>
#include <Streaming.h>
#include <TinyGPS.h> 
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
//#include "BMP085.h"

File telemetry_file;
TinyGPS gps;
MPU6050 mpu;
//BMP085 bmp;
OneWire oneWire(temp_bus);
DallasTemperature sensors(&oneWire);

float external_temp;
float internal_temp;
float latitude;
float longitude;
float gps_alt;
//float bar_alt;
float accX, accY, accZ;
bool gps_fix;
const int toneFreq = 4000;
void printinserial(UARTClass &s);

void setup() {
  external_temp = 0.;
  internal_temp = 0.;
  latitude = 0.;
  longitude = 0.;
  gps_alt = 0.;
  //bar_alt = 0.;
  accX = 0.;
  accY = 0.;
  accZ = 0.;

  digitalWrite(LED, HIGH);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  //Temp
  sensors.begin(); 
  
  if(use_debug) Serial.begin(115200);
  else Serial1.begin(115200);
  
  //SD card init
  if (SD.begin(SD_CS)){
    log("SD card initialized.");
  } else {
    log("SD card initialization error, check wiring.");
    while (1) {}
  }
  String filename = "flight";
  filename = check_file(filename);
  telemetry_file = SD.open(filename + ".csv", FILE_WRITE);
  if (telemetry_file) {
    log("Writing header to file...");
    telemetry_file.println("Internal temp.,External temp.,Altitude,Latitude,Longitude,AX,AY,AZ");
    telemetry_file.flush();
    log("Done");
  } else {
    log("Error opening logfile");
  }
    
  //GPS init
  Serial2.begin(9600);
  log("GPS initialized.");
     
  //IMU init
  Wire.begin();
  mpu.initialize();
  //bmp.initialize();
  if(!mpu.testConnection()/* && !bmp.testConnection()*/){
    if(use_debug) Serial.println("IMU initialization error, check wiring.");
    else Serial1.println("IMU initialization error, check wiring."); 
    while (1) {}
  }
  
  log("IMU initialized.\nExecuting...\n");

  tone(50);
  digitalWrite(LED, LOW);
  
  delay(2000);
}

void loop() {
  telemetry();
  digitalWrite(LED, HIGH);
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
        log("No fix detected");
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
  //bmp.setControl(BMP085_MODE_PRESSURE_1);
  //uint32_t pressure = bmp.getPressure();
  //delay(50);
  //Serial.println(pressure);
  //bar_alt = bmp.getAltitude(pressure);
  //Accelerometer
  int16_t X,Y,Z;
  mpu.getAcceleration(&X, &Y, &Z);
  accX = X/16384.;
  accY = Y/16384.;
  accZ = Z/16384.;
  
  if(use_debug) printinserial(Serial);
  else printinserial(Serial1);
  
  //Writing to file
  telemetry_file.print(internal_temp);
  telemetry_file.print(',');
  telemetry_file.print(external_temp);
  telemetry_file.print(',');
  telemetry_file.print(gps_alt);
  telemetry_file.print(',');
  //telemetry_file.print(bar_alt);
  //telemetry_file.print(',');
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

void log(String message)
{
  if(use_debug) Serial.println(message);
  else Serial1.println(message);
}

void printinserial(UARTClass &s)
{
  s.print("Int.t: ");
  s.print(internal_temp);
  s.print('\t');
  s.print("Ext.t: ");
  s.print(external_temp);
  s.print('\t');
  s.print("Alt: ");
  s.print(gps_alt);
  s.print('\t');
  //s.print("Alt: ");
  //s.print(bar_alt);
  //s.print('\t');
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
  const uint32_t rcVal = VARIANT_MCK/256/toneFreq;  // target value for counter, before it resets (= 82 for 4kHz)
  toggles = 2*toneFreq*duration/1000;               // calculate no of waveform edges (rises/falls) for the tone burst
  setupTC(TC1,0,TC3_IRQn,toneFreq);                 // Start Timer/Counter 1, channel 0, interrupt, frequency
  }
  
void setupTC(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t freq){
  pmc_set_writeprotect(false);                       // disable write protection of timer registers
  pmc_enable_periph_clk((uint32_t) irq);             // enable clock / interrupt
//pmc_enable_periph_clk((uint32_t) ID_TC3);          // alternate syntax, using PMC_id instead
  TC_Configure(tc, channel,            
               TC_CMR_TCCLKS_TIMER_CLOCK4 |          // TIMER_CLOCK4: MCK/128=656,250Hz. 16 bits so 656,250/65,536=~10Hz/bit
               TC_CMR_WAVE |                         // Waveform mode
               TC_CMR_WAVSEL_UP_RC );                // Counter running up and reset when = Register C value (rcVal)
  const uint32_t rcVal = VARIANT_MCK/256/freq;       // target value for counter, before it resets
//Serial << "rcVal: " << rcVal << " toggles: " << toggles << '\n';
//TC_SetRA(tc, channel, rcVal/2);                    // could also use Register A for 50% duty cycle square wave
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
  //else noTone();                                   // seems superfluous ?
  }

