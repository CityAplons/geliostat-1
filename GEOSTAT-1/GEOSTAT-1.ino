#define LED 2
#define BUZZER 3
#define SD_CS 23
#define IRpin 5
#define steps 30
#define temp_bus 4
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
#include "skms5611.h"

File telemetry_file;
TinyGPS gps;
MPU6050 mpu;
SKMS5611 bmp(&Wire);
OneWire oneWire(temp_bus);
DallasTemperature sensors(&oneWire);

uint8_t mode;
float external_temp;
float internal_temp;
float latitude;
float longitude;
float gps_alt;
float bar_alt;
float bar;
float init_bar;
float accX, accY, accZ;
bool gps_fix;
const int toneFreq = 4000;
uint8_t iterator = steps;

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
  mode = 1;

  digitalWrite(LED, HIGH);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(IRpin, OUTPUT);
  
  //Temp
  sensors.begin(); 
  sensors.setResolution(10);
  
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
    telemetry_file.println("Internal temp.,External temp.,GPS Alt,Bar. Alt,Air Pressure,Latitude,Longitude,AX,AY,AZ");
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
  bmp.connect();
  if(!mpu.testConnection()){
    if(use_debug) Serial.println("IMU initialization error, check wiring.");
    else Serial1.println("IMU initialization error, check wiring."); 
    while (1) {}
  }

  bmp.ReadProm();
  bmp.Readout();
  init_bar = bmp.GetPres();
  
  log("IMU initialized.\nExecuting...\n");

  tone(50);
  digitalWrite(LED, LOW);
  
  delay(2000);
  Serial.println("CHEEZE :D");
}

void loop() {
  if(iterator == 0) {
    iterator = steps;
    takePhoto();
    Serial.println("CHEEZE :D");
  }
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
  iterator--;
}

void init_test()
{
  //Initialization test
}

void ps()
{
  //Power safe mode and beaconing
}

void telemetry(){
  //Turn off led to ensure that this function is executing
  digitalWrite(LED, HIGH);
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
  bmp.ReadProm();
  bmp.Readout();
  bar = bmp.GetPres();
  bar_alt = getAlt(bar,init_bar,bmp.GetTemp()/100);
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

  delay(500);
}

double getAlt(float pressure, float init_p, float temp)
{
  return 8.3145*(temp+273.151)/(0.029*9.807)*log(init_p/pressure);
}

void takePhoto(void) {
  int i;
  for (i = 0; i < 76; i++) {
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(7);
  }
  delay(27);
  delayMicroseconds(810);
  for (i = 0; i < 16; i++) {
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(7);
  }
  delayMicroseconds(1540);
  for (i = 0; i < 16; i++) {
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(7);
  }
  delayMicroseconds(3545);
  for (i = 0; i < 16; i++) {
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(7);
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
