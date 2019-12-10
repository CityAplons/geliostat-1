#define BUZZER 3
#define SD_CS 23
#define use_debug 0

#include <SPI.h>
#include <SD.h>
#include <Streaming.h>
#include <Wire.h>
#include "MPU6050_tockn.h"

File telemetry_file;
MPU6050 mpu(Wire);

uint8_t mode;
float accX, accY, accZ;
const int toneFreq = 4000;
void setup() {
  accX = 0.;
  accY = 0.;
  accZ = 0.;

  pinMode(BUZZER, OUTPUT);
  
  if(use_debug) Serial.begin(115200);
  
  //SD card init
  if (SD.begin(SD_CS)){
    log("SD card initialized.");
  } else {
    log("SD card initialization error, check wiring.");
    while (1) {}
  }
  String filename = "shock";
  filename = check_file(filename);
  telemetry_file = SD.open(filename + ".csv", FILE_WRITE);
  if (telemetry_file) {
    log("Writing header to file...");
    telemetry_file.println("Time,AX,AY,AZ");
    telemetry_file.flush();
    log("Done");
  } else {
    log("Error opening logfile");
  }
     
  //IMU init
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(false);
  tone(50);
 
}

void loop() {
  telemetry();
  
}

void telemetry(){
  mpu.update();
  //Accelerometer
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  if(use_debug) printinserial(Serial);
  
  //Writing to file
  telemetry_file.print(millis());
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
}

void printinserial(UARTClass &s)
{
  s.print("Time: ");
  s.print(millis());
  s.print('\t');
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
