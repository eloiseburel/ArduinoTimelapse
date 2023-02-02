#include <Wire.h>           //I2C pour le capteur de temperature SHT20
#include "DFRobot_SHT20.h"  //bibliotheque capteur de temperature SHT20
#include <SoftwareSerial.h> //Serial simulé sur port de sortie pour radio LoRa
#include <RH_RF95.h>        //bibliotheque radio LoRa RF95 434MHz
//#include <SD.h>
#include <RTClib.h>

#define CLOCK_INTERRUPT_PIN 2 //Interruption RTC
#define LORA_TIMEOUT_MS 3000  //Temps d'attente de reponse LoRa
#define FILENAME datalog.txt  //Nom du fichier de datalog.txt
#define DATE_FORMAT "YYYY-MM-DDThh:mm:ss"
#define FRAME_SIZE (sizeof(DATE_FORMAT) + 2)

//Capteur de temperature/humidite SHT20 (en I2C)
DFRobot_SHT20    sht20;      // SDA (bleu), SCL (blanc), GND (vert), 3.3V ou 5V (rouge)

SoftwareSerial ss(4, 5);     // pin 4(Rx) (jaune), pin 5(Tx) (blanc), GND (noir), 3.3V ou 5V (rouge)
RH_RF95 rf95(ss);            // Singleton instance of the radio driver

RTC_DS3231  rtc;

void erreur_init(String cause){
  Serial.print("Init impossible :");
  Serial.println(cause);
  Serial.flush();
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  while(1) { delay(10); } 
}

bool get_hourly(){

  float temp = sht20.readTemperature();

  //determination du mode bi-journalier / chaque-heure en fonction de la temp
  //return false;

  return false; //
}

// ========================== Radio LoRa RF95 (en Software SERIAL) ===================================
void init_RF95(){
  if (!rf95.init()){
    erreur_init("LoRa"); 
  }
  rf95.setFrequency(434.0);
}

void LORA_send_mode_time(){
  char format[] = DATE_FORMAT;
  String str = "";
  if(get_hourly())
    str = "1@";
  else
    str = "0@";
  str += rtc.now().toString(format);

  Serial.println(str);
  rf95.send(reinterpret_cast<uint8_t*>(&str[0]), FRAME_SIZE);
  rf95.waitPacketSent();
}

// ===================================================================================================

// ====================================== RTC DS3231 (I2C) ===========================================
// RTC DS3231 communication I2C, interruption (pin SQW) sur la pin 2 Arduino (CLOCK_INTERRUPT_PIN)
void init_RTC(){
  if (!rtc.begin()) {
    erreur_init("RTC");
    Serial.print("Init impossible :");
  }
  
  //ajustement de l'horloge de la RTC avec celle de l'ordinateur
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  else{ //UNIQUEMENT POUR LA DEMO, necessaire qu'une seule fois
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //    DS3231_A1_PerSecond   Alarm once per second - chaque seconde
  //    DS3231_A1_Second  Alarm when seconds match - chaque minute
  //    DS3231_A1_Minute  Alarm when minutes and seconds match - chaque heure
  //    DS3231_A1_Hour  Alarm when hours, minutes and seconds match - chaque jour
  rtc.setAlarm1(DateTime(0,0,0,0,0,0),DS3231_A1_Second);
  rtc.clearAlarm(1);
}
// ===================================================================================================

// ======================================== SLEEP ====================================================
void init_sleep(){
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  // Activation de l'interruption externe sur pin2 (SQW de la RTC)
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
}

void sleep(){
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
}
// ===================================================================================================

// ============================== Main ===============================================================
//

void setup() {
  Serial.begin(115200);
  sht20.initSHT20(); //capteur de temperature //sht20.checkSHT20();
  init_RF95();
  init_RTC();  
  //init_SD();   
  init_sleep();
  //verif init OK
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(2000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  Serial.println("init OK");
}




void loop() { 

  LORA_send_mode_time(); //envoi des donnees en LoRa
  rtc.clearAlarm(1); //clear le flag d'alarme RTC
  
  sleep();

}

void onAlarm(){
  //ISR RTC
}
