/* Ce programme correspond à la partie réception du message LoRa.
Le programme est similaire à celui nommé LoRa_Arduino_Photo.ino, mais la prise
de photo a été retirée pour des raisons d'occupation mémoire. */

#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <RH_RF95.h>

#define DATE_FORMAT "YYYY-MM-DDThh:mm:ss"			//Format de l'horodatage dans la trame LoRa
#define FRAME_SIZE (sizeof(DATE_FORMAT)+2)		//Taille de la trame LoRa

// Singleton instance of the radio driver
SoftwareSerial ss(4,5);
RH_RF95 rf95(ss);

// set pin 7 as the slave select for the digital pot:
#define CS 7

#define CLOCK_INTERRUPT_PIN 2

// ======================================== SLEEP ==========================================
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

// ========================================= RTC ===========================================
//Initiation de la Real Time Clock presente sur le shiend Datalogging Arduino Shield PCF8523
#include "RTClib.h" // Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
RTC_PCF8523 rtc;

void init_rtc(){
  // Init RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  //ajustement de l'horloge de la RTC avec celle de l'ordinateur
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();

  //Activation du compte a rebour
  rtc.enableCountdownTimer(PCF8523_FrequencyMinute , 1, PCF8523_LowPulse10x64Hz  ); // Unite d'attente, nombre d'unité a attendre, duree de l'impulsion d'interruption // PCF8523_FrequencyHour
}
// =========================================================================================

void setup() {
  Serial.begin(115200);

  init_sleep();
  init_rtc();
  
  if(!rf95.init()) {
      Serial.println("init failed");
      while(1);
  } 
  Serial.println("LoRa radio init OK!");
  rf95.setFrequency(434.0);
}


void loop() {

  //Get current time
  DateTime now = rtc.now();
  //Get current hour
  uint8_t currentHour = rtc.now().hour();  

  if(rf95.waitAvailableTimeout(30000))		//attente d'un message LoRa pendant 30sec
  {
    //Récupération de la trame de données reçue  
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if(rf95.recv(buf, &len))
    {
        Serial.print("Message recu : ");        
        Serial.println((char*)buf);
        delay(200);

	  //Le premier élément de la trame reçue correspond à la fréquence de prise de photo
        if(buf[0]=='0') {
          if ((currentHour == 16 || currentHour == 9)) {
            Serial.println("Start capture");		//Remplacement de la prise de photo par un message console
          }
        }
        else if(buf[0]=='1') {
          if ((currentHour < 16) && (currentHour > 9)) {
            Serial.println("Start capture");
          }
        }
        else {
          Serial.println("No mode detected");
        }

        unsigned char i = 1;
        while(buf[i]!='\0') {		//Détermination de la taille de la trame reçue
          i++;
        }

        if ((i+1) != FRAME_SIZE) {
          Serial.println("La trame n'a pas la taille attendue : ");
          Serial.println(i); 
        }

        //Mise à jour de l'horloge avec le temps reçu par trame LoRa
        DateTime dt((char*)(buf+2));
        rtc.adjust(dt);
        Serial.print("new date : ");
        char format[] = DATE_FORMAT;
        Serial.println(rtc.now().toString(format));  
        delay(200);

    }
    else
    {
        Serial.println("Erreur dans la reception");
    }
  }
    
    sleep();		//endormissement de la carte Arduino
}

void onAlarm() {}
