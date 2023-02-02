#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <RH_RF95.h>


#define DATE_FORMAT "YYYY-MM-DDThh:mm:ss"			//Format de l'horodatage dans la trame LoRa
#define FRAME_SIZE (sizeof(DATE_FORMAT)+2)		//Taille de la trame LoRa

// Singleton instance of the radio driver
SoftwareSerial ss(4,5);
RH_RF95 rf95(ss);

//On vérifie que c'est bien la caméra 2MP Plus qui a été branchée
#if !(defined (OV2640_MINI_2MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

// A modifier en fonction du nombre de photos par prise souhaité
// 0x00 = 1 photo/prise jusqu'à 0x06 = 7 photos/prise
#define   FRAMES_NUM    0x02		//3 photos/prise
// set pin 7 as the slave select for the digital pot:
#define CS 7
// set pin 4 for seeed shield, pin 10 for adafruit shield
#define SD_CS 10

//Nécessaires pour la prise de photo
bool is_header = false;
int total_time = 0;

#if defined (OV2640_MINI_2MP_PLUS)
ArduCAM myCAM( OV2640, CS );
#endif

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

uint8_t read_fifo_burst(ArduCAM myCAM);

//Procédure de prise de photo et gestion des flags
void take_picture(ArduCAM myCAM) {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
#if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);		//Le paramètre peut être modifié en fonction de la taille de la photo souhaitée
#endif
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start capture."));
  total_time = millis();
  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println(F("CAM Capture Done."));
  total_time = millis() - total_time;
  Serial.print(F("capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  total_time = millis();
  read_fifo_burst(myCAM);
  total_time = millis() - total_time;
  Serial.print(F("save capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
}

void setup() {
  init_sleep();
  init_rtc();
  
  uint8_t vid, pid;
  uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif
  Serial.begin(115200);
  Serial.println(F("ArduCAM Start!"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    } else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
#if defined (OV2640_MINI_2MP_PLUS)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module!"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV2640 detected.")); break;
    }
  }
#endif

  if(!rf95.init()) {
      Serial.println("init failed");
      while(1);
  } 
  Serial.println("LoRa radio init OK!");
  rf95.setFrequency(434.0);

  //Initialize SD Card
  while (!SD.begin(SD_CS))
  {
    Serial.println(F("SD Card Error!")); delay(1000);
  }
  Serial.println(F("SD Card detected."));
  //Change to JPEG capture mode and initialize the camera module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.clear_fifo_flag();
  myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
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
	  //Si mode 2 prises/jour et heure actuelle correspondante alors on prend un photo
          if ((currentHour == 16 || currentHour == 9)) {
            take_picture(myCAM);
          }
        }
        else if(buf[0]=='1') {
 	  //Si mode 1 prise/heure et heure actuelle dans l'intervalle souhaité alors on prend un photo
          if ((currentHour < 16) && (currentHour > 9)) {
            take_picture(myCAM);
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
          Serial.println("La trame n'a pas la taille attendue");
          Serial.println(i); 
        }

        
        //Mise à jour de l'horloge avec le temps reçu par trame LoRa
        DateTime dt((char*)(buf+2));
        rtc.adjust(dt);
        Serial.print("new date : ");
        char format[] = DATE_FORMAT;
        Serial.println(rtc.now().toString(format));  
        delay(200);
        
        //Accusé de réception de la trame LoRa
/*
        uint8_t data[] = "OK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();	
*/
        
    }
    else
    {
        Serial.println("Erreur dans la reception");
    }
  }
    
    sleep();		//endormissement de la carte Arduino
}

//Procédure de sauvegarde des photos
uint8_t read_fifo_burst(ArduCAM myCAM)
{ 
  String pic_name;
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  File outFile;
  byte buf[256];
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      //Create a avi file
      pic_name = rtc.now().toString("MMDDhh");
      pic_name += k;
      pic_name += ".jpg";
      Serial.println("Save pic with name : ");
      Serial.print(pic_name);
      //Open the new file
      outFile = SD.open(pic_name, O_WRITE | O_CREAT | O_TRUNC);
      if (! outFile)
      {
        Serial.println(F("File open failed"));
        while (1);
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
      //new index for pic
      
      k=k+1;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}

void onAlarm() {}