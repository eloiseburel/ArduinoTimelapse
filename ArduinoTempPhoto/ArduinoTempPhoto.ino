#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include "DFRobot_SHT20.h"  //bibliotheque capteur de temperature SHT20

//On vérifie que c'est bien la caméra 2MP Plus ou 5MP Plus qui a été branchée
#if !(defined (OV5640_MINI_5MP_PLUS)||defined (OV5642_MINI_5MP_PLUS)||defined (OV_2640_MINI_2MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

// A modifier en fonction du nombre de photos par prise souhaité
// 0x00 = 1 photo/prise jusqu'à 0x06 = 7 photos/prise
#define   FRAMES_NUM    0x06		//3 photos/prise
// set pin 7 as the slave select for the digital pot:
#define CS 7
// set pin 4 for seeed and 10 for adafruit
#define SD_CS 10

//Nécessaires pour la prise de photo
bool is_header = false;
int total_time = 0;

DFRobot_SHT20    sht20;      // SDA (bleu), SCL (blanc), GND (vert), 3.3V ou 5V (rouge)

#if defined (OV2640_MINI_2MP_PLUS)
ArduCAM myCAM( OV2640, CS );
#endif
#if defined (OV5640_MINI_5MP_PLUS)
  ArduCAM myCAM(OV5640, CS);
#endif
#if defined (OV5642_MINI_5MP_PLUS)
  ArduCAM myCAM(OV5642, CS);
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

//Permet d'obtenir la fréquence de prise de photo en fonction de la température
bool hourly_mode() {
  if (sht20.readTemperature() > 12.0)
    return true;    //hourly mode
  else
    return false;   //daily mode
}

//Procédure de prise de photo et gestion des flags
void take_picture(ArduCAM myCAM) {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
#if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
#endif
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.OV5640_set_JPEG_size(OV5640_1600x1200);delay(1000);
#endif
#if defined (OV5642_MINI_5MP_PLUS)
  myCAM.OV5642_set_JPEG_size(OV5642_1600x1200);delay(1000);
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
  init_rtc();
  init_sleep();
  sht20.initSHT20(); //capteur de temperature //sht20.checkSHT20();
  
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

#if defined (OV5640_MINI_5MP_PLUS)
while(1){
  //Check if the camera module type is OV5640
  myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x40)){
    Serial.println(F("Can't find OV5640 module!"));
    delay(1000); continue;
  }else{
    Serial.println(F("OV5640 detected."));break;      
  }
}
#endif

#if defined (OV5642_MINI_5MP_PLUS)
while(1){
  //Check if the camera module type is OV5642
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)){
    Serial.println(F("Can't find OV5642 module!"));
    delay(1000);continue;
  }else{
    Serial.println(F("OV5642 detected."));break;      
  }
}
#endif

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

  //Si mode 1 prise/heure et heure actuelle dans l'intervalle souhaité alors on prend un photo
  if ( hourly_mode() && (currentHour < 19) && (currentHour > 9)) {
    take_picture(myCAM);
    Serial.println("Hourly mode");
  }
  //Si mode 2 prises/jour et heure actuelle correspondante alors on prend un photo
  if ( (!hourly_mode()) && (currentHour == 16 || currentHour == 9)) {
    take_picture(myCAM);
    Serial.println("Daily mode");
  }

  Serial.println("Temperature = ");
  Serial.println(sht20.readTemperature());
  Serial.println("Heure = ");
  Serial.println(rtc.now().toString("hh"));

  delay(200);
  sleep();
}

//Procédure de sauvegarde des photos
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  String pic_name;
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static char k = 'a';
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
      /*
      if (k == ('a' + FRAMES_NUM))
        k = 'a';
      else
        k = k + 1;
      */
      k=k+1;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}

void onAlarm() {}
