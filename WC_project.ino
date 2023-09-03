//********************************************************************************//
//                      PARAMETERS NEED TO BE SET                                 //
//********************************************************************************//

int default_limit_length_detection = 30;    //in cm
int signal_detect_counter = 5;              //in seconds
int ident = 1;                             //0->reciver ; 10->for setting parameters
int danger_zone_time = 15;                  //yellow color in seconds
float brightness_RGB = 0.15;                //led brightness (set between 0-1)
int sensor_high_detect = 33;                //set only if ident is set on 10 

//*********************************************************************************//
//                      START OF THE PROGRAM                                       //
//*********************************************************************************//
#include "pitches.h"
#include <esp_now.h>
#include <WiFi.h>
//*******************LED AND BUTTON PINS DEFINES**********************************//
#define GREEN_LED_WOMAN 12
#define YELLOW_LED_WOMAN 14
#define RED_LED_WOMAN 13
#define GREEN_LED_MAN 26
#define YELLOW_LED_MAN 27
#define RED_LED_MAN 25
#define ECHOPIN 18      //echo pin for sensor
#define TRIGPIN 19      //trigger pin for sensor
#define Buzzer 33
#define Reciver_not_working 23    //led for no connection signal
#define Set_sensor_detection 22     //button for sensor limit detection  

//Reserved BUTTON pins
#define LED2 34
#define LED3 35
#define LED5 32

//reserved LED pins
#define BUTTON 2
#define BUTTON 4
#define BUTTON 5
#define BUTTON 15

//**********************VARIABLES DEFINE******************************************//

int brightness = 0;
boolean Button_state = LOW;
int Button_pressed_time = 0;
int limit_length_detection;
bool MAN_state = LOW;
bool WOMAN_state = LOW;
int Woman_esp32_no_signal = 0;
int Man_esp32_no_signal = 0;
int WOMAN_esp_signal_check = 0;
int MAN_esp_signal_check = 0;
int reciver_check_signal = 0;
int yellow_led_state = 0;
int yellow_led_timer_woman = 15;
int yellow_led_timer_man = 15;
int on_detection_counter = 0;
int off_detection_counter = 0;
int BUTTONstate_new;
int BUTTONstate;
int distance;
bool buttonDown = false;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct struct_message {     // Define data structure
  bool a;    //stanje tipke
  float b;    //izpis zaznane višine
  int c;      //ident
  int d;      //nastavitev mejne višine
  int e;      //nastavitev svetlosti led 
} struct_message;

struct_message myData;


struct {     // Define data structure
  int red;
  int green;
  int blue;
}

//****RGB COLLORS FEFINE*****//

Blue = {0, 254, 0},
Red = {254, 0, 0},
Pink = {0, 254, 0},
Yellow = {254, 200, 0};
//Pink = {254,20,147}  //original

//**************************//



//settings for song GoT
int melody[] = {    // Game of Thrones
  //https://github.com/robsoncouto/arduino-songs/blob/master/gameofthrones/gameofthrones.ino
  NOTE_G4, -4, NOTE_C4, -4,
  NOTE_DS4, 16, NOTE_F4, 16, NOTE_G4, 4, NOTE_C4, 4, NOTE_DS4, 16, NOTE_F4, 16,
  NOTE_D4, -1,
};
int tempo = 85, divider = 0, noteDuration = 0;;
int wholenote = (60000 * 4) / tempo;      //calculates duration of a whole note in ms
int notes = sizeof(melody) / sizeof(melody[0]) / 2;


//********************************************************************************//
//                           FUNCTIONS DEFINE                                     //
//********************************************************************************//

void LED_Initialize(void) {
  // assign a led pins to a channel
  ledcAttachPin(RED_LED_MAN, 0);
  ledcAttachPin(GREEN_LED_MAN, 1);
  ledcAttachPin(YELLOW_LED_MAN, 2);
  ledcAttachPin(RED_LED_WOMAN, 3);
  ledcAttachPin(GREEN_LED_WOMAN, 4);
  ledcAttachPin(YELLOW_LED_WOMAN, 5);
  // 12 kHz PWM, 8-bit resolution
  ledcSetup(0, 4000, 8);
  ledcSetup(1, 4000, 8);
  ledcSetup(2, 4000, 8);
  ledcSetup(3, 4000, 8);
  ledcSetup(4, 4000, 8);
  ledcSetup(5, 4000, 8);
}


void Check_for_transmission_error(void) {
  if (reciver_check_signal < 5) {
    reciver_check_signal++;
    digitalWrite(Reciver_not_working, LOW);
  }
  if (reciver_check_signal == 5) {
    digitalWrite(Reciver_not_working, HIGH);
  }
  if (ident == 0) {
    if (MAN_esp_signal_check < 5) {
      MAN_esp_signal_check++;
    }
    if (MAN_esp_signal_check == 5) {
      MAN_state = !MAN_state;
      digitalWrite(YELLOW_LED_MAN, MAN_state);
      digitalWrite(RED_LED_MAN, MAN_state);
      digitalWrite(GREEN_LED_MAN, MAN_state);
      yellow_led_timer_man = danger_zone_time;
    }
    if (WOMAN_esp_signal_check < 5) {
      WOMAN_esp_signal_check++;
    }
    if (WOMAN_esp_signal_check == 5) {
      WOMAN_state = !WOMAN_state;
      digitalWrite(YELLOW_LED_WOMAN, WOMAN_state);
      digitalWrite(RED_LED_WOMAN, WOMAN_state);
      digitalWrite(GREEN_LED_WOMAN, WOMAN_state);
      yellow_led_timer_woman = danger_zone_time;
    }
  }
}


void noTone(byte pin) {
  ledcSetup(0, 0, 0); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, 0); // play tone
}


void tone(byte pin, int freq, int note) {
  ledcSetup(0, 0, note); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
}


void song(void) {
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    divider = melody[thisNote + 1];     // calculates the duration of each note
    if (divider > 0) {
      noteDuration = (wholenote) / divider;     // regular note, just proceed
    }

    else if (divider < 0) {     // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }
    tone(Buzzer, melody[thisNote], noteDuration * 0.9);       // play 90%, 10% as a pause
    delay(noteDuration);
    noTone(Buzzer);     //stop music
  }
}


void Check_distance(void) {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIGPIN, LOW);
  distance = pulseIn(ECHOPIN, HIGH, 26000);
  distance = distance / 58;
  delay(50);
}


//******************DATA RECIVE FUNCTION***************************//
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {

  struct_message* myData = (struct_message*) data;
  switch (myData->c) {

    case 0:
      reciver_check_signal = 0;
      break;

    //************esp1 WOMAN ******************//

    case 1:
      Woman_esp32_no_signal = 0;
      WOMAN_esp_signal_check = 0;
      if (myData->a == true) {
        //digitalWrite(YELLOW_LED_WOMAN, LOW);
        //digitalWrite(RED_LED_WOMAN, HIGH);
        //digitalWrite(GREEN_LED_WOMAN, LOW);
        ledcWrite(3, Red.red * brightness_RGB); // set the brightness of the LED
        ledcWrite(4, Red.green * brightness_RGB);
        ledcWrite(5, Red.blue * brightness_RGB);
        yellow_led_timer_woman = 0;

      }
      else {
        if (yellow_led_timer_woman < danger_zone_time) {
          //digitalWrite(RED_LED_WOMAN, LOW);
          //digitalWrite(YELLOW_LED_WOMAN, HIGH);
          ledcWrite(3, Yellow.red * brightness_RGB); // set the brightness of the LED
          ledcWrite(4, Yellow.green * brightness_RGB);
          ledcWrite(5, Yellow.blue * brightness_RGB);
          yellow_led_timer_woman++;
        }
        else {
          //digitalWrite(RED_LED_WOMAN, LOW);
          // digitalWrite(YELLOW_LED_WOMAN, LOW);
          //digitalWrite(GREEN_LED_WOMAN, HIGH);
          ledcWrite(3, Pink.red * brightness_RGB); // set the brightness of the LED
          ledcWrite(4, Pink.green * brightness_RGB);
          ledcWrite(5, Pink.blue * brightness_RGB);
        }
      }
      Serial.print("WOMAN Sensor detected:  ");
      Serial.print(myData->e);
      Serial.println(" cm    ");
      break;


    //*************esp1 MAN ******************//

    case 2:
      Man_esp32_no_signal = 0;
      MAN_esp_signal_check = 0;
      if (myData->a == true) {
        ledcWrite(0, Red.red * brightness_RGB); // set the brightness of the LED
        ledcWrite(1, Red.green * brightness_RGB);
        ledcWrite(2, Red.blue * brightness_RGB);
        yellow_led_timer_man = 0;
      }
      else {
        if (yellow_led_timer_man < danger_zone_time) {
          ledcWrite(0, Yellow.red * brightness_RGB); // set the brightness of the LED
          ledcWrite(1, Yellow.green * brightness_RGB);
          ledcWrite(2, Yellow.blue * brightness_RGB);
          yellow_led_timer_man++;
        }
        else {
          ledcWrite(0, Blue.red * brightness_RGB); // set the brightness of the LED
          ledcWrite(1, Blue.green * brightness_RGB);
          ledcWrite(2, Blue.blue * brightness_RGB);
        }
      }
      Serial.print("MAN Sensor detected:  ");
      Serial.print(myData->e);
      Serial.println(" cm    ");
      break;


    //********** ESP for set parameters*********//
    
    case 10:
      limit_length_detection = myData->d;
      brightness_RGB = myData->b;
      break;

      
    default:
      break;
  }
}


//********************************************************************************//
//                             END OF FUNCTIONS                                   //
//********************************************************************************//
void setup() {

  pinMode(RED_LED_MAN, OUTPUT);
  pinMode(GREEN_LED_MAN, OUTPUT);
  pinMode(RED_LED_WOMAN, OUTPUT);
  pinMode(GREEN_LED_WOMAN, OUTPUT);
  pinMode(YELLOW_LED_MAN, OUTPUT);
  pinMode(YELLOW_LED_WOMAN, OUTPUT);
  pinMode(ECHOPIN, INPUT_PULLUP);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(Reciver_not_working, OUTPUT);
  pinMode(Set_sensor_detection, INPUT);
  digitalWrite(ECHOPIN, HIGH);
  pinMode(Buzzer, OUTPUT);

  LED_Initialize();

  myData.c = ident;
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    delay(3000);
    ESP.restart();
    return;
  }

  // register peer
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;


  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  limit_length_detection = default_limit_length_detection;
}

//********************************************************************************//

void loop() {

  //for easyer dtect which esp is on set COM port
  Serial.print("Ident: ");
  Serial.println(myData.c);
  Serial.println(limit_length_detection);
  

  
  Check_distance();
  Button_state = digitalRead(Set_sensor_detection);     //check button status


  
  //function for set and reset limit sensor detection
  if (Button_state == HIGH) {
    if (Button_pressed_time < 3) {      //press 3s or more
      limit_length_detection = distance;    
      Button_pressed_time++;
    }
    else {
      limit_length_detection = default_limit_length_detection;      //set current position
    }
  }
  if (Button_state == LOW) {
    Button_pressed_time = 0;      //reset button counter for snsor limit set
  }



  if (distance < limit_length_detection) {
    on_detection_counter++;
    off_detection_counter = 0;
    if (on_detection_counter == signal_detect_counter) {
      buttonDown = true;      //sensor on
      myData.a = buttonDown;    //save sensor state to structure
      myData.e = distance;
      myData.d = sensor_high_detect;
      myData.b = brightness_RGB;
      on_detection_counter = 0;
    }
  }

  else {
    off_detection_counter++;
    on_detection_counter = 0;
    if (off_detection_counter == signal_detect_counter) {
      buttonDown = false;       // Reset the button state
      myData.a = buttonDown;
      myData.e = distance;
      myData.d = sensor_high_detect;
      myData.b = brightness_RGB;
      off_detection_counter = 0;
    }
  }



  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(struct_message));   //send myData packet to other ESP32

  if (result != ESP_OK) {     //check for successfully transmit
    Serial.println("Error sending the data");
  }
  Check_for_transmission_error();   //check if all esp is connected to Wi-Fi

  delay(1000);
}
