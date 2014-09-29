// FastLED includes
#include <FastLED.h>

#include <Wire.h>
#include "./mpu.h"

#define DEBUG
#define DEBUG_GYRO
#define DEBUG_LEDS
#define DEBUG_BT


// #define NUM_LEDS       181
// #define NUM_LEDS_H1    0
// #define NUM_LEDS_H2    90
// #define NUM_LEDS_Q1    0
// #define NUM_LEDS_Q2    45
// #define NUM_LEDS_Q3    90
// #define NUM_LEDS_Q4    135
#define NUM_LEDS       21
#define NUM_LEDS_H1    0
#define NUM_LEDS_H2    10
#define NUM_LEDS_Q1    0
#define NUM_LEDS_Q2    5
#define NUM_LEDS_Q3    10
#define NUM_LEDS_Q4    15

#define MODE_RAINBOW           0
#define MODE_GYRO_RAINBOW      1
#define MODE_WAVE              2
#define MODE_GYRO_WAVE1        3
#define MODE_POLICE_FLASH      4
#define MODE_STROB             5

#define MODE_TERMOMETER        64    // Этот режим будем включать только вручную

#define MODE_COUNT             6

#define MAX_CHANGE_STEPS       50

CRGB leds[NUM_LEDS];

unsigned char mode = MODE_TERMOMETER;

const int buttonPin = 2;     // номер входа, подключенный к кнопке
const int ledPin =  12;      // номер выхода светодиода
bool checkButton(){
    static int lastButtonState = 0;
    int buttonState = 0;         // переменная для хранения состояния кнопки
    // считываем значения с входа кнопки
    buttonState = digitalRead(buttonPin);

    // проверяем нажата ли кнопка
    // если нажата, то buttonState будет HIGH:
    if (buttonState == HIGH) {
        digitalWrite(ledPin, HIGH);
        if (lastButtonState == 0){
            lastButtonState = 1;
            return true;
        }
        else{
            return false;
        }
    }
    else {
        digitalWrite(ledPin, LOW);
        lastButtonState = 0;
        return false;
    }
}

void initSerial(){
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(19200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
}

void initLEDS(){
    FastLED.addLeds<WS2812, 6, GRB>(leds, NUM_LEDS);
}

#define LED_PIN 13
bool blinkState = false;

void setup() {
    initLEDS();
    initSerial();
    mpu_setup();
    
    // инициализируем пин, подключенный к кнопке, как вход
    pinMode(buttonPin, INPUT);
    pinMode(ledPin, OUTPUT);
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

//void set_led_flashing(int num){
//    for(int i=0; i < NUM_LEDS;i++){
//        if (i == num){
//            leds[i] = CRGB::White;
//        }
//        else if ((i+1)%NUM_LEDS == num){
//            leds[i] = CRGB(128,128,128);
//        }
//        else if ((i+2)%NUM_LEDS == num){
//            leds[i] = CRGB(64,64,64);
//        }
//        else{
//            leds[i] = CRGB::Black;
//        }
//    }
//}

// Создает движущуюся "радугу"
// rspeed - скорость движения радуги
// rlength - "длина волны" - количество диодов охватывающее весь спектр
// насыщенность и яркость - понятно что
// Двухсторонняя направленна и круговая последовательная версии
void rainbowWheel2(float rspeed, uint8_t rlength, uint8_t saturation=255, uint8_t britness=255){
    static float shift = 0;
    const float speed_delimiter = 2.0;
    shift += rspeed / speed_delimiter;
    uint16_t i=0;
    for(; i < NUM_LEDS_H2;i++){
        leds[i] = CHSV( ((i << 8)/rlength + (int)shift) & 0xFF, saturation, britness);
    }
    for(; i < NUM_LEDS;i++){
        leds[NUM_LEDS - (i - NUM_LEDS_H2 + 1)] = CHSV( ((i << 8)/rlength + (int)shift) & 0xFF, saturation, britness);
    }
}
void rainbowWheel(float rspeed, uint8_t rlength, uint8_t saturation=255, uint8_t britness=255){
    static float shift = 0;
    const float speed_delimiter = 2.0;
    shift += rspeed / speed_delimiter;
    uint16_t i=0;
    for(; i < NUM_LEDS;i++){
        leds[i] = CHSV( ((i << 8)/rlength + (int)shift) & 0xFF, saturation, britness);
    }
}

// Создает движущуюся волну одного оттенка
// hue - оттенок
// rspeed - скорость движения радуги
// rlength - "длина волны"
// насыщенность
void wave(uint8_t hue, float rspeed, uint8_t rlength, uint8_t saturation=255){
    static float shift = 0;
    shift += rspeed;
    if (shift > rlength) shift -= rlength;
    for(uint16_t i=0; i < NUM_LEDS;i++){
        static uint16_t brit;
        brit = ((int)(((i + shift) * 512.0) / rlength)) & 0x1FF;
        if (brit & 0x100) brit = 511 - brit;
        leds[i] = CHSV( hue, saturation, brit);
    }
}

// Создает волну на позиции
// hue - оттенок
// rspeed - скорость движения радуги
// rlength - "длина волны"
// насыщенность
void waveAt(uint8_t hue, uint8_t pos, uint8_t rlength, uint8_t saturation=255){
    for(uint16_t i=0; i < NUM_LEDS;i++){
        leds[i] = CRGB::Black;
    }
    for(uint16_t i = pos; i < NUM_LEDS && i < pos + rlength/2; i++){
        leds[i] = CHSV( hue, saturation, 255 - (int)(((i - pos)*255.0)/rlength));
    }
    for(int i = pos; i >= 0 && i > pos - rlength/2; i--){
        leds[i] = CHSV( hue, saturation, 255 - (int)(((pos - i)*255.0)/rlength));
    }
}

// Создает волну на позиции
// hue - оттенок
// rspeed - скорость движения радуги
// rlength - "длина волны"
// насыщенность
void shineAt3(uint8_t hue, uint8_t pos, uint8_t saturation=255){
    if (pos >= NUM_LEDS) return;
    for(uint16_t i=0; i < NUM_LEDS;i++){
        leds[i] = CRGB::Black;
    }
    leds[pos] = CHSV( hue, saturation, 255);
    if (pos > 0) leds[pos - 1] = CHSV( hue, saturation, 128);
    if (pos < NUM_LEDS - 1) leds[pos + 1] = CHSV( hue, saturation, 128);
}
// Создает волну на позиции
// hue - оттенок
// rspeed - скорость движения радуги
// rlength - "длина волны"
// насыщенность
void shine(uint8_t hue, uint8_t pos, uint8_t rlength, uint8_t saturation=255){
    int i;
    for(i = 0; i < NUM_LEDS;i++){
        leds[i] = CRGB::Black;
    }
    for (i = 0; i < rlength; i++){
        int val = i + pos - rlength/2;
        if (val > 0 && val < NUM_LEDS){
            leds[i] = CHSV( hue, saturation, 255 - abs(255 - i*510/rlength));
        }
    }
}

// Эффект полицейской мигалки
// Одна сторона мигает красным, вторая синим
void policeFlash(uint16_t& animate_step){
    const int flash_length = 24;
    const int speed_delimeter = 4;
    int local_step = animate_step / speed_delimeter;            // Замедление анимации
    uint16_t i=0;
    for( i = 0; i < NUM_LEDS ; i++){
        leds[i] = CRGB::Black;
    }
    if (local_step % 2 == 0) return;
    if (local_step < flash_length / 2 - 4){
        for ( i = NUM_LEDS_Q1; i < NUM_LEDS_Q2; i++ ){
            leds[i] = CRGB::Red;
        }
        for ( i = NUM_LEDS_Q4; i < NUM_LEDS; i++ ){
            leds[i] = CRGB::Red;
        }
    }
    
    if (local_step > flash_length / 2 && local_step < flash_length - 4){
        for ( i = NUM_LEDS_Q2; i < NUM_LEDS_Q4; i++ ){
            leds[i] = CRGB::Blue;
        }
    }
    
    animate_step = animate_step % (flash_length*speed_delimeter);
}

// Эффект стробоскопа
void strob(uint16_t& animate_step){
//    const int flash_length = 24;
    const int speed_delimeter = 2;
    int local_step = animate_step / speed_delimeter;            // Замедление анимации
    uint16_t i=0;
    if (local_step % 2 == 0){
        for( i = 0; i < NUM_LEDS ; i++){
            leds[i] = CRGB::Black;
        }
    }
    else {
        for( i = 0; i < NUM_LEDS ; i++){
            leds[i] = CRGB::White;
        }
    }
}

// Показывает температуру
void showTemperature(const int temp){
    uint16_t i=0;
    for( i = 0; i < NUM_LEDS ; i++){
        leds[i] = CRGB::Black;
    }
    
    for( i = 0; i < ((temp*NUM_LEDS_H2)/40) ; i++){
        leds[i] = CRGB::Blue;
    }
    for( i = NUM_LEDS_H2; i < NUM_LEDS_H2 + ((temp*(NUM_LEDS - NUM_LEDS_H2))/40) ; i++){
        leds[i] = CRGB::Blue;
    }
}

void change(uint8_t &chg_step){
    uint16_t i = 0;
    uint8_t value = 255 - abs(255 - (chg_step*510/MAX_CHANGE_STEPS));
    for( i = 0; i < NUM_LEDS ; i++){
        leds[i] = CRGB(value,value,value);
    }
    chg_step--;
}

static uint16_t ani_step = 0;
static uint8_t chg_step = 0;
void loop() {
// blink LED to indicate activity
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);

    mpu_get();
//    Serial.print(F("GYR:"));
//    Serial.print(get_last_x_angle());
//    Serial.print(F(","));
//    Serial.print(get_last_y_angle());
//    Serial.print(F(","));
//    Serial.print(get_last_z_angle());
//    Serial.println();

//    Serial.println(( (double) gyro.getTemperature() + 12412.0) / 340.0);
    if (checkButton()){
        mode = (mode + 1) % MODE_COUNT;
        chg_step = MAX_CHANGE_STEPS;
    }

    if (chg_step){
        change(chg_step);
    }
    else{
        static uint8_t hue = 200;
        switch(mode){
        case MODE_RAINBOW:
            rainbowWheel(6, 20);
            break;
        case MODE_GYRO_RAINBOW:
            rainbowWheel2(get_last_y_angle(), 20 - (abs(get_last_x_angle())/9));
            break;
        case MODE_WAVE:
            hue += get_last_x_angle() / 45.0;
            wave(hue, get_last_y_angle()/30.0, 20);
            break;
        case MODE_GYRO_WAVE1:
            hue += get_last_x_angle() / 45.0;
            shine(hue, (int)((get_last_y_angle() + 90.0)/180.0 * NUM_LEDS), 5);
            break;
//        case MODE_GYRO_WAVE2:
//            shineAt3((uint8_t)(((get_last_y_angle() + 90.0)/90.0)*128), (int)((get_last_x_angle() + 90.0)/180.0 * NUM_LEDS));
//            break;
        case MODE_POLICE_FLASH:
            policeFlash(ani_step);
            break;
        case MODE_STROB:
            strob(ani_step);
            break;
        case MODE_TERMOMETER:
            showTemperature(get_last_temperature());
            break;
        }
    }
    FastLED.show();
    ani_step++;
    delay(10);
}
