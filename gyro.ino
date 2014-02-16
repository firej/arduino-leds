// FastLED includes
#include <FastLED.h>

// gyroscope includes
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DEBUG
#define DEBUG_GYRO
#define DEBUG_LEDS
#define DEBUG_BT


#define NUM_LEDS 10
CRGB leds[NUM_LEDS];


MPU6050 gyro;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void initGyro(){
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    gyro.initialize();

    // verify connection
    gyro.testConnection();

    // load and configure the DMP
    devStatus = gyro.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    gyro.setXGyroOffset(220);
    gyro.setYGyroOffset(76);
    gyro.setZGyroOffset(-85);
    gyro.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        gyro.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = gyro.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = gyro.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
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
    initGyro();
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void set_led_flashing(int num){
    for(int i=0; i < NUM_LEDS;i++){
        if (i == num){
            leds[i] = CRGB::White;
        }
        else if ((i+1)%NUM_LEDS == num){
            leds[i] = CRGB(128,128,128);
        }
        else if ((i+2)%NUM_LEDS == num){
            leds[i] = CRGB(64,64,64);
        }
        else{
            leds[i] = CRGB::Black;
        }
    }
}

// Создает движущуюся "радугу"
// rspeed - скорость движения радуги
// rlenght - "длина волны" - количество диодов охватывающее весь спектр
// насыщенность и яркость - понятно что
void rainbowWheel(uint8_t rspeed, uint8_t rlenght, uint8_t saturation=255, uint8_t britness=255){
    static uint8_t shift = 0;
    shift = (shift + rspeed) & 0xFF;
    for(uint16_t i=0; i < NUM_LEDS;i++){
        leds[i] = CHSV( ((i << 8)/rlenght + shift) & 0xFF, saturation, britness);
    }
}

// Создает движущуюся волну одного оттенка
// hue - оттенок
// rspeed - скорость движения радуги
// rlenght - "длина волны"
// насыщенность
void wave(uint8_t hue, float rspeed, uint8_t rlenght, uint8_t saturation=255){
    static float shift = 0;
    shift += rspeed;
    if (shift > rlenght) shift -= rlenght;
    for(uint16_t i=0; i < NUM_LEDS;i++){
        static uint16_t brit;
        brit = ((int)(((i + shift) * 512.0) / rlenght)) & 0x1FF;
        if (brit & 0x100) brit = 511 - brit;
        leds[i] = CHSV( hue, saturation, brit);
    }
}

// Создает волну на позиции
// hue - оттенок
// rspeed - скорость движения радуги
// rlenght - "длина волны"
// насыщенность
void waveAt(uint8_t hue, uint8_t pos, uint8_t rlenght, uint8_t saturation=255){
    for(uint16_t i=0; i < NUM_LEDS;i++){
        leds[i] = CRGB::Black;
    }
    for(uint16_t i = pos; i < NUM_LEDS && i < pos + rlenght/2; i++){
        leds[i] = CHSV( hue, saturation, 255 - (int)(((i - pos)*255.0)/rlenght));
    }
    for(int i = pos; i >= 0 && i > pos - rlenght/2; i--){
        leds[i] = CHSV( hue, saturation, 255 - (int)(((pos - i)*255.0)/rlenght));
    }
}

// Создает волну на позиции
// hue - оттенок
// rspeed - скорость движения радуги
// rlenght - "длина волны"
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


//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
#define OUTPUT_GRAVITY


// Получаем данные с гироскопа
void refreshGyroData(){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = gyro.getIntStatus();

    // get current FIFO count
    fifoCount = gyro.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        gyro.resetFIFO();
//        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = gyro.getFIFOCount();

        // read a packet from FIFO
        gyro.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_GRAVITY
            // display Euler angles in degrees
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            gyro.dmpGetGravity(&gravity, &q);
            Serial.print("gravity:\t");
            Serial.print(gravity.x);
            Serial.print("\t");
            Serial.print(gravity.y);
            Serial.print("\t");
            Serial.println(gravity.z);
        #endif

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            gyro.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            gyro.dmpGetGravity(&gravity, &q);
            gyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            gyro.dmpGetAccel(&aa, fifoBuffer);
            gyro.dmpGetGravity(&gravity, &q);
            gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            gyro.dmpGetQuaternion(&q, fifoBuffer);
            gyro.dmpGetAccel(&aa, fifoBuffer);
            gyro.dmpGetGravity(&gravity, &q);
            gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            gyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    }
}


void loop() {
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    refreshGyroData();

//    Serial.println(( (double) gyro.getTemperature() + 12412.0) / 340.0);

//    rainbowWheel(3, 20);
//    wave(200, 0.5, 20);
//    waveAt(200, i, 10);
    shineAt3((uint8_t)((gravity.y + 1.0)*128), (int)((gravity.x + 1.0)/2.0 * NUM_LEDS));
    FastLED.show();
}
