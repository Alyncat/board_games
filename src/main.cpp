#include <Arduino.h>
#include <FastLED.h>

#define RX_R200 19
#define TX_R200 20

HardwareSerial r200_serial(2);

#define NUM_LEDS 2
#define DATA_PIN 18
CRGB leds[NUM_LEDS];

unsigned char selectTAG3[26] = {
    0XAA, 0X00, 0X0C, 0X00, 0X13, 0X01, 0X00, 0X00,
    0X00, 0X20, 0X60, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X03,
    0XA3, 0XDD
};
unsigned char ReadMulti[10] = {0XAA, 0X00, 0X27, 0X00, 0X03, 0X22, 0XFF, 0XFF, 0X4A, 0XDD};

unsigned char BlinkLED[17] = {
    0XAA, 0X00, 0X39, 0X00, 0X09, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X04, 0X00, 0X01, 0X47, 0XDD
};

unsigned char selectTAG2[26] = {
    0XAA, 0X00, 0X0C, 0X00, 0X13, 0X01, 0X00, 0X00,
    0X00, 0X20, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X02,
    0X42, 0XDD
};
unsigned int timeSec = 0;
unsigned int timemin = 0;
unsigned int dataAdd = 0;
unsigned int incomedate = 0;
unsigned int parState = 0;
unsigned int codeState = 0;

void setup() {
    Serial.begin(115200);
    pinMode(17, OUTPUT);
    CFastLED::addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Red;
    FastLED.show();
    leds[1] = CRGB::Blue;
    FastLED.show();
    digitalWrite(17, HIGH);
    r200_serial.begin(115200, SERIAL_8N1, RX_R200, TX_R200);
}

void loop() {
    timeSec++;
    if (timeSec >= 50000) {
        timemin++;
        timeSec = 0;
        if (timemin >= 20) {
            timemin = 0;
            r200_serial.write(ReadMulti, 10);
        }
    }
    if (r200_serial.available() > 0) {
        incomedate = r200_serial.read();
        if ((incomedate == 0x02) & (parState == 0)) {
            parState = 1;
        } else if ((parState == 1) & (incomedate == 0x22) & (codeState == 0)) {
            codeState = 1;
            dataAdd = 3;
        } else if (codeState == 1) {
            dataAdd++;
            if ((dataAdd >= 9) & (dataAdd <= 20)) {
                if (dataAdd == 9) {
                    Serial.print("EPC:");
                }
                Serial.print(incomedate, HEX);
            } else if (dataAdd >= 21) {
                Serial.println(" ");
                dataAdd = 0;
                parState = 0;
                codeState = 0;
            }
        } else {
            r200_serial.write(selectTAG3, 26);
            delay(2);
            dataAdd = 0;
            parState = 0;
            codeState = 0;
            r200_serial.write(BlinkLED, 17);
        }
    }
}
