#define IM920_RX_PIN 0
#define IM920_TX_PIN 1
#define in 11
#define out 12
#define out_led 3

#include <SoftwareSerial.h>
#include <IM920Driver.h>

int default_status;
int now_high_cou = 0;
int now_low_cou = 0;
unsigned long time_data_fre;
unsigned long now_fre;
unsigned long time_data_high = 0;
unsigned long time_data_low = 0;
unsigned long now_high = 0;
unsigned long now_low = 0;

char buff[IM920_BUFFER_SIZE];
String BUFF;
String str;

SoftwareSerial IM920Serial(IM920_RX_PIN, IM920_TX_PIN);
IM920Driver im920(&IM920Serial);

void setup(){
    pinMode(in, INPUT);
    pinMode(out, OUTPUT);
    pinMode(out_led, OUTPUT);

    default_status = digitalRead(in);
    IM920Serial.begin(19200);  // デフォルトでは19200bps(設定で変更可)
    Serial.begin(19200);
}

void loop(){

    while(im920.available()){
        im920.read();

        if(im920.get_line_changed() != 0){
            continue;
        }
        im920.get_last_line(buff);

        if (strcmp(buff, "OK\r\n") != 0 && strcmp(buff, "NG\r\n") != 0){
            BUFF = buff;
            str = BUFF.substring(11, 13);

            if (str == "31") {
                Serial.println(str);
                digitalWrite(out_led, LOW);
                // Serial.println("TXDU0002 33");
            }

            else if (str == "30") {
                Serial.println(str);
                digitalWrite(out_led, HIGH);
                // Serial.println("TXDU0002 32");
            } 

            else {
            Serial.println(str);
            }
        }
    }
}