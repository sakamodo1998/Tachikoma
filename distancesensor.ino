#define LED_PIN 13
byte val = 0; //----------for serial 2 raspbery

int serialStartFlag = 0;
int x, y;

float buff[2];
float sensorData[5];

float Vcc = 5.0; //-------get distance
float distance_1;
float distance_2;
float getdestance(int port)
{
    distance_1 = Vcc * analogRead(port) / 1023;     //(5.0V*センサ数値/1023)1023は5V入力時の値
    distance_2 = 26.549 * pow(distance_1, -1.2091); //距離換算
    return distance_2;
}

void setup()
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);
}

void loop()
{
    digitalWrite(13, HIGH);
    for (x = 0; x <= 5; x++)
    {
        for (y = 0; y < 2; y++)
        {
            buff[y] = getdestance(x);
        }
        sensorData[x] = (buff[0] + buff[1]) / 2;
    }
    for (x = 0; x <= 5; x++)
    {
        Serial.print(getdestance(x));
        if (x < 5)
        {
            Serial.print(",");
        }
        else
        {
            Serial.println("$");
        }
    }
    delay(10);
    //Serial.println("LED OFF");
    digitalWrite(13, LOW);
    //delay(1000);
}
