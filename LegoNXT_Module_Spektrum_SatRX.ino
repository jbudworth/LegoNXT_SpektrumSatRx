// Intended to allow an Arduino to mimic a Hitech IRRceiver for Radio Control
// http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIR1032

// Arduino ground -> NXT pin 2 and 3
// Arduino 5V     -> NXT cable pin 4
// Arduino pin A4 -> NXT cable pin 6
// Arduino pin A5 -> NXT cable pin 5

//  For Spektrum Satellite Rx decode Library
// hxxp://www.dogfight.no/2011/01/spectrum-receiver-satellite-to-arduino.html
// Not used, had to be re-written due to major issues

//  For Spektrum Satellite Rx binding code
// http://www.diydrones.com/profiles/blogs/ardupilotmega-spektrum-satellite-rx

#include <Wire.h>
#include <SatelliteReceiver.h>

//  Address for the Arduino to announce itself as
#define ID 0x01
#define ADDRESS ID
#define ADDRESS_SEND 2
#define ADDRESS_RECV (ADDRESS_SEND - 1)

uint8_t SensorVersion[9] = " V0.1   ";
uint8_t SensorName[9] =    "Arduino ";
uint8_t SensorType[9] =    "RadioCon";

byte x;
byte buffer[9] = {
  0,0,0,0,0,0,0,0};
byte sendbuffer[] = {
  0, 0};
int PWM_MODE[] = {
  0, 16, 30, 44, 58, 72, 86, 100, -128};

int Bind_pin = 2;
int Rx_PowerPin = 0;

SatelliteReceiver Rx;

void setup() {
  pinMode(Rx_PowerPin, OUTPUT);
  pinMode(Bind_pin, INPUT);
  digitalWrite(Bind_pin, HIGH);
  if ( digitalRead(Bind_pin) == LOW){
    SpektrumBind(); // Place Sat into bind mode to interface with the Spektrum sat
  }

  /*
  UCSR0B &= ~(1 << TXCIE0); // disable Tx interrupt
   UCSR0B &= ~(1 << TXEN0); // disable USART1 Tx
   PORTD &= ~(1 << PORTD1); // disable pull-up
   DDRD |= (1 << DDD1); // Tx as output
   PORTD |= (1 << PORTD1); // Set HIGH to enable adapter regulator
   */

  digitalWrite(Rx_PowerPin, HIGH);
  Serial.begin(115200); // Receive data from Rx
  Wire.begin(ID);                
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);  
  delay(100);
}

void loop(){
  Rx.getFrame();

  PWM_trans(map(Rx.getRudd(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),0);
  PWM_trans(map(Rx.getThro(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),1);
  PWM_trans(map(Rx.getAile(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),2);
  PWM_trans(map(Rx.getElev(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),3);
  PWM_trans(map(Rx.getGear(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),4);
  PWM_trans(map(Rx.getFlap(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),5);
  PWM_trans(map(Rx.getAux2(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),6);
  //PWM_trans(map(Rx.getAux3(), RXCENTER-RXTRAVEL, RXCENTER+RXTRAVEL, 0, 1000),7);
}

// ---------------Convert to Lego PowerFunctions speeds-----------------------

void PWM_trans(int val, int out){ //7 FWD
  if ( val > 940){
    buffer[out] = PWM_MODE[7];
  }
  else if (val >= 880 && val <= 940){ // 6 FWD
    buffer[out] = PWM_MODE[6];
  }
  else if (val >= 820 && val <= 880){ // 5 FWD
    buffer[out] = PWM_MODE[5];
  }
  else if (val >= 760 && val <= 820){ // 4 FWD
    buffer[out] = PWM_MODE[4];
  }
  else if (val >= 700 && val <= 760){ // 3 FWD
    buffer[out] = PWM_MODE[3];
  }
  else if (val >= 640 && val <= 700){ // 2 FWD
    buffer[out] = PWM_MODE[2];
  }
  else if (val >= 580 && val <= 640){ // 1 FWD
    buffer[out] = PWM_MODE[1];
  }
  else if (val >= 420 && val <= 580){ // FLT
    buffer[out] = PWM_MODE[0];
  }
  else if (val >= 360 && val <= 420){ // 1 REV
    buffer[out] = 0 - PWM_MODE[1];
  }
  else if (val >= 300 && val <= 360){ // 2 REV
    buffer[out] = 0 - PWM_MODE[2];
  }
  else if (val >= 240 && val <= 300){ // 3 REV
    buffer[out] = 0 - PWM_MODE[3];
  }
  else if (val >= 180 && val <= 240){ // 4 REV
    buffer[out] = 0 - PWM_MODE[4];
  }
  else if (val >= 120 && val <= 180){ // 5 REV
    buffer[out] = 0 - PWM_MODE[5];
  }
  else if (val >= 60 && val <= 120){ // 6 REV
    buffer[out] = 0 - PWM_MODE[6];
  }
  else if (val < 60){                // 7 REV
    buffer[out] = 0 - PWM_MODE[7];
  }
}

//---------------------------------I2C Events-------------------------------//

void receiveEvent(int howMany){
  x = Wire.read(); // receive byte as an integer
}

void requestEvent(){
  if (x == 0x00){
    Wire.write(SensorVersion, 8);
  }
  else if (x == 0x08){
    Wire.write(SensorName, 8);
  }
  else if (x == 0x10){
    Wire.write(SensorType, 8);
  }
  else if (x == 0x42){// Channel 1, Motor 1A and 1B
    sendbuffer[0] = buffer[0];
    sendbuffer[1] = buffer[1];
    Wire.write(sendbuffer, 2);
  }
  else if (x == 0x44){// Channel 2, Motor 2A and 2B
    sendbuffer[0] = buffer[2];
    sendbuffer[1] = buffer[3];
    Wire.write(sendbuffer, 2);
  }
  else if (x == 0x46){// Channel 3, Motor 3A and 3B
    sendbuffer[0] = buffer[4];
    sendbuffer[1] = buffer[5];
    Wire.write(sendbuffer, 2);
  }
  else if (x == 0x48){// Channel 4, Motor 4A and 4B
    sendbuffer[0] = buffer[6];
    sendbuffer[1] = buffer[7];
    Wire.write(sendbuffer, 2);
  }
} 

//---------------------------------Spektrum Bind Routine-------------------------------//
void SpektrumBind(void)
{
  unsigned long time;
  unsigned char connected = 0;

  /*
  UCSR0B &= ~(1 << TXCIE0); // disable Tx interrupt
   UCSR0B &= ~(1 << TXEN0); // disable USART1 Tx
   PORTD &= ~(1 << PORTD1); // disable pull-up
   DDRD |= (1 << DDD1); // Tx as output
   PORTD |= (1 << PORTD1); // Set HIGH to enable adapter regulator
   */


  // Connect the power for the Rx to pin this is brought
  // high and turns on a transistor.
  digitalWrite(Rx_PowerPin, HIGH);

  UCSR0B &= ~(1 << RXCIE0); // disable Rx interrupt
  UCSR0B &= ~(1 << RXEN0); // disable USART1 Rx
  PORTD &= ~(1 << PORTD0); // disable pull-up

  while(time <= 10000) // Wait 10 seconds for spektrum sat connection
  {
    time = millis();
    if(PIND & (1 << PORTD0))
    {
      connected = 1;
      break;
    }
  }

  if(connected)
  {

    DDRD |= (1 << DDD0); // Rx as output
    delay(90); // Delay after Rx startup

    // === Update 2011-08-18 ===
    // Bind mode data gathered from Spektrum DX8
    // 2 low pulses: DSM2 1024/22ms (this works with Doug Weibel's PPM Encoder firmware)
    // 3 low pulses: no result
    // 4 low pulses: DSM2 2048/11ms
    // 5 low pulses: no result
    // 6 low pulses: DSMX 22ms
    // 7 low pulses: no result
    // 8 low pulses: DSMX 11ms

    PORTD &= ~(1 << PORTD0);    
    delayMicroseconds(116);  // 1 LOW
    PORTD |= (1 << PORTD0);     
    delayMicroseconds(116);
    PORTD &= ~(1 << PORTD0);    
    delayMicroseconds(116);  // 2 LOW
    PORTD |= (1 << PORTD0);     
    delayMicroseconds(116);
    //PORTD &= ~(1 << PORTD0);    delayMicroseconds(116);  // 3 LOW
    //PORTD |= (1 << PORTD0);     delayMicroseconds(116);
    //PORTD &= ~(1 << PORTD0);    delayMicroseconds(116);  // 4 LOW
    //PORTD |= (1 << PORTD0);     delayMicroseconds(116);
    //PORTD &= ~(1 << PORTD0);    delayMicroseconds(116);  // 5 LOW
    //PORTD |= (1 << PORTD0);     delayMicroseconds(116);
    //PORTD &= ~(1 << PORTD0);    delayMicroseconds(116);  // 6 LOW
    //PORTD |= (1 << PORTD0);     delayMicroseconds(116);
    //    PORTD &= ~(1 << PORTD0); delayMicroseconds(116);   // 7 LOW
    //    PORTD |= (1 << PORTD0); delayMicroseconds(116);    
    //    PORTD &= ~(1 << PORTD0); delayMicroseconds(116);   // 8 LOW
    //    PORTD |= (1 << PORTD0); delayMicroseconds(116);   

  }
  else {
    DDRD &= ~(1 << DDD0); // Rx as input
  }
  PORTD &= ~(1 << PORTD0);
}


