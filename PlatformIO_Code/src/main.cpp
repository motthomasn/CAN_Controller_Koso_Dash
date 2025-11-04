/*
 * Copyright 2020 THOMAS NAUGHTON
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * CBR015-0026_DASH_CONTROLLER_ASSY_CODE
 * Version 1.0  03-06-2020
 * Version 2.0  05-06-2023      CAN IDs updated to match CAN config v3.0
 *                              Shift lights functionality added
 * Version 3.0  29-10-2023      Code adjusted to suit Rev02 PCB layout & functionality
 *                              CAN Rx adjusted to match CAN config v3.1
 *                              Selectable measurement display in place of vehicle speed
 *                              Auto adjustment of shift light LED brighness depending on ambient light conditions
 *                              Added CAN transmission of light intensity value and LED PWM duty cycle
 * Version 3.1  04-11-2025      Added shift lights test mode via CAN message     
 * 
 * 
 * This code is designed for use with the CBR015-0026 DASH CONTROLLER ASSY utilising CBR015-0027 Rev02 PCB layout
 * CAN Config: CBR250RRi_CAN_CONFIG_v3.6.dbc
 * Messages recieved via CAN are MSB First or Big Endian byte order
 * 
 * See documentation for details & overview
 */

#include <Arduino.h>
#include <FlexCAN.h>
#include <SPI.h>
#include <CircularBuffer.hpp>

// ===============================================================================================================
// Declare constants most likely to be adjusted by user in this block

#define TACHUpdatePeriod 20000      // Tach output update rate in µs
#define VSSUpdatePeriod 200000      // VSS output update rate in µs
#define IndUpdatePeriod 500000      // Indicators output update rate in µs
#define can_speed 500000            // CAN-Bus baud rate in bps

#define RPMmin 50                   // minimum engine speed before outputting signal
#define VSSmin 1                    // minimum vehicle speed before outputting signal in kph
#define circ 2                      // tyre circumference in metres
#define pulses 4                    // VSS pulses per rev

#define SHIFTUpdatePeriod 50000     // Shift lights state update rate in µs
#define SHIFTFlashPeriod 50000      // Shift lights flash rate in µs
#define brightnessCheck 1000000     // Frequncy to check ambient light brightness in µs, 1s
#define SHIFTGstage 1               // changeLightStage to switch on green shift light
#define SHIFTYstage 2               // changeLightStage to switch on yellow shift light
#define SHIFTRstage 3               // changeLightStage to switch on red shift light
#define SHIFTBstage 4               // changeLightStage to switch on blue shift light
#define SHIFTFstage 5               // changeLightStage to flash shift lights

// ===============================================================================================================

// Declare SPI pin numbers according to CBR015-0027 Rev00 schematic
#define chipSelect 10
#define mosi 11
#define miso 12
#define sck 13

// Declare other output pins according to CBR015-0027 Rev00 schematic
#define TACHpin 14
#define VSSpin 15
#define EOBDpin 19
#define OILPpin 20
#define SHIFTGpin 9
#define SHIFTYpin 6
#define SHIFTRpin 5
#define SHIFTBpin 23
#define LIGHTpin A3
#define ROTARY1pin 8
#define ROTARY2pin 16

// Define light to LED brightness relationship
#define LIGHT_BUFFER_SIZE 5   // Buffer 5 brightness readings
#define darkness 17           // light sensor reading corresponding to darkness (min 0)
#define sunlight 734          // light sensor reading corresponding to bright sunlight (max 1023)
#define dimLED 1              // LED PWM duty for darkness (min 0)
#define brightLED 255         // LED PWM duty for direct sunlight (max 255)

// Declare can bus and can messages
static CAN_message_t rxmsg;
// only interested in IDs 0x100, 0x101, 0x250, 0x7F0
#define filterID1 0x000
#define maskID1 0x4AE
#define filterID2 0x7F0
#define maskID2 0x7FF
#define txID 0x750      // message ID for light intensity value
static CAN_message_t txmsg;

// Declare variables used by timers
uint32_t currentMicros;
uint32_t PrevTACHUpdateMicros=0, PrevVSSUpdateMicros=0, PrevIndUpdateMicros=0, PrevSHIFTUpdateMicros=0, PrevSHIFTFlashMicros=0, PrevBrightCheckMicros=0, PrevCANtxMicros=0;

// Declare variables used by dig out pins
// TACH
bool TACHpinState = LOW;
uint32_t previousTACHpinMicros=0, TACHperiod=UINT32_MAX;
int16_t RPM=0;

// VSS
bool VSSpinState = LOW;
uint32_t previousVSSpinMicros=0, VSSperiod=UINT32_MAX;
float facVSSperiod;
uint8_t VSS=0, *VSSdisp;

// EOBD
uint8_t engineEnable=0; // initialise at 0 => OK
bool EOBDpinState=LOW;

// OILP
bool OILPpinState=LOW;
uint8_t lowEopLight=0; // initialise at 0 => OFF

// Shift Lights
uint8_t changeLightStage=0, ledPWM=0;
uint16_t ambLight;
CircularBuffer<uint16_t, LIGHT_BUFFER_SIZE> lightBuffer;
bool shiftGState=LOW, shiftYState=LOW, shiftRState=LOW, shiftBState=LOW, shiftFlashState=LOW, shiftTest=false;

float ECT, EOT;
uint8_t TPS=0, GEAR=2;

// Declare AD5235
// Source: https://forum.arduino.cc/index.php?topic=356088.0
// based on code provided by GROZEA Ion
//www.grozeaion.com
//Control Registers
#define CMD__NOTHING B00000000 //0  - Do nothing
#define CMD_MEM2RDAC B00010000 //1  - Restore EEMEM (A0) contents to RDAC (A0) register. See Table 16.
#define CMD_RDAC2MEM B00100000 //2  - Store wiper setting. Store RDAC (A0) setting to EEMEM (A0). See Table 15. - Use a delay of 50ms!!!
#define CMD_USER2MEM B00110000 //3  - Store contents of Serial Register Data Byte 0 and Serial Register Data Bytes 1 (total 16 bits) to EEMEM (ADDR). See Table 18.- Use a delay of 50ms!!!
#define CMD_DECRE6DB B01000000 //4  - Decrement by 6 dB. Right-shift contents of RDAC (A0) register, stop at all 0s.
#define CMD_DEALL6DB B01010000 //5  - Decrement all by 6 dB. Right-shift contents of all RDAC registers, stop at all 0s.
#define CMD_DECR1STP B01100000 //6  - Decrement contents of RDAC (A0) by 1, stop at all 0s.
#define CMD_DECA1STP B01110000 //7  - Decrement contents of all RDAC registers by 1, stop at all 0s.
#define CMD_ALL2RDAC B10000000 //8  - Reset. Refresh all RDACs with their corresponding EEMEM previously stored values. - Use a delay of 30us!!!
#define CMD_GETEMEM B10010000  //9  - Read contents of EEMEM (ADDR) from SDO output in the next frame. See Table 19. - Use a delay of 30us!!!
#define CMD_GET_RDAC B10100000 //10 - Read RDAC wiper setting from SDO output in the next frame. See Table 20. - Use a delay of 30us!!!
#define CMD_SET_RDAC B10110000 //11 - Write contents of Serial Register Data Byte 0 and Serial Register Data Byte 1 (total 10 bits) to RDAC (A0). See Table 14.
#define CMD_INCRE6DB B11000000 //12 - Increment by 6 dB: Left-shift contents of RDAC (A0),stop at all 1s. See Table 17.
#define CMD_INALL6DB B11010000 //13 - Increment all by 6 dB. Left-shift contents of all RDAC registers, stop at all 1s.
#define CMD_INCR1STP B11100000 //14 - Increment contents of RDAC (A0) by 1, stop at all 1s. See Table 15.
#define CMD_IALL1STP B11000000 //15 - Increment contents of all RDAC registers by 1, stop at all 1s.

SPISettings settingsA(4000000, MSBFIRST, SPI_MODE0); // Default setting: 4MHz bus speed, MSB first byte order & Mode0

#define ECTwiper 0    // ECT pot is wiper 0
#define EOTwiper 1    // EOT pot is wiper 1
#define Beta 4005.678 // beta value for Koso BF030000 sensor between 25-100degC
#define R0 49120      // Ohms
#define T0 298.15     // degK
float logR2, R2, ECTResistance, EOTResistance, Rmin, AD5235inECT, AD5235inEOT;
uint8_t myVal = 0;
// Steinhart-Hart constants for Koso BF030000 sensor
//#define c1 0.007579331099
//#define c2 0.0002332380527
//#define c3 0.00000006080582592

// ===============================================================================================================
uint8_t transferData(uint8_t data)
{
  SPDR = data;                  // send the data
  while (!(SPSR & (1 << SPIF))) // wait until transmission is complete
    ;
  return SPDR;
}

// ===============================================================================================================
uint8_t PT65112(uint8_t pin1, uint8_t pin2)
{
  // function to read state of PT65112 rotary switch
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  bool state1 = !digitalRead(pin1); // invert truthism here as pins are active when pulled low
  bool state2 = !digitalRead(pin2);

  return state1 | (state2<<1);

}

union uint16_split {
  uint16_t value;
  uint8_t bytes[2];
};

// ===============================================================================================================
void setWiper(uint8_t w, uint16_t value)
{
  //SPI.beginTransaction(settingsA); // allow access to SPI bus
  digitalWrite(chipSelect, LOW); // pull CS low
  myVal = transferData(CMD_SET_RDAC + w);
  myVal = transferData(value >> 8);
  myVal = transferData(value & 0xFF);
  digitalWrite(chipSelect, HIGH); // set CS high
  //SPI.endTransaction(); // remove access to SPI bus
}

// ===============================================================================================================
void can_recieve()
{

  if (Can0.available())
  {

    Can0.read(rxmsg);

    switch (rxmsg.id)
    {

    case 0x100:
      RPM = ((int16_t((rxmsg.buf[1]) | (rxmsg.buf[0] << 8))));         //rpm
      VSS = uint8_t(round((int16_t((rxmsg.buf[3]) | (rxmsg.buf[2] << 8))) * 0.036)); //kph No decimal place display on speedometer
      break;
    case 0x101:
      if (!shiftTest)
      {
        // ignore changeLightStage from ECU if test mode active
        changeLightStage = uint8_t(rxmsg.buf[1]); 
      }
      // changeLightState = uint8_t(rxmsg.buf[3]);
      lowEopLight = uint8_t(rxmsg.buf[5]);
      engineEnable = uint8_t(rxmsg.buf[7]);
      break;
    case 0x250:
      ECT = ((int16_t((rxmsg.buf[1]) | (rxmsg.buf[0] << 8))) * 0.1); //degC
      EOT = ((int16_t((rxmsg.buf[3]) | (rxmsg.buf[2] << 8))) * 0.1); //degC
      TPS = uint8_t(round((int16_t((rxmsg.buf[5]) | (rxmsg.buf[4] << 8))) * 0.012207031)); //% No decimal place display on speedometer
      GEAR = uint8_t(rxmsg.buf[7]);
      break;
    case 0x7F0:
      shiftTest = bool( 1 & rxmsg.buf[0] ); // bitwise AND to extract bit 0
      if (shiftTest)
      {
        // update changeLightStage from tester message if bit0 active
        changeLightStage = uint8_t(rxmsg.buf[0] >> 1); // right shift by 1 to remove bit 0 request
      }
    }
  }
}

// ===============================================================================================================
// CAN Send
void CANsend()
{
  uint16_split light;
  light.value = ambLight;

  // ID 0x750
  txmsg.buf[1] = light.bytes[0]; // LSB
  txmsg.buf[0] = light.bytes[1]; // MSB

  txmsg.buf[2] = ledPWM;

  Can0.write(txmsg);
}
// ===============================================================================================================
void shiftLBrightness()
{
  uint16_t lightReading = analogRead(LIGHTpin); // read light sensor pin

  // add light reading to buffer. Buffer is used because there is a lot of variance in the light readings.
  // i.e. shining a bright LED light directly at the sensor provides a generally high reading of ~1010 but these readings are often interspersed with readings of ~30 for 1 or 2 readings in between
  lightBuffer.push(lightReading); 

  ambLight = 0; // init
  for (decltype(lightBuffer)::index_t i = 0; i < lightBuffer.size(); i++) {
    // mathematically, max(a+b) = ( (a+b) + |(a-b)| ) / 2
    ambLight = ( (ambLight+lightBuffer[i]) + abs((ambLight-lightBuffer[i])) ) / 2;
  }

  // Serial.print("Brightness reading: ");
  // Serial.println(lightReading);
  // Serial.print("Brightness buffer max: ");
  // Serial.println(ambLight);

  ambLight = constrain(ambLight, darkness, sunlight); // constrain readings to predefined limits

  ledPWM = map(ambLight, darkness, sunlight, dimLED, brightLED); // determine corresponding PWM duty

  CANsend();
}

// ===============================================================================================================
void UpdateIndicators()
{

  // check oil pressure and set pin state
  if ( ( (lowEopLight == 1) || (engineEnable == 100) ) && (OILPpinState == LOW) )
  {
    // only change & write pin state if required
    OILPpinState = HIGH;
    digitalWrite(OILPpin, OILPpinState);
  }
  else if ( (lowEopLight == 0) && (engineEnable != 100) && (OILPpinState == HIGH) )
  {
    OILPpinState = LOW;
    digitalWrite(OILPpin, OILPpinState);
  }

  // check engine enable state and set pin state
  if ((engineEnable > 0) && (EOBDpinState == LOW))
  {
    // only change & write pin state if required
    EOBDpinState = HIGH;
    digitalWrite(EOBDpin, EOBDpinState);
  }
  else if ((engineEnable == 0) && (EOBDpinState == HIGH))
  {
    EOBDpinState = LOW;
    digitalWrite(EOBDpin, EOBDpinState);
  }

  // update coolant temp & get digital pot ADC request
  ECT = ECT + 273.15; // convert to degK

  ECTResistance = R0 * (exp(Beta * ((1.0 / ECT) - (1.0 / T0)))); //Ohms

  AD5235inECT = (ECTResistance - Rmin) * (1024 - 1) / (250000 - Rmin) + 1; // same as map function but gives float output

  AD5235inECT = round(AD5235inECT); // round to nearest whole number

  // update oil temp & get digital pot ADC request
  EOT = EOT + 273.15; // convert to degK

  EOTResistance = R0 * (exp(Beta * ((1.0 / EOT) - (1.0 / T0)))); // Ohms

  AD5235inEOT = (EOTResistance - Rmin) * (1024 - 1) / (250000 - Rmin) + 1; // same as map function but gives float output

  AD5235inEOT = round(AD5235inEOT); // round to nearest whole number

  // Send ADC values to AD5235
  setWiper(ECTwiper, AD5235inECT);
  setWiper(EOTwiper, AD5235inEOT);
}

// ===============================================================================================================
void TACHpulse()
{

  TACHpinState = !TACHpinState; // switch state of pin
  digitalWrite(TACHpin, TACHpinState);
}

// ===============================================================================================================
void VSSpulse()
{

  VSSpinState = !VSSpinState; // switch state of pin
  digitalWrite(VSSpin, VSSpinState);
}

// ===============================================================================================================
void TACHupdate()
{

  // calculate timer period for pulses
  // RPM
  if (RPM < RPMmin)
  {
    TACHperiod = UINT32_MAX; // set period to max possible to avoid triggering
  }
  else
  {
    TACHperiod = (30000000 / RPM); // Should match Screen 7, P - 1, High impulse.
  }
}

// ===============================================================================================================
void VSSupdate()
{

  // calculate timer period for pulses
  // VSS
  if (*VSSdisp <= VSSmin)
  {
    VSSperiod = UINT32_MAX;
  }
  else
  {
    VSSperiod = facVSSperiod / *VSSdisp;
  }
}

// ===============================================================================================================
void shiftLightsFlash()
{
  // flashes all shift lights
  shiftFlashState = !shiftFlashState; // switch state
  shiftGState = shiftFlashState;
  shiftYState = shiftFlashState;
  shiftRState = shiftFlashState;
  shiftBState = shiftFlashState;

  uint8_t pwm;

  if (shiftFlashState==HIGH)
  {
    pwm = ledPWM;
  }
  else
  {
    pwm = 0;
  }

  analogWrite(SHIFTGpin, pwm);
  analogWrite(SHIFTYpin, pwm);
  analogWrite(SHIFTRpin, pwm);
  analogWrite(SHIFTBpin, pwm);
  
}

// ===============================================================================================================
void shiftLights()
{
  // Flash
  if (changeLightStage >= SHIFTFstage) {
    if (currentMicros - PrevSHIFTFlashMicros > SHIFTFlashPeriod)
    {
      PrevSHIFTFlashMicros = currentMicros;
      shiftLightsFlash();
    }
  }
  else {
    shiftFlashState = LOW;

    // Green
    if ( (changeLightStage >= SHIFTGstage) && (shiftGState==LOW) ) {
      shiftGState = HIGH;
      analogWrite(SHIFTGpin, ledPWM);
    }
    else if ( (changeLightStage < SHIFTGstage) && (shiftGState==HIGH) ) {
      shiftGState = LOW;
      analogWrite(SHIFTGpin, 0);
    }

    // Yellow
    if ( (changeLightStage >= SHIFTYstage) && (shiftYState==LOW) ) {
      shiftYState = HIGH;
      analogWrite(SHIFTYpin, ledPWM);
    }
    else if ( (changeLightStage < SHIFTYstage) && (shiftYState==HIGH) ) {
      shiftYState = LOW;
      analogWrite(SHIFTYpin, 0);
    }

    // Red
    if ( (changeLightStage >= SHIFTRstage) && (shiftRState==LOW) ) {
      shiftRState = HIGH;
      analogWrite(SHIFTRpin, ledPWM);
    }
    else if ( (changeLightStage < SHIFTRstage) && (shiftRState==HIGH) ) {
      shiftRState = LOW;
      analogWrite(SHIFTRpin, 0);
    }

    // Blue
    if ( (changeLightStage >= SHIFTBstage) && (shiftBState==LOW) ) {
      shiftBState = HIGH;
      analogWrite(SHIFTBpin, ledPWM);
    }
    else if ( (changeLightStage < SHIFTBstage) && (shiftBState==HIGH) ) {
      shiftBState = LOW;
      analogWrite(SHIFTBpin, 0);
    }
  }
}

// ===============================================================================================================
void setup()
{
  delay(5000);
  // Serial.begin(115200); // only used for USB debugging

  // CAN setup
  CAN_filter_t filter1;
  filter1.id = filterID1;
  filter1.ext = 0; //defines if filter is extended or standard frame. 0 = std, 1 = extended
  filter1.rtr = 0;

  CAN_filter_t filter2;
  filter2.id = filterID2;
  filter2.ext = 0; //defines if filter is extended or standard frame. 0 = std, 1 = extended
  filter2.rtr = 0;

  Can0.begin(can_speed);

  Can0.setMask(maskID2 << 18, 0); //bitshift is necessary for 11-bit id's
  Can0.setFilter(filter2, 0);

  // all filters must be populated so just set the remaining ones the same
  for (uint8_t filterNum = 1; filterNum < 14; filterNum++)
  {
    Can0.setMask(maskID1 << 18, filterNum); //bitshift is necessary for 11-bit id's
    Can0.setFilter(filter1, filterNum);
  }

  // SPI Setup
  SPI.setMOSI(mosi);
  SPI.setMISO(miso);
  SPI.setSCK(sck);
  pinMode(chipSelect, OUTPUT);
  SPI.begin();

  // Digital Pins Setup
  pinMode(TACHpin, OUTPUT);
  pinMode(VSSpin, OUTPUT);
  pinMode(EOBDpin, OUTPUT);
  pinMode(OILPpin, OUTPUT);
  pinMode(SHIFTGpin, OUTPUT);
  pinMode(SHIFTYpin, OUTPUT);
  pinMode(SHIFTRpin, OUTPUT);
  pinMode(SHIFTBpin, OUTPUT);
  pinMode(LIGHTpin, INPUT);

  // Read state of rotary switch & point to different variable depending on state.
  uint8_t swtState = PT65112(ROTARY1pin, ROTARY2pin);
  // Serial.print("Switch state: ");
  // Serial.println(swtState); // swtState not working. Always shows 0

  switch (swtState)
  {
    case 0:
      VSSdisp = &VSS;
      break;
    case 1:
      VSSdisp = &GEAR;
      break;
    case 2:
      VSSdisp = &TPS;
      break;
  }

  // calculate min resistance for AD5235
  Rmin = 250000.0 / 1024.0;

  // calculate factor to convert VSS to required pulse period
  facVSSperiod = ( ( 1800000 * circ ) / pulses );

  txmsg.id = txID;
  txmsg.len = 8;

  // get required shift lights brightness before entering loops
  shiftLBrightness();

}

// ===============================================================================================================
void loop()
{

  can_recieve(); // Carry out CAN recieve function as often as possible

  currentMicros = micros();

  // Debug only
  //Serial.print("rpm: ");
  //Serial.println(RPM);

  if (currentMicros - previousTACHpinMicros > TACHperiod)
  {
    previousTACHpinMicros = currentMicros;
    TACHpulse();
  }
  if (currentMicros - previousVSSpinMicros > VSSperiod)
  {
    previousVSSpinMicros = currentMicros;
    VSSpulse();
  }
  if (currentMicros - PrevTACHUpdateMicros > TACHUpdatePeriod)
  {
    PrevTACHUpdateMicros = currentMicros;
    TACHupdate();
  }
  if (currentMicros - PrevVSSUpdateMicros > VSSUpdatePeriod)
  {
    PrevVSSUpdateMicros = currentMicros;
    VSSupdate();
  }
  if (currentMicros - PrevSHIFTUpdateMicros > SHIFTUpdatePeriod) 
  {
    PrevSHIFTUpdateMicros = currentMicros;
    // Serial.print("Test active: ");
    // Serial.println(shiftTest);
    // Serial.print("Shift light stage: ");
    // Serial.println(changeLightStage);

    shiftLights();
  }
  if (currentMicros - PrevIndUpdateMicros > IndUpdatePeriod)
  {
    PrevIndUpdateMicros = currentMicros;
    // Debug only
    //Serial.print("vss: ");
    //Serial.println(VSS);
    //Serial.print("ect: ");
    //Serial.println(ECT);
    //Serial.print("EOT: ");
    //Serial.println(EOT);
    //Serial.print("EngineEnable: ");
    //Serial.println(engineEnable);
    //Serial.print("EOP: ");
    //Serial.println(OILP);
    // End Debug
    UpdateIndicators();
  }
  if (currentMicros - PrevBrightCheckMicros > brightnessCheck)
  {
    PrevBrightCheckMicros = currentMicros;

    shiftLBrightness();
  }
}
