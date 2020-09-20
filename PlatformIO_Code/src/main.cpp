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
 * 
 * This code is designed for use with the CBR015-0026 DASH CONTROLLER ASSY utilising CBR015-0027 Rev00 PCB layout
 * CAN Config: 200507_LOGGER_CAN_CONFIG_v2.0.xlsx
 * Messages recieved via CAN are MSB First or Big Endian byte order
 * 
 * See documentation for details & overview
 */

#include <Arduino.h>
#include <FlexCAN.h>
#include <SPI.h>

// ===============================================================================================================
// Declare constants most likely to be adjusted by user in this block

#define TACHUpdateFreq 50 // Tach output update rate in Hz
#define VSSUpdateFreq 5   // VSS output update rate in Hz
#define IndUpdateFreq 2   // Indicators output update rate in Hz
#define can_speed 500000  // CAN-Bus baud rate in bps

#define RPMmin 50 // minimum engine speed before outputting signal
#define VSSmin 3  // minimum vehicle speed before outputting signal in kph
#define circ 2    // tyre circumference in metres
#define pulses 4  // VSS pulses per rev

// limits
#define OILPmin 300   // mBarA, minimum oil pressure for oil light switching
#define errorFlagmax 2 // maximum value of error flags allowed before illuminating EOBD light. This is not used in v1.0. Instead

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
#define SHIFTGpin 5
#define SHIFTYpin 0
#define SHIFTRpin 1
#define SHIFTBpin 2

// Declare can bus and can messages
static CAN_message_t rxmsg;
#define filterID 0x600
#define maskID 0x7FE

// Declare variables used by timers
uint32_t PrevTACHUpdateMicros, PrevVSSUpdateMicros, PrevIndUpdateMicros, TACHUpdatePeriod, VSSUpdatePeriod, IndUpdatePeriod;

// Declare variables used by dig out pins
// TACH
bool TACHpinState;
uint32_t previousTACHpinMicros, TACHperiod;
int16_t RPM;

// VSS
bool VSSpinState;
uint32_t previousVSSpinMicros, VSSperiod;
float VSS;

// EOBD
uint16_t engineEnable;
bool EOBDpinState;

// OILP
bool OILPpinState;
int16_t OILP;

float ECT, EOT;

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

    case 0x600:
      RPM = ((int16_t((rxmsg.buf[1]) | (rxmsg.buf[0] << 8))));         //rpm
      VSS = ((int16_t((rxmsg.buf[3]) | (rxmsg.buf[2] << 8))) * 0.036); //kph
      break;
    case 0x601:
      ECT = ((int16_t((rxmsg.buf[1]) | (rxmsg.buf[0] << 8))) * 0.1); //degC
      EOT = ((int16_t((rxmsg.buf[3]) | (rxmsg.buf[2] << 8))) * 0.1); //degC
      OILP = ((int16_t((rxmsg.buf[5]) | (rxmsg.buf[4] << 8))));      //mBarA
      engineEnable = ((uint16_t((rxmsg.buf[7]) | (rxmsg.buf[6] << 8))));
      break;
    }
  }
}

// ===============================================================================================================
void UpdateIndicators()
{

  // check oil pressure and set pin state
  if ((OILP < OILPmin) && (OILPpinState == LOW))
  {
    // only change & write pin state if required
    Serial.println("OilP Low");
    OILPpinState = HIGH;
    digitalWrite(OILPpin, OILPpinState);
  }
  else if ((OILP > OILPmin) && (OILPpinState == HIGH))
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
    TACHperiod = (30000000 / RPM); // confirm dash settings and calculation. Should match Screen 7, P - 1, High impulse.
  }
}

// ===============================================================================================================
void VSSupdate()
{

  // calculate timer period for pulses
  // VSS
  if (VSS < VSSmin)
  {
    VSSperiod = UINT32_MAX;
  }
  else
  {
    VSSperiod = ((1800000 * circ) / (VSS * pulses)); // confirm dash settings and calculation. Also consider moving most of the calculation to setup so this step is an operation with only 2 variables
  }
}

// ===============================================================================================================
void setup()
{

  //Serial.begin(115200); // only used for USB debugging

  // CAN setup
  CAN_filter_t filter;
  filter.id = filterID;
  filter.ext = 0; //defines if filter is extended or standard frame. 0 = std, 1 = extended
  filter.rtr = 0;

  Can0.begin(can_speed);

  for (uint8_t filterNum = 0; filterNum < 14; filterNum++)
  {
    Can0.setMask(maskID << 18, filterNum); //bitshift is necessary for 11-bit id's
    Can0.setFilter(filter, filterNum);
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

  // Calculate update periods
  TACHUpdatePeriod = (1000000 / TACHUpdateFreq);
  VSSUpdatePeriod = (1000000 / VSSUpdateFreq);
  IndUpdatePeriod = (1000000 / IndUpdateFreq);

  // calculate min resistance for AD5235
  Rmin = 250000.0 / 1024.0;

  // initialise variables before they are updated within the loop
  PrevTACHUpdateMicros = 0;
  PrevVSSUpdateMicros = 0;
  PrevIndUpdateMicros = 0;
  TACHpinState = LOW;
  previousTACHpinMicros = 0;
  TACHperiod = 2147483647; // initialise at max to avoid triggering
  RPM = 0;
  VSSpinState = LOW;
  previousVSSpinMicros = 0;
  VSSperiod = 2147483647; // initialise at max to avoid triggering
  VSS = 0;
  engineEnable = 0; // initialise at 0 => OK
  EOBDpinState = LOW;
  OILP = 0;
  OILPpinState = LOW;
}

// ===============================================================================================================
void loop()
{

  can_recieve(); // Carry out CAN recieve function as often as possible

  uint32_t currentMicros = micros();

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
}