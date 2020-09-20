# CAN_Controller_Koso_Dash
Teensy 3.2 based controller board designed to recieve engine data from ECU via CAN-Bus and use it to control a Koso RX2N motorcycle dash

This module was built for my CBR250RRi project. The aim was to run my aftermarket Koso RX2N dash using the same sensors (coolant & oil temp) as the ECU to avoid having to double up on sensors. The idea was to recieve all necessary signals from the ECU via CAN-Bus and then have the Teensy replicate the analogue signals that the dash expects.

The Koso RX2N dash has the following possible inputs for indication:
1. Engine speed         - variable frequency square wave signal
1. Vehicle speed        - variable frequency square wave signal
1. Coolant temperature  - thermistor
1. Oil temperature      - thermistor
1. Oil pressure light   - digital on/off signal, low side switch
1. EOBD light           - digital on/off signal, low side switch
1. Neutral light        - digital on/off signal, low side switch
1. Fuel level           - digital on/off signal, low side switch
1. Hi-beam light        - digital on/off signal, high side switch
1. RH Indicator         - digital on/off signal, high side switch
1. LH Indicator         - digital on/off signal, high side switch

The LH/RH indicators, hi-beam, fuel level & neutral signals were all provided directly from the sensors and wire harness. All other signals relied on sensors which would be connected directly to the ECU and therefore required the controller to provide the signals based on data recieved via CAN-Bus.

**Engine & Vehicle Speed**
Although the ECU could provide the required signal directly to the dash using an injector output, I decided to save the ECU output and hand the job over to the dash controller. A 50% duty cycle variable frequency signal is provided by the Teensy switching 12V to gound. The frequency is updated at 50Hz to avoid visible stuttering of the dash needle during fast engine speed ramps. 
A similar tactic as engine speed was employed to the vehicle speed signal apart from the signal being 5V level and the signal frequency is updated at only 5Hz as vehicle speed is displayed on a 7-segment display. 

**Coolant & Oil Temperature**
A pair of digital potentiometers are employed to replicate the thermistor resistance. The R-T curve for the sensors supplied with the RX2N had been provided by Koso tech support. The linear response and resolution of the digital potentiometers means that there could be an error of up to 3°C between the actual and displayed values between 90-110°C temperature. The dash display is only an indication so this level of accuracy is acceptable.

**Oil Pressure Light**
My bike and most others from the 1990's came fitted with an oil pressure switch. The original switch could be connected directly to the dash but as I have replaced the pressure switch with an actual pressure sensor, I have added the possibility of controlling the light via the dash controller. In the current version of the code, the actual oil pressure is compared to a single threshold, so operates the light much the same as the original switch. I plan to update the code to add a varying pressure threshold with engine speed and oil temperature after I have collected some data to build a normal oil pressure map.

**EOBD Light**
The EOBD light is a handy indicator of something being wrong on the ECU side. The measure of "something wrong" depends on the ECU but on my Life Racing F88R I use a flag called *engineEnable* and set the EOBD light if the flag is not equal to 0 (OK).


I included tracks on the controller board for USB to allow the Teensy code to be updated without opening the enclosure. However, I did not adhere to recommendations around USB bus design and so this does not work. I plan to update the board to meet guidelines in the future.



For further details on the CBR250RRi project, refer to the website https://www.cbr250rri.com/.
