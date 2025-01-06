# DTSU666_CHINT_to_HUAWEI_translator_arduino
This poject is based on a great project  "DTSU666_CHINT_to_HUAWEI_translator" https://github.com/salakrzy/DTSU666_CHINT_to_HUAWEI_translator/tree/main
I only try use this on arduino IDE.
This project uses ESP32 to translate MODBUS messages of CHINT's DTSU666 Energy Meter into HUAWEI DTSU666H format.
This project translates the DTSU666 CHINT Power Meter registers addreses to Huawei DTSU666H Power Meter addreses.

All data received from CHINT counter could be send to MQTT Broker.

Project base on the eMODBUS librrary  https://github.com/eModbus/eModbus 

Information on how to prepare enviroment , compile and flash ESP32 modules can be found at
https://store.arduino.cc/en-pl


Description how to assembling the hardware you will found in the /doc project directory . 

In used RS485 interface the R5 and R6 resistors are 20k ohm it is to much for MOdbus standard, if you have transmission errors, change this resistors to 2k ohm.
If you change the R5 and R6 resistors,  you don't need remove the R7 120 ohm resistor.

Good Luck:-)
