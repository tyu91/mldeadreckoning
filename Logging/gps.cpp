#include "gps.h"
#include "TinyGPS++.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// int main(void) {
//     printf("GPS test...\n");
//     I2CGPS gps;
//     TinyGPSPlus tgps;
//     if (gps.begin()) {
//         printf("started correctly?\n");

//         printf("Updated speed?\n");

//         gps.sendMTKpacket("$PMTK220,100*1d\r\n"); // set 100hz update speed

//         while(true) {
//             uint8_t dataBytes = gps.available();
//             // printf("%d bytes available\n", dataBytes);
//             for(int i = 0; i < dataBytes; i++) {
//                 tgps.encode(gps.read());
//                 //printf("%c", gps.read());
//             }
//             //printf("\n");

//             if (tgps.location.isUpdated()) {
//                 printf("LAT=%f\n", tgps.location.lat());
//                 printf("LNG=%f\n", tgps.location.lng());
//                 printf("TIME=%d:%d:%d.%d\n", tgps.time.hour(), tgps.time.minute(), tgps.time.second(), tgps.time.centisecond());
//             }
//             //sleep(1);
//         }
//     }
// }

//Sets up the sensor for constant read
//Returns false if sensor does not respond

bool I2CGPS::begin(int address)
{
    //Bring in the user's choices
    _i2c_fd = wiringPiI2CSetup(address);
    _head = 0; //Reset the location holder
    _tail = 0;

    wiringPiI2CWrite(_i2c_fd, 0);

    if (wiringPiI2CRead(_i2c_fd)) // might fail if current is unstable on GPIO pins

        // set the update speed?
        sendMTKpacket("$PMTK220,200*2C\r\n");

        return (true);

    return false;
}

//Polls the GPS module to see if new data is available
//Reads a 255 byte packet from GPS module
//If new data is there, appends it to the gpsData array
//Function requires 26ms @ 100kHz I2C, 9ms @ 400kHz I2C so call sparingly
void I2CGPS::check()
{
    for (uint8_t x = 0; x < MAX_PACKET_SIZE; x++)
    {

        uint8_t incoming;
        incoming = wiringPiI2CRead(_i2c_fd);

        if (incoming != 0x0A)
        {
            //Record this byte
            gpsData[_head++] = incoming;
            _head %= MAX_PACKET_SIZE; //Wrap variable
        }
    }
}

//Returns # of available bytes that can be read
uint8_t I2CGPS::available()
{
    //If tail=head then no new data is available in the local buffer
    //So now check to see if the module has anything new in its buffer
    if (_tail == _head)
    {
        check(); //Check to module to see if new I2C bytes are available
    }

    //Return new data count
    if (_head > _tail)
        return (_head - _tail);
    if (_tail > _head)
        return (MAX_PACKET_SIZE - _tail + _head);
    return (0); //No data available
}

//Returns the next available byte from the gpsData array
//Returns 0 if no byte available
uint8_t I2CGPS::read(void)
{
    if (_tail != _head)
    {
        uint8_t datum = gpsData[_tail++];
        if (_tail == MAX_PACKET_SIZE)
            _tail = 0; //Wrap variable
        return (datum);
    }
    else
        return (0); //No new data
}

//Turn off printing of GPS character streams
void I2CGPS::disableDebugging()
{
    _printDebug = false; //Turn off extra print statements
}

//Functions for sending commands to the GPS module
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a given command or configuration string to the module
//The input buffer on the MTK is 255 bytes. Caller must keep strings shorter than 255 bytes
//Any time you end transmission you must give the module 10ms to process bytes

bool I2CGPS::sendMTKpacket(std::string command)
{
    if (command.length() > 255)
    {
        if (_printDebug == true)
            printf("Command message too long!\n");
        return (false);
    }


    const char* command_bytes = command.c_str();
    write(_i2c_fd, (u_int8_t *)command_bytes, command.length());

    // for (int i = 0; i < command.length(); i++)
    // {
    //     wiringPiI2CWrite(_i2c_fd, command[i]);
    // }

    //_i2cPort->write(MT333x_ADDR << 1, command.c_str(), command.length());
    // ThisThread::sleep_for(10);

    return (true);
}

//Given a packetType and any settings, return string that is a full
//config sentence complete with CRC and \r \n ending bytes
//PMTK uses a different packet numbers to do configure the module.
//These vary from 0 to 999. See 'MTK NMEA Packet' datasheet for more info.

// #elif defined(__MBED__)
// string I2CGPS::createMTKpacket(uint16_t packetType, string dataField)
// {
//     //Build config sentence using packetType
//     string configSentence = "";
// #endif
//     configSentence += "$PMTK"; //Default header for all GPS config messages

//     //Attach the packetType number
//     //Append any leading zeros
//     if (packetType < 100)
//         configSentence += "0";
//     if (packetType < 10)
//         configSentence += "0";
// #if defined(ARDUINO)
//     configSentence += String(packetType);
// #elif defined(__MBED__)
//     configSentence += to_string(packetType);
// #endif

//     //Attach any settings
//     if (dataField.length() > 0)
//     {
//         configSentence += dataField; //Attach the string of flags
//     }

//     configSentence += "*"; //Attach end tag

//     configSentence += calcCRCforMTK(configSentence); //Attach CRC

//     //Attach ending bytes
//     configSentence += '\r'; //Carriage return
//     configSentence += '\n'; //Line feed

//     return (configSentence);
// }

// //Calculate CRC for MTK messages
// //Given a string of characters, XOR them all together and return CRC in string form
// #if defined(ARDUINO)
// String I2CGPS::calcCRCforMTK(String sentence)
// {
// #elif defined(__MBED__)
// string I2CGPS::calcCRCforMTK(string sentence)
// {
// #endif
//     uint8_t crc = 0;

//     //We need to ignore the first character $
//     //And the last character *
//     for (uint8_t x = 1; x < sentence.length() - 1; x++)
//         crc ^= sentence[x]; //XOR this byte with all the others

// #if defined(ARDUINO)
//     String output = "";
// #elif defined(__MBED__)
//     string output = "";
// #endif
//     if (crc < 10)
//         output += "0"; //Append leading zero if needed

// #if defined(ARDUINO)
//     output += String(crc, HEX);
// #elif defined(__MBED__)
//     static char outhex[4];
//     sprintf(outhex, "%02X", crc);

//     output += outhex;
// #endif

//     return (output);
// }

// #if defined(ARDUINO)
// boolean I2CGPS::sendPGCMDpacket(String command)
// {
// #elif defined(__MBED__)
// bool I2CGPS::sendPGCMDpacket(string command)
// {
// #endif
//     return (sendMTKpacket(command)); // Send process is the same, re-named to ease user's minds
// }

// #if defined(ARDUINO)
// String I2CGPS::createPGCMDpacket(uint16_t packetType, String dataField)
// {
//     //Build config sentence using packetType
//     String configSentence = "";
// #elif defined(__MBED__)
// string I2CGPS::createPGCMDpacket(uint16_t packetType, string dataField)
// {
//     //Build config sentence using packetType
//     string configSentence = "";
// #endif
//     configSentence += "$PGCMD,"; //Default header for all PGCMD messages

//     //Attach the packetType number
//     //Append any leading zeros
//     if (packetType < 100)
//         configSentence += "0";
//     if (packetType < 10)
//         configSentence += "0";
// #if defined(ARDUINO)
//     configSentence += String(packetType);
// #elif defined(__MBED__)
//     configSentence += to_string(packetType);
// #endif

//     //Attach any settings
//     if (dataField.length() > 0)
//     {
//         configSentence += dataField; //Attach the string of flags
//     }

//     configSentence += "*"; //Attach end tag

//     configSentence += calcCRCforMTK(configSentence); //Attach CRC - uses the same crc as PTMK

//     //Attach ending bytes
//     configSentence += '\r'; //Carriage return
//     configSentence += '\n'; //Line feed

//     return (configSentence);
// }