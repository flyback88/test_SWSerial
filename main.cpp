#include <Arduino.h>

#define IORxpin 04      // D2 de nodeMCU
#define SOFTSERIAL_BAUDRATE	1200
#define SOFTSERIAL_LENGTH	7		// length in byte, could be 7 or 8
#define SOFTSERIAL_PARITY	2		// could be 0=NONE , 2=EVEN, 1=ODD
#define SOFTSERIAL_NSTOPS	1 		// for Tx only, not checked in Rx

#include "SWSerial.hpp"

void setup() {
    SERIALPORT.begin(1000000);
    SERIALPORT.println("\r\nSTART");

    TICSerial_begin();
}

void loop() {
    int16_t ii,N;
    N=TICSerial_available();
    if(N>0){
        //for(ii=0;ii<N;ii++){TICSerial_read();}
    }
    delay(10);
}