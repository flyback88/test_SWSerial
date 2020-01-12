/*
    Manage a Serial Channel in software, using only GPIO interrupts
    	receive only routine made custom for the Linky electricity counter
	    works at 1200 baud, 7 bits, parity even (7E1)


	characters in put at buffer[head] and then head++ % size
	and retreived from buffer[tail] and then tail++ % size

                                        char in ---+
                                                   |											 
 	             tail                              |
	   -----------x--------------------------------x---------
                  |                              head
                  |
                  +---> char out



	IORxpin, baudrate, length, partity are hardcoded through defines
    this supposes that this is defined before including
	    #define IOTICdata	xxxx


    Serial receive software driver
    _______                                                                                           ____S____     __
           |____S____|XXXX0XXXX|XXXX1XXXX|XXXX2XXXX|XXXX3XXXX|XXXX4XXXX|XXXX5XXXX|XXXX6XXXX|XXXXPXXXX|         XXXXX  |____S__

    _______                     _________                                                   ______________S____     __
    STX    |____S______________|         |_________________________________________________|                   XXXXX  |____S__

           ISR       ISR       ISR       ISR       ISR       ISR       ISR       ISR       ISR                        ISR
     wfs     wfb                                                                                    wfs
     e# xxx0                   1         2                                                 3                          4
	 b# xxxxx1                   3         4                                                 9                        11

        falling/rising edges trigger GPIO interrupt
		compute dT since last edge
		if dT is big enough --> startbit, reset edge/bit counter
		if dT=N*bits, shift in N bits either ONE or ZERO
		if Nbits


	The rising/falling edges might be significatively different, for example is IOrx is after an optocoupler

GPIO ISR management is done using the "wiring" framework, defined in core_esp8266_wiring_digital.cpp 
    attachInterrupt(IORxpin, ISR_SoftSerial_GPIO, FALLING);
        FunctionInfo* fi = new FunctionInfo;
        fi->reqFunction = ISR_SoftSerial_GPIO;
        ArgStructure* as = new ArgStructure;
        as->interruptInfo = nullptr;
        as->functionInfo = fi;
        __attachInterruptArg (IORxpin, ISR_SoftSerial_GPIO, as, mode);

            __attachInterruptArg(uint8_t pin, voidFuncPtr userFunc, void *arg, int mode) {
            ETS_GPIO_INTR_DISABLE();   // ets_isr_mask((1<<ETS_GPIO_INUM));
            interrupt_handler_t *handler = &interrupt_handlers[IORxpin];
            handler->mode = mode;
            handler->fn = userFunc;
            if (handler->arg) cleanupFunctional(handler->arg);// Clean when new attach without detach
            handler->arg = arg;
            interrupt_reg |= (1 << IORxpin);
            GPC(IORxpin) &= ~(0xF << GPCI);//INT mode disabled
            GPIEC = (1 << IORxpin); //Clear Interrupt for this pin
            GPC(pin) |= ((mode & 0xF) << GPCI);//INT mode "mode"
            ets_isr_attach(ETS_GPIO_INUM, (int_handler_t)(interrupt_handler), (void *)(&interrupt_reg))
            ets_isr_unmask((1<<ETS_GPIO_INUM))

*/

#define rcRxGlitchError 0x01	// glitch on the line, level is not stable over 32 samples
#define rcRxFrameError	0x02	// edges found not on bit boundaries
#define rcRxStartError	0x04    // start bit is not zero
#define rcRxStopError	0x08	// stop bit not equal 1
#define rcRxParityError	0x10
#define rcRxOverRun		0x20	// overrun in the serial interface
#define rcRxOverRun2    0x40	// overrun in the receiving buffer
#define rcRxOverRun3    0x80	// overrun, one more character received while a previous buffer is received closed


#ifndef IORxpin
	#error "IORxpin pin is not defined"
#endif
#define IORxmask (1<<IORxpin)

#ifndef SOFTSERIAL_BAUDRATE
	#define SOFTSERIAL_BAUDRATE	1200
#endif
#ifndef SOFTSERIAL_LENGTH
	#define SOFTSERIAL_LENGTH	7		// length in byte, could be 7 or 8
#endif
#ifndef SOFTSERIAL_PARITY
	#define SOFTSERIAL_PARITY	2		// could be 0=NONE , 2=EVEN, 1=ODD
#endif
#ifndef SOFTSERIAL_NSTOPS
	#define SOFTSERIAL_NSTOPS	1 		// for Tx only, not checked in Rx
#endif

#if SOFTSERIAL_PARITY>0
  #define SOFTSERIAL_NBITSTOPROCESS (1+SOFTSERIAL_LENGTH+1+SOFTSERIAL_NSTOPS)
#else
  #define SOFTSERIAL_NBITSTOPROCESS (1+SOFTSERIAL_LENGTH+SOFTSERIAL_NSTOPS)
#endif

#define SOFTSERIAL_MAXSILENCE 2000  // time out in ms for saying link is not operational
#define SOFTSERIAL_STOPSILENCE 20	// time out in #bits for saying next edge is a start bit
#define systemtime() (*(volatile uint32 *)(0x3ff20c00)) // also defined as WDEV_NOW()
#define SYSTEMTIMERESOLUTION 1	// in microseconds
#define Tbit (1000000/SOFTSERIAL_BAUDRATE/SYSTEMTIMERESOLUTION)
#define Tbit_error 2	// timing error on the bit rate, in percent
#define Tbit_offset 0	// tlow/thigh dutcycle in percent
#define Tbit_lh_l ((100-Tbit_error)*(100+Tbit_offset)*100/SOFTSERIAL_BAUDRATE/SYSTEMTIMERESOLUTION)
#define Tbit_lh_h ((100+Tbit_error)*(100+Tbit_offset)*100/SOFTSERIAL_BAUDRATE/SYSTEMTIMERESOLUTION)
#define Tbit_hl_l ((100-Tbit_error)*(100-Tbit_offset)*100/SOFTSERIAL_BAUDRATE/SYSTEMTIMERESOLUTION)
#define Tbit_hl_h ((100+Tbit_error)*(100-Tbit_offset)*100/SOFTSERIAL_BAUDRATE/SYSTEMTIMERESOLUTION)

#define SoftMode 1	// 1,2 for std Arduino, 3 for customized GPIOinterrupts.hpp

#include "Common.h"
#include "Time.hpp"

#if SoftMode==3	// dont use Arduino std routines, but use GPIOinterrupts.hpp
	extern void ICACHE_RAM_ATTR ISR_SoftSerial_GPIO(void);
	#if IORxpin==0
		#define GPIO_ISR_0 ISR_SoftSerial_GPIO()
	#elif IORxpin==2
		#define GPIO_ISR_2 ISR_SoftSerial_GPIO()
	#elif IORxpin==4
		#define GPIO_ISR_4 ISR_SoftSerial_GPIO()
	#elif IORxpin==5
		#define GPIO_ISR_5 ISR_SoftSerial_GPIO()
	#elif IORxpin==12
		#define GPIO_ISR_12 ISR_SoftSerial_GPIO()
	#elif IORxpin==13
		#define GPIO_ISR_13 ISR_SoftSerial_GPIO()
	#elif IORxpin==14
		#define GPIO_ISR_14 ISR_SoftSerial_GPIO()
	#elif IORxpin==15
		#define GPIO_ISR_15 ISR_SoftSerial_GPIO()
	#else
		#error SoftSerial IORxpin out of range
	#endif
	#include "GPIOinterrupts.hpp"
#endif

#if 1   // mise au point
  #define SSerialMiseAuPoint
  #define SSerialMaP(fmt, ...)  MaPT(fmt,##__VA_ARGS__)
  #define SSerialMaP_(fmt, ...)	MaP_(fmt,##__VA_ARGS__)
  #define soDBGedgetime 32
  uint32_t DBGedgetime[soDBGedgetime];    // start + 8 + P + stop
  #define SSerialMaPPut(char) USF(UART0)=char   // mise au point/debug, envoi immediat d'un charactère à serial0
#else
  #define SSerialMaP(fmt, ...)
  #define SSerialMaP_(fmt, ...)
  #define SSerialMaPPut(char)
#endif

#ifndef SOFTSERIAL_RX_BUFFER_SIZE
	#define SOFTSERIAL_RX_BUFFER_SIZE 200   // standard Linky message is around 150 bytes
#endif
#if  (SOFTSERIAL_RX_BUFFER_SIZE>256)
	typedef uint16_t rx_buffer_index_t;
#else
	typedef uint8_t rx_buffer_index_t;
#endif

int16_t _error;
uint32_t _Tedge,_Tlastedge;
int32_t _dT;
uint8_t _SSedgenumber=0;
uint8_t _SSBitNumber=0;
uint16_t _SSBitMask=0x01;    // next bit to receive
uint16_t _SSByte;            // byte being received, includes start, parity and stop
enum SSState_t:int8_t { SSOff=0, SSWaitForStart, SSWaitForBit, SSSuspended};
SSState_t _SSState=SSOff;      // gives the current status of the SoftSerial interrupt machine
enum SSStatus_t:int8_t { SSnotOperational=0, SSOperational};
SSStatus_t _SSStatus=SSnotOperational;
uint32_t SSTimeLastByteOK;
bool _newcarin, _run, _SSByteAvailable;
rx_buffer_index_t _rx_buffer_head,_rx_buffer_tail;
uint8_t _rx_buffer[SOFTSERIAL_RX_BUFFER_SIZE];


inline __attribute__((always_inline)) void TICSerial_reset0(void){
    _SSedgenumber=0;
    _SSBitNumber=0;_SSBitMask=0x01;
    _error=0;
    }


void ICACHE_RAM_ATTR ISR_SoftSerial_GPIO(void){        // GPIO falling edge interrupt
	/*
		edge f/r interrupt service to decode a UART stream
	*/
  uint8_t level0;
	// sample 32 times, this lasts 10 micro seconds (measured ESP8266 80MHz)
	level0=0;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	if((GPI&IORxmask)==0)level0++;
	// now test the value, if we don't get the good number, this is a glitch, return and wait for next falling edge
	// if we got 32 times a zero, this is a zero level, if we got 0 time a zero, this is a one level
	if(level0&0x1F) _error |= rcRxGlitchError;    // error, but continue to next bit
	level0=level0>>5;   // now level0=1 if it was a valid ZERO level, otherwise level0=0
	_Tedge=systemtime();
	_dT=_Tedge-_Tlastedge;
	_Tlastedge=_Tedge;

    if(_SSState==SSWaitForStart){
        if(_dT>(SOFTSERIAL_STOPSILENCE*Tbit)){
            if(level0){ // falling edge
                #ifdef SSerialMiseAuPoint
                    SSerialMaPPut('S');
                #endif
                _SSedgenumber=0;
                _SSBitNumber=1;_SSBitMask=0x02;
                _error=0;
                _SSByte=0;
                _SSState=SSWaitForBit;
            }else{      // rising edge
                _error |= rcRxStartError;	// start bit not ok
            }
        }else{
            if(level0){ // falling edge
                #ifdef SSerialMiseAuPoint
                    SSerialMaPPut('s');
                #endif
                _SSedgenumber=0;
                _SSBitNumber=1;_SSBitMask=0x02;
                _error=0;
                _SSByte=0;
                _SSState=SSWaitForBit;
            }else{
                
            }
        }
		return;
    }
    
    if(_SSState!=SSWaitForBit) return;
    #ifdef SSerialMiseAuPoint
        SSerialMaPPut('|');
        DBGedgetime[_SSedgenumber] = _dT;
    #endif
	_SSedgenumber++;

	if(level0){	// falling edge
		if(_dT<Tbit_hl_l){	// too short
			_error |= rcRxFrameError;
			return;
		}else{
			_dT=_dT-Tbit_hl_l;
            while(_dT>Tbit_hl_l){
				_dT=_dT-Tbit_hl_l;
				_SSByte |= _SSBitMask;	// there were some bits one
				_SSBitNumber++;_SSBitMask=_SSBitMask<<1;    // next bit
			}
            _SSByte &= (~_SSBitMask);	// current one is a bit zero
            _SSBitNumber++;_SSBitMask=_SSBitMask<<1;    // next bit
		}
	}else{	// rising edge
		if(_dT<Tbit_lh_l){	// too short
			_error |= rcRxFrameError;
			return;
		}else{
			_dT=_dT-Tbit_hl_l;
            while(_dT>Tbit_lh_l){
				_dT=_dT-Tbit_hl_l;
				_SSByte &= (~_SSBitMask);	// there were some bits zero
				_SSBitNumber++;_SSBitMask=_SSBitMask<<1;    // next bit
			}
            _SSByte |= _SSBitMask;	// current one is a bit one
            _SSBitNumber++;_SSBitMask=_SSBitMask<<1;    // next bit
		}
	}

	if(_SSBitNumber<(SOFTSERIAL_NBITSTOPROCESS-1)) return;
    // _SSBitNumber should be SOFTSERIAL_NBITSTOPROCESS-2 or -1 or -0
    #ifdef SSerialMiseAuPoint
        SSerialMaPPut('0'+_SSBitNumber);
    #endif
	// here, we have shifted in N+P+p bits
	//if((_SSByte & (1<<(SOFTSERIAL_NBITSTOPROCESS-2)))==0) _error |= rcRxStopError;	// stop bit not ok

	// check parity, the number of the 8 bits at one should be even if parity==2
	#if SOFTSERIAL_PARITY>0
		uint16_t _SSBitMask=1<<(SOFTSERIAL_LENGTH);
		uint8_t count=0;
		while(_SSBitMask!=0){
			if(_SSByte&_SSBitMask)count++;
			_SSBitMask=_SSBitMask>>1;
		}
        #ifdef SSerialMiseAuPoint
            count&=1;
            SSerialMaPPut('p'+count);
        #endif
		#if SOFTSERIAL_PARITY==2
			if((count&1)!=0) _error |= rcRxParityError;		// flag the error but continue
		#elif SOFTSERIAL_PARITY==1
			if((count&1)==0) _error |= rcRxParityError;		// flag the error but continue
		#endif
	#endif
	#if SOFTSERIAL_LENGTH==7
		_SSByte=_SSByte&0x7F;	// remove parity & stop bit
	#else
		_SSByte=_SSByte&0xFF;	// remove parity & stop bit
	#endif
	_SSByteAvailable=true;

    if(!_error){    // if we got a good byte: start, parity OK, and stop, the line is operational
        _SSStatus=SSOperational;
        //todo: the following works with micros64() or system_get_time(), but does not work with _my_micros64()
        //SSTimeLastByteOK=micros64();
        //SSTimeLastByteOK=_my_micros64();
        SSTimeLastByteOK=systemtime();
    }

  rx_buffer_index_t ii;
    ii = (rx_buffer_index_t)(_rx_buffer_head + 1) % SOFTSERIAL_RX_BUFFER_SIZE;		
    /*
        we should be storing the received character into the location [head]
        but if head=tail-1 modulo buffer size, 
        we're about to overflow the buffer and so we don't write the character.
    */
    if (ii != _rx_buffer_tail) {
        _rx_buffer[_rx_buffer_head] = (uint8_t)_SSByte;
        _newcarin = true;
        _rx_buffer_head = ii;
    }else{
        _error |= rcRxOverRun2;		//		_rx_buffer_head + 1 = _rx_buffer_tail
    }

    _SSState=SSWaitForStart;
    return;
}

//---------------------------------------------------------------------------------------------------------------------------------
void TICSerial_reset(){
	_rx_buffer_head=0;
	_rx_buffer_tail=0;
    _run=true;
	_newcarin = false;
	memset((void*)_rx_buffer,0,SOFTSERIAL_RX_BUFFER_SIZE);

	//_receivingMessage = false;
	//_messageAvailable = false;
    //_TICstatus=TICnotOperational;
    //_TICmessageTime = 0;
    TICSerial_reset0();
}

void TICSerial_wdog(void){
    uint32_t tmp32;
    if(_SSStatus==SSOperational){
        tmp32=systemtime() - SSTimeLastByteOK;
        if(tmp32 > (SOFTSERIAL_MAXSILENCE*1000/SYSTEMTIMERESOLUTION)){
            _SSStatus=SSnotOperational;
            _SSState=SSWaitForStart;
        }
    }
    #ifdef MapGPIO
    MapGPIOIntPush();   // send debug buffer of GPIOinterrupts.hpp to serial
    #endif
}

bool TICSerial_Operational(){ TICSerial_wdog(); if(_SSStatus==SSOperational) return(true); else return(false); }

int8_t TICSerial_state(void){ TICSerial_wdog(); return(_SSState); }

int8_t TICSerial_status(void){ TICSerial_wdog(); return(_SSStatus); }

uint8_t TICSerial_error(){ TICSerial_wdog(); return(_error); }

void TICSerial_begin(void){
    if(_SSState) return;        // already started ??
	pinMode(IORxpin, INPUT);    // Linky TIC serial input ; no pullup
	TICSerial_reset();

    #if SoftMode==1
	attachInterrupt(IORxpin, ISR_SoftSerial_GPIO, CHANGE );
    #elif SoftMode==2
    __attachInterruptArg(IORxpin, ISR_SoftSerial_GPIO, 0, CHANGE );
    #elif SoftMode==3
    GPIO_attachInterrupt(IORxpin,0,0,CHANGE );
    #endif 
    _SSStatus=SSnotOperational;
    _SSState=SSWaitForStart;
    SSerialMaP("IO=" xstr(IORxpin) ", baudrate=" xstr(SOFTSERIAL_BAUDRATE) ", mode=" xstr(SOFTSERIAL_LENGTH) "-" xstr(SOFTSERIAL_PARITY) "-" xstr(SOFTSERIAL_NSTOPS) 
        ", tbit=%d, error=%d%%, offset=%d%%, delays:lh[%d:%d],hl[%d:%d]\r\n",
        Tbit, Tbit_error, Tbit_offset, Tbit_lh_l,Tbit_lh_h,Tbit_hl_l,Tbit_hl_h);
}

uint8_t TICSerial_read(void){
	// get THE last character received, and increment pointer
    TICSerial_wdog();
	uint8_t ccc;
	ccc=0;
	if (_rx_buffer_head != _rx_buffer_tail) {
		ccc = _rx_buffer[_rx_buffer_tail];
		_rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SOFTSERIAL_RX_BUFFER_SIZE;
	}

    SSerialMaP("read 0x%0.2X, head-tail:0x%0.2X-0x%0.2X\r\n",ccc,_rx_buffer_head,_rx_buffer_tail);
	_error=0;
	_newcarin=false;
	return ccc;
}

int16_t TICSerial_available(void){
	// should be called periodically
    // return 0 is nothing is available
	// return errorcode<0 if overflow/parity error/frame error has occured
	// return N>0 is characters are available in a complete message

	#ifdef SSerialMiseAuPoint    // code pour mise au point fine, au bit près
        /*
        if(_SSedgenumber>6){
            SSerialMaP("Edges=%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
				DBGedgetime[0],DBGedgetime[1],DBGedgetime[2],DBGedgetime[3],DBGedgetime[4],DBGedgetime[5],DBGedgetime[6],DBGedgetime[7],DBGedgetime[8]);
            _SSedgenumber=0;
        }*/
        if(_SSByteAvailable){
            SSerialMaP("Byte=%0.2X, error=%0.2X timings=%d,%d,%d,%d,%d,%d,%d,%d,%d, head-tail:0x%0.2X-0x%0.2X\r\n",
				_SSByte, _error, DBGedgetime[0],DBGedgetime[1],DBGedgetime[2],DBGedgetime[3],DBGedgetime[4],DBGedgetime[5],DBGedgetime[6],DBGedgetime[7],DBGedgetime[8],_rx_buffer_head,_rx_buffer_tail);
            memset((void*)DBGedgetime,0,soDBGedgetime);
            _SSByteAvailable=false;
        }
    #endif
    TICSerial_wdog();
    if(_SSState==SSOff) return(0);
    if(_SSState==SSSuspended) return(0);
   	if(TICSerial_Operational() && (_error!=0)) {
        SSerialMaP("Error reading Rx bitstream on IO %d, error=0x%0.2X, status=%d, state=%d\r\n",IORxpin,_error,_SSStatus,_SSState);
		return(-_error);
	}
	//if(!_messageAvailable) return 0;
	//if(_TICstatus!=TICmessageAvailable) return 0;
	// ici nous avons recu un message STX...ETX complet

  int16_t tmp16;
	tmp16=((int16_t)((SOFTSERIAL_RX_BUFFER_SIZE + _rx_buffer_head) - _rx_buffer_tail)) % SOFTSERIAL_RX_BUFFER_SIZE;
    #ifdef toot//SSerialMiseAuPoint    // code pour mise au point fine, au bit près
      int16_t ii,jj;
        if(tmp16!=0 & _newcarin) {
            SSerialMaP("available=%d char, head-tail:0x%0.2X-0x%0.2X : ",tmp16,_rx_buffer_head,_rx_buffer_tail);
            for(ii=0;ii<tmp16;ii++){
                jj=(_rx_buffer_tail+ii)%SOFTSERIAL_RX_BUFFER_SIZE;
                uint8_t cc=_rx_buffer[jj];
                if(cc==0x0A) cc='n';
                else if(cc==0x0D) cc='r';
                SSerialMaP_("%c",cc);
            }
            SSerialMaP_("\r\n");
           	_newcarin=false;
        }
    #endif
	return(tmp16);
}

