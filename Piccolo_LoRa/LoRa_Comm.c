//###########################################################################
//
//  FILE:  Example_2803xSci_Echoback.c
//
//  TITLE: SCI Echo Back example
//
//!  \addtogroup f2803x_example_list
//!  <h1>SCI Echo Back(sci_echoback)</h1>
//!
//!  This test receives and echo-backs data through the SCI-A port.
//!
//!  The PC application 'hypterterminal' can be used to view the data
//!  from the SCI and to send information to the SCI.  Characters received
//!  by the SCI port are sent back to the host.
//!
//!  \b Running \b the \b Application
//!  -# Configure hyperterminal:
//!  Use the included hyperterminal configuration file SCI_96.ht.
//!  To load this configuration in hyperterminal
//!    -# Open hyperterminal
//!    -# Go to file->open
//!    -# Browse to the location of the project and
//!       select the SCI_96.ht file.
//!  -# Check the COM port.
//!  The configuration file is currently setup for COM1.
//!  If this is not correct, disconnect (Call->Disconnect)
//!  Open the File-Properties dialog and select the correct COM port.
//!  -# Connect hyperterminal Call->Call
//!  and then start the 2803x SCI echoback program execution.
//!  -# The program will print out a greeting and then ask you to
//!  enter a character which it will echo back to hyperterminal.
//!
//!  \note If you are unable to open the .ht file, you can create
//!  a new one with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!  - \b LoopCount, for the number of characters sent
//!  - ErrorCount
//!
//! \b External \b Connections \n
//!  Connect the SCI-A port to a PC via a transceiver and cable.
//!  - GPIO28 is SCI_A-RXD (Connect to Pin3, PC-TX  , of serial DB9 cable)
//!  - GPIO29 is SCI_A-TXD (Connect to Pin2, PC-RX, of serial DB9 cable)
//
//###########################################################################
// $TI Release: F2803x Support Library v2.02.00.00 $
// $Release Date: Sun Oct  4 16:06:22 IST 2020 $
// $Copyright:
// Copyright (C) 2009-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Flash2803x_API_Library.h"
//#include "DSP2803x_Adc.c"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
//#include <time.h>
//
// Functions Prototypes
//
//
// Defines
//
// Define AUTOBAUD to use the autobaud lock feature
//#define AUTOBAUD





//
// LoraWAN parameters this must be different for each device
//
// ABP parameters
char *NwSKey = "66C0736375DB3FA2331D26949262F356\r\n";
char *AppSKey = "8788B4374A546CED366A614187B519BA\r\n";
char *DevAddr = "260B2EB7\r\n";                         //MUST be unique for every device



//OTAA parameters
char *DevEui = "0018E605391D0467\r\n\0";                //Device Unique Identifier must be assigned by the network or use the hardware number.
char *AppEui = "70B3D57ED0032BB6\r\n";
char *AppKey = "F8575B42766E4BCA2ED77C2648FBCB09\r\n";


// Type of join to the network
bool joinMode = false; //0 = ABP, 1 = OTAA


//
//String Definition for concatenation
//


// MAC set Commands
char *setDr = "mac set dr "; // Datarate set to 0 SF10BW250
char *setPwrIdx = "mac set pwridx "; // Power Index 5 => (30 - 2 * 5) = 20 dBm (18.5 Max)
char *setReTxN = "mac set retx ";
char *setRx2 = "mac set rx2 ";
char *setRxDly = "mac set rxdelay1 "; //5 Second recommendation of TTN
char *txUnconf = "mac tx uncnf 3 "; //Transmit on port 1


char *hexPayload; //momentary payload for testing
char *receivedChar;
//
// Function Prototypes
//

//
//General Functions
//
void startTimer(void);
void currentTime(void);
void StopTimer(void);
void str2Hex(char *in);
void hex2Str (char *in);

//
//SCI
//
void setChannelTI(int channel);
void scia_fifo_init();
void scia_echoback_init();
void scia_msg(char * msg);
void scia_xmit(int a);
void StartTimer(void);
void StopTimer(void);
void setChannel(int channel);
void startRn(void);
void delay_loop();
void macSave(void);
void loRaUnconfTx(char *payload);
bool macJoinABP();
bool macJoinOTAA();
bool setABP();
void setDevEUI();
void setOTAA();
void setAppKEY();
void setAppEUI();
void setNwSKey();
void setAppSKey();
void setDevAdd();
/*
//
//ADC
//
__interrupt void adc_isr(void);
void Adc_Config(void);

Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];
Uint16 Voltage3[10];
*/

//
// Global Variables
//
char hexChars[16] = "0123456789ABCDEF";
Uint16 LoopCount;
long ErrorCount;
Uint32 startTime;
Uint32 crrntTime_;
long timeSec;
Uint16 buffer_c[64]; //RX buffer
char temp[3];
char temport[32];
char receivedChars[32];
char *asciiPayload;
long i;
//
// Main
//




void main(void)
{
    int k;


    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2803x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2803x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio(); Skipped for this example

    //
    // For this example, only init the pins for the SCI-A port.
    // This function is found in the DSP2803x_Sci.c file.
    //
    InitSciaGpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
    // This function is found in DSP2803x_PieVect.c.
    //
    InitPieVectTable();
    /*

    EALLOW;             // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT1 = &adc_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER |= M_INT1;                     // Enable CPU Interrupt 1
    EINT;                              // Enable Global interrupt INTM
    ERTM;                              // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    ConversionCount = 0;

    //
    // Configure ADC
    // Note: Channel ADCINA4  will be double sampled to workaround the
    // ADC 1st sample issue for rev0 silicon errata
    //
    EALLOW;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; //ADCINT1 trips after AdcResults latch
    AdcRegs.INTSEL1N2.bit.INT1E     = 1; // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0; // Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 2;  // setup EOC2 to trigger
                                          // ADCINT1 to fire

    //
    // set SOC0 channel select to ADCINA4
    // (dummy sample for rev0 errata workaround)
    //
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 4;

    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 4;  //set SOC1 channel select to ADCINA4
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;  //set SOC2 channel select to ADCINA2
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 6;  //set SOC3 channel select to ADCINA6

    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1, then SOC2
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;

    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1, then SOC2
    //
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;

    //
    // set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1, then SOC2
    //
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;


    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;

    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;

    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;

    //
    // set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;



    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;
    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;    // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;    // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA    = 0x0080;       // Set compare A value
    EPwm1Regs.TBPRD                 = 0xFFFF;   // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE     = 0;        // count up and start
    //
    // Step 4. Initialize all the Device Peripherals:
    // Not required for this example
    //
    */
    //
    // Step 5. User specific code:
    //
    LoopCount = 1; // Only used for range testing


    scia_fifo_init();	                // Initialize the SCI FIFO
    scia_echoback_init();               // Initialize SCI for echoback

    InitCpuTimers();
    startTimer();
    //ConfigCpuTimer(&CpuTimer1, 0.0006, 0xFFFFFFFF); // Initialize CPU Timer 1, with a clock of 1 Hz, With the biggest period possible

    for(;;){ // Wait 1S for RN to start
        currentTime();
        if (crrntTime_ >= 1570){
            break;
        }
    }
    //
    //Turn on the 8 channels
    //
    setChannel(8); //916.8 MHz
    setChannel(9); //917.0 MHz
    setChannel(10); //917.2 MHz
    setChannel(11); //917.4 MHz
    setChannel(12); //917.6 MHz
    setChannel(13); //917.8 MHz
    setChannel(14); //918.0 MHz
    setChannel(15); //918.2 MHz

    scia_msg("mac set dr 0\r\n"); //Set Data Rate to SF10BW125
    delay_loop();
    scia_msg("mac set pwridx 5\r\n"); //Set Power Index to 20 dBm (max 18.5, hardware limit)
    delay_loop();
    scia_msg("mac set rxdelay1 5000\r\n"); // Set First Window of RX to 5 Seconds
    delay_loop();
    //
    //Save Mac states, variables and join with the respective procedure
    //

    if (joinMode){//Send ABP parameters and join
        setDevEUI();
        delay_loop();
        setAppKEY();
        delay_loop();
        setAppEUI();
        delay_loop();
        scia_msg("mac save\r\n\0");
        macJoinOTAA(); //Send OTAA parameters and join
    }
    else{
        delay_loop();
        setDevEUI();
        delay_loop();
        setDevAdd();
        delay_loop();
        setNwSKey();
        delay_loop();
        setAppSKey();
        delay_loop();
        delay_loop();
        delay_loop();
        //scia_msg("mac save\r\n");


        scia_msg("mac join abp\r\n");
        for (i = 0; i < 1000000; i++) {} //wait for "accepted" message


    }

    //macSave();
//
//Main Loop
//
    startTimer();

    for(;;)
    {

        //Transmit Code
        currentTime();
        if (crrntTime_ >= 95770){ //1570 approx 1 sec // 1 m = 95770 //
            ltoa(LoopCount, temp, 10);
            char stringfCC[13] = "Packet-";
            strcat(stringfCC, temp);
            str2Hex(stringfCC);
            hexPayload = strcat(temport, "\r\n");
            startTimer();
            loRaUnconfTx(hexPayload);
            LoopCount++;
        }

        //Receive Code
        i = 0;
        while (SciaRegs.SCIFFRX.bit.RXFFST == 1){
            buffer_c[i] = SciaRegs.SCIRXBUF.all;
            i++;
        }


        if (i != 0){
            for (k = 0; k < i; k++){
                scia_xmit(buffer_c[k]);
                receivedChar = strcat(receivedChar, (char *)buffer_c[k]);
            }
            scia_msg(receivedChar);
        }

    }
}


// Calculate time using cpuTimer1


void startTimer(void){
    EALLOW;
    CpuTimer1Regs.TCR.bit.TSS = 1;                    //Stop the timer
    CpuTimer1Regs.TPR.all = 0x8700;                   //Set the prescale divider
    CpuTimer1Regs.TPRH.all = 0x0393;
    CpuTimer1Regs.PRD.all = 0xFFFFFFFF;               //Set the largest possible period
    CpuTimer1Regs.TCR.bit.TRB = 1;                    //Reload the timer and prescale counters
    CpuTimer1Regs.TCR.bit.TSS = 0;                    //Start the timer
    EDIS;
}


void currentTime(void){ //"Millis" function
    crrntTime_ = 0xFFFFFFFF - CpuTimer1Regs.TIM.all;    //Get the current time 1570 approx 1s
}


//
//LoraWAN functions for RN2903
//
void loRaUnconfTx(char *payload){ //Tx payload through LoRa, channel 1, unconfirmed.
    scia_msg(txUnconf);
    scia_msg(payload);
}




//Function to setup channel
void setChannel(int channel){
    switch (channel) {
        case 8:
            scia_msg("mac set ch status 8 on\r\n");
            break;
        case 9:
            scia_msg("mac set ch status 9 on\r\n");
            break;
        case 10:
            scia_msg("mac set ch status 10 on\r\n");
            break;
        case 11:
            scia_msg("mac set ch status 11 on\r\n");
            break;
        case 12:
            scia_msg("mac set ch status 12 on\r\n");
            break;
        case 13:
            scia_msg("mac set ch status 13 on\r\n");
            break;
        case 14:
            scia_msg("mac set ch status 14 on\r\n");
            break;
        case 15:
            scia_msg("mac set ch status 15 on\r\n");
            break;
    }
    delay_loop();
    }


//Function that performs join procedure (either ABP or OTAA)
bool macJoinABP(){
    //char *resp;
    scia_msg("mac join abp\r\n");
    delay_loop();
    delay_loop();
    return true;
}


bool macJoinOTAA(){
    //char *resp;
    scia_msg("mac join otaa\r\n\0");
    delay_loop();
    return true;
}


// Function that sets OTAA parameters.
void setOTAA(){
    //delay_loop();
    setDevEUI();
    delay_loop();
    setAppKEY();
    delay_loop();
    setAppEUI();
    delay_loop();
}


//Function that sets ABP parameters.
bool setABP(){
    setDevEUI();
    delay_loop();
    setDevAdd();
    delay_loop();
    setNwSKey();
    delay_loop();
    setAppSKey();
    delay_loop();
    return true;
}
//Function that sets AppSKey
void setAppSKey(){
    scia_msg("mac set appskey ");
    scia_msg(AppSKey);
}


//Function that sets NwSKey
void setNwSKey(){
    scia_msg("mac set nwkskey ");
    scia_msg(NwSKey);
    delay_loop();
}


//Function that sets DevAdd
void setDevAdd(){
    scia_msg("mac set devaddr ");
    scia_msg(DevAddr);
    delay_loop();

}


//Function that sets DevEui
void setDevEUI(){
    scia_msg("mac set deveui ");
    scia_msg(DevEui);
    delay_loop(); //4ms of delay for the answer
}


//Function that sets appEui
void setAppEUI(){
    char *send;
    send = strcat("mac set appeui ", AppEui);
    scia_msg(send);
    delay_loop(); //4ms of delay for the answer
}


//Function that sets appKey
void setAppKEY(){
    char *send3;
    send3 = strcat("mac set appkey ", AppKey);
    scia_msg(send3);
    delay_loop(); //4ms of delay for the answer

}




//
// scia_echoback_init - Test 1, SCIA  DLB, 8-bit word, baud rate 0x000F,
// default, 1 STOP bit, no parity
//
void scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral in the InitSysCtrl()
    // function
    //
    
    //
    // 1 stop bit,  No loopback, No parity, 8 char bits, async mode, 
    // idle-line protocol
    //
 	SciaRegs.SCICCR.all = 0x0007;
    
    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
	SciaRegs.SCICTL1.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

	SciaRegs.SCIHBAUD    = 0x0000;  // 9600 baud @LSPCLK = 15MHz (60 MHz SYSCLK)
    SciaRegs.SCILBAUD    = 0x0020;  //C2 for 9600; 20 for 57600 approx.

    SciaRegs.SCICCR.bit.LOOPBKENA = 0; //Enable Loop back
	SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a){
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {
        
    }
    SciaRegs.SCITXBUF = (int) a;
}

//
// scia_msg - 
//

void scia_msg(char *msg){
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init(){
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}


//
//Shameful way to delay operation
//
void delay_loop(){ //4 ms approx.
    for (i = 0; i < 100000; i++) {}
}



/*
//
//ADC Functions
//
__interrupt void
adc_isr(void)
{
    //
    // discard ADCRESULT0 as part of the workaround to the
    // 1st sample errata for rev0
    //
    Voltage1[ConversionCount] = AdcResult.ADCRESULT1;

    Voltage2[ConversionCount] = AdcResult.ADCRESULT2;

    Voltage3[ConversionCount] = AdcResult.ADCRESULT3;
    //
    // If 20 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }

    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

*/



// Hex Conversion
void str2Hex(char *in){
   int index;
   int rest;
   int inlen = strlen(in);
   for (i = 0; i < 30; i++){
       if (i == inlen){
           temport[2*i] = NULL;
           break;
       }
        index = ((int) in[i]) / 16;
        rest = ((int) in[i]) % 16;
        temport[2*i] = hexChars[index];
        temport[2*i + 1] = hexChars[rest];
   }

}


//This theoretically works
//ASCII Conversion
void hex2Str (char *in){
    int p;
    int index_;
    int rest_;
    int total;
    int inlen = strlen(in);
    for (p = 1; p <= inlen/2; p++){
        index_ = atoi(in[2*p-1]);
        rest_ = atoi(in[2*p]);
        total = index_ * 16 + rest_;
        receivedChars[p] = (char) total;
    }

}








//
// Flash simulated EEPROM
//




//
// End of File
//

