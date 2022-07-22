#include <p32xxxx.h>
#include <plib.h>
/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_midi.h"
#include "USB/usb_function_cdc.h"
#include "BspInit.h"
#include "UsbProcess.h"
#include "Player.h"

/** CONFIGURATION **************************************************/

#pragma config UPLLEN = ON      // USB PLL Enabled
#pragma config FPLLMUL = MUL_15 // PLL Multiplier
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider
#pragma config FPLLODIV = DIV_1 // PLL Output Divider
#pragma config FPBDIV = DIV_1   // Peripheral Clock divisor
#pragma config FWDTEN = OFF     // Watchdog Timer
#pragma config WDTPS = PS1      // Watchdog Timer Postscale
//#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF    // CLKO Enable
#pragma config POSCMOD = XT      // Primary Oscillator
#pragma config IESO = OFF        // Internal/External Switch-over
#pragma config FSOSCEN = OFF     // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC = PRIPLL    // Oscillator Selection
#pragma config CP = OFF          // Code Protect
#pragma config BWP = OFF         // Boot Flash Write Protect
#pragma config PWP = OFF         // Program Flash Write Protect
#pragma config ICESEL = ICS_PGx4 // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF
#pragma config FVBUSONIO = OFF

/** VARIABLES ******************************************************/

extern unsigned char ReceivedDataBuffer[64];
extern unsigned char ToSendDataBuffer[64];
extern USB_AUDIO_MIDI_EVENT_PACKET midiData;

extern USB_HANDLE USBTxHandle;
extern USB_HANDLE USBRxHandle;

extern USB_VOLATILE BYTE msCounter;

void synth_wave(short *buffer_pp, int len);

Player mPlayer;

/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
void ProcessIO(void);
void UserInit(void);
#define ONE_FIX_POINT 255
int lastGain = ONE_FIX_POINT;
#define timeConstAttack 10
#define timeConstRelease 200
int lastPeak = 0;
int compressEffect(int sampleIn,int threshold)
{
	int currentGain;
	int currentPeak;
	if (abs(sampleIn) > threshold)
	{
		currentGain = threshold * ONE_FIX_POINT / abs(sampleIn);
	}
	else
	{
		currentGain = ONE_FIX_POINT;
	}
	currentPeak = currentGain;

	if (lastPeak > currentPeak)
	{
		currentPeak = (timeConstAttack*lastPeak + (ONE_FIX_POINT-timeConstAttack) *currentGain)/ ONE_FIX_POINT;
	}
	else
	{
		currentPeak = (timeConstRelease*lastPeak + (ONE_FIX_POINT - timeConstRelease) *currentGain) / ONE_FIX_POINT;
	}
	lastPeak = currentPeak;

	return sampleIn * currentPeak / ONE_FIX_POINT;
}
/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
int main(void)
{
    UserInit();
    InitializeSystem();
    USBDeviceAttach();
    PlayerInit(&mPlayer);
    PlayerPlay(&mPlayer);
    while (1)
    {
        // source: http://chipkit.net/forum/viewtopic.php?t=3137
        if (bufferAFull == 0)
        {
            synth_wave(&buffer_a[0], BUFFER_LENGTH);
            bufferAFull = 1;
        }
        if (bufferBFull == 0)
        {
            synth_wave(&buffer_b[0], BUFFER_LENGTH);
            bufferBFull = 1;
        }
        //UsbProcess();
        PlayerProcess(&mPlayer);

    } //end while
} //end main

void synth_wave(short *buffer_pp, int len)
{
    PORTBbits.RB15 = 1;

    mPlayer.mainSynthesizer.volume = ReadADC10(0);

    for (int i = 0; i < len; i += 2)
    {
        Player32kProc(&mPlayer);


		int32_t rawSynthOutput = mPlayer.mainSynthesizer.mixOut;
		//rawSynthOutput = compressEffect(rawSynthOutput, 31000);
		if (rawSynthOutput < -32768)
		{
			rawSynthOutput = -32768;
		}	
		else if (rawSynthOutput > 32767)
		{
			rawSynthOutput = 32767;
		}
        
        buffer_pp[i] = rawSynthOutput;
        buffer_pp[i + 1] = rawSynthOutput;
    }
    OC1R = abs(buffer_pp[0]);
    PORTBbits.RB15 = 0;
}

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{
    //initialize the variable holding the handle for the last
    // transmission
    USBTxHandle = NULL;
    USBRxHandle = NULL;
} //end UserInit

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    if (USBSuspendControl == 1)
    {
    }
    else
    {
        if (USBDeviceState == DETACHED_STATE)
        {
        }
        else if (USBDeviceState == ATTACHED_STATE)
        {
        }
        else if (USBDeviceState == POWERED_STATE)
        {
        }
        else if (USBDeviceState == DEFAULT_STATE)
        {
        }
        else if (USBDeviceState == ADDRESS_STATE)
        {
        }
        else if (USBDeviceState == CONFIGURED_STATE)
        {
        } //end if(...)
    }     //end if(UCONbits.SUSPND...)

} //end BlinkUSBStatus

/** EOF main.c *************************************************/
