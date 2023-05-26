//*****************************************************************************
//
// enet_lwip.c - Sample WebServer Application using lwIP.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C129EXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#include "utils/locator.h"
#include "utils/lwiplib.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"

#include "httpserver_raw/httpd.h"

#include "drivers/pinout.h"

// added from github
#include <string.h>
#include <inc/hw_types.h>
#include "driverlib/emac.h"
#include "driverlib/rom.h"
#include "utils/ptpdlib.h"
#include "utils/random.h"
#include "lwip/sys.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Ethernet with lwIP (enet_lwip)</h1>
//!
//! This example application demonstrates the operation of the Tiva
//! Ethernet controller using the lwIP TCP/IP Stack.  DHCP is used to obtain
//! an Ethernet address.  If DHCP times out without obtaining an address,
//! AutoIP will be used to obtain a link-local address.  The address that is
//! selected will be shown on the UART.
//!
//! UART0, connected to the ICDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application. Use the
//! following command to re-build the any file system files that change.
//!
//!     ../../../../tools/bin/makefsfile -i fs -o enet_fsdata.h -r -h -q
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)
#define SYSTICKUS               (1000000 / SYSTICKHZ)
#define SYSTICKNS               (1000000000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

//*****************************************************************************
//
// A set of flags used to track the state of the application.
//
//*****************************************************************************
static volatile unsigned long g_ulFlags;
#define FLAG_PPSOUT             0           // PPS Output is on.
#define FLAG_PPSOFF             1           // PPS Output should be turned off.
#define FLAG_PTPDINIT           2           // PTPd has been initialized.
#define FLAG_PTPTIMESET         3           // PTP set GMT time.
#define FLAG_IPUPDATE           4           // The IP address has changed.

//*****************************************************************************
//
// The current IP address.
//
//*****************************************************************************
uint32_t g_ui32IPAddress;

//*****************************************************************************
//
// The system clock frequency.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
uint32_t g_tickNs;

//*****************************************************************************
//
// Local data for clocks and timers.
//
//*****************************************************************************
static volatile unsigned long g_ulNewSystemTickReload = 0;
static volatile unsigned long g_ulSystemTickHigh = 0;
static volatile unsigned long g_ulSystemTickReload = 0;

//*****************************************************************************
//
// Statically allocated runtime options and parameters for PTPd.
//
//*****************************************************************************
static PtpClock g_sPTPClock;
static ForeignMasterRecord g_psForeignMasterRec[DEFAULT_MAX_FOREIGN_RECORDS];
static RunTimeOpts g_sRtOpts;

//*****************************************************************************
//
// System Run Time - Ticks
//
//*****************************************************************************
volatile unsigned long g_ulSystemTimeTicks;

//*****************************************************************************
//
// Local function prototypes.
//
//*****************************************************************************
static void ptpd_init(void);
static void ptpd_tick(void);

//*****************************************************************************
//
// A twirling line used to indicate that DHCP/AutoIP address acquisition is in
// progress.
//
//*****************************************************************************
static char g_pcTwirl[4] = { '\\', '|', '/', '-' };

//*****************************************************************************
//
// The index into the twirling line array of the next line orientation to be
// printed.
//
//*****************************************************************************
static unsigned long g_ulTwirlPos = 0;

//*****************************************************************************
//
// A mapping from day of the week numbers to the name of the day.
//
//*****************************************************************************
const char *g_ppcDay[7] =
{
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

//*****************************************************************************
//
// A mapping from month numbers to the name of the month.
//
//*****************************************************************************
const char *g_ppcMonth[12] =
{
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

//*****************************************************************************
//
// The most recently assigned IP address.  This is used to detect when the IP
// address has changed (due to DHCP/AutoIP) so that the new address can be
// printed.
//
//*****************************************************************************
static unsigned long g_ulLastIPAddr = 0;

//*****************************************************************************
//
// System Time - Internal representaion.
//
//*****************************************************************************
volatile unsigned long g_ulSystemTimeSeconds;
volatile unsigned long g_ulSystemTimeNanoSeconds;

//*****************************************************************************
//
// The random number seed, which corresponds to the most recently returned
// random number.  This is set based on the entropy-generated random number
// by RandomSeed().
//
//*****************************************************************************
static unsigned long g_ulRandomSeed = 0;

//*****************************************************************************
//
// Generate a new random number.  The number returned would more accruately be
// described as a pseudo-random number since a linear congruence generator is
// being used.
//
//*****************************************************************************
unsigned long
RandomNumber(void)
{
    //
    // Generate a new pseudo-random number with a linear congruence random
    // number generator.  This new random number becomes the seed for the next
    // random number.
    //
    g_ulRandomSeed = (g_ulRandomSeed * 1664525) + 1013904223;

    //
    // Return the new random number.
    //
    return(g_ulRandomSeed);
}


//*****************************************************************************
//
// Volatile global flag to manage LED blinking, since it is used in interrupt
// and main application.  The LED blinks at the rate of SYSTICKHZ.
//
//*****************************************************************************
volatile bool g_bLED;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

    //
    // Display the string.
    //
    UARTprintf(pcBuf);
}

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void
lwIPHostTimerHandler(void)
{
#ifdef ewarm
    volatile unsigned long ulIPAddress;
#else
    unsigned long ulIPAddress;
#endif

    //
    // Get the local IP address.
    //
    ulIPAddress = lwIPLocalIPAddrGet();

    //
    // If IP Address has not yet been assigned, update the display accordingly
    //
    if(ulIPAddress == 0)
    {
        //
        // Draw a spinning line to indicate that the IP address is being
        // discovered.
        //
        UARTprintf("\b%c", g_pcTwirl[g_ulTwirlPos]);

        //
        // Update the index into the twirl.
        //
        g_ulTwirlPos = (g_ulTwirlPos + 1) & 3;
    }

    //
    // Check if IP address has changed, and display if it has.
    //
    else if(g_ulLastIPAddr != ulIPAddress)
    {
        //
        // Save the new IP address.
        //
        g_ulLastIPAddr = ulIPAddress;

        //
        // Tell the main task to update the display with the new IP information.
        //
        HWREGBITW(&g_ulFlags, FLAG_IPUPDATE) = 1;
    }

    //
    // If IP address has been assigned, initialize the PTPD software (if
    // not already initialized).
    //
    if(ulIPAddress && !HWREGBITW(&g_ulFlags, FLAG_PTPDINIT))
    {
        ptpd_init();
        HWREGBITW(&g_ulFlags, FLAG_PTPDINIT) = 1;
    }

    //
    // If PTPD software has been initialized, run the ptpd tick.
    //
    if(HWREGBITW(&g_ulFlags, FLAG_PTPDINIT))
    {
        ptpd_tick();
    }
}

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    unsigned long ulTemp;
    struct tm sLocalTime;

    //
    // Update internal time and set PPS output, if needed.
    //
    g_ulSystemTimeNanoSeconds += SYSTICKNS;
    if(g_ulSystemTimeNanoSeconds >= 1000000000)
    {
//        ROM_GPIOPinWrite(PPS_GPIO_BASE, PPS_GPIO_PIN, PPS_GPIO_PIN);
        g_ulSystemTimeNanoSeconds -= 1000000000;
        g_ulSystemTimeSeconds += 1;
        HWREGBITW(&g_ulFlags, FLAG_PPSOUT) = 1;
    }

    //
    // Set a new System Tick Reload Value.
    //
    ulTemp = g_ulSystemTickReload;
    if(ulTemp != g_ulNewSystemTickReload)
    {
        g_ulSystemTickReload = g_ulNewSystemTickReload;

        g_ulSystemTimeNanoSeconds = ((g_ulSystemTimeNanoSeconds / SYSTICKNS) *
                                     SYSTICKNS);
    }

    //
    // For each tick, set the next reload value for fine tuning the clock.
    //
    ulTemp = g_ulSystemTimeTicks;
    if((ulTemp % g_tickNs) < g_ulSystemTickHigh)
    {
        MAP_SysTickPeriodSet(g_ulSystemTickReload + 1);
    }
    else
    {
        MAP_SysTickPeriodSet(g_ulSystemTickReload);
    }

    //
    // Service the PTPd Timer.
    //
    timerTick(SYSTICKMS);

    //
    // Increment the run-time tick counter.
    //
    g_ulSystemTimeTicks++;

    //
    // Clear PPS output when needed and display time of day.
    //
    if(HWREGBITW(&g_ulFlags, FLAG_PPSOFF))
    {
        //
        // Negate the PPS output.
        //
//        ROM_GPIOPinWrite(PPS_GPIO_BASE, PPS_GPIO_PIN, 0);

        //
        // Indicate that the PPS output has been negated.
        //
        HWREGBITW(&g_ulFlags, FLAG_PPSOFF) = 0;

        //
        // Only print the date and time if PTPd has been started (in other
        // words, if an IP address has been obtained).
        //
        if(HWREGBITW(&g_ulFlags, FLAG_PTPDINIT))
        {
            //
            // Convert the elapsed seconds (ulSeconds) into time structure.
            //
            ulocaltime(g_ulSystemTimeSeconds, &sLocalTime);

            //
            // Print out the date and time.
            //
            UARTprintf("\r%3s %3s %2d, %4d %02d:%02d:%02d (GMT)",
                       g_ppcDay[sLocalTime.tm_wday],
                       g_ppcMonth[sLocalTime.tm_mon], sLocalTime.tm_mday,
                       sLocalTime.tm_year, sLocalTime.tm_hour, sLocalTime.tm_min,
                       sLocalTime.tm_sec);
        }
    }

    //
    // Setup to disable the PPS output on the next pass.
    //
    if(HWREGBITW(&g_ulFlags, FLAG_PPSOUT))
    {
        //
        // Setup to turn off the PPS output.
        //
        HWREGBITW(&g_ulFlags, FLAG_PPSOUT) = 0;
        HWREGBITW(&g_ulFlags, FLAG_PPSOFF) = 1;
    }

    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(SYSTICKMS);
}

//*****************************************************************************
//
// Display Statistics.  For now, do nothing, but this could be used to either
// update a web page, send data to the serial port, or to the OLED display.
//
// Refer to the ptpd software "src/dep/sys.c" for example code.
//
//*****************************************************************************
void
displayStats(RunTimeOpts *rtOpts, PtpClock *ptpClock)
{
}

//*****************************************************************************
//
// This function returns the local time (in PTPd internal time format).  This
// time is maintained by the SysTick interrupt.
//
// Note: It is very important to ensure that we detect cases where the system
// tick rolls over during this function.  If we don't do this, there is a race
// condition that will cause the reported time to be off by a second or so
// once in a blue moon.  This, in turn, causes large perturbations in the
// 1588 time controller resulting in large deltas for many seconds as the
// controller tries to compensate.
//
//*****************************************************************************
void
getTime(TimeInternal *time)
{
    unsigned long ulTime1;
    unsigned long ulTime2;
    unsigned long ulSeconds;
    unsigned long ulPeriod;
    unsigned long ulNanoseconds;

    //
    // We read the SysTick value twice, sandwiching taking snapshots of
    // the seconds, nanoseconds and period values.  If the second SysTick read
    // gives us a higher number than the first read, we know that it wrapped
    // somewhere between the two reads so our seconds and nanoseconds
    // snapshots are suspect.  If this occurs, we go round again.  Note that
    // it is not sufficient merely to read the values with interrupts disabled
    // since the SysTick counter keeps counting regardless of whether or not
    // the wrap interrupt has been serviced.
    //
    do
    {
        ulTime1 = MAP_SysTickValueGet();
        ulSeconds = g_ulSystemTimeSeconds;
        ulNanoseconds = g_ulSystemTimeNanoSeconds;
        ulPeriod = MAP_SysTickPeriodGet();
        ulTime2 = MAP_SysTickValueGet();

#ifdef DEBUG
        //
        // In debug builds, keep track of the number of times this function was
        // called just as the SysTick wrapped.
        //
        if(ulTime2 > ulTime1)
        {
            g_ulSysTickWrapDetect++;
            g_ulSysTickWrapTime = ulSeconds;
        }
#endif
    }
    while(ulTime2 > ulTime1);

    //
    // Fill in the seconds field from the snapshot we just took.
    //
    time->seconds = ulSeconds;

    //
    // Fill in the nanoseconds field from the snapshots.
    //
    time->nanoseconds = (ulNanoseconds + (ulPeriod - ulTime2) * g_tickNs);

    //
    // Adjust for any case where we accumulate more than 1 second's worth of
    // nanoseconds.
    //
    if(time->nanoseconds >= 1000000000)
    {
#ifdef DEBUG
        g_ulGetTimeWrapCount++;
#endif
        time->seconds++;
        time->nanoseconds -= 1000000000;
    }
}

//*****************************************************************************
//
// Get the RX Timestamp.  This is called from the lwIP low_level_input function
// when configured to include PTPd support.
//
//*****************************************************************************
void
lwIPHostGetTime(u32_t *time_s, u32_t *time_ns)
{
    TimeInternal psRxTime;

    //
    // Get the current IEEE1588 time.
    //
    getTime(&psRxTime);

    //
    // Fill in the appropriate return values.
    //
    *time_s = psRxTime.seconds;
    *time_ns = psRxTime.nanoseconds;
}

//*****************************************************************************
//
// This function returns a random number, using the functions in random.c.
//
//*****************************************************************************
UInteger16
getRand(UInteger32 *seed)
{
    unsigned long ulTemp;
    UInteger16 uiTemp;

    //
    // Re-seed the random number generator.
    //
    RandomAddEntropy(*seed);
    RandomSeed();

    //
    // Get a random number and return a 16-bit, truncated version.
    //
    ulTemp = RandomNumber();
    uiTemp = (UInteger16)(ulTemp & 0xFFFF);
    return(uiTemp);
}

//*****************************************************************************
//
// This function will set the local time (provided in PTPd internal time
// format).  This time is maintained by the SysTick interrupt.
//
//*****************************************************************************
void
setTime(TimeInternal *time)
{
    sys_prot_t sProt;

    //
    // Update the System Tick Handler time values from the given PTPd time
    // (fine-tuning is handled in the System Tick handler).  We need to update
    // these variables with interrupts disabled since the update must be
    // atomic.
    //
    sProt = sys_arch_protect();
    g_ulSystemTimeSeconds = time->seconds;
    g_ulSystemTimeNanoSeconds = time->nanoseconds;

    //
    // Set the flag indicating that PTP has set our system clock.
    //
    HWREGBITW(&g_ulFlags, FLAG_PTPTIMESET) = 1;

    sys_arch_unprotect(sProt);
}

//*****************************************************************************
//
// Based on the value (adj) provided by the PTPd Clock Servo routine, this
// function will adjust the SysTick periodic interval to allow fine-tuning of
// the PTP Clock.
//
//*****************************************************************************
Boolean
adjFreq(Integer32 adj)
{
    unsigned long ulTemp;

    //
    // Check for max/min value of adjustment.
    //
    if(adj > ADJ_MAX)
    {
        adj = ADJ_MAX;
    }
    else if(adj < -ADJ_MAX)
    {
        adj = -ADJ_MAX;
    }

    //
    // Convert input to nanoseconds / systick.
    //
    adj = adj / SYSTICKHZ;

    //
    // Get the nominal tick reload value and convert to nano seconds.
    //
    ulTemp = (g_ui32SysClock / SYSTICKHZ) * g_tickNs;

    //
    // Factor in the adjustment.
    //
    ulTemp -= adj;

    //
    // Get a modulo count of nanoseconds for fine tuning.
    //
    g_ulSystemTickHigh = ulTemp % g_tickNs;

    //
    // Set the reload value.
    //
    g_ulNewSystemTickReload = ulTemp / g_tickNs;

    //
    // Return.
    //
    return(TRUE);
}

//*****************************************************************************
//
// Initialization code for PTPD software.
//
//*****************************************************************************
static void
ptpd_init(void)
{
    uint32_t ulTemp;
    uint32_t ui32User0, ui32User1;

    //
    // Clear out all of the run time options and protocol stack options.
    //
    memset(&g_sRtOpts, 0, sizeof(g_sRtOpts));
    memset(&g_sPTPClock, 0, sizeof(g_sPTPClock));

    //
    // Initialize all PTPd run time options to a valid, default value.
    //
    g_sRtOpts.syncInterval = DEFAULT_SYNC_INTERVAL;
    memcpy(g_sRtOpts.subdomainName, DEFAULT_PTP_DOMAIN_NAME,
           PTP_SUBDOMAIN_NAME_LENGTH);
    memcpy(g_sRtOpts.clockIdentifier, IDENTIFIER_DFLT, PTP_CODE_STRING_LENGTH);
    g_sRtOpts.clockVariance = (UInteger32)DEFAULT_CLOCK_VARIANCE;
    g_sRtOpts.clockStratum = DEFAULT_CLOCK_STRATUM;
    g_sRtOpts.clockPreferred = FALSE;
    g_sRtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;
    g_sRtOpts.epochNumber = 0;
    memcpy(g_sRtOpts.ifaceName, "LMI", strlen("LMI"));
    g_sRtOpts.noResetClock = DEFAULT_NO_RESET_CLOCK;
    g_sRtOpts.noAdjust = FALSE;
    g_sRtOpts.displayStats = FALSE;
    g_sRtOpts.csvStats = FALSE;
    g_sRtOpts.unicastAddress[0] = 0;
    g_sRtOpts.ap = DEFAULT_AP;
    g_sRtOpts.ai = DEFAULT_AI;
    g_sRtOpts.s = DEFAULT_DELAY_S;
    g_sRtOpts.inboundLatency.seconds = 0;
    g_sRtOpts.inboundLatency.nanoseconds = DEFAULT_INBOUND_LATENCY;
    g_sRtOpts.outboundLatency.seconds = 0;
    g_sRtOpts.outboundLatency.nanoseconds = DEFAULT_OUTBOUND_LATENCY;
    g_sRtOpts.max_foreign_records = DEFAULT_MAX_FOREIGN_RECORDS;
    g_sRtOpts.slaveOnly = TRUE;
    g_sRtOpts.probe = FALSE;
    g_sRtOpts.probe_management_key = 0;
    g_sRtOpts.probe_record_key = 0;
    g_sRtOpts.halfEpoch = FALSE;

    //
    // Initialize the PTP Clock Fields.
    //
    g_sPTPClock.foreign = &g_psForeignMasterRec[0];

    //
    // Configure port "uuid" parameters.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    g_sPTPClock.port_communication_technology = PTP_ETHER;
    g_sPTPClock.port_uuid_field[0] = ((ui32User0 >>  0) & 0xff);
    g_sPTPClock.port_uuid_field[1] = ((ui32User0 >>  8) & 0xff);
    g_sPTPClock.port_uuid_field[2] = ((ui32User0 >> 16) & 0xff);
    g_sPTPClock.port_uuid_field[3] = ((ui32User1 >>  0) & 0xff);
    g_sPTPClock.port_uuid_field[4] = ((ui32User1 >>  8) & 0xff);
    g_sPTPClock.port_uuid_field[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Enable Ethernet Multicast Reception (required for PTPd operation).
    // Note:  This must follow lwIP/Ethernet initialization.
    //
    ulTemp = EMACFrameFilterGet (EMAC0_BASE);
    ulTemp |= EMAC_FRMFILTER_HASH_MULTICAST;
    EMACFrameFilterSet (EMAC0_BASE, ulTemp);

    //
    // Run the protocol engine for the first time to initialize the state
    // machines.
    //
    protocol_first(&g_sRtOpts, &g_sPTPClock);
}

//*****************************************************************************
//
// Run the protocol engine loop/poll.
//
//*****************************************************************************
static void
ptpd_tick(void)
{
    //
    // Run the protocol engine for each pass through the main process loop.
    //
    protocol_loop(&g_sRtOpts, &g_sPTPClock);
}



int main(void)
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACArray[8];

    //
    // Make sure the main oscillator is enabled because this is required by
    // the PHY.  The system must have a 25MHz crystal attached to the OSC
    // pins.  The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
    // frequency is 10MHz or higher.
    //
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

    //
    // Run from the PLL at 40 MHz. (chosen so that g_tickNs is an integer value)
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 40000000);
    g_tickNs = 1000000000 / g_ui32SysClock;

    //
    // Configure the device pins.
    //
    PinoutSet(true, false);

    //
    // Configure debug port for internal use.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);

    //
    // Clear the terminal and print a banner.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("Ethernet with PTPd\n\n");

    //
    // Configure SysTick for a periodic interrupt.
    //
    ROM_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    g_ulSystemTickReload = ROM_SysTickPeriodGet();
    g_ulNewSystemTickReload = g_ulSystemTickReload;
    MAP_SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();

    //
    // Configure the hardware MAC address for Ethernet Controller filtering of
    // incoming packets.  The MAC address will be stored in the non-volatile
    // USER0 and USER1 registers.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);
    if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
    {
        //
        // Let the user know there is no MAC address
        //
        UARTprintf("No MAC programmed!\n");

        while(1)
        {
        }
    }

    //
    // Tell the user what we are doing just now.
    //
    UARTprintf("Waiting for IP.\n");

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split
    // MAC address needed to program the hardware registers, then program
    // the MAC address into the Ethernet Controller registers.
    //
    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

    //
    // Initialze the lwIP library, using DHCP.
    //
    lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);

    for (;;)
    {

    }

}

