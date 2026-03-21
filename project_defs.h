
#ifndef PROJECT_DEFS_H_
#define PROJECT_DEFS_H_

#include "ttypes.h"
#include "stm32f407xx.h"
#include "cmsis_gcc.h"
#include "core_cm4.h"

// bigger buffer for accepting long hexscii sequences
#define CLI_PARAMETERS

#define CLI_TITLE "ActiveRobot"

#define DCELLS 20  // number of data stack cells
#define RCELLS 20  // number of return stack cells
#define LINE_LENGTH 400 // number of characters allowed in tib
#define EMITQ_SIZE 400
#define KEYQ_SIZE 400
#define PAD_SIZE 20
#define PROMPTSTRING "ar: "
#define CUSHION LINE_LENGTH // how much space to maintain for HERE
#define HERE_SPACE 5000 // small here space
#define OUTPUT_BLOCKED output() // deal with by running machines
#define OUTPUT_FLUSH output()
#define FLOAT_SUPPORT true
#define NAN (__builtin_nanf(""))

void output_flush();
void output();

// interrupt support
// max for PPS timing; hi for time events; lo for uarts; min for dma rings
#define INT_MAX_PRIO    0
#define INT_HI_PRIO     1
#define INT_LO_PRIO     2
#define INT_MIN_PRIO    3

// UTC and time event clocks
#define CLOCK_MHZ 168u
#define ONE_SECOND (10000)	// for UTC 100us res
#define TE_SECOND ONE_SECOND    // for Delta timer

// Hi res time measurements
// 168 MHz clock ticks; wraps after 25 seconds; 5.95 ns resolution
#define sysTicks()  (Long)(DWT->CYCCNT)

#define SYS_TO_NS(n) ((unsigned long long)(n)*1000/CLOCK_MHZ)
#define SYS_TO_US(n) ((n)/CLOCK_MHZ)
#define SYS_TO_MS(n) ((unsigned long long)(n)/(CLOCK_MHZ*1000))
#define US_TO_SYS(n)	 ((n)*CLOCK_MHZ)

#define IN_INTERRUPT() 	(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk)

#define ENTER_REGION()                                          \
	{                                                           \
		uint32_t primask_bit = __get_PRIMASK();					\
		__disable_irq();

#define LEAVE_REGION()                                          \
		  __set_PRIMASK(primask_bit);                           \
	}

#define ENTER_SAFE_REGION() ENTER_REGION()
#define LEAVE_SAFE_REGION() LEAVE_REGION()

// Record event parameters
#define NUM_ACTIONS 80
#define NUM_TE 80

#define N_EVENTS 400
#define FIRST_EVENT (const char *)secs(5)

// define space for action stats
#define TEA_TABLE HASH10

// black hole reasons - addendum
#define DMA_OVERBOOKED 7

// user flow control
void user_monitor();
bool user_stop();
void user_return();

#define DEFAULT_SLEEP_MODE 1

/*!
 * \brief Convert CPU cycles into micro-seconds.
 *
 * \param  cy:      Number of CPU cycles.
 * \param  fcpu_hz: CPU frequency in Hz.
 *
 * \return the converted number of micro-second.
 */
#define cpu_cy_2_us(cy, fcpu_hz) (((Octet)(cy) * 1000000 + (fcpu_hz)-1) / (fcpu_hz))

#endif
