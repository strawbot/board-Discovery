// Clocks

#include "tea.h"
#include "gpio.h"
#include "printers.h"
#include "cli.h"
#include "project_defs.h"
#include "tim.h"

void over_due() { /* incCtr(overDueTea); */ }

// must translate to local timer scale; UTC seconds to DELTA seconds; 1 if same clock
void set_delta_alarm(Long t) {
	LL_TIM_SetAutoReload(TIM2, t - 1);  // count to t ticks
	LL_TIM_SetCounter(TIM2, 0);  // reset counter
	LL_TIM_EnableCounter(TIM2);  // start — stops automatically at ARR in one-pulse mode
}	

void delta_alarm() { LL_TIM_ClearFlag_UPDATE(TIM2);  now(*alarmEvent); }

Long get_ticks() { return LL_TIM_GetCounter(TIM5); } // 100 us ticks

void show_timer() {
	print("UTC:");
	dotnb(8, 8, get_ticks(), 16);
	print("  ticks/S:");
	printDec(ONE_SECOND);
	print("  Timer:");
	dotnb(8, 8, TIM2->CNT, 16);
	print("  ticks/S:");
	printDec(TE_SECOND);
}

void micro_sleep() { __WFI(); }

#define set_pin(pin) LL_GPIO_SetOutputPin(pin##_GPIO_Port, pin##_Pin)
#define reset_pin(pin) LL_GPIO_ResetOutputPin(pin##_GPIO_Port, pin##_Pin)

#define gron() set_pin(LD4)
#define groff() reset_pin(LD4)
#define oron() set_pin(LD3)
#define oroff() reset_pin(LD3)
#define blon() set_pin(LD6)
#define bloff() reset_pin(LD6)
#define reon() set_pin(LD5)
#define reoff() reset_pin(LD5)

static void blink_leds() {
	static enum {GREEN,GREENORANGE,ORANGE,ORANGERED,RED,REDBLUE,BLUE,BLUEGREEN} color = GREEN;
	switch (color) {
	case GREEN:  		bloff();	color = GREENORANGE; 	break;
	case GREENORANGE:	oron();		color = ORANGE; 		break;
	case ORANGE:  		groff();	color = ORANGERED; 		break;
	case ORANGERED:		reon();		color = RED; 			break;
	case RED:     		oroff();	color = REDBLUE;		break;
	case REDBLUE:		blon();		color = BLUE; 			break;
	case BLUE:   		reoff();	color = BLUEGREEN; 		break;
	case BLUEGREEN:		gron();		color = GREEN; 			break;
	}
	after(msec(125), blink_leds);
}

// compare times over an interval: sysTicks();
void init_clocks() {
	// start a 32 bit counter based on processor frequency
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	// SysTick->CTRL  = 0;                   /* Disable the Systick Timer */
	// LL_SYSTICK_DisableIT();
	
	never(alarmEvent);
	LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM5);
	later(blink_leds);
	namedAction(blink_leds);
}
