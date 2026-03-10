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
	LL_TIM_SetAutoReload(TIM2, t);
	LL_TIM_EnableCounter(TIM2);
}	

void delta_alarm() { LL_TIM_ClearFlag_UPDATE(TIM2);  (*alarmEvent)(); }

void show_timer() {
	print("UTC:");
	dotnb(8, 8, get_ticks(), 16);
	print("  ticks/S:");
	printDec(ONE_SECOND);
	print("  Timer:");
	dotnb(4, 4, TIM2->CNT, 16);
	print("  ticks/S:");
	printDec(TE_SECOND);
}

void micro_sleep() { __WFI(); }

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
}
