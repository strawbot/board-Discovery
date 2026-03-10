#include "ttypes.h"
#include "project_defs.h"

#ifndef CLOCKS_H_
#define CLOCKS_H_

// ticks per second
#define TO_US(ticks) ((Octet)(ticks)*1000000/ONE_SECOND)
#define TO_MS(ticks) ((Octet)(ticks)*1000/ONE_SECOND)

// ticks proportional to milliseconds via ONE_SECOND
Long get_ticks();
void show_timer();
void init_clocks(void);
void set_delta_alarm(Long t);
void over_due();
void micro_sleep();

#endif
