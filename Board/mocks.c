#include "ttypes.h"
#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "timeout.h"
#include "printers.h"

void system_failure(Long reason) { while (true); }

static Long dropped_chars = 0;

void show_cli() { print("\nDropped chars: "), printDec(dropped_chars); }

void output() {
    Timeout box;
    setTimeout(secs(1), &box);
    while(fullbq(emitq))
        if (checkTimeout(&box))
            pullbq(emitq);
        else
            action_slice();
}
