#include <stdlib.h>
#include <string.h>
#include "printers.h"
#include "cli.h"
#include <ctype.h>

bool visible_word(char *s);

static char *filter;

const char *strcasestr_r(const char *haystack, const char *needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0) return haystack;
    for (; *haystack; haystack++) {
        if (strncasecmp(haystack, needle, nlen) == 0) return haystack;
    }
    return NULL;
}

static void printif(char *s) {
	if (strcasestr_r(s, filter) != NULL && visible_word(s))
		print(s);
}

static void helphelp() {
	print("Print commands with one line help with wild card filtering: help <filter>");
    print("\nCommands with Arguments");
    print("\n Some arguments precede the command while others have arguments after.");
    print("\n Generally, numbers come before while strings come after. A number as");
    print("\n a string will come after such as the commands setting port addresses");
    print("\n or ip addresses.");
    print("\n\n  Use:");
    print("\n   <s> for strings following a command");
    print("\n   (n) for parameters preceding a command");
    print("\n       if there are results they are preceded by -");
    print("\n       so ( a - n ) uses a and returns n");
    print("\n  For example:");
    print("\n    	rm   <pattern> remove files whose name matches pattern");
    print("\n      seek   ( n )  seek to position n in the file; -1 to end");
    print("\n        s!   ( h a - ) store next into memory using top as address (16 bit)");
    print("\n    negate   ( n - -n ) two's complement of top data stack item");
    print("\n\n  Command examples:");
    print("\n   rm somefilename");
    print("\n   124 seek");
    print("\n   1001 portaddress s!");
    print("\n   100 negate");
}

void help(void) {
	cursorReturn();
	parse(0);
	here();
	filter = (char *)ret()+1;
	if (strcmp("help", filter) == 0) {
		helphelp();
		return;
	}
    printif("!   ( n a - ) store next into memory using top as address (processor sized)\n");
    printif("#   ( n - n' ) convert a digit from n\n");
    printif("#>   ( n - a c ) finish number sequence and return address and count\n");
    printif("#s   ( n - 0 ) convert all digits in n\n");
    printif("(  [i]  start of comment till end of line or )\n");
    printif("*   ( n m - p ) multiply next data stack item by top and leave on top\n");
    printif("+   ( n m - p ) add top two data stack items and leave on top\n");
    printif("+b   ( b a - ) turn on b bits at address a: 0b10001 em +b\n");
    printif(",   ( n -s ) allocate 1 cell and put n into it\n");
    printif("-   ( n m - p ) subtract top data stack item from next item and leave on top\n");
    printif("-b   ( b a - ) turn off b bits at address a: 0b10001 em -b\n");
    printif(".   ( n - ) print n in current number base\n");
    printif(".b   ( n - ) print number in binary\n");
    printif(".d   ( n - ) print number in decimal\n");
    printif(".f   ( f - )  print f as a floating point number\n");
    printif(".h   ( n - ) print number in hex\n");
    printif(".r   ( m n - ) print m in right field of n digits\n");
    printif(".s   print number of items on data stack and items\n");
    printif("/   ( n m - q ) divide next data stack item by top and leave on top\n");
    printif("/mod   ( n m - q r ) return divide and modulus from top item into next item\n");
    printif("0latency   zero the latency stats; sampled every 10s\n");
    printif("0stats   initialize stats to zero\n");
    printif(":   <string> start a macro definition named string\n");
    printif(";  [i]  end a macro build\n");
    printif("<   ( n m - f ) leave a boolean on stack indicating if next is less than top\n");
    printif("<#   initiate a number sequence\n");
    printif("=   ( n m - f ) leave a boolean on stack after equating top two data stack items\n");
    printif(">   ( n m - f ) leave a boolean on stack indicating if next is greater than top\n");
    printif(">r   ( n - ) (R - n ) push the top item of the data stack onto the return stack\n");
    printif("?dup   ( n - n n | - 0 ) duplicate top data stack item if not 0\n");
    printif("@   ( a - n ) return contents of memory using top stack item as the address (processor sized)\n");
    printif("[  [i]  exit macro build\n");
    printif("]   enter macro build\n");
    printif("abs   ( n - |n|) top data stack item is made positive\n");
    printif("accel_init   init accelerometer interface\n");
    printif("accoff   stop accelerometer\n");
    printif("accon   start accerloeromete\n");
    printif("accread   ( a n ) read n accelerometer registers starting at a\n");
    printif("accwrite   ( n a ) write n to accelerometer register a\n");
    printif("acos   ( f -  f  ) calculate the inverse cos of f\n");
    printif("again  [i]  end of a continuous loop construct\n");
    printif("allot   ( n - ) reserve n bytes after end of dictionary\n");
    printif("and   ( n m - p ) bitwise AND top two data stack items and leave on top\n");
    printif("asin   ( f -  f  ) calculate the inverse sine of f\n");
    printif("atan   ( f -  f  ) calculate the inverse tan of f\n");
    printif("atan2   ( f m - m  ) calculate the inverse tan of f/m\n");
    printif("begin  [i]  start of a loop construct\n");
    printif("bin   switch to binary numbers\n");
    printif("c!   ( c a - ) store next into memory using top as address (8 bit)\n");
    printif("c,   ( c - ) allocate and 1 byte and put value in it\n");
    printif("c@   ( a - c ) return contents of memory using top stack item as the address (8 bit)\n");
    printif("cal-wire   run two-point self-calibration: R_max at 0% then R_min at 70%\n");
    printif("cbrt   ( f - f  ) calculate the cube root of f\n");
    printif("cmove   ( s d n - ) move n bytes from s to d\n");
    printif("constant   ( n - ) <string> give n a name\n");
    printif("cos   ( f -  f  ) calculate the cosine of f\n");
    printif("cr   send end of line to output device\n");
    printif("dacoff   stop sine wave on PA4\n");
    printif("dacon   start sine wave on PA4\n");
    printif("dacset   ( f ) set sine wave to frequency f in hz from 1 to 1,000,000\n");
    printif("decimal   interpret all subsequent numbers as decimal\n");
    printif("decimals\n");
    printif("drop   ( n - ) throw away the top data stack item\n");
    printif("dump   ( a n - ) dump n 16-byte rows of memory starting at address a\n");
    printif("dump-profile   emit captured profile as CSV to CLI output\n");
    printif("dup   ( n - n n ) make a copy of the top data stack item\n");
    printif("echooff   turn off key echo\n");
    printif("echoon   turn on key echo\n");
    printif("else  [i]  otherwise part of an if statement\n");
    printif("emit   ( c - ) send c to output device\n");
    printif("end   print time from start\n");
    printif("endif  [i]  end of else or if statement\n");
    printif("erase   ( s n - ) erase n bytes from s\n");
    printif("execute   ( a - ) use the top data stack item as a function call\n");
    printif("exit  [i]  exit macro\n");
    printif("exp   ( f - f  ) calculate the value of e to the power of f\n");
    printif("f*   ( f m - m  ) multiply two floats\n");
    printif("f+   ( f m - f  ) add two floats\n");
    printif("f-   ( f m - f  ) subtract float m from f\n");
    printif("f/   ( f m - m  ) divide float f by float m\n");
    printif("f>   ( f m - flag  ) flag is true if f is greater\n");
    printif("f>i   ( f - f  ) convert 32 bit float to integer\n");
    printif("fabs   ( f - f  ) get absolute value of float\n");
    printif("fill   ( s n x - )fill n bytes from s with x\n");
    printif("fneg   ( f - f  ) reverse the sign of the float\n");
    printif("for  [i]  ( n - ) start of a loop which runs n times\n");
    printif("gnetif   ( - a ) LwIP netif structure for Ethernet\n");
    printif("gtt   print tick and time\n");
    printif("help   <filtering> print words with one line help; allow wild card <filtering>; parenthesis show ( args - results ) and precede the command; angle brackets show arguments that follow commands\n");
    printif("here   ( - a ) return address of end of dictionary\n");
    printif("hex   interpret all following numbers as hex\n");
    printif("hold   ( c - ) hold a character in number sequence\n");
    printif("i>f   ( i - f ) convert integer to 32 bit float\n");
    printif("if  [i]  ( n - ) execute following code if top of stack is non-zero\n");
    printif("key   ( - c ) return character c from key queue or 0\n");
    printif("key?   ( - f ) return true if there is a key in the keyq\n");
    printif("l!   (n a - )store next into memory using top as address (processor sized)\n");
    printif("l@   ( a - n )return contents of memory using top stack item as the address (32 bit)\n");
    printif("latency   show latency stats for time events and actions\n");
    printif("ln   ( f - f  ) calculate the natural log of f\n");
    printif("log   ( f - f  ) calculate the log 10 of f\n");
    printif("max   ( n m - n|m) leave maximum of top two stack items\n");
    printif("min    ( n m - n|m) leave minimum of top two stack items\n");
    printif("mod   ( n m - r ) modulus next data stack item by top and leave on top\n");
    printif("mstats   list stats for machines\n");
    printif("nap   (n) take a nap for n milliseconds\n");
    printif("negate   ( n - -n ) two's complement of top data stack item\n");
    printif("next  [i]  end of a for loop\n");
    printif("not   ( n - n' ) invert all bits on the top data stack item\n");
    printif("oct   switch to octal numbers\n");
    printif("or    ( n m - p ) bitwise OR top two data stack items and leave on top\n");
    printif("over   ( n m - n m n ) copy 2nd data stack item to top of data stack\n");
    printif("pa   print actions in queue\n");
    printif("pi   ( - f ) return 32 bit value for pi\n");
    printif("pins   show all pins and states\n");
    printif("play   play out events in event queue and restart recording\n");
    printif("pow   ( f m - f  ) calculate the power of f to m\n");
    printif("r   ( - n ) (R n - n ) copy the top item of the return stack onto the data stack\n");
    printif("r>    ( - n ) (R n - ) move top item on return stack to data stack\n");
    printif("reboot   reboot the device via NVIC system reset\n");
    printif("record   start recording events\n");
    printif("repeat  [i]  go back to the begin part\n");
    printif("resetcli   reset cli including removing all macros\n");
    printif("s!   ( h a - ) store next into memory using top as address (16 bit)\n");
    printif("s@   ( a - h ) return contents of memory using top stack item as the address (16 bit)\n");
    printif("setpwm   ( n ) set muscle wire PWM duty to n percent (0-70)\n");
    printif("shift   ( n m - p ) shift n by m bit left for minus and right for positive\n");
    printif("show-ADC   show all adc inputs\n");
    printif("show-acc   show accelerometer type, orientation (pitch/roll) and tap count\n");
    printif("show-cal   show calibration state, R_max, R_min and measured travel span\n");
    printif("show-cli   display cli status\n");
    printif("show-dac   show DAC sine wave output: frequency, samples, timer registers\n");
    printif("show-dma   show DMA controller usage: active streams, channels, state, error flags\n");
    printif("show-eth   show Ethernet link status, speed, duplex and PHY info\n");
    printif("show-http   show HTTP server state and active connections\n");
    printif("show-ip   show IP address, gateway, netmask and DHCP state\n");
    printif("show-net   show LwIP network statistics: RX/TX counts, errors\n");
    printif("show-pwm   show PWM duty, supply/node voltages, wire resistance and contraction\n");
    printif("show-stats   show packet stats\n");
    printif("show-sys   show system info: clock frequencies and uptime\n");
    printif("show-telnet   show Telnet server state and active connections\n");
    printif("show-time   show delta timer state and UTC tick counter\n");
    printif("show-timers   dump TIM1-TIM14 and RTC: clock gate, CEN, direction, PSC, ARR, CNT, active CC channels\n");
    printif("show-usb   show USB connection state and CDC line status\n");
    printif("sign   ( m n - n ) prepend sign to number sequence if m is negative\n");
    printif("sin   ( f -  f  ) calculate the sine of f\n");
    printif("sp!   ( ... - ) empty the data stack\n");
    printif("sqrt   ( f - f  ) calculate the square root of f\n");
    printif("start   create a reference point\n");
    printif("start-profile   ( n ) apply n% PWM and begin 25.6 s resistance profile capture\n");
    printif("stop   stop recording events\n");
    printif("strlen   ( a - c ) return length of a string\n");
    printif("swap   ( n m - m n ) swap top two items on the data stack\n");
    printif("tabto   (n) move ahead to line position n\n");
    printif("tan   ( f -  f  ) calculate the tangent of f\n");
    printif("te   print counter, compare value and list of time event actions with due dates\n");
    printif("teakeycosts   show key hashes and clustering of teatimes table\n");
    printif("telist   print TE todo and done list of time events\n");
    printif("testtime   ( s ) test ticks, timeouts and time for s seconds\n");
    printif("tickms   ( tick - ms ) convert tick to milliseconds\n");
    printif("tn   dump out names in action name array\n");
    printif("type   ( a n - ) output n characters starting at a\n");
    printif("until  [i]  ( n - ) go back to the begin statement if stack is zero\n");
    printif("variable   ( n - ) <string> give n a place to be stored at a name\n");
    printif("while  [i]  ( n - ) conditional choice in a loop construct\n");
    printif("words   list all words in dictionary\n");
    printif("xor   ( n m - p ) bitwise XOR top two data stack items and leave on top\n");
    printif("xyz   display current accelerometer values\n");
}
