// Board/arch/cc.h — shim that shadows Middlewares/.../system/arch/cc.h
//
// How it works:
//   lwip/arch.h includes "arch/cc.h".  GCC searches -I directories in order;
//   because Board/ precedes Middlewares/Third_Party/LwIP/system in the project
//   include path, this file is found first.
//
//   #include_next "arch/cc.h" then skips past Board/ and picks up the real
//   generated file from Middlewares/.../system/arch/cc.h, bringing in all of
//   its content (PACK_STRUCT macros, LWIP_RAND, etc.).
//
//   After that we #undef and redefine LWIP_PLATFORM_ASSERT to route assertion
//   failures to lwip_assert_handler() instead of printf() (which is a no-op
//   on this target without semihosting).
//
// CubeMX-safe:
//   CubeMX regenerates Middlewares/.../system/arch/cc.h freely — this shim
//   never touches that file.  The include-path ordering (Board before
//   Middlewares/Third_Party/LwIP/system) is stored in .cproject under
//   user-managed settings and is preserved across CubeMX regeneration.
//
// #pragma GCC system_header:
//   Marks this file and the generated cc.h it pulls in as system headers,
//   suppressing the macro-redefinition warning that the generated cc.h would
//   otherwise emit (it defines LWIP_PLATFORM_ASSERT unconditionally, without
//   a #ifndef guard).

#ifndef BOARD_ARCH_CC_H
#define BOARD_ARCH_CC_H

#pragma GCC system_header          // suppress diagnostics from this file and
                                   // from the generated cc.h included below
#include_next "arch/cc.h"          // pull in Middlewares/.../system/arch/cc.h

// Replace the printf-based assert with the TimbreOS serial handler.
// #undef first because the generated cc.h defines it unconditionally.
#undef  LWIP_PLATFORM_ASSERT
#define LWIP_PLATFORM_ASSERT(x)                                 \
    do {                                                        \
        extern void lwip_assert_handler(const char *msg);       \
        lwip_assert_handler(x);                                 \
    } while(0)

#endif // BOARD_ARCH_CC_H
