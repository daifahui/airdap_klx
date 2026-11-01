#ifndef CH572_ATTRIBUTES_H
#define CH572_ATTRIBUTES_H

#define __ALWAYS_INLINE     inline __attribute__((always_inline))
#define __NO_INLINE         __attribute__((noinline))
#define __NO_RETURN         __attribute__((noreturn))
#define __WEAK              __attribute__((weak))
#define __UNUSED            __attribute__((unused))
#define __MAY_ALIAS         __attribute__((may_alias))
#define __PACKED            __attribute__((packed))
#define __ALIGNED(bytes)    __attribute__((aligned(bytes)))
#define __SECTION(name)     __attribute__((section(name)))
#define __ALIAS(name)       __attribute__((alias(name)))
#define __OPTIMIZE(options) __attribute__((optimize(options)))
#define __INTERRUPT         __attribute__((interrupt("WCH-Interrupt-fast")))

#define __HIGH_CODE               __SECTION(".highcode")
#define __HIGH_CODE_SECTION(name) __SECTION(".highcode." name)
#define __COLD_FLASH              __SECTION(".cold_flash")
#endif
