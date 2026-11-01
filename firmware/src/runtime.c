/* Flash memcpy uses MCPY; SRAM callers use ordinary memory accesses. */

#include <stddef.h>
#include "ch572/attributes.h"
#include "runtime.h"

#include "ch572/ch572.h"

#define __RUNTIME_RAM __HIGH_CODE

void *__RUNTIME_RAM RUNTIME_CopyRam(void *destination, const void *source, size_t n) {
    unsigned char       *d         = destination;
    const unsigned char *s         = source;
    uintptr_t            alignment = (uintptr_t)d | (uintptr_t)s;

    if (!(alignment & 3U)) {
        runtime_word_t       *words = (runtime_word_t *)d;
        const runtime_word_t *input = (const runtime_word_t *)s;
        const runtime_word_t *end   = input + n / sizeof(*input);

        n &= 3U;
        while (input != end) {
            *words++ = *input++;
        }
        d = (unsigned char *)words;
        s = (const unsigned char *)input;
    }
    if (!(alignment & 1U)) {
        runtime_half_t       *halves = (runtime_half_t *)d;
        const runtime_half_t *input  = (const runtime_half_t *)s;
        const runtime_half_t *end    = input + n / sizeof(*input);

        n &= 1U;
        while (input != end) {
            *halves++ = *input++;
        }
        d = (unsigned char *)halves;
        s = (const unsigned char *)input;
    }
    while (n--) {
        *d++ = *s++;
    }
    return destination;
}

void *memcpy(void *destination, const void *source, size_t n) {
    const unsigned char *s = source;

    if (n) {
        __MCPY(destination, (void *)s, (void *)(s + n));
    }
    return destination;
}

void *memset(void *destination, int value, size_t n) {
    unsigned char *d = destination;

    while (n--) {
        *d++ = (unsigned char)value;
    }
    return destination;
}

int memcmp(const void *first, const void *second, size_t n) {
    const unsigned char *a = first, *b = second;

    while (n--) {
        if (*a != *b) {
            return (int)*a - (int)*b;
        }
        ++a;
        ++b;
    }
    return 0;
}

size_t strlen(const char *s) {
    const char *p = s;
    while (*p) {
        ++p;
    }
    return p - s;
}
