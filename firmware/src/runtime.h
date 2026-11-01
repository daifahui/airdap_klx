#ifndef RUNTIME_H
#define RUNTIME_H

#include <stddef.h>
#include "ch572/attributes.h"
#include <stdint.h>

/* Explicit C copy for code executing in SRAM. Standard memcpy stays in Flash. */

void *RUNTIME_CopyRam(void *destination, const void *source, size_t n);

/* Ordinary-memory copies may alias byte buffers. Callers must establish the
 * matching address alignment before using either access type. */
typedef uint32_t runtime_word_t __MAY_ALIAS;
typedef uint16_t runtime_half_t __MAY_ALIAS;
#endif
