/*
 * rf_regs.h - CH572 2.4 GHz RF register access
 *
 * Bases and access widths are checked against the original linked RFIP_RegInit
 * and ordered machine-code accesses. Offsets in C are byte offsets.
 */

#ifndef RF_REGS_H
#define RF_REGS_H

#include <stdint.h>

/* Register block bases, verified against hardware accesses. */

/* RFIP_RegInit stores confirm LLE=0xc200, RFEND=0xd000 and BB=0xc100. */
#define RF_BB_BASE    0x4000C100UL /* gptrBBReg    */
#define RF_RFEND_BASE 0x4000D000UL /* gptrRFENDReg */
#define RF_LLE_BASE   0x4000C200UL /* gptrLLEReg   */

/* Register access helpers. */
#define RF_BB(off)    (*(volatile uint32_t *)(RF_BB_BASE + (off)))
#define RF_RFEND(off) (*(volatile uint32_t *)(RF_RFEND_BASE + (off)))
#define RF_LLE(off)   (*(volatile uint32_t *)(RF_LLE_BASE + (off)))

/* Narrow accesses for 16-bit registers, including length fields. */
#define RF_BB16(off)    (*(volatile uint16_t *)(RF_BB_BASE + (off)))
#define RF_RFEND16(off) (*(volatile uint16_t *)(RF_RFEND_BASE + (off)))
#define RF_LLE16(off)   (*(volatile uint16_t *)(RF_LLE_BASE + (off)))
#define RF_BB8(off)     (*(volatile uint8_t *)(RF_BB_BASE + (off)))
#define RF_RFEND8(off)  (*(volatile uint8_t *)(RF_RFEND_BASE + (off)))
#define RF_LLE8(off)    (*(volatile uint8_t *)(RF_LLE_BASE + (off)))
#endif /* RF_REGS_H */
