#ifndef MAIN_H
#define MAIN_H

#include <ioCC254x_bitdef.h>
#include <ioCC2541.h>
#include <stdint.h>

/*User functions*/

void sys_setup(void);

/*MACRO DEFENITIONS*/

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define TOGGLE_BIT(REG, BIT)  ((REG) ^= (BIT))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define R_W0_REG(REG, BIT)    ((REG) = ~(BIT))

#endif /*MAIN_H*/
