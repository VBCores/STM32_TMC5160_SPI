/*
 * 5160_Constants.h
 *
 *  Created on: Sep 7, 2023
 *      Author: VR
 */

#ifndef INC_5160_CONSTANTS_H_
#define INC_5160_CONSTANTS_H_

#define TMC5160_REGISTER_COUNT   128
#define TMC5160_MOTORS           1
#define TMC5160_WRITE_BIT        0x80
#define TMC5160_ADDRESS_MASK     0x7F
#define TMC5160_MAX_VELOCITY     8388096
#define TMC5160_MAX_ACCELERATION 65535

// ramp modes (Register TMC5161_RAMPMODE)
#define TMC5160_MODE_POSITION  0
#define TMC5160_MODE_VELPOS    1
#define TMC5160_MODE_VELNEG    2
#define TMC5160_MODE_HOLD      3

// limit switch mode bits (Register TMC5160_SWMODE)
#define TMC5160_SW_STOPL_ENABLE    0x0001
#define TMC5160_SW_STOPR_ENABLE    0x0002
#define TMC5160_SW_STOPL_POLARITY  0x0004
#define TMC5160_SW_STOPR_POLARITY  0x0008
#define TMC5160_SW_SWAP_LR         0x0010
#define TMC5160_SW_LATCH_L_ACT     0x0020
#define TMC5160_SW_LATCH_L_INACT   0x0040
#define TMC5160_SW_LATCH_R_ACT     0x0080
#define TMC5160_SW_LATCH_R_INACT   0x0100
#define TMC5160_SW_LATCH_ENC       0x0200
#define TMC5160_SW_SG_STOP         0x0400
#define TMC5160_SW_SOFTSTOP        0x0800

// Status bits (Register TMC5160_RAMPSTAT)
#define TMC5160_RS_STOPL          0x0001
#define TMC5160_RS_STOPR          0x0002
#define TMC5160_RS_LATCHL         0x0004
#define TMC5160_RS_LATCHR         0x0008
#define TMC5160_RS_EV_STOPL       0x0010
#define TMC5160_RS_EV_STOPR       0x0020
#define TMC5160_RS_EV_STOP_SG     0x0040
#define TMC5160_RS_EV_POSREACHED  0x0080
#define TMC5160_RS_VELREACHED     0x0100
#define TMC5160_RS_POSREACHED     0x0200
#define TMC5160_RS_VZERO          0x0400
#define TMC5160_RS_ZEROWAIT       0x0800
#define TMC5160_RS_SECONDMOVE     0x1000
#define TMC5160_RS_SG             0x2000

// Encoderbits (Register TMC5160_ENCMODE)
#define TMC5160_EM_DECIMAL     0x0400
#define TMC5160_EM_LATCH_XACT  0x0200
#define TMC5160_EM_CLR_XENC    0x0100
#define TMC5160_EM_NEG_EDGE    0x0080
#define TMC5160_EM_POS_EDGE    0x0040
#define TMC5160_EM_CLR_ONCE    0x0020
#define TMC5160_EM_CLR_CONT    0x0010
#define TMC5160_EM_IGNORE_AB   0x0008
#define TMC5160_EM_POL_N       0x0004
#define TMC5160_EM_POL_B       0x0002
#define TMC5160_EM_POL_A       0x0001

#endif /* 5160_CONSTANTS_H_ */
