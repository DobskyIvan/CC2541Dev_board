/*********************************************************************
 * DESCRIPION
 *          This file contains the Cycling Power (CP) service for use 
 * with the power meter app.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus/2023
 */

#ifndef CYCLINGPOWERSERVICE_H
#define CYCLINGPOWERSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */ 

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

#define CYCLING_POWER_SERVICE               0x00000001

//ATT error Codes
//TODO
#define CPS_ERR_INAPPR_CONN_PARAM           0x80

//CP service Parameters
//TODO

//CP supported Features
#define CP_NO_SUPPORT                       0x00
#define CP_PEDAL_POWER_BALANCE_SUPP         (uint32_t)1<<0
#define CP_ACCUM_TORQUE_SUPP                (uint32_t)1<<1
#define CP_WHEEL_REV_SUPP                   (uint32_t)1<<2
#define CP_CRANK_REV_SUPP                   (uint32_t)1<<3
#define CP_EXTREME_MAGNITUDES_SUPP          (uint32_t)1<<4
#define CP_EXTREME_ANGELS_SUPP              (uint32_t)1<<5
#define CP_TOP_BOTTOM_DEAD_SPOT_SUPP        (uint32_t)1<<6
#define CP_ACCUM_ENERGY_SUPP                (uint32_t)1<<7
#define CP_OFFSET_COMP_INDICATOR_SUPP       (uint32_t)1<<8
#define CP_OFFSET_COMP_SUPP                 (uint32_t)1<<9
#define CP_CYCLING_MEAS_CHAR_MASK_SUPP      (uint32_t)1<<10
#define CP_MULTI_SENS_SUPP                  (uint32_t)1<<11
#define CP_CRANK_LEN_ADJ_SUPP               (uint32_t)1<<12
#define CP_CHAIN_LEN_ADJ_SUPP               (uint32_t)1<<13
#define CP_CHAIN_WEIGHT_ADJ_SUPP            (uint32_t)1<<14
#define CP_SPAN_LEN_ADJ_SUPP                (uint32_t)1<<15
#define CP_SENSOR_MEAS_CONTEXT_SUPP         (uint32_t)1<<16
#define CP_INST_MEAS_DIR_SUPP               (uint32_t)1<<17
#define CP_FACTORY_CALIBR_DATE_SUPP         (uint32_t)1<<18
#define CP_ENHA_OFFSET_COMP_SUPP            (uint32_t)1<<19
#define CP_NOT_FOR_USE_IN_DIST_SYS          (uint32_t)1<<20
#define CP_CAN_BE_USED_IN_DIST_SYS          (uint32_t)1<<21
#define CP_FULL_SUPPORT                     0x002FFFFF

//GATT spec. says there are 16 possible.
#define CP_MAX_SENSOR_LOCS                 17

//CP sensor Locations
#define CP_SENSOR_LOC_OTHER                0
#define CP_SENSOR_LOC_TOP_OF_SHOE          1
#define CP_SENSOR_LOC_IN_SHOE              2
#define CP_SENSOR_LOC_HIP                  3
#define CP_SENSOR_LOC_FRONT_WHEEL          4
#define CP_SENSOR_LOC_LEFT_CRANK           5
#define CP_SENSOR_LOC_RIGHT_CRANK          6
#define CP_SENSOR_LOC_LEFT_PEDAL           7
#define CP_SENSOR_LOC_RIGHT_PEDAL          8
#define CP_SENSOR_LOC_FRONT_HUB            9
#define CP_SENSOR_LOC_REAR_DROPOUT         10
#define CP_SENSOR_LOC_CHAINSTAY            11
#define CP_SENSOR_LOC_REAR_WHEEL           12
#define CP_SENSOR_LOC_REAR_HUB             13
#define CP_SENSOR_LOC_CHEST                14
#define CP_SENSOR_LOC_SPIDER               15
#define CP_SENSOR_LOC_CHAIN_RING           16

//CP commands
#define CP_SET_CUMM_VAL            1
#define CP_START_SENS_CALIB        2
#define CP_UPDATE_SENS_LOC         3
#define CP_REQ_SUPP_SENS_LOC       4
#define CP_COMMAND_RSP             16

/*********************************************************************
 * TYPEDEFS
 */

// CP service callback function
typedef void (*cyclingPowerServiceCB_t)( uint8 event, uint32 *pNewCummVal );

#endif /* CYCLINGPOWERSERVICE_H *
