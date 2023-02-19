/*********************************************************************
 * DESCRIPION
 *          This file contains the Cycling Power (CP) service for use 
 * with the power meter app.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus 02/2023 Ivan Dobsky
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

#define CYCLING_POWER_SERVICE               0x00000001 // TODO: check this!

// ATT error Codes
#define CPS_ERR_INAPPR_CONN_PARAM           0x80

// CP service Parameters
#define CP_FEATURE_PARAM                    1
#define CP_SENSOR_LOC_PARAM                 2
#define CP_CONTROL_PARAM                    3

// Sensor locations
#define CP_SENSOR_LOC_OTHER                 0
#define CP_SENSOR_LOC_TOP_OF_SHOE           1
#define CP_SENSOR_LOC_IN_SHOE               2
#define CP_SENSOR_LOC_HIP                   3
#define CP_SENSOR_LOC_FRONT_WHEEL           4
#define CP_SENSOR_LOC_LEFT_CRANK            5
#define CP_SENSOR_LOC_RIGHT_CRANK           6
#define CP_SENSOR_LOC_LEFT_PEDAL            7
#define CP_SENSOR_LOC_RIGHT_PEDAL           8
#define CP_SENSOR_LOC_FRONT_HUB             9
#define CP_SENSOR_LOC_REAR_DROPOUT          10
#define CP_SENSOR_LOC_CHAINSTAY             11
#define CP_SENSOR_LOC_REAR_WHEEL            12
#define CP_SENSOR_LOC_REAR_HUB              13
#define CP_SENSOR_LOC_CHEST                 14
#define CP_SENSOR_LOC_SPIDER                15
#define CP_SENSOR_LOC_CHAIN_RING            16

// GATT spec. says there are 17 possible.
#define CP_MAX_SENSOR_LOCS                  17

// CP supported Features
#define CP_NO_SUPPORT                       0x00
#define CP_PEDAL_POWER_BALANCE_SUPP         (uint32)1<<0
#define CP_ACCUM_TORQUE_SUPP                (uint32)1<<1
#define CP_WHEEL_REV_SUPP                   (uint32)1<<2
#define CP_CRANK_REV_SUPP                   (uint32)1<<3
#define CP_EXTREME_MAGNITUDES_SUPP          (uint32)1<<4
#define CP_EXTREME_ANGELS_SUPP              (uint32)1<<5
#define CP_TOP_BOTTOM_DEAD_SPOT_SUPP        (uint32)1<<6
#define CP_ACCUM_ENERGY_SUPP                (uint32)1<<7
#define CP_OFFSET_COMP_INDICATOR_SUPP       (uint32)1<<8
#define CP_OFFSET_COMP_SUPP                 (uint32)1<<9
#define CP_CYCLING_MEAS_CHAR_MASK_SUPP      (uint32)1<<10
#define CP_MULTI_SENS_SUPP                  (uint32)1<<11
#define CP_CRANK_LEN_ADJ_SUPP               (uint32)1<<12
#define CP_CHAIN_LEN_ADJ_SUPP               (uint32)1<<13
#define CP_CHAIN_WEIGHT_ADJ_SUPP            (uint32)1<<14
#define CP_SPAN_LEN_ADJ_SUPP                (uint32)1<<15
#define CP_SENSOR_MEAS_CONTEXT_SUPP         (uint32)1<<16
#define CP_INST_MEAS_DIR_SUPP               (uint32)1<<17
#define CP_FACTORY_CALIBR_DATE_SUPP         (uint32)1<<18
#define CP_ENHA_OFFSET_COMP_SUPP            (uint32)1<<19
#define CP_NOT_FOR_USE_IN_DIST_SYS          (uint32)1<<20
#define CP_CAN_BE_USED_IN_DIST_SYS          (uint32)1<<21
#define CP_FULL_SUPPORT                     0x002FFFFF

// CP control point stuff
#define CPS_CMD_LEN                         (3 + CP_MAX_SENSOR_LOCS) // TODO: check this!

// CP service task events
// TODO: add service task events
#define CPS_CMD_IND_SEND_EVT                0x0001
#define CP_MEAS_NOTI_ENABLED                0x0011
#define CP_MEAS_NOTI_DISABLED               0x0012
#define CP_READ_ATTR                        0x0013
#define CP_WRITE_ATTR                       0x0014


// CP control point Op codes 
#define CP_SET_CUMU_VAL                     0x01
#define CP_UPDATE_SENS_LOC                  0x02
#define CP_REQ_SUPP_SENS_LOC                0x03
#define CP_SET_CRANK_LENGTH                 0x04
#define CP_REQ_CRANK_LENGTH                 0x05
#define CP_SET_CHAIN_LENGTH                 0x06
#define CP_REQ_CHAIN_LENGTH                 0x07
#define CP_SET_CHAIN_WEIGHT                 0x08
#define CP_REQ_CHAIN_WEIGHT                 0x09
#define CP_SET_SPAN_LENGTH                  0x0A
#define CP_REQ_SPAN_LENGTH                  0x0B
#define CP_START_OFFSET_COMPENSATION        0x0C
#define CP_MASK_CYCLING_POWER_MEAS_CHAR     0x0D
#define CP_REQ_SAMPL_RATE                   0x0E
#define CP_REQ_FACTORY_CALIB_DATE           0x0F
#define CP_START_ENHA_OFFSET_CALIBR         0x10
#define CP_RESP_CODE                        0x20

// CP control point response code values
#define CP_RESP_SUCCESS                     0x01
#define CP_RESP_OP_CODE_NOT_SUPP            0x02
#define CP_RESP_INVALID_OPERAND             0x03
#define CP_RESP_OPERATION_FAILED            0x04

// CP measurement flags
#define CP_FLAG_AT_REST                     0x00
#define CP_FALG_PEDAL_PW_BALANCE_PRESENT    (uint32)1<<0
#define CP_FLAG_PEDAL_PW_BALANCE_REF_LEFT   (uint32)1<<1
#define CP_FLAG_ACCUM_TORQUE_PRESENT        (uint32)1<<2
#define CP_FLAG_ACCUM_TORQUE_SOURCE_CRANK   (uint32)1<<3
#define CP_FLAG_WHEEL_REV_DATA_PRESENT      (uint32)1<<4
#define CP_FLAG_CRANK_REV_DATA_PRESENT      (uint32)1<<5
#define CP_FLAG_EXTREME_FORCE_MAGNITUDES    (uint32)1<<6
#define CP_FLAG_EXTREME_TORQUE_MAGNITUDES   (uint32)1<<7
#define CP_FLAG_EXTREME_ANGLES              (uint32)1<<8
#define CP_FLAG_TOP_DEAD_SPOT_ANGLE         (uint32)1<<9
#define CP_FLAG_BOTTOM_DEAD_SPOT_ANGLE      (uint32)1<<10
#define CP_FLAG_ACCUM_ENERGY                (uint32)1<<11
#define CP_FLAG_OFFSET_COMP_INDICATOR_TRUE  (uint32)1<<12

#define INCLUDE_CP_VECTOR
#ifdef INCLUDE_CP_VECTOR

// CP vecotr
// TODO: add vector fields DEFINES

// CP vector flags
// TODO: add vector flags DEFINES

#endif /* INCLUDE_CP_VECTOR */

/*********************************************************************
 * TYPEDEFS
 */

// CP service callback function
typedef void (*cyclingPowerServiceCB_t)( uint8 event, uint32 *pNewCummVal );


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      CyclingPowerService_Init
 *
 * @brief   collect the OSAL task ID.
 *
 * @param   task_id - OSAL task ID.
 *
 * @return  none
 */
extern void CyclingPowerService_Init( uint8 task_id );

/*********************************************************************
 * @fn      CyclinPowerService_ProcessEvent
 *
 * @brief   process incoming event.
 *
 * @param   task_id - OSAL task id.
 *
 * @param   events - event bit(s) set for the task(s)
 *
 * @return  remaining event bits
 */
extern uint16 CyclingPowerService_ProcessEvent( uint8 task_id, uint16 events );

/*
 * @fn      CyclingPower_AddService
 *
 * @brief   Initializes the CP service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t CyclingPower_AddService( uint32 services );

/*
 * @fn      CyclingPower_Register
 *
 * @brief   Register a callback function with the
 *          CP Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  none
 */

extern void CyclingPower_Register( cyclingPowerServiceCB_t pfnServiceCB );

/*
 * @fn      CyclingPower_SetParameter
 *
 * @brief   Set a CP parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t CyclingPower_SetParameter( uint8 param, uint8 len, void *value );

/*********************************************************************
 * @fn      CyclingPower_GetParameter
 *
 * @brief   Get a CP parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CyclingPower_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          CyclingPower_MeasNotify
 *
 * @brief       Send a notification containing a CP
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t CyclingPower_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );

#ifdef INCLUDE_CP_VECTOR
/*********************************************************************
 * @fn          CyclingPower_VectorNotify
 *
 * @brief       Send a notification containing a CP
 *              vector.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
extern bStatus_t CyclingPower_VectorNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );
#endif /* INCLUDE_CP_VECTOR */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif /* __cplusplus */ 

#endif /* CYCLINGPOWERSERVICE_H */