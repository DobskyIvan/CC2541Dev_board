/*********************************************************************
 * DESCRIPION
 *          This file contains power meter application definitions and
 * prototypes.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus 02/2023 Ivan Dobsky
 */

#ifndef CYCLINGPOWERSENSOR_H
#define CYCLINGPOWERSENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Cycling Sensor Task Events
#define START_DEVICE_EVT                                (uint16)1<<0
#define CP_PERIODIC_EVT                                 (uint16)1<<1
#define CP_CONN_PARAM_UPDATE_EVT                        (uint16)1<<2
#define CP_NEGLECT_TIMEOUT_EVT                          (uint16)1<<3
#define CP_RESET_EVT                                    (uint16)1<<4

/*********************************************************************
 * MACROS
 */
  

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void CyclingPowerSensor_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 CyclingPowerSensor_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CYCLINGPOWERSENSOR_H */
