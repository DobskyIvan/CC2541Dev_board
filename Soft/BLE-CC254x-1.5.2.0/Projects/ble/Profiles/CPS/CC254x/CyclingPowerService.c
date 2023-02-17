/*********************************************************************
 * DESCRIPION
 *          This file contains the Cycling Power (CP) service for use 
 * with the power meter app.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus/2023
 */



/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "CyclingPowerService.h"

/*********************************************************************
 * TYPEDEFS
 */
// CP service Task Events
// TODO
#define CPS_CMD_IND_SEND_EVT   0x0001

/*********************************************************************
 * GLOBAL VARIABLES
 */

// CP service
CONST uint8 cyclingPowerServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_SERV_UUID), HI_UINT16(CYCPWR_SERV_UUID)
};

// CP feature characteristic
CONST uint8 cyclingPowerFeatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_FEATURE_UUID), HI_UINT16(CYCPWR_FEATURE_UUID)
};

// CP measurement characteristic
CONST uint8 cyclingPowerMeasUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_MEAS_UUID), HI_UINT16(CYCPWR_MEAS_UUID)
};

// sensor location characteristic
CONST uint8 sensLocationUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_LOC_UUID), HI_UINT16(SENSOR_LOC_UUID)
};

// CP control characteristic
CONST uint8 cyclingPowerControlPointUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_CTRL_PT_UUID), HI_UINT16(CYCPWR_CTRL_PT_UUID)
};

// CP vector characteristic
CONST uint8 cyclingPowerVectorUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_VECTOR_UUID), HI_UINT16(CYCPWR_VECTOR_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Preset cumulative value 
static cyclingPowerServiceCB_t cyclingPowerServiceCB = NULL; 

static uint8 supportedSensors = 0; 
static bool scOpInProgress = FALSE; // Control point busy flag

// Variables used in CP command processing
// TODO


/*********************************************************************
 * Profile Attributes - variables
 */

// TaskID
uint8 cyclingPowerService_TaskID = 0;

// CSC Service attribute
static CONST gattAttrType_t cyclingPowerService = { ATT_BT_UUID_SIZE, cyclingPowerServUUID };
