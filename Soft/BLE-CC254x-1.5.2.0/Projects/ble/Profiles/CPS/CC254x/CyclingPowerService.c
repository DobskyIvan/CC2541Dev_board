/*********************************************************************
 * DESCRIPION
 *          This file contains the Cycling Power (CP) service for use 
 * with the power meter app.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus 02/2023 Ivan Dobsky
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

// Sensor location characteristic
CONST uint8 sensLocationUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_LOC_UUID), HI_UINT16(SENSOR_LOC_UUID)
};

// CP control point characteristic
CONST uint8 cyclingPowerControlPointUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_CTRL_PT_UUID), HI_UINT16(CYCPWR_CTRL_PT_UUID)
};

#ifdef INCLUDE_CP_VECTOR
// CP vector characteristic
CONST uint8 cyclingPowerVectorUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(CYCPWR_VECTOR_UUID), HI_UINT16(CYCPWR_VECTOR_UUID)
};
#endif /* INCLUDE_CP_VECTOR */

/*********************************************************************
 * LOCAL VARIABLES
 */
 
static cyclingPowerServiceCB_t cyclingPowerServiceCB = NULL; 

static bool scOpInProgress = FALSE; // Control point busy flag

// Variables used in CP command processing
static uint16 connectionHandle;
static attHandleValueInd_t cpsCmdInd;

// Available sensor locations
static uint8 supportedSensors = 0; 
static uint8 supportedSensorLocations[CP_MAX_SENSOR_LOCS];

//TODO: add other var e.g. crankLength

static uint16 crankLength = 0;


/*********************************************************************
 * Profile Attributes - variables 
 */

// TaskID
uint8 cyclingPowerService_TaskID = 0;

// CP Service attribute
static CONST gattAttrType_t cyclingPowerService = { ATT_BT_UUID_SIZE, cyclingPowerServUUID };

// TODO: check types of variables e.g. uint8...

// CP feature Characteristic
static uint8 cyclingPowerFeatureProps = GATT_PROP_READ;
static uint32 cyclingPowerFeatures = CP_NO_SUPPORT;
static uint8 cyclingPowerFeatureUserDesc[]="CPS feature support\0";

// CP measurement Characteristic
static uint8 cyclingPowerMeasProps = GATT_PROP_NOTIFY;
static uint8 cyclingPowerMeas = 0;
// TODO: check that cyclingPowerMeas may be struct, see spec
static gattCharCfg_t *cyclingPowerMeasClientCharCfg;
static uint8 cyclingPowerMeasUserDesc[]="CPS measurement variable\0";

// Sensor location characteristic
static uint8 cyclingPowerSensLocProps = GATT_PROP_READ;
static uint8 cyclingPowerSensLoc = CP_SENSOR_LOC_OTHER;
static uint8 cyclingPowerSensLocUserDesc[]="CPS sensor location\0";

// CP control point characteristic
static uint8 cyclingPowerControlPointProps = GATT_PROP_WRITE | GATT_PROP_INDICATE;
static uint8 cyclingPowerControlPoint = 0;
static gattCharCfg_t *cyclingPowerControlPointClientCharCfg;
static uint8 cyclingPowerControlPointUserDesc[]="CPS control point\0";

#ifdef INCLUDE_CP_VECTOR
// CP vector characteristic
static uint8 cyclingPowerVectorProps = GATT_PROP_NOTIFY;
static uint8 cyclingPowerVector = 0;
static gattCharCfg_t *cyclingPowerVectorClientCharCfg;
static uint8 cyclingPowerVectorUserDesc[]="CPS vector variable\0";
#endif /* INCLUDE_CP_VECTOR */

/*********************************************************************
 * Profile Attributes - Table
 */

//Еable layout, it is important that the elements correspond to the indexes:
#define CP_MEAS_VALUE_POS                     5
#define CP_MEAS_CFG_POS                       6
#define CP_COMMAND_CFG_POS                    13
#ifdef INCLUDE_CP_VECTOR
#define CP_VECTOR_VALUE_POS                   16
#define CP_VECTOR_CFG_POS                     17
#endif /* INCLUDE_CP_VECTOR */

static gattAttribute_t cyclingPowerAttrTbl[] = // TODO: check types of variables e.g. uint8...
{
  // CP service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
/*0*/ (uint8 *) &cyclingPowerService            /* pValue */
  },

    // CP feature declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
/*1*/ &cyclingPowerFeatureProps
    },

      // feature Value
      {
        { ATT_BT_UUID_SIZE, cyclingPowerFeatureUUID },
        GATT_PERMIT_READ,
        0,
/*2*/   (uint8 *) &cyclingPowerFeatures
      },

      // CP feature client user description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
/*3*/   cyclingPowerFeatureUserDesc
      },

    // CP measurement declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
/*4*/ &cyclingPowerMeasProps
    },

      // CP measurement value
      {
        { ATT_BT_UUID_SIZE, cyclingPowerMeasUUID },
        0,
        0,
/*5*/   &cyclingPowerMeas
      },

      // CP measurement client characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
       0,
/*6*/  (uint8 *) &cyclingPowerMeasClientCharCfg
      },

      // CP measurement client user description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
/*7*/   cyclingPowerMeasUserDesc
      },

    // CP sensor location declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
/*8*/ &cyclingPowerSensLocProps
    },

      // Sensor location Value
      {
        { ATT_BT_UUID_SIZE, sensLocationUUID },
        GATT_PERMIT_READ,
        0,
/*9*/   &cyclingPowerSensLoc
      },

      // CP sensor location client user description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
/*10*/  cyclingPowerSensLocUserDesc
      },

    // CP control point declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
/*11*/&cyclingPowerControlPointProps
    },

      // CP control point value
      {
        { ATT_BT_UUID_SIZE, cyclingPowerControlPointUUID },
        GATT_PERMIT_WRITE,
        0,
/*12*/  &cyclingPowerControlPoint
      },

      // CP control point client characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
/*13*/  (uint8 *) &cyclingPowerControlPointClientCharCfg
      },

      // CP control point client user description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
/*14*/  cyclingPowerControlPointUserDesc
      }

      #ifdef INCLUDE_CP_VECTOR
      ,
      // CP vector declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
/*15*/&cyclingPowerVectorProps
    },

    // CP vector value
      {
        { ATT_BT_UUID_SIZE, cyclingPowerVectorUUID },
        GATT_PERMIT_READ,
        0,
/*16*/  &cyclingPowerVector
      },

      // CP vector client characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
/*17*/  (uint8 *) &cyclingPowerControlPointClientCharCfg
      },

      // CP vector client user description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
/*18*/  cyclingPowerVectorUserDesc
      }

      #endif /* INCLUDE_CP_VECTOR */
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t cyclingPower_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint8 *pLen, uint16 offset,
                                     uint8 maxLen, uint8 method );
static bStatus_t cyclingPower_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint8 len, uint16 offset,
                                      uint8 method );
static void cyclingPower_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void cyclingPower_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void cyclingPower_ProcessCPSCmd( uint16 attrHandle, uint8 *pValue, uint8 len );

static bool cyclingPower_SensorLocSupported( uint8 sensorLoc );
// TODO: add some other functions 
//1. Add function that will power off notification when lose connection e.g.: 
//static void cyclingPower_HandleConnStatusCB( uint16 connHandle, uint8 changeType );
 

/*********************************************************************
 * PROFILE CALLBACKS
 */

// CP Service Callbacks
CONST gattServiceCBs_t cyclingPowerCBs ={
  cyclingPower_ReadAttrCB,  // Read callback function pointer
  cyclingPower_WriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      cyclingPower_SensorLocSupported
 *
 * @brief   check to see if sensor location is supported
 *
 * @param   sensorLoc - location to check for
 *
 * @return  TRUE if supported, FALSE otherwise
 */
static bool cyclingPower_SensorLocSupported( uint8 sensorLoc ){
  uint8 i;
  for (i = 0; i <= supportedSensors; i++){
    if (supportedSensorLocations[i] == sensorLoc){
      return TRUE;
    }
  }
  return FALSE;
}

/*********************************************************************
 * @fn      CyclingPowerService_Init
 *
 * @brief   collect the OSAL task ID.
 *
 * @param   task_id - OSAL task ID.
 *
 * @return  none
 */
void CyclingPowerService_Init( uint8 task_id ){
  // Only purpose is to obtain task ID
  cyclingPowerService_TaskID = task_id;
}

/*********************************************************************
 * @fn      CyclingPowerService_ProcessEvent
 *
 * @brief   process incoming event.
 *
 * @param   task_id - OSAL task id.
 *
 * @param   events - event bit(s) set for the task(s)
 *
 * @return  none
 */
uint16 CyclingPowerService_ProcessEvent( uint8 task_id, uint16 events ){
  VOID task_id;

  if ( events & SYS_EVENT_MSG ){
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( cyclingPowerService_TaskID )) != NULL ){
      cyclingPower_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & CPS_CMD_IND_SEND_EVT ){
    if ( GATT_Indication( connectionHandle, &cpsCmdInd, FALSE, 
                          cyclingPowerService_TaskID ) != SUCCESS ){

      GATT_bm_free( (gattMsg_t *)&cpsCmdInd, ATT_HANDLE_VALUE_IND );
    }
  
    // Clear out this indication.
    VOID osal_memset( &cpsCmdInd, 0, sizeof(attHandleValueInd_t) );

    return ( events ^ CPS_CMD_IND_SEND_EVT );
  }

  return 0;
}

/*********************************************************************
 * @fn      cyclingPower_ProcessOSALMsg
 *
 * @brief   process incoming OSAL msg.
 *
 * @param   pMsg- pointer to messag to be read.
 *
 * @return  none
 */
void cyclingPower_ProcessOSALMsg( osal_event_hdr_t *pMsg ){
  switch ( pMsg->event ){
    case GATT_MSG_EVENT:
      cyclingPower_ProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;

    default: // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      cyclingPower_ProcessGATTMsg
 *
 * @brief   process incoming GATT msg.
 *
 * @param   pMsg- pointer to messag to be read.
 *
 * @return  none
 */
void cyclingPower_ProcessGATTMsg( gattMsgEvent_t *pMsg ){
  if ( pMsg->method == ATT_HANDLE_VALUE_CFM ){
    // Indication receipt was confirmed by the client.
    // Set Control Point Cfg done
    scOpInProgress = FALSE;
  }
}

/*********************************************************************
 * @fn      CyclingPower_AddService
 *
 * @brief   Initializes the CP service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t CyclingPower_AddService( uint32 services ){
  uint8 status;

  // Allocate client characteristic configuration table
  cyclingPowerMeasClientCharCfg = (gattCharCfg_t *)osal_mem_alloc( sizeof ( gattCharCfg_t ) * 
                                                              linkDBNumConns );
  if ( cyclingPowerMeasClientCharCfg == NULL ){
    return ( bleMemAllocError );
  }
  
  // Allocate client characteristic configuration table
  cyclingPowerControlPointClientCharCfg = (gattCharCfg_t *)osal_mem_alloc( sizeof ( gattCharCfg_t ) * 
                                                                 linkDBNumConns );
  if ( cyclingPowerControlPointClientCharCfg == NULL ){
    // Free already allocated data
    osal_mem_free( cyclingPowerControlPointClientCharCfg );
    
    return ( bleMemAllocError );
  }
  #ifdef INCLUDE_CP_VECTOR
  // Allocate client characteristic configuration table
  cyclingPowerVectorClientCharCfg = (gattCharCfg_t *)osal_mem_alloc( sizeof ( gattCharCfg_t ) * 
                                                                  linkDBNumConns );
  if ( cyclingPowerVectorClientCharCfg == NULL ){
    // Free already allocated data
    osal_mem_free( cyclingPowerVectorClientCharCfg );
    
    return ( bleMemAllocError );
  }
  #endif /* INCLUDE_CP_VECTOR */
  // Initialize client characteristic configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cyclingPowerMeasClientCharCfg );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cyclingPowerControlPointClientCharCfg);
  #ifdef INCLUDE_CP_VECTOR
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cyclingPowerVectorClientCharCfg);
  #endif /* INCLUDE_CP_VECTOR */

  if ( services & CYCLING_POWER_SERVICE ){
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( cyclingPowerAttrTbl,
                                          GATT_NUM_ATTRS( cyclingPowerAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &cyclingPowerCBs );
  }
  else{
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      CyclingPower_Register
 *
 * @brief   Register a callback function with the CP Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void CyclingPower_Register( cyclingPowerServiceCB_t pfnServiceCB ){
  cyclingPowerServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      CyclingPower_SetParameter
 *
 * @brief   Set a CP parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CyclingPower_SetParameter( uint8 param, void *pValue ){
  bStatus_t ret = SUCCESS;

  switch ( param ){
    case CP_FEATURE_PARAM:
      cyclingPowerFeatures = *((uint32*)pValue);
    break;

    case CP_SENSOR_LOC_PARAM:
      cyclingPowerSensLoc = *((uint8*)pValue);
    break;

    case CP_AVAIL_SENS_LOCS:
      if (supportedSensors  < CP_MAX_SENSOR_LOCS){
        supportedSensorLocations[supportedSensors++] = *((uint8*)pValue);
      }
    break;

    default:
      ret = INVALIDPARAMETER;
    break;
  }

  return ( ret );
}

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
bStatus_t CyclingPower_GetParameter( uint8 param, void *value ){
  bStatus_t ret = SUCCESS;

  switch ( param ){
    case CP_FEATURE_PARAM:
      *((uint32*)value) = cyclingPowerFeatures;

    case CP_SENSOR_LOC_PARAM:
      *((uint8*)value) = cyclingPowerSensLoc;
      break;

    case CP_CONTROL_PARAM: // TODO: check if it is necessary
      *((uint8*)value) = cyclingPowerControlPoint;
      break;

    //TODO: add other parametrs cases

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

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
bStatus_t CyclingPower_MeasNotify( uint16 connHandle, attHandleValueNoti_t *pNoti ){

  uint16 value = GATTServApp_ReadCharCfg( connHandle, cyclingPowerMeasClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY ){
    // Set the handle
    pNoti->handle = cyclingPowerAttrTbl[CP_MEAS_VALUE_POS].handle;

    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}

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
bStatus_t CyclingPower_VectorNotify( uint16 connHandle, attHandleValueNoti_t *pNoti ){
  uint16 value = GATTServApp_ReadCharCfg( connHandle, cyclingPowerVectorClientCharCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY ){
    // Set the handle
    pNoti->handle = cyclingPowerAttrTbl[CP_VECTOR_VALUE_POS].handle;

    // Send the notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}
#endif /* INCLUDE_CP_VECTOR */

/*********************************************************************
 * @fn      cyclingPower_ProcessCPSCmd
 *
 * @brief   process an incoming CP command.
 *
 * @param   attrHandle - attribute handle
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 *
 * @return  none
 */
static void cyclingPower_ProcessCPSCmd( uint16 attrHandle, uint8 *pValue, uint8 len ){
  uint8 cpsStatus = CP_RESP_SUCCESS;

  // See if need to alloc payload for new indication.
  if (cpsCmdInd.pValue == NULL){
    cpsCmdInd.pValue = GATT_bm_alloc(connectionHandle, ATT_HANDLE_VALUE_IND, 
                                     CPS_CMD_LEN, NULL);
    if (cpsCmdInd.pValue == NULL){
      return; // failed to allocate space!
    }
  }
  
  // Set Control Point Cfg in progress
  scOpInProgress = TRUE;

  // Set indication info to be sent out
  cpsCmdInd.handle = attrHandle;

  cpsCmdInd.len = 3; // TODO: check this!
  cpsCmdInd.pValue[0] = CP_RESP_CODE;
  cpsCmdInd.pValue[1] = pValue[0];

  switch ( pValue[0] ){
    // TODO: add some extra conditions from specification (e.g. the bicycle is not moving)
    case CP_SET_CUMU_VAL:
      // If wheel revolutions is a feature
      if ( ( len <= 5 ) && ( cyclingPowerFeatures & CP_WHEEL_REV_SUPP ) ){
        uint32 cumuWheelRevolutions;

        // full 32 bits were specified.
        if (( len - 1 ) == 4){
          cumuWheelRevolutions = BUILD_UINT32( pValue[1], pValue[2], pValue[3], pValue[4]);
        }
        else{
          cumuWheelRevolutions = 0;
          // In case only lower bits were specified and upper bits remain zero.
          for( int i = 0; i < (len - 1); ++i ){
            cumuWheelRevolutions += pValue[i + 1] << (i*8);
          }
        }
        // Notify app
        if ( cyclingPowerServiceCB != NULL ){
          (*cyclingPowerServiceCB)( CP_SET_CUMU_VAL, &cumuWheelRevolutions );
        }
      }
      else{ // characteristic not supported.
        cpsStatus = CP_RESP_INVALID_OPERAND;
      }
      break;

      case CP_UPDATE_SENS_LOC:
      // If multiple sensor locations is supported and that this is a valid location.
      if ( ( len == 2 )                              &&
           ( cyclingPowerFeatures & CP_MULTI_SENS_SUPP ) &&
           ( cyclingPower_SensorLocSupported( pValue[1] ) == TRUE ) ){

        // Update sensor location
        cyclingPowerSensLoc = pValue[1];
        
        // Notify app
        if ( cyclingPowerServiceCB != NULL ){
          (*cyclingPowerServiceCB)( CP_UPDATE_SENS_LOC, NULL );
        }
      }
      else{ // characteristic not supported.
        cpsStatus = CP_RESP_INVALID_OPERAND;
      }
      break;

      case CP_REQ_SUPP_SENS_LOC:
      // If multiple sensor locations are supported and list requested
      if ( ( len == 1 ) && ( cyclingPowerFeatures & CP_MULTI_SENS_SUPP ) ){
        cpsCmdInd.len += supportedSensors;
        osal_memcpy( &(cpsCmdInd.pValue[3]), supportedSensorLocations, supportedSensors );
      }
      else{ // characteristic not supported.
        // Send an indication with the list.
        cpsStatus = CP_RESP_INVALID_OPERAND;
      }
      break;

      case CP_SET_CRANK_LENGTH:
      // If crank length adjustment is a feature
      if ( ( len <= 3 ) && ( cyclingPowerFeatures & CP_CRANK_LEN_ADJ_SUPP ) ){

        // full 16 bits were specified.
        if (( len - 1 ) == 2){
          crankLength = BUILD_UINT16( pValue[1], pValue[2]);
        }
        else{
          crankLength = 0;
          // In case only lower bit were specified and upper bit remain zero.
          for( int i = 0; i < (len - 1); ++i ){
            crankLength += pValue[i + 1] << (i*8);
          }
        }
        // Notify app
        if ( cyclingPowerServiceCB != NULL ){
          (*cyclingPowerServiceCB)( CP_SET_CRANK_LENGTH, (uint32*) &crankLength );
        }
      }
      else{ // characteristic not supported.
        cpsStatus = CP_RESP_INVALID_OPERAND;
      }
      break;

      case CP_REQ_CRANK_LENGTH:
      // If crank length adjustment is a feature
      if ( ( len == 1 ) && ( cyclingPowerFeatures & CP_CRANK_LEN_ADJ_SUPP ) ){
        cpsCmdInd.len += 1; // add Hi-8 bit part 
        cpsCmdInd.pValue[3] = LO_UINT16(crankLength);
        cpsCmdInd.pValue[4] = HI_UINT16(crankLength); 
      }
      else{ // characteristic not supported.
        // Send an indication with the list.
        cpsStatus = CP_RESP_INVALID_OPERAND;
      }
      break;

      // TODO: add other suported op codes

      default:
      // Send an indication with opcode not suported response
      cpsStatus = CP_RESP_OP_CODE_NOT_SUPP;
      break;
  }

  // Send indication of operation result
  cpsCmdInd.pValue[2] = cpsStatus;

  // Ask our task to send out indication
  osal_set_event( cyclingPowerService_TaskID, CPS_CMD_IND_SEND_EVT );
}

/*********************************************************************
 * @fn          cyclingPower_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t cyclingPower_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint8 *pLen, uint16 offset,
                                     uint8 maxLen, uint8 method ){

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 ){
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch ( uuid ){
    case SENSOR_LOC_UUID:{
      // Read sensor location
      *pLen = 1;
      pValue[0] = pAttr->pValue[0];
    }
    break;

    case CYCPWR_FEATURE_UUID:{
      //Read cycling power feature
      *pLen = 4;
      
      pValue[0] = pAttr->pValue[0];
      pValue[1] = pAttr->pValue[1];
      pValue[2] = pAttr->pValue[2];
      pValue[3] = pAttr->pValue[3];
    }
    break;

    /* No need for "GATT_SERVICE_UUID", "GATT_CLIENT_CHAR_CFG_UUID" or 
    "GATT_CHAR_USER_DESC_UUID" cases;
    gattserverapp handles this type for reads */

    // TODO: add other cases

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
    break;
  }

  // Notify app
  if ( cyclingPowerServiceCB != NULL ){
    (*cyclingPowerServiceCB)( CP_READ_ATTR, NULL );
  }

  return ( status );
}

/*********************************************************************
 * @fn      cyclingPower_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t cyclingPower_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                      uint8 *pValue, uint8 len, uint16 offset,
                                      uint8 method ){

  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if ( offset > 0 ){
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  switch ( uuid ){
    case CYCPWR_CTRL_PT_UUID:
      // Make sure Control Point Cfg is not already in progress
      if ( scOpInProgress == TRUE ){
        status = CP_RESP_OPERATION_FAILED;
      }
      // Make sure Control Point Cfg is configured for Indications
      else if ( (cyclingPowerControlPointClientCharCfg[connHandle].value 
                                    & GATT_CLIENT_CFG_INDICATE) == FALSE ){
        status = CP_RESP_OPERATION_FAILED;
      }
      else{
        // Process CP control point cmd
        cyclingPower_ProcessCPSCmd( pAttr->handle, pValue, len );
        connectionHandle = connHandle;
      }
      break;

      // For measure, vector and commands CCC
    case GATT_CLIENT_CHAR_CFG_UUID:
      if ( pAttr->handle == cyclingPowerAttrTbl[CP_COMMAND_CFG_POS].handle ){
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        // Notify app
        if ( cyclingPowerServiceCB != NULL ){
          (*cyclingPowerServiceCB)( CP_WRITE_ATTR, NULL );
        }
      }
      else if ( pAttr->handle == cyclingPowerAttrTbl[CP_MEAS_CFG_POS].handle ){
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS ){
          // Notify app
          if ( cyclingPowerServiceCB != NULL ){
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] ); // TODO: check this

            (*cyclingPowerServiceCB)( ((charCfg == GATT_CFG_NO_OPERATION) ?
                                   CP_MEAS_NOTI_DISABLED :
                                   CP_MEAS_NOTI_ENABLED ), NULL );
          }
        }
      }
      #ifdef INCLUDE_CP_VECTOR
      else if ( pAttr->handle == cyclingPowerAttrTbl[CP_VECTOR_CFG_POS].handle ){
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS ){
          // Notify app
          if ( cyclingPowerServiceCB != NULL ){
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] ); // TODO: check this

            (*cyclingPowerServiceCB)( ((charCfg == GATT_CFG_NO_OPERATION) ?
                                   CP_VECTOR_NOTI_DISABLED :
                                   CP_VECTOR_NOTI_ENABLED ), NULL );
          }
        }
      }
      #endif /*INCLUDE_CP_VECTOR*/
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  return ( status );
}

/*********************************************************************
*********************************************************************/