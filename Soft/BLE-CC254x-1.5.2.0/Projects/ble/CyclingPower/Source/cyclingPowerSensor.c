/*********************************************************************
 * DESCRIPION
 *          This file contains the power meter application for use 
 * with the Bluetooth Low Energy Protocol Stack.
 * 
 *  Target Device: CC2540, CC2541
 *  Rus 02/2023 Ivan Dobsky
 */

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
//#include "hal_led.h" // FOR FUTURE USE
#include "linkdb.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "CyclingPowerSensor.h"
#include "CyclingPowerService.h"

/*********************************************************************
* TODO: 
* 1. Add capability to save some recent value to flash e.g. sensor location
* 2. Comment or delete unnecessary includes in files cyclingPowerSensor_Main.c
*    OSAL_cyclingPowerSensor.c
*/

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units.  625*32 = 20ms (recommended)
#define DEFAULT_FAST_ADV_INTERVAL                32
// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION                0 // endless, TEST
// Duration of advertising to white list members only after link termination
#define DEFAULT_WHITE_LIST_ADV_DURATION          0 // endless, TEST
// Slow advertising interval in 625us units.  625*1704 = 1065ms (recommended)
#define DEFAULT_SLOW_ADV_INTERVAL                0 // endless, TEST
// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION                20000
// How often to perform sensor's periodic event (ms)
#define DEFAULT_CP_PERIOD                        1000
// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST            FALSE
// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL        200
// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL        1600
// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY            0
// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT             1000
// Sensor sends a slave security request.
#define DEFAULT_PAIRING_PARAMETER                GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
// Bonded devices' addresses are stored in white list.
#define USING_WHITE_LIST                         FALSE
// Request bonding.
#define REQ_BONDING                              FALSE
// Time alloted for service discovery before requesting more energy efficient connection parameters
#define SVC_DISC_DELAY                           5000
// After 15 seconds of no user input with notifications off, terminate connection
#define NEGLECT_TIMEOUT_DELAY                    15000
// Setting this to true lets this device disconnect after a period of no use.
#define USING_NEGLECT_TIMEOUT                    FALSE
// delay for reset of device's bonds, connections, alerts
#define CSC_RESET_DELAY                          3000 // in ms, 3 seconds

// TODO: specify the length of measurement data (need to allocate memmory for 
// meas notify)this value needs to be variable!
#define CP_MEAS_LEN                               14 // TEST


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 sensorPower_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint8 sensorUsingWhiteList = FALSE;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] ={
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'C',
  'P',
  ' ',
  'S',
  'e',
  'n',
  's',
  'o',
  'r',
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm, default Tx power level
};

static uint8 advertData[] ={
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUIDs
  0x03,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(CYCPWR_SERV_UUID),
  HI_UINT16(CYCPWR_SERV_UUID),
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "CP Sensor";
// GAP connection handle
static uint16 gapConnHandle;

// Advertising user-cancelled state
static bool sensorAdvCancelled = FALSE;

// Flags for measurements
// TODO: add flags
static uint16 sensorFlags = CP_FLAG_WHEEL_REV_DATA_PRESENT 
| CP_FLAG_CRANK_REV_DATA_PRESENT | CP_FLAG_OFFSET_COMP_INDICATOR_TRUE; // TEST


// CP parameters
// TODO: add other used parametrs; add capabilty to restore cached values
uint32 cummWheelRevs = 0; // TEST
uint16 cummCrankRevs = 0; // TEST
#define REV_INCREMENT_1 250 // TEST
#define REV_INCREMENT_2 700 // TEST
uint16 lastWheelEvtTime = 0; // TEST
uint16 lastCrankEvtTime = 0; // TEST
int16 instantaneousPower = 150; // TEST
uint8 sensorLocationCurrent = CP_SENSOR_LOC_LEFT_CRANK;
uint16 crankLengthCurrent = 0;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
// TODO: add sensorPowerVectorNotify
static void sensorPower_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void SensorPowerGapStateCB( gaprole_States_t newState );
static void sensorPowerPeriodicTask( void );
static void sensorPowerMeasNotify( void );
static void SensorPowerCB( uint8 event, uint32 *pNewCummVal );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t cyclingPowerPeripheralCB ={
  SensorPowerGapStateCB,       // Profile State Change Callbacks
  NULL                    // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t cyclingPowerBondCB ={
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CyclingPowerSensor_Init
 *
 * @brief   Initialization function for the Cycling Power App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void CyclingPowerSensor_Init( uint8 task_id ){
  sensorPower_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // Device start advertising at once
    // TODO: add condition to start advertising; setup advertising see
    // sensor_HandleKeys() in cyclingSensor.c
    uint8 initial_advertising_enable = TRUE;

    // if not in a connection, toggle advertising on and off
    if ( gapProfileState != GAPROLE_CONNECTED ){
      uint8 status;

      // Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_WHITE_LIST_ADV_DURATION );

      // toggle GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      status = !status;

      // If not already using white list, begin to do so.
      // Only do so if about to begin advertising
      if ( USING_WHITE_LIST && status == TRUE ){
        uint8 bondCount = 0;

        GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCount );

        if ((sensorUsingWhiteList == FALSE) && (bondCount > 0) ){
          uint8 value = GAP_FILTER_POLICY_WHITE;

          GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &value);

          sensorUsingWhiteList = TRUE;
        }
      }

      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status );

      // Set state variable
      if (status == FALSE){
        sensorAdvCancelled = TRUE;
      }
    }

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 000000; // passkey "000000"
    uint8 pairMode = DEFAULT_PAIRING_PARAMETER;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8 bonding = REQ_BONDING;
    uint8 autoSync = USING_WHITE_LIST;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
    GAPBondMgr_SetParameter( GAPBOND_AUTO_SYNC_WL, sizeof ( uint8 ), &autoSync );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  CyclingPower_AddService( GATT_ALL_SERVICES );
  DevInfo_AddService( );

  // Register for cycling service callback
  CyclingPower_Register( SensorPowerCB );

  // Setup CP profile data
  // TODO: setup service data
  {
    uint32 features = CP_WHEEL_REV_SUPP | CP_MULTI_SENS_SUPP 
      | CP_CRANK_LEN_ADJ_SUPP | CP_NOT_FOR_USE_IN_DIST_SYS;
    uint8 sensorLocation1 = CP_SENSOR_LOC_REAR_DROPOUT; // TEST
    uint8 sensorLocation2 = CP_SENSOR_LOC_TOP_OF_SHOE; // TEST
    uint8 sensorLocation3 = CP_SENSOR_LOC_REAR_WHEEL; // TEST
    uint8 sensorLocation4 = CP_SENSOR_LOC_HIP; // TEST

    // Add available sensor locations
    CyclingPower_SetParameter(CP_AVAIL_SENS_LOCS, &sensorLocation1); // TEST
    CyclingPower_SetParameter(CP_AVAIL_SENS_LOCS, &sensorLocation2); // TEST
    CyclingPower_SetParameter(CP_AVAIL_SENS_LOCS, &sensorLocation3); // TEST
    CyclingPower_SetParameter(CP_AVAIL_SENS_LOCS, &sensorLocation4); // TEST

    // Set sensor location
    CyclingPower_SetParameter(CP_SENSOR_LOC_PARAM, &sensorLocationCurrent);

    //TODO: set crankLength

    // Set supported features
    CyclingPower_SetParameter(CP_FEATURE_PARAM, &features);
  }

  // TODO: turn on first led // TEST
  P1SEL &= ~ ((uint8)1<<0 | (uint8)1<<1); // P1_0, P1_1 as GPIO
  P1DIR |= ((uint8)1<<0 | (uint8)1<<1); // P1_0, P1_1 as output
  //P1 |= ((uint8)1<<0 | (uint8)1<<1); // P1_0, P1_1 HIGH
  P1 &= ~ ((uint8)1<<0 | (uint8)1<<1); // P1_0, P1_1 LOW

  // Setup a delayed profile startup
  osal_set_event( sensorPower_TaskID, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      CyclingPowerSensor_ProcessEvent
 *
 * @brief   Cycling Power Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 CyclingPowerSensor_ProcessEvent( uint8 task_id, uint16 events ){

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG ){
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sensorPower_TaskID )) != NULL ){
      sensorPower_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT ){
    // Start the Device
    VOID GAPRole_StartDevice( &cyclingPowerPeripheralCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &cyclingPowerBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & CP_PERIODIC_EVT ){
    // Perform sensor's periodic task
    sensorPowerPeriodicTask();

    return (events ^ CP_PERIODIC_EVT);
  }

  if ( events & CP_CONN_PARAM_UPDATE_EVT ){
    // Send param update.  If it fails, retry until successful.
    GAPRole_SendUpdateParam( DEFAULT_DESIRED_MIN_CONN_INTERVAL, DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                             DEFAULT_DESIRED_SLAVE_LATENCY, DEFAULT_DESIRED_CONN_TIMEOUT,
                             GAPROLE_RESEND_PARAM_UPDATE );

    // Assuming service discovery complete, start neglect timer
    if ( USING_NEGLECT_TIMEOUT ){
      osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
    }

    return (events ^ CP_CONN_PARAM_UPDATE_EVT);
  }

  if ( events & CP_NEGLECT_TIMEOUT_EVT ){
    // No user input, terminate connection.
    GAPRole_TerminateConnection();

    return ( events ^ CP_NEGLECT_TIMEOUT_EVT );
  }

  if ( events & CP_RESET_EVT ){
    // Soft reset in action
    if ( gapProfileState == GAPROLE_CONNECTED ){
      // Exit the connection
      GAPRole_TerminateConnection();

      // There is no callback for manual termination of the link.  change state variable here.
      gapProfileState = GAPROLE_WAITING;

      // Set timer to give the end advertising event time to finish
      osal_start_timerEx( sensorPower_TaskID, CP_RESET_EVT, 500 );
    }
    else if ( gapProfileState == GAPROLE_ADVERTISING ){
      uint8 value = FALSE;

      // Turn off advertising
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &value );

      // Set timer to give the end advertising event time to finish
      osal_start_timerEx( sensorPower_TaskID, CP_RESET_EVT, 500 );
    }
    else if ( USING_WHITE_LIST == TRUE ){
      //temporary variable
      uint8 value = GAP_FILTER_POLICY_ALL;

      // Turn off white list filter policy
      GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &value);

      sensorUsingWhiteList = FALSE;

      // Clear the white list
      HCI_LE_ClearWhiteListCmd();

      // Set timer to give the end advertising event time to finish
      osal_start_timerEx( sensorPower_TaskID, CP_RESET_EVT, 500 );
    }
    else if ( (gapProfileState == GAPROLE_STARTED) ||
              (gapProfileState == GAPROLE_WAITING) ||
              (gapProfileState == GAPROLE_WAITING_AFTER_TIMEOUT) ){
      uint8 eraseBonds = TRUE;

      // Erase all bonds
      GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, &eraseBonds );

      // TODO: turn on second led to indicate. // TEST
    }

    return (events ^ CP_RESET_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      sensorPower_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void sensorPower_ProcessOSALMsg( osal_event_hdr_t *pMsg ){
  // TODO: add some action..
  ; 
}

/*********************************************************************
 * @fn      sensorPowerMeasNotify
 *
 * @brief   Prepare and send a CP measurement notification
 *
 * @return  none
 */
static void sensorPowerMeasNotify( void ){
  
  static uint8 sw = 1, sw_ = 1; //TEST
  
  attHandleValueNoti_t sensorPowerMeas;
  
  sensorPowerMeas.pValue = GATT_bm_alloc( gapConnHandle, ATT_HANDLE_VALUE_NOTI,
                                     CP_MEAS_LEN, NULL );
  if ( sensorPowerMeas.pValue != NULL ){
    uint8 *p = sensorPowerMeas.pValue;

  /* TODO: 
  * 1. Take measurement data from periodic meas func;
  * 2. add some other flags handlers
  * 3. add data availability check!
  */

    // Build CP measurement structure from data
    
    //Flags, necessary
    *p++ = LO_UINT16(sensorFlags); // [0]
    *p++ = HI_UINT16(sensorFlags); // [1]
    
    // TODO: check signification
    //Instantaneous Power, necessary
    *p++ = LO_UINT16(instantaneousPower); // [2]
    *p++ = HI_UINT16(instantaneousPower); // [3]

    if (instantaneousPower == 150) instantaneousPower = 180; // TEST
    else instantaneousPower = 150; // TEST

    // If present, add wheel rev. data into measurement
    if (sensorFlags & CP_FLAG_WHEEL_REV_DATA_PRESENT){
    
      //TODO: add check for wheel rev. data to roll over

      *p++ = BREAK_UINT32(cummWheelRevs, 0);
      *p++ = BREAK_UINT32(cummWheelRevs, 1);
      *p++ = BREAK_UINT32(cummWheelRevs, 2);
      *p++ = BREAK_UINT32(cummWheelRevs, 3);

      *p++ = LO_UINT16(lastWheelEvtTime);
      *p++ = HI_UINT16(lastWheelEvtTime);
      
      cummWheelRevs++; // TEST
      
      // TEST
      if (sw){
        lastWheelEvtTime += REV_INCREMENT_1; 
        sw = 0;
      }
      else{
        lastWheelEvtTime += REV_INCREMENT_2; 
        sw = 1;
      }

    }

    // If present, add crank rev. data into measurement
    if (sensorFlags & CP_FLAG_CRANK_REV_DATA_PRESENT){
      *p++ = LO_UINT16(cummCrankRevs);
      *p++ = HI_UINT16(cummCrankRevs);

      *p++ = LO_UINT16(lastCrankEvtTime);
      *p++ = HI_UINT16(lastCrankEvtTime);

      cummCrankRevs++; // TEST
      
            // TEST
      if (sw_){
        lastCrankEvtTime += REV_INCREMENT_1; 
        sw_ = 0;
      }
      else{
        lastCrankEvtTime += REV_INCREMENT_2; 
        sw_ = 1;
      }
      
    }

    // Get length
    sensorPowerMeas.len = (uint8) (p - sensorPowerMeas.pValue);

    // Send to service to send the notification
    if ( CyclingPower_MeasNotify( gapConnHandle, &sensorPowerMeas ) != SUCCESS ){
      GATT_bm_free( (gattMsg_t *)&sensorPowerMeas, ATT_HANDLE_VALUE_NOTI );
    }
  }
}

/*********************************************************************
 * @fn      SensorPowerGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SensorPowerGapStateCB( gaprole_States_t newState ){
  // If connected
  if (newState == GAPROLE_CONNECTED){
    P1 |= (uint8)1<<0; // TEST
    // Get connection handle
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);

    // Set timer to update connection parameters
    // 5 seconds should allow enough time for Service Discovery by the collector to finish
    osal_start_timerEx( sensorPower_TaskID, CP_CONN_PARAM_UPDATE_EVT, SVC_DISC_DELAY);
  }
  // If disconnected
  else if (gapProfileState == GAPROLE_CONNECTED &&
           newState != GAPROLE_CONNECTED){
    P1 &= ~ (uint8)1<<0; // TEST
    uint8 advState = TRUE;
    uint8 bondCount = 0;

    // Stop periodic measurement
    osal_stop_timerEx( sensorPower_TaskID, CP_PERIODIC_EVT );

    // If not already using white list, begin to do so.
    GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCount );

    if( USING_WHITE_LIST && sensorUsingWhiteList == FALSE && bondCount > 0 ){
      uint8 value = GAP_FILTER_POLICY_WHITE;

      GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &value);

      sensorUsingWhiteList = TRUE;
    }

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT ){
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_WHITE_LIST_ADV_DURATION );
    }
    else{
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_WHITE_LIST_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
  }
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING &&
            newState == GAPROLE_WAITING ){
    uint8 whiteListUsed = FALSE;
    
    P1 &= ~ (uint8)1<<1; // TEST
    // if white list is in use, disable to allow general access
    if( sensorUsingWhiteList == TRUE ){
      uint8 value = GAP_FILTER_POLICY_ALL;

      GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, sizeof( uint8), &value);

      whiteListUsed = TRUE;

      sensorUsingWhiteList = FALSE;
    }

    // if advertising stopped by user
    if ( sensorAdvCancelled ){
      sensorAdvCancelled = FALSE;
    }
    // if fast advertising interrupted to cancel white list
    else if ( ( (!USING_WHITE_LIST) || whiteListUsed) &&
              (GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL ) ){
      uint8 advState = TRUE;

      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
    }
    // if fast advertising switch to slow or if was already slow but using white list.
    else if ( ((!USING_WHITE_LIST) || whiteListUsed) ||
             (GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL) ){
      uint8 advState = TRUE;

      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
    }
  }
  // if started
  else if (newState == GAPROLE_STARTED){
    P1 |= (uint8)1<<1; // TEST
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];

    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;

    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      SensorPowerCB
 *
 * @brief   Callback function for CP service.
 *
 * @param   event - service event
 * @param   pNewVal - pointer to new data if specified by event.  NULL otherwise.
 * @return  none
 */
static void SensorPowerCB( uint8 event, uint32 *pNewVal )
{
  static uint8 notificationsEnabled = FALSE;
  // TODO: add other suported callbacks
  switch ( event ){
    case CP_SET_CUMU_VAL:
      // Cancel neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
      }

      cummWheelRevs = *pNewVal;

      // Start neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    case CP_UPDATE_SENS_LOC:
      // Cancel neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
      }

      // Get updated sensor location
      CyclingPower_GetParameter( CP_SENSOR_LOC_PARAM, &sensorLocationCurrent );

      // Start neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    case CP_SET_CRANK_LENGTH:
      // Cancel neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
      }

      // Get new crank length value
      crankLengthCurrent = (uint16) *pNewVal;

      // Start neglect timer
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    case CP_MEAS_NOTI_ENABLED:
      // Cancel neglect timer
      if ( USING_NEGLECT_TIMEOUT ){
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
      }
      
      // If connected start periodic measurement
      if (gapProfileState == GAPROLE_CONNECTED){
        notificationsEnabled = TRUE;
        
        osal_start_timerEx( sensorPower_TaskID, CP_PERIODIC_EVT, DEFAULT_CP_PERIOD );
      }
      break;

    case CP_MEAS_NOTI_DISABLED:
      // Stop periodic measurement
      osal_stop_timerEx( sensorPower_TaskID, CP_PERIODIC_EVT );

      notificationsEnabled = FALSE;

      // Start neglect timer
      if ( USING_NEGLECT_TIMEOUT ){
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    case CP_READ_ATTR:
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        // Cancel neglect timer
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
        // TODO: add some action..
        // Start neglect timer
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    case CP_WRITE_ATTR:
      if ( USING_NEGLECT_TIMEOUT && !notificationsEnabled ){
        // Cancel neglect timer
        osal_stop_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT );
        // TODO: add some action..
        // Start neglect timer
        osal_start_timerEx( sensorPower_TaskID, CP_NEGLECT_TIMEOUT_EVT, NEGLECT_TIMEOUT_DELAY );
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      sensorPowerPeriodicTask
 *
 * @brief   Perform a periodic cycling power application task.
 *
 * @param   none
 *
 * @return  none
 */
static void sensorPowerPeriodicTask( void ){
  if (gapProfileState == GAPROLE_CONNECTED){
    // Send measurement notification 

    sensorPowerMeasNotify();

    // Restart timer
    osal_start_timerEx( sensorPower_TaskID, CP_PERIODIC_EVT, DEFAULT_CP_PERIOD );
  }
}

/*********************************************************************
*********************************************************************/
