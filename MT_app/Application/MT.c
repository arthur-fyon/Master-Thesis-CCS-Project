/******************************************************************************

   @file  MT.c

   @brief This file contains the Arthur Fyon Master Thesis application for use
        with the CC2640 Bluetooth Low Energy Protocol Stack.

   Group: CMCU, LPRF
   Target Device: cc2640
   

 ******************************************************************************
   
 Copyright (c) 2015-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************

 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>
#include <ti/drivers/I2C.h>

#include <uartlog/UartLog.h>  // Comment out if using xdc Log

#include <ti/display/AnsiColor.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include <icall.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Bluetooth Profiles */
#include <devinfoservice.h>
#include <services/battery_service.h>
#include <services/temp_service.h>
#include <services/ecg_service.h>
#include <services/ppg_service.h>

/* Application specific includes */
#include <MT_Board.h>

#include <MT.h>
#include <util.h>

/* 32-kHz Crystal-Less Mode */
#include "rcosc_calibration.h"

/*********************************************************************
 * MACROS
 */

// Spin if the expression is not true
#define APP_ASSERT(expr) if(!(expr)) {project_zero_spin();}

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0

/*********************************************************************
 * CONSTANTS
 */
// Task configuration
#define DEFAULT_CLOCK_TIMEOUT                10
// Both not equals to avoid triggering the interrupt at the same time
#define BQ_CLOCK_TIMEOUT                     30521 // Nearly 30 seconds
#define TMP_CLOCK_TIMEOUT                    29886 // Nearly 30 seconds

#define PZ_TASK_PRIORITY                     1

#ifndef PZ_TASK_STACK_SIZE
#define PZ_TASK_STACK_SIZE                   2048
#endif

// Internal Events for RTOS application
#define PZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PZ_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define PZ_ALL_EVENTS                        (PZ_ICALL_EVT | \
                                              PZ_APP_MSG_EVT)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define PZ_SERVICE_WRITE_EVT     0  /* A characteristic value has been written     */
#define PZ_SERVICE_CFG_EVT       1  /* A characteristic configuration has changed  */
#define PZ_UPDATE_CHARVAL_EVT    2  /* Request from ourselves to update a value    */
#define PZ_PAIRSTATE_EVT         3  /* The pairing state is updated                */
#define PZ_PASSCODE_EVT          4  /* A pass-code/PIN is requested during pairing */
#define PZ_ADV_EVT               5  /* A subscribed advertisement activity         */
#define PZ_START_ADV_EVT         6  /* Request advertisement start from task ctx   */
#define PZ_SEND_PARAM_UPD_EVT    7  /* Request parameter update req be sent        */
#define PZ_CONN_EVT              8  /* Connection Event End notice                 */
#define PZ_PERIODIC_TIMER        9  /* Periodic timer timeout                      */
#define BQ_PERIODIC_TIMER        10 /* Periodic timer timeout for BQ25125          */
#define BQSOC_TIMER              20 /* SoC conversion over for BQ25125             */
#define START_ADPD               30 /* Start ADPD4101                              */
#define TMP_PERIODIC_TIMER       40 /* Periodic timer timeout for TMP1075          */
#define GET_TMP_PERIODIC_TIMER   50 /* Timeout for getting TMP1075 measurement     */
#define INT_X_TRIGGERED          60 /* Interrupt X is triggered                    */
#define INT_Y_TRIGGERED          70 /* Interrupt Y is triggered                    */

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     12

// Maximum connection interval (units of 1.25ms, 800=1000ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     36

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// Delay (in ms) after connection establishment before sending a parameter update requst
#define PZ_SEND_PARAM_UPDATE_DELAY            6000

// I2C addresses
#define TMP1075_ADDR        0x49
#define ADPD4101_ADDR       0x24
#define BQ25125_ADDR        0x6A

// BQ Registers
#define BQ25125_STATUS_REG  0x00 //Status and Ship Mode Control Register
#define BQ25125_TS_INT_REG  0x02 //TS Control and Faults Masks Register
#define BQ25125_FAST_REG    0x03 //Fast Charge Control Register
#define BQ25125_TERM_REG    0x04 //Termination/Pre-Charge Register
#define BQ25125_SYSVOUT_REG 0x06 //SYS VOUT Control Register
#define BQ25125_LDO_REG     0x07 //Load Switch and LDO Control Register
#define BQ25125_ILIM_REG    0x09 //ILIM and Battery UVLO Control Register
#define BQ25125_VB_REG      0x0A //Voltage Based Battery Monitor Register

// TMP Registers
#define TMP1075_RES_REG     0x00 //Result Register
#define TMP1075_OS_REG      0x01 //OS Register

// ADPD Registers
#define ADPD4101_FIFO_REG   0x0000 //FIFO_STATUS Register
#define ADPD4101_FIFOTH_REG 0x0006 //FIFO_TH Register
#define ADPD4101_LFOSC_REG  0x000F //SYS_CTL Register
#define ADPD4101_TS_REG     0x0010 //OPMODE Register
#define ADPD4101_INTX_REG   0x0014 //INT_ENABLE_XD Register
#define ADPD4101_INTY_REG   0x0015 //INT_ENABLE_YD Register
#define ADPD4101_CFG_REG    0x0021 //INPUT_CFG Register
#define ADPD4101_GPIO_REG   0x0022 //GPIO_CFG Register
#define ADPD4101_GPIO01_REG 0x0023 //GPIO01 Register

#define ADPD4101_SIG_AL_REG 0x0030 //SIGNAL1_L_A Register
#define ADPD4101_SIG_AH_REG 0x0031 //SIGNAL1_H_A Register
#define ADPD4101_SIG_BL_REG 0x0038 //SIGNAL1_L_B Register
#define ADPD4101_SIG_BH_REG 0x0039 //SIGNAL1_H_B Register
#define ADPD4101_SIG_CL_REG 0x0040 //SIGNAL1_L_C Register
#define ADPD4101_SIG_CH_REG 0x0041 //SIGNAL1_H_C Register
#define ADPD4101_SIG_DL_REG 0x004A //SIGNAL1_L_D Register
#define ADPD4101_SIG_DH_REG 0x004B //SIGNAL1_H_D Register
#define ADPD4101_SIG_EL_REG 0x0050 //SIGNAL1_L_E Register
#define ADPD4101_SIG_EH_REG 0x0051 //SIGNAL1_H_E Register

#define ADPD4101_INA_REG    0x0102 //INPUTS_A Register
#define ADPD4101_CATHA_REG  0x0103 //CATHODE_A Register
#define ADPD4101_LED12A_REG 0x0105 //LED_POW12_A Register
#define ADPD4101_LED34A_REG 0x0106 //LED_POW34_A Register
#define ADPD4101_PATA_REG   0x010D //PATTERN_A Register
#define ADPD4101_SIGA_REG   0x0110 //DATA_FORMAT_A Register
#define ADPD4101_LITA_REG   0x0111 //LIT_DATA_FORMAT_A Register

#define ADPD4101_INB_REG    0x0122 //INPUTS_B Register
#define ADPD4101_CATHB_REG  0x0123 //CATHODE_B Register
#define ADPD4101_LED12B_REG 0x0125 //LED_POW12_B Register
#define ADPD4101_LED34B_REG 0x0126 //LED_POW34_B Register
#define ADPD4101_PATB_REG   0x012D //PATTERN_B Register
#define ADPD4101_SIGB_REG   0x0130 //DATA_FORMAT_B Register
#define ADPD4101_LITB_REG   0x0131 //LIT_DATA_FORMAT_B Register

#define ADPD4101_INC_REG    0x0142 //INPUTS_C Register
#define ADPD4101_CATHC_REG  0x0143 //CATHODE_C Register
#define ADPD4101_LED12C_REG 0x0145 //LED_POW12_C Register
#define ADPD4101_LED34C_REG 0x0146 //LED_POW34_C Register
#define ADPD4101_PATC_REG   0x014D //PATTERN_C Register
#define ADPD4101_SIGC_REG   0x0150 //DATA_FORMAT_C Register
#define ADPD4101_LITC_REG   0x0151 //LIT_DATA_FORMAT_C Register

#define ADPD4101_PATHD_REG  0x0161 //TS_PATH_D Register
#define ADPD4101_IND_REG    0x0162 //INPUTS_D Register
#define ADPD4101_AFED_REG   0x0164 //AFE_TRIM_D Register
#define ADPD4101_CNTD_REG   0x0167 //COUNTS_D Register
#define ADPD4101_PERD_REG   0x0168 //PERIOD_D Register
#define ADPD4101_INTEGD_REG 0x016A //INTEG_SETUP_D Register
#define ADPD4101_IOFFD_REG  0x016B //INTEG_OS_D Register
#define ADPD4101_MODPD_REG  0x016C //MOD_PULSE_D Register
#define ADPD4101_SIGD_REG   0x0170 //DATA_FORMAT_D Register
#define ADPD4101_LITD_REG   0x0171 //LIT_DATA_FORMAT_D Register

#define ADPD4101_INE_REG    0x0182 //INPUTS_E Register
#define ADPD4101_CATHE_REG  0x0183 //CATHODE_E Register
#define ADPD4101_AFEE_REG   0x0184 //AFE_TRIM_E Register
#define ADPD4101_CNTE_REG   0x0187 //COUNTS_E Register
#define ADPD4101_INTEGE_REG 0x018A //INTEG_SETUP_E Register
#define ADPD4101_IOFFE_REG  0x018B //INTEG_OS_E Register
#define ADPD4101_MODPE_REG  0x018C //MOD_PULSE_E Register
#define ADPD4101_SIGE_REG   0x0190 //DATA_FORMAT_E Register
#define ADPD4101_LITE_REG   0x0191 //LIT_DATA_FORMAT_E Register


/*********************************************************************
 * TYPEDEFS
 */
// Struct for messages sent to the application task
typedef struct
{
    uint8_t event;
    void    *pData;
} pzMsg_t;

// Struct for messages about characteristic data
typedef struct
{
    uint16_t svcUUID; // UUID of the service
    uint16_t dataLen; //
    uint8_t paramID; // Index of the characteristic
    uint8_t data[]; // Flexible array member, extended to malloc - sizeof(.)
} pzCharacteristicData_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} pzPasscodeReq_t;

// Struct for message about a pending parameter update request.
typedef struct
{
    uint16_t connHandle;
} pzSendParamReq_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} pzPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t deviceAddr[B_ADDR_LEN];
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} pzPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} pzGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t *connHandle;
} pzConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                    // Connection Handle
    Clock_Struct* pUpdateClock;             // pointer to clock struct
    bool phyCngRq;                          // Set to true if PHY change request is in progress
    uint8_t currPhy;                        // The active PHY for a connection
    uint8_t rqPhy;                          // The requested PHY for a connection
    uint8_t phyRqFailCnt;                   // PHY change request fail count
} pzConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task configuration
Task_Struct pzTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(appTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t appTaskStack[PZ_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Zero";

// Advertisement data
static uint8_t advertData[] =
{
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // complete name
    13, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'M',
    'a',
    's',
    't',
    'e',
    'r',
    'T',
    'h',
    'e',
    's',
    'i',
    's',
};

// Scan Response Data
static uint8_t scanRspData[] =
{
    // service UUID, to notify central devices what services are included
    // in this peripheral
    (ATT_UUID_SIZE + 0x01),   // length of this data, battery service UUID + header
    GAP_ADTYPE_128BIT_MORE,   // some of the UUID's, but not all
    BATTERY_SERVICE_SERV_UUID_BASE128(BATTERY_SERVICE_SERV_UUID),
};

// Advertising handles
static uint8_t advHandleLegacy;

// Per-handle connection info
static pzConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

/* Pin driver handles */
static PIN_Handle CDPinHandle;
static PIN_Handle intPinHandle;
static PIN_Handle ChargePinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State CDPinState;
static PIN_State intPinState;
static PIN_State ChargePinState;

/*
 * Initial CD pin configuration table
 *   - CD is off.
 */
PIN_Config CDPinTable[] = {
    CC2640_PIN_CD | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
    PIN_TERMINATE
};

/*
 * Initial Charge pin configuration table
 */
PIN_Config ChargePinTable[] = {
    CC2640_PIN_CHARGE | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    PIN_TERMINATE
};

/*
 * Application interrupt pin configuration table:
 *   - Interrupt is configured to trigger on rising edge.
 */
PIN_Config intPinTable[] = {
    Board_PIN_INTX | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE | PIN_HYSTERESIS,
    Board_PIN_INTY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE | PIN_HYSTERESIS,
    PIN_TERMINATE
};

// Clock objects
static Clock_Struct myClock;
static Clock_Handle myClockHandle;
static Clock_Struct BQClock;
static Clock_Handle BQClockHandle;
static Clock_Struct BQSoCClock;
static Clock_Handle BQSoCClockHandle;
static Clock_Struct TMPClock;
static Clock_Handle TMPClockHandle;
static Clock_Struct GetTMPClock;
static Clock_Handle GetTMPClockHandle;

// Update variable
static uint8_t batteryVal = 0;
static uint16_t tempVal = 0;
static uint32_t ecgVal = 0;
static uint32_t ecgLeadOff = 0;
static uint32_t ppgVals[3] = {0};

// I2C utils
static I2C_Handle i2c;
static I2C_Params i2cParams;
static uint8_t txBuffer[4];
static uint8_t rxBuffer[4];

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/* Task functions */
static void ProjectZero_init(void);
static void I2C_slave_init(void);
static void ProjectZero_taskFxn(UArg a0, UArg a1);

/* I2C functions */
static void I2C_writeRegister(const uint8_t slave_addr, const uint8_t reg, const uint8_t val);
static uint8_t I2C_readRegister(const uint8_t slave_addr, const uint8_t reg);
static void I2C_writeLongRegister(const uint8_t slave_addr, const uint16_t reg, const uint16_t val);
static uint16_t I2C_readLongRegister(const uint8_t slave_addr, const uint16_t reg);
static void I2C_start_TMP(void);
static void I2C_get_TMP(void);
static void I2C_reset_BQ(void);
static uint8_t I2C_get_status_BQ(void);
static void I2C_start_SoC_BQ(void);
static void I2C_get_SoC_BQ(void);
static void I2C_start_ADPD(void);
static void I2C_clear_FIFO_ADPD(void);
static void I2C_read_ADPD(void);

/* Event message processing functions */
static void ProjectZero_processStackEvent(uint32_t stack_event);
static void ProjectZero_processApplicationMessage(pzMsg_t *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_processGapMessage(gapEventHdr_t *pMsg);
static void ProjectZero_processHCIMsg(ICall_HciExtEvt *pMsg);
static void ProjectZero_processPairState(pzPairStateData_t *pPairState);
static void ProjectZero_processPasscode(pzPasscodeReq_t *pReq);
static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void ProjectZero_processAdvEvent(pzGapAdvEventData_t *pEventData);

/* Stack or profile callback function */
static void ProjectZero_advCallback(uint32_t event,
                                    void *pBuf,
                                    uintptr_t arg);
static void ProjectZero_passcodeCb(uint8_t *pDeviceAddr,
                                   uint16_t connHandle,
                                   uint8_t uiInputs,
                                   uint8_t uiOutputs,
                                   uint32_t numComparison);
static void ProjectZero_pairStateCb(uint16_t connHandle,
                                    uint8_t state,
                                    uint8_t status);

/* Interrupt callback function */
static void interruptCallbackFxn(PIN_Handle handle, PIN_Id pinId);

/* Connection handling functions */
static uint8_t ProjectZero_getConnIndex(uint16_t connHandle);
static uint8_t ProjectZero_clearConnListEntry(uint16_t connHandle);
static uint8_t ProjectZero_addConn(uint16_t connHandle);
static uint8_t ProjectZero_removeConn(uint16_t connHandle);
static void ProjectZero_updatePHYStat(uint16_t eventCode,
                                      uint8_t *pMsg);
static void ProjectZero_handleUpdateLinkParamReq(
    gapUpdateLinkParamReqEvent_t *pReq);
static void ProjectZero_sendParamUpdate(uint16_t connHandle);
static void ProjectZero_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);
static void ProjectZero_paramUpdClockHandler(UArg arg);
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport);

/* Utility functions */
static status_t ProjectZero_enqueueMsg(uint8_t event,
                                   void *pData);
static char * util_arrtohex(uint8_t const *src,
                            uint8_t src_len,
                            uint8_t       *dst,
                            uint8_t dst_len,
                            uint8_t reverse);
static char * util_getLocalNameStr(const uint8_t *advData, uint8_t len);
static void ProjectZero_processL2CAPMsg(l2capSignalEvent_t *pMsg);

static void myClockSwiFxn(uintptr_t arg0);
static void BQClockSwiFxn(uintptr_t arg0);
static void BQSoCClockSwiFxn(uintptr_t arg0);
static void TMPClockSwiFxn(uintptr_t arg0);
static void GetTMPClockSwiFxn(uintptr_t arg0);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause,
                          uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Bond Manager Callbacks
static gapBondCBs_t ProjectZero_BondMgrCBs =
{
    ProjectZero_passcodeCb,     // Passcode callback
    ProjectZero_pairStateCb     // Pairing/Bonding state Callback
};



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn     project_zero_spin
 *
 * @brief   Spin forever
 */
static void project_zero_spin(void)
{
  volatile uint8_t x = 0;;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      ProjectZero_createTask
 *
 * @brief   Task creation function for the Project Zero.
 */
void ProjectZero_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = PZ_TASK_STACK_SIZE;
    taskParams.priority = PZ_TASK_PRIORITY;

    Task_construct(&pzTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      ProjectZero_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void ProjectZero_init(void)
{
    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&appMsgQueue, NULL);
    appMsgQueueHandle = Queue_handle(&appMsgQueue);

    // ******************************************************************
    // Hardware initialization
    // ******************************************************************

    // Open CD pin
    CDPinHandle = PIN_open(&CDPinState, CDPinTable);
    if(!CDPinHandle)
    {
        Log_error0("Error initializing CD pin");
        Task_exit();
    }

    // Open Charge pin
    ChargePinHandle = PIN_open(&ChargePinState, ChargePinTable);
    if(!ChargePinHandle)
    {
        Log_error0("Error initializing board Charge pin");
        Task_exit();
    }

    // Open interrupt pin
    intPinHandle = PIN_open(&intPinState, intPinTable);
    if(!intPinHandle)
    {
        Log_error0("Error initializing interrupt pins");
        Task_exit();
    }
    
    // Setup callback for interrupt pins
    if(PIN_registerIntCb(intPinHandle, &interruptCallbackFxn) != 0)
    {
        Log_error0("Error registering interrupt callback function");
        Task_exit();
    }
    
    myClockHandle = Util_constructClock(&myClock,
                                        myClockSwiFxn, DEFAULT_CLOCK_TIMEOUT,
                                        DEFAULT_CLOCK_TIMEOUT,
                                        0,
                                        NULL);  
    
    // Clock each 10s to reset watchdog timer and having the value of the battery                                    
    BQClockHandle = Util_constructClock(&BQClock,
                                        BQClockSwiFxn, 0,
                                        BQ_CLOCK_TIMEOUT,
                                        0,
                                        NULL);
                                        
    // Clock to trigger end of SoC conversion                                 
    BQSoCClockHandle = Util_constructClock(&BQSoCClock,
                                        BQSoCClockSwiFxn, 5,
                                        0,
                                        0,
                                        NULL);
                                        
    // Clock each 10s to trigger temperature conversion                                    
    TMPClockHandle = Util_constructClock(&TMPClock,
                                        TMPClockSwiFxn, 0,
                                        TMP_CLOCK_TIMEOUT,
                                        0,
                                        NULL);
                                        
    // Clock to trigger end of SoC conversion                                 
    GetTMPClockHandle = Util_constructClock(&GetTMPClock,
                                        GetTMPClockSwiFxn, 5,
                                        0,
                                        0,
                                        NULL);

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configure GAP for param update
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
    // section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    {
        // Don't send a pairing request after connecting (the peer device must
        // initiate pairing)
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        // Use authenticated pairing: require passcode.
        uint8_t mitm = TRUE;
        // This device only has display capabilities. Therefore, it will display the
        // passcode during pairing. However, since the default passcode is being
        // used, there is no need to display anything.
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        // Request bonding (storing long-term keys for re-encryption upon subsequent
        // connections without repairing)
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t),
                                &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t),
                                &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t),
                                &bonding);
    }

    // ******************************************************************
    // BLE Service initialization
    // ******************************************************************
    GGS_AddService(GATT_ALL_SERVICES);         // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT Service
    DevInfo_AddService();                      // Device Information Service

    // Add services to GATT server and give ID of this task for Indication acks.
    Battery_service_AddService(selfEntity);
    Temp_service_AddService(selfEntity);
    Ecg_service_AddService(selfEntity);
    Ppg_service_AddService(selfEntity);

    // Placeholder variable for characteristic intialization
    uint8_t battery_service_batteryLevel_initVal[BATTERY_SERVICE_BATTERYLEVEL_LEN] = {0};
    uint8_t temp_service_tempValue_initVal[TEMP_SERVICE_TEMPVALUE_LEN] = {0};
    uint8_t ecg_service_ecgValue_initVal[ECG_SERVICE_ECGVALUE_LEN] = {0};
    uint8_t ecg_service_ecgLeadOff_initVal[ECG_SERVICE_ECGLEADOFF_LEN] = {0};
    uint8_t ppg_service_ppgValue_initVal[PPG_SERVICE_PPGVALUE_LEN] = {0};
    
    // Initalization of characteristics in battery_service that are readable.                             
    Battery_service_SetParameter(BATTERY_SERVICE_BATTERYLEVEL_ID, 
                                 BATTERY_SERVICE_BATTERYLEVEL_LEN, 
                                 battery_service_batteryLevel_initVal);
                                
    // Initalization of characteristics in temp_service that are readable.
    Temp_service_SetParameter(TEMP_SERVICE_TEMPVALUE_ID, 
                              TEMP_SERVICE_TEMPVALUE_LEN, 
                              temp_service_tempValue_initVal);
                              
    // Initalization of characteristics in ecg_service that are readable.
    Ecg_service_SetParameter(ECG_SERVICE_ECGVALUE_ID, 
                             ECG_SERVICE_ECGVALUE_LEN, 
                             ecg_service_ecgValue_initVal);
                             
    Ecg_service_SetParameter(ECG_SERVICE_ECGLEADOFF_ID, 
                             ECG_SERVICE_ECGLEADOFF_LEN, 
                             ecg_service_ecgLeadOff_initVal);
                             
    // Initalization of characteristics in ppg_service that are readable.
    Ppg_service_SetParameter(PPG_SERVICE_PPGVALUE_ID, 
                             PPG_SERVICE_PPGVALUE_LEN, 
                             ppg_service_ppgValue_initVal);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&ProjectZero_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the HCI section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set default values for Data Length Extension
    // Extended Data Length Feature is already enabled by default
    {
      // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
      // Some brand smartphone is essentially needing 251/2120, so we set them here.
      #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
      #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

      // This API is documented in hci.h
      // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
      // http://software-dl.ti.com/lprf/ble5stack-latest/
      HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
    GATT_InitClient();

    // Initialize Connection List
    ProjectZero_clearConnListEntry(CONNHANDLE_ALL);

    //Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, ADDRMODE_PUBLIC, NULL);
    
    // Initialize calibration of the internal 32 kHz oscillator
    RCOSC_enableCalibration();
}

/*********************************************************************
 * @fn      I2C_slave_init
 *
 * @brief   Called during initialization and contains I2C
 *          specific initialization
 */
static void I2C_slave_init(void)
{
    /* Initializing the BQ25125 */
    if (PIN_getInputValue(CC2640_PIN_CHARGE) == 0)
    {
        PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_ENABLED);
    }
    I2C_writeRegister(BQ25125_ADDR, BQ25125_SYSVOUT_REG, 0xAA); //Enable SW and choose 1.8V
    I2C_writeRegister(BQ25125_ADDR, BQ25125_LDO_REG, 0xE4); //Enable LS/LDO at 3.3V
    I2C_writeRegister(BQ25125_ADDR, BQ25125_TERM_REG, 0x0E); //TERM 2mA enabled
    I2C_writeRegister(BQ25125_ADDR, BQ25125_FAST_REG, 0xF8); //FAST enabled (290mA, avoid 300mA = MAX) + charger enabled + not high impedance module
    I2C_writeRegister(BQ25125_ADDR, BQ25125_ILIM_REG, 0x3A); //ILIM = 400mA and BUVLO = 3V
    I2C_writeRegister(BQ25125_ADDR, BQ25125_TS_INT_REG, 0x00); //Disable TS and INT
    PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_DISABLED);
    
    /* Initializing the TMP1075 */
    I2C_writeRegister(TMP1075_ADDR, TMP1075_OS_REG, 0x05); //Shutdown mode with high alert
    
    /* Initializing the ADPD4101 */
    // General init
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LFOSC_REG, 0x0006); //1MHz low frequency oscillator
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_TS_REG, 0x0400); //Time slots ABCDE enabled
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CFG_REG, 0x0008); //IN7-8 as differential
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_GPIO_REG, 0x0012); //GPIO0 and 1 as normal output
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_GPIO01_REG, 0x0302); //GPIO0 = INTX/ GPIO1 = INTY
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_FIFOTH_REG, 0x01FD); //FIFO_TH = 509
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INTX_REG, 0x8000); //INT X = INT_FIFO_TH
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INTY_REG, 0x0010); //INT Y = INT_DATA_E

    // PPG measurement (green LED) on time slot A
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INA_REG, 0x0001); //Enable IN1 on channel 1
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CATHA_REG, 0x5002); //Preconditionned to TIA_VREF and VC1 = TIA_VREF + 215mV
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED12A_REG, 0x0000); //Activate green LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED34A_REG, 0x0000); //Deactivate R and IR LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_PATA_REG, 0x00AA); //Integrator chop mode one pulse 2 and 4
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_SIGA_REG, 0x0002); //2 signal data bytes in the FIFO
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LITA_REG, 0x0000); //No lit data in the FIFO

    // PPG measurement (R LED) on time slot B
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INB_REG, 0x0001); //Enable IN1 on channel 1
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CATHB_REG, 0x5002); //Preconditionned to TIA_VREF and VC1 = TIA_VREF + 215mV
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED12B_REG, 0x8080); //Deactivate green LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED34B_REG, 0x0080); //Activate R and deactivate IR LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_PATB_REG, 0x00AA); //Integrator chop mode one pulse 2 and 4
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_SIGB_REG, 0x0002); //2 signal data bytes in the FIFO
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LITB_REG, 0x0000); //No lit data in the FIFO

    // PPG measurement (IR LED) on time slot C
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INC_REG, 0x0001); //Enable IN1 on channel 1
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CATHC_REG, 0x5002); //Preconditionned to TIA_VREF and VC1 = TIA_VREF + 215mV
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED12C_REG, 0x8080); //Deactivate green LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LED34C_REG, 0x8000); //Deactivate R and activate IR LEDs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_PATC_REG, 0x00AA); //Integrator chop mode one pulse 2 and 4
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_SIGC_REG, 0x0002); //2 signal data bytes in the FIFO
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LITC_REG, 0x0000); //No lit data in the FIFO

    // ECG measurement on time slot D
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_PATHD_REG, 0x00E6); //TIA, integrator and ADC, bypass the BPF
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_IND_REG, 0x7000); //Enable IN7-8 on channel 1
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_AFED_REG, 0x02C1); //TIA_VREF = 0.9V and TIA gain = 100kOhms
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CNTD_REG, 0x0102); //2 pulses
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_PERD_REG, 0x1000); //Float type operation
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INTEGD_REG, 0x0203); //Integrator pulse width 3µs, bandpass filter powered down
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_IOFFD_REG, 0x01A0); //Integrator pulse timing offset 13µs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_MODPD_REG, 0x0210); //MOD WIDTH 2µs and MOD OFFSET 16µs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_SIGD_REG, 0x0002); //2 signal data bytes in the FIFO
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LITD_REG, 0x0000); //No lit data in the FIFO

    // Lead off on time slot E
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INE_REG, 0x0010); //IN3 to CH1, IN4 disconnected
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CATHE_REG, 0x5A45); //Precondition to TIA_VREF, pulse VC2_VREF by 215 mV
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_AFEE_REG, 0xE212); //50 kΩ TIA GAIN both channels, TIA_VREF=0.88V
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_CNTE_REG, 0x0110); // 16 pulses
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_INTEGE_REG, 0x0003); //Integrator pulse width 3µs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_IOFFE_REG, 0x0216); //Integrator pulse timing offset 16.9375µs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_MODPE_REG, 0x0210); //MOD WIDTH 2µs and MOD OFFSET 16µs
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_SIGE_REG, 0x0002); //2 signal data bytes in the FIFO
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_LITE_REG, 0x0000); //No lit data in the FIFO
}

/*********************************************************************
 * @fn      ProjectZero_taskFxn
 *
 * @brief   Application task entry point for the Project Zero.
 *
 * @param   a0, a1 - not used.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    ProjectZero_init();
    
    // Call drivers init function
    I2C_init();
    
    // Initialize I2C
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode  = I2C_MODE_BLOCKING;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (!i2c) {
        Log_error0("Error Initializing I2C\n");
        while (1);
    }
    else {
        Log_info0("I2C Initialized!\n");
    }
    
    // Initialize I2C slaves
    I2C_slave_init();

    // Start clock to reset watchdog timer of BQ25125
    Util_startClock((Clock_Struct *)BQClockHandle);

    // Application main loop
    for(;; )
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, PZ_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if(events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if(ICall_fetchServiceMsg(&src, &dest,
                                     (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                    // Check for BLE stack events first
                    if(pEvt->signature == 0xffff)
                    {
                        // Process stack events
                        ProjectZero_processStackEvent(pEvt->event_flag);
                    }
                    else
                    {
                        switch(pMsg->hdr.event)
                        {
                        case GAP_MSG_EVENT:
                            // Process GAP message
                            ProjectZero_processGapMessage((gapEventHdr_t*) pMsg);
                            break;

                        case GATT_MSG_EVENT:
                            // Process GATT message
                            safeToDealloc =
                                ProjectZero_processGATTMsg(
                                    (gattMsgEvent_t *)pMsg);
                            break;

                        case HCI_GAP_EVENT_EVENT:
                            ProjectZero_processHCIMsg(pMsg);
                            break;

                        case L2CAP_SIGNAL_EVENT:
                            // Process L2CAP free buffer notification
                            ProjectZero_processL2CAPMsg(
                                (l2capSignalEvent_t *)pMsg);
                            break;

                        default:
                            // do nothing
                            break;
                        }
                    }
                }

                if(pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // Process messages sent from another task or another context.
            while(!Queue_empty(appMsgQueueHandle))
            {
                pzMsg_t *pMsg = (pzMsg_t *)Util_dequeueMsg(appMsgQueueHandle);
                if(pMsg)
                {
                    // Process application-layer message probably sent from ourselves.
                    ProjectZero_processApplicationMessage(pMsg);
                    // Free the received message.
                    ICall_free(pMsg);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      ProjectZero_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @param   pMsg - L2CAP signal buffer from stack
 *
 * @return  None
 */
static void ProjectZero_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
    switch(pMsg->opcode)
    {
      case L2CAP_NUM_CTRL_DATA_PKT_EVT:
          break;
      default:
          break;
    }
}


/*********************************************************************
 * @fn      ProjectZero_processStackEvent
 *
 * @brief   Process stack event. The event flags received are user-selected
 *          via previous calls to stack APIs.
 *
 * @param   stack_event - mask of events received
 *
 * @return  none
 */
static void ProjectZero_processStackEvent(uint32_t stack_event)
{
    // Intentionally blank
}

/*********************************************************************
 * @fn      ProjectZero_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
    if(pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Log_error1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if(pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return(TRUE);
}

/*********************************************************************
 * @fn      ProjectZero_processApplicationMessage
 *
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type pzMsg_t.
 */
static void ProjectZero_processApplicationMessage(pzMsg_t *pMsg)
{
    // Cast to pzCharacteristicData_t* here since it's a common message pdu type.
    //pzCharacteristicData_t *pCharData = (pzCharacteristicData_t *)pMsg->pData;

    switch(pMsg->event)
    {
      case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

      case PZ_SERVICE_WRITE_EVT: /* Message about received value write */
          /* Call different handler per service */

          break;

      case PZ_SERVICE_CFG_EVT: /* Message about received CCCD write */
          /* Call different handler per service */
          //switch(pCharData->svcUUID)
          //{

          //}
          break;

      case PZ_UPDATE_CHARVAL_EVT: /* Message from ourselves to send  */
          break;

      case PZ_ADV_EVT:
          ProjectZero_processAdvEvent((pzGapAdvEventData_t*)(pMsg->pData));
          break;

      case PZ_SEND_PARAM_UPD_EVT:
      {
          // Send connection parameter update
          pzSendParamReq_t* req = (pzSendParamReq_t *)pMsg->pData;
          ProjectZero_sendParamUpdate(req->connHandle);
      }
      break;

      case PZ_START_ADV_EVT:
          if(linkDB_NumActive() < MAX_NUM_BLE_CONNS)
          {
              // Enable advertising if there is room for more connections
              GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
          }
          break;

      case PZ_PAIRSTATE_EVT: /* Message about the pairing state */
          ProjectZero_processPairState((pzPairStateData_t*)(pMsg->pData));
          break;

      case PZ_PASSCODE_EVT: /* Message about pairing PIN request */
      {
          pzPasscodeReq_t *pReq = (pzPasscodeReq_t *)pMsg->pData;
          ProjectZero_processPasscode(pReq);
      }
      break;

      case PZ_CONN_EVT:
        ProjectZero_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        break;
        
      case PZ_PERIODIC_TIMER:
        
        break;
        
      case BQ_PERIODIC_TIMER:
        if (PIN_getInputValue(CC2640_PIN_CHARGE) == 0)
        {
            PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_ENABLED);
        }
        I2C_start_SoC_BQ();
        PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_DISABLED);
        Util_startClock((Clock_Struct *)BQSoCClockHandle);
        break;
        
      case BQSOC_TIMER:
        if (PIN_getInputValue(CC2640_PIN_CHARGE) == 0)
        {
            PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_ENABLED);
        }
        I2C_get_SoC_BQ();
        PIN_setOutputValue(CDPinHandle, CC2640_PIN_CD, CC2640_CD_I2C_DISABLED);
        Battery_service_SetParameter(BATTERY_SERVICE_BATTERYLEVEL_ID, 
                                 BATTERY_SERVICE_BATTERYLEVEL_LEN, 
                                 &batteryVal);
      
      case START_ADPD:
        I2C_start_ADPD();
        
      case TMP_PERIODIC_TIMER:
        I2C_start_TMP();
        Util_startClock((Clock_Struct *)GetTMPClockHandle);
        break;
        
      case GET_TMP_PERIODIC_TIMER:
        I2C_get_TMP();
        Temp_service_SetParameter(TEMP_SERVICE_TEMPVALUE_ID, 
                              TEMP_SERVICE_TEMPVALUE_LEN, 
                              &tempVal);
      
      case INT_X_TRIGGERED:
        I2C_clear_FIFO_ADPD();
        
      case INT_Y_TRIGGERED:
        I2C_read_ADPD();
        Ppg_service_SetParameter(PPG_SERVICE_PPGVALUE_ID, 
                             PPG_SERVICE_PPGVALUE_LEN, 
                             ppgVals);
        Ecg_service_SetParameter(ECG_SERVICE_ECGVALUE_ID, 
                              ECG_SERVICE_ECGVALUE_LEN, 
                              &ecgVal);
        Ecg_service_SetParameter(ECG_SERVICE_ECGLEADOFF_ID, 
                             ECG_SERVICE_ECGLEADOFF_LEN, 
                             &ecgLeadOff);
                              
      default:
        break;
    }

    if(pMsg->pData != NULL)
    {
        ICall_free(pMsg->pData);
    }
}

/*********************************************************************
 * @fn      ProjectZero_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void ProjectZero_processGapMessage(gapEventHdr_t *pMsg)
{
    switch(pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        bStatus_t status = FAILURE;

        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

        if(pPkt->hdr.status == SUCCESS)
        {
            // Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

            // Set Device Info Service Parameter
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);

            // Display device address
            // Need static so string persists until printed in idle thread.
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("GAP is started. Our address: " \
                      ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);

            // Setup and start Advertising
            // For more information, see the GAP section in the User's Guide:
            // http://software-dl.ti.com/lprf/ble5stack-latest/

            // Temporary memory for advertising parameters for set #1. These will be copied
            // by the GapAdv module
            GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

            // Create Advertisement set #1 and assign handle
            status = GapAdv_create(&ProjectZero_advCallback, &advParamLegacy,
                                   &advHandleLegacy);
            APP_ASSERT(status == SUCCESS);

            Log_info1("Name in advertData array: " \
                      ANSI_COLOR(FG_YELLOW) "%s" ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)util_getLocalNameStr(advertData,
                                                      sizeof(advertData)));

            // Load advertising data for set #1 that is statically allocated by the app
            status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                         sizeof(advertData), advertData);
            APP_ASSERT(status == SUCCESS);

            // Load scan response data for set #1 that is statically allocated by the app
            status =
                GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                    sizeof(scanRspData),
                                    scanRspData);
            APP_ASSERT(status == SUCCESS);

            // Set event mask for set #1
            status = GapAdv_setEventMask(advHandleLegacy,
                                         GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                         GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                         GAP_ADV_EVT_MASK_SET_TERMINATED);

            // Enable legacy advertising for set #1
            status =
                GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX,
                              0);
            APP_ASSERT(status == SUCCESS);
        }

        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info2("Link establish event, status 0x%02x. Num Conns: %d",
                  pPkt->hdr.status,
                  linkDB_NumActive());

        if(pPkt->hdr.status == SUCCESS)
        {
            // Add connection to list
            ProjectZero_addConn(pPkt->connectionHandle);

            // Display the address of this connection
            static uint8_t addrStr[3 * B_ADDR_LEN + 1];
            util_arrtohex(pPkt->devAddr, B_ADDR_LEN, addrStr, sizeof addrStr,
                          UTIL_ARRTOHEX_REVERSE);
            Log_info1("Connected. Peer address: " \
                        ANSI_COLOR(FG_GREEN)"%s"ANSI_COLOR(ATTR_RESET),
                      (uintptr_t)addrStr);
        }

        if(linkDB_NumActive() < MAX_NUM_BLE_CONNS)
        {
            Log_info1("Continue to Advertise, %d possible connection remain", MAX_NUM_BLE_CONNS - linkDB_NumActive());
            // Start advertising since there is room for more connections
            GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        }
        else
        {
            Log_info1("Max Number of Connection reach: %d, Adv. will not be enable again", linkDB_NumActive());
            //Util_startClock((Clock_Struct *)myClockHandle);
            ProjectZero_enqueueMsg(START_ADPD, NULL);
            Util_startClock((Clock_Struct *)TMPClockHandle);
            Log_info0("Clocks started");
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

        // Display the amount of current connections
        Log_info0("Device Disconnected!");
        Log_info1("Num Conns: %d", linkDB_NumActive());

        // Remove the connection from the list and disable RSSI if needed
        ProjectZero_removeConn(pPkt->connectionHandle);

        // GapAdv_enable will return success only if the maximum number of connections 
		// has been reached, and adv was not re-enable in GAP_LINK_ESTABLISHED_EVENT
		// switch case.
        // If less connection were in used, Advertisement will have been restart in 
		// the GAP_LINK_ESTABLISHED_EVENT switch case and calling GapAdv_enable will 
		// just return an error.
        if ( GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0) == SUCCESS)
        {
            Log_info1("Restart Advertising, %d possible connection remain", MAX_NUM_BLE_CONNS - linkDB_NumActive());
            //Util_stopClock((Clock_Struct *)myClockHandle);
            Log_info0("Clock stopped");
        }
    }
    break;

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
        ProjectZero_handleUpdateLinkParamReq(
            (gapUpdateLinkParamReqEvent_t *)pMsg);
        break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
        ProjectZero_handleUpdateLinkEvent((gapLinkUpdateEvent_t *)pMsg);
        break;

    case GAP_PAIRING_REQ_EVENT:
        // Disable advertising so that the peer device can be added to
        // the resolving list
        GapAdv_disable(advHandleLegacy);
        break;

    default:
        break;
    }
}

void ProjectZero_processHCIMsg(ICall_HciExtEvt *pEvt)
{
    ICall_Hdr *pMsg = (ICall_Hdr *)pEvt;

    // Process HCI message
    switch(pMsg->status)
    {
    case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        ProjectZero_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
        break;

    case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
        break;

    // HCI Commands Events
    case HCI_COMMAND_STATUS_EVENT_CODE:
    {
        hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
        switch(pMyMsg->cmdOpcode)
        {
        case HCI_LE_SET_PHY:
        {
            if(pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                Log_info0("PHY Change failure, peer does not support this");
            }
            else
            {
                Log_info1("PHY Update Status Event: 0x%x",
                          pMyMsg->cmdStatus);
            }

            ProjectZero_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
        }
        break;

        default:
            break;
        }
    }
    break;

    // LE Events
    case HCI_LE_EVENT_CODE:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        // A Phy Update Has Completed or Failed
        if(pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
        {
            if(pPUC->status != SUCCESS)
            {
                Log_info0("PHY Change failure");
            }
            else
            {
                // Only symmetrical PHY is supported.
                // rxPhy should be equal to txPhy.
                Log_info1("PHY Updated to %s",
                          (uintptr_t)((pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
                                      (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value"));
            }

            ProjectZero_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT,
                                      (uint8_t *)pMsg);
        }
    }
    break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      ProjectZero_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static void ProjectZero_processAdvEvent(pzGapAdvEventData_t *pEventData)
{
    switch(pEventData->event)
    {
    /* Sent on the first advertisement after a GapAdv_enable */
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        Log_info1("Adv Set %d Enabled", *(uint8_t *)(pEventData->pBuf));
        break;

    /* Sent after advertising stops due to a GapAdv_disable */
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        Log_info1("Adv Set %d Disabled", *(uint8_t *)(pEventData->pBuf));
        break;

    /* Sent at the beginning of each advertisement. (Note that this event
     * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_START:
        break;

    /* Sent after each advertisement. (Note that this event is not enabled
     * by default, see GapAdv_setEventMask). */
    case GAP_EVT_ADV_END:
        break;

    /* Sent when an advertisement set is terminated due to a
     * connection establishment */
    case GAP_EVT_ADV_SET_TERMINATED:
    {
        GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

        Log_info2("Adv Set %d disabled after conn %d",
                  advSetTerm->handle, advSetTerm->connHandle);
    }
    break;

    /* Sent when a scan request is received. (Note that this event
     * is not enabled by default, see GapAdv_setEventMask). */
    case GAP_EVT_SCAN_REQ_RECEIVED:
        break;

    /* Sent when an operation could not complete because of a lack of memory.
       This message is not allocated on the heap and must not be freed */
    case GAP_EVT_INSUFFICIENT_MEMORY:
        break;

    default:
        break;
    }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

/*********************************************************************
 * @fn      ProjectZero_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   pPairData - pointer to pair state data container
 */
static void ProjectZero_processPairState(pzPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;

    switch(state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
        Log_info0("Pairing started");
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        if(status == SUCCESS)
        {
            Log_info0("Pairing success");
        }
        else
        {
            Log_info1("Pairing fail: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
        if(status == SUCCESS)
        {
            Log_info0("Encryption success");
        }
        else
        {
            Log_info1("Encryption failed: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
        if(status == SUCCESS)
        {
            Log_info0("Bond save success");
        }
        else
        {
            Log_info1("Bond save failed: %d", status);
        }
        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      ProjectZero_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @param   pReq - pointer to passcode req
 */
static void ProjectZero_processPasscode(pzPasscodeReq_t *pReq)
{
    Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
              (uintptr_t)(pReq->uiInputs ? "Sending" : "Displaying"),
              B_APP_DEFAULT_PASSCODE);

    // Send passcode response.
    GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}
/*********************************************************************
 * @fn      ProjectZero_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  Log_info1("Connection event done for connHandle: %d", pReport->handle);
}

/*********************************************************************
 * @fn      ProjectZero_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 */
static void ProjectZero_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    uint8_t status = pMsg->pReturnParam[0];

    //Find which command this command complete is for
    switch(pMsg->cmdOpcode)
    {
    case HCI_READ_RSSI:
    {
    		int8 rssi = (int8)pMsg->pReturnParam[3];
    
        // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
        if(status == SUCCESS)
        {
            uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1],
                                           pMsg->pReturnParam[2]);

            Log_info2("RSSI:%d, connHandle %d",
                      (uint32_t)(rssi),
                      (uint32_t)handle);
        } // end of if (status == SUCCESS)
        break;
    }

    case HCI_LE_READ_PHY:
    {
        if(status == SUCCESS)
        {
            Log_info2("RXPh: %d, TXPh: %d",
                      pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
        }
        break;
    }

    default:
        break;
    } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
 * @fn      ProjectZero_handleUpdateLinkParamReq
 *
 * @brief   Receive and respond to a parameter update request sent by
 *          a peer device
 *
 * @param   pReq - pointer to stack request message
 */
static void ProjectZero_handleUpdateLinkParamReq(
    gapUpdateLinkParamReqEvent_t *pReq)
{
    gapUpdateLinkParamReqReply_t rsp;

    rsp.connectionHandle = pReq->req.connectionHandle;
    rsp.signalIdentifier = pReq->req.signalIdentifier;

    // Only accept connection intervals with slave latency of 0
    // This is just an example of how the application can send a response
    if(pReq->req.connLatency == 0)
    {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
    }
    else
    {
        rsp.accepted = FALSE;
    }

    // Send Reply
    VOID GAP_UpdateLinkParamReqReply(&rsp);
}

/*********************************************************************
 * @fn      ProjectZero_handleUpdateLinkEvent
 *
 * @brief   Receive and parse a parameter update that has occurred.
 *
 * @param   pEvt - pointer to stack event message
 */
static void ProjectZero_handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt)
{
    // Get the address from the connection handle
    linkDBInfo_t linkInfo;
    linkDB_GetInfo(pEvt->connectionHandle, &linkInfo);

    static uint8_t addrStr[3 * B_ADDR_LEN + 1];
    util_arrtohex(linkInfo.addr, B_ADDR_LEN, addrStr, sizeof addrStr,
                  UTIL_ARRTOHEX_REVERSE);

    if(pEvt->status == SUCCESS)
    {
        uint8_t ConnIntervalFracture = 25*(pEvt->connInterval % 4);
        // Display the address of the connection update
        Log_info5(
            "Updated params for %s, interval: %d.%d ms, latency: %d, timeout: %d ms",
            (uintptr_t)addrStr,
            (uintptr_t)(pEvt->connInterval*CONN_INTERVAL_MS_CONVERSION),
            ConnIntervalFracture,
            pEvt->connLatency,
            pEvt->connTimeout*CONN_TIMEOUT_MS_CONVERSION);
    }
    else
    {
        // Display the address of the connection update failure
        Log_info2("Update Failed 0x%02x: %s", pEvt->opcode, (uintptr_t)addrStr);
    }

    // Check if there are any queued parameter updates
    pzConnHandleEntry_t *connHandleEntry = (pzConnHandleEntry_t *)List_get(
        &paramUpdateList);
    if(connHandleEntry != NULL)
    {
        // Attempt to send queued update now
        ProjectZero_sendParamUpdate(*(connHandleEntry->connHandle));

        // Free list element
        ICall_free(connHandleEntry->connHandle);
        ICall_free(connHandleEntry);
    }
}

/*********************************************************************
 * @fn      ProjectZero_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  bleMemAllocError if a param update event could not be sent. Else SUCCESS.
 */
static uint8_t ProjectZero_addConn(uint16_t connHandle)
{
    uint8_t i;
    uint8_t status = bleNoResources;

    // Try to find an available entry
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;

            // Create a clock object and start
            connList[i].pUpdateClock
              = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

            if (connList[i].pUpdateClock)
            {
              Util_constructClock(connList[i].pUpdateClock,
                                  ProjectZero_paramUpdClockHandler,
                                  PZ_SEND_PARAM_UPDATE_DELAY, 0, true,
                                  (uintptr_t)connHandle);
            }

            // Set default PHY to 1M
            connList[i].currPhy = HCI_PHY_1_MBPS; // TODO: Is this true, neccessarily?

            break;
        }
    }

    return(status);
}

/*********************************************************************
 * @fn      ProjectZero_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @param   connHandle - connection handle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ProjectZero_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if(connList[i].connHandle == connHandle)
        {
            return(i);
        }
    }

    return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      ProjectZero_clearConnListEntry
 *
 * @brief   Clear the connection information structure held locally.
 *
 * @param   connHandle - connection handle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t ProjectZero_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if(connHandle != CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = ProjectZero_getConnIndex(connHandle);
        if(connIndex >= MAX_NUM_BLE_CONNS)
        {
            return(bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if((connIndex == i) || (connHandle == CONNHANDLE_ALL))
        {
            connList[i].connHandle = CONNHANDLE_INVALID;
            connList[i].currPhy = 0;
            connList[i].phyCngRq = 0;
            connList[i].phyRqFailCnt = 0;
            connList[i].rqPhy = 0;
        }
    }

    return(SUCCESS);
}

/*********************************************************************
 * @fn      ProjectZero_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t ProjectZero_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = ProjectZero_getConnIndex(connHandle);

    if(connIndex < MAX_NUM_BLE_CONNS)
    {
      Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

      if (pUpdateClock != NULL)
      {
        // Stop and destruct the RTOS clock if it's still alive
        if (Util_isActive(pUpdateClock))
        {
          Util_stopClock(pUpdateClock);
        }

        // Destruct the clock object
        Clock_destruct(pUpdateClock);
        // Free clock struct
        ICall_free(pUpdateClock);
      }
      // Clear Connection List Entry
      ProjectZero_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/*********************************************************************
 * @fn      ProjectZero_sendParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @param   connHandle - connection handle
 */
static void ProjectZero_sendParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    connIndex = ProjectZero_getConnIndex(connHandle);
    APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct
    ICall_free(connList[connIndex].pUpdateClock);
    connList[connIndex].pUpdateClock = NULL;

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the update completes
    if(status == bleAlreadyInRequestedMode)
    {
        pzConnHandleEntry_t *connHandleEntry =
            ICall_malloc(sizeof(pzConnHandleEntry_t));
        if(connHandleEntry)
        {
            connHandleEntry->connHandle = ICall_malloc(sizeof(uint16_t));

            if(connHandleEntry->connHandle)
            {
                *(connHandleEntry->connHandle) = connHandle;

                List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
            }
        }
    }
}

/*********************************************************************
 * @fn      ProjectZero_updatePHYStat
 *
 * @brief   Update the auto phy update state machine
 *
 * @param   eventCode - HCI LE Event code
 *          pMsg - message to process
 */
static void ProjectZero_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
    uint8_t connIndex;
    pzConnHandleEntry_t *connHandleEntry;

    switch(eventCode)
    {
    case HCI_LE_SET_PHY:
    {
        // Get connection handle from list
        connHandleEntry = (pzConnHandleEntry_t *)List_get(&setPhyCommStatList);

        if(connHandleEntry)
        {
            // Get index from connection handle
            connIndex = ProjectZero_getConnIndex(*(connHandleEntry->connHandle));
            APP_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

            ICall_free(connHandleEntry->connHandle);
            ICall_free(connHandleEntry);

            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

            if(pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
            {
                // Update the phy change request status for active RSSI tracking connection
                connList[connIndex].phyCngRq = FALSE;
                connList[connIndex].phyRqFailCnt++;
            }
        }
        break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
        hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

        if(pPUC)
        {
            // Get index from connection handle
            uint8_t index = ProjectZero_getConnIndex(pPUC->connHandle);
            APP_ASSERT(index < MAX_NUM_BLE_CONNS);

            // Update the phychange request status for active RSSI tracking connection
            connList[index].phyCngRq = FALSE;

            if(pPUC->status == SUCCESS)
            {
                connList[index].currPhy = pPUC->rxPhy;
            }
            if(pPUC->rxPhy != connList[index].rqPhy)
            {
                connList[index].phyRqFailCnt++;
            }
            else
            {
                // Reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
                connList[index].rqPhy = 0;
            }
        }

        break;
    }

    default:
        break;
    } // end of switch (eventCode)
}

/*
 * @brief   SWI handler function for periodic clock expiry
 *
 * @param   arg0 - Passed by TI-RTOS clock module
 */
static void myClockSwiFxn(uintptr_t arg0)
{
  // Can't call blocking TI-RTOS calls or BLE APIs from here.
  // .. Send a message to the Task that something is afoot.
  ProjectZero_enqueueMsg(PZ_PERIODIC_TIMER, NULL);
}

/*
 * @brief   SWI handler function for periodic clock expiry
 *
 * @param   arg0 - Passed by TI-RTOS clock module
 */
static void BQClockSwiFxn(uintptr_t arg0)
{
  // Can't call blocking TI-RTOS calls or BLE APIs from here.
  // .. Send a message to the Task that something is afoot.
  ProjectZero_enqueueMsg(BQ_PERIODIC_TIMER, NULL);
}

/*
 * @brief   SWI handler function for periodic clock expiry
 *
 * @param   arg0 - Passed by TI-RTOS clock module
 */
static void BQSoCClockSwiFxn(uintptr_t arg0)
{
  // Can't call blocking TI-RTOS calls or BLE APIs from here.
  // .. Send a message to the Task that something is afoot.
  ProjectZero_enqueueMsg(BQSOC_TIMER, NULL);
}

/*
 * @brief   SWI handler function for periodic clock expiry
 *
 * @param   arg0 - Passed by TI-RTOS clock module
 */
static void TMPClockSwiFxn(uintptr_t arg0)
{
  // Can't call blocking TI-RTOS calls or BLE APIs from here.
  // .. Send a message to the Task that something is afoot.
  ProjectZero_enqueueMsg(TMP_PERIODIC_TIMER, NULL);
}

/*
 * @brief   SWI handler function for periodic clock expiry
 *
 * @param   arg0 - Passed by TI-RTOS clock module
 */
static void GetTMPClockSwiFxn(uintptr_t arg0)
{
  // Can't call blocking TI-RTOS calls or BLE APIs from here.
  // .. Send a message to the Task that something is afoot.
  ProjectZero_enqueueMsg(GET_TMP_PERIODIC_TIMER, NULL);
}

/*********************************************************************
 * @fn      ProjectZero_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 *          pBuf - data potentially accompanying event
 *          arg - not used
 */
static void ProjectZero_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    pzGapAdvEventData_t *eventData = ICall_malloc(sizeof(pzGapAdvEventData_t));

    if(eventData != NULL)
    {
        eventData->event = event;
        eventData->pBuf = pBuf;

        if(ProjectZero_enqueueMsg(PZ_ADV_EVT, eventData) != SUCCESS)
        {
          ICall_free(eventData);
        }
    }
}

/*********************************************************************
 * @fn      ProjectZero_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 *          state - pair state
 *          status - pair status
 */
static void ProjectZero_pairStateCb(uint16_t connHandle, uint8_t state,
                                    uint8_t status)
{
    pzPairStateData_t *pairState =
        (pzPairStateData_t *)ICall_malloc(sizeof(pzPairStateData_t));

    if(pairState != NULL)
    {
        pairState->state = state;
        pairState->connHandle = connHandle;
        pairState->status = status;

        if(ProjectZero_enqueueMsg(PZ_PAIRSTATE_EVT, pairState) != SUCCESS)
        {
          ICall_free(pairState);
        }
    }
}

/*********************************************************************
 * @fn      ProjectZero_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @param   pDeviceAddr - not used
 *          connHandle - connection handle
 *          uiInpuits - if TRUE, the local device should accept a passcode input
 *          uiOutputs - if TRUE, the local device should display the passcode
 *          numComparison - the code that should be displayed for numeric
 *          comparison pairing. If this is zero, then passcode pairing is occurring.
 */
static void ProjectZero_passcodeCb(uint8_t *pDeviceAddr,
                                   uint16_t connHandle,
                                   uint8_t uiInputs,
                                   uint8_t uiOutputs,
                                   uint32_t numComparison)
{
    pzPasscodeReq_t *req =
        (pzPasscodeReq_t *)ICall_malloc(sizeof(pzPasscodeReq_t));
    if(req != NULL)
    {
        req->connHandle = connHandle;
        req->uiInputs = uiInputs;
        req->uiOutputs = uiOutputs;
        req->numComparison = numComparison;

        if(ProjectZero_enqueueMsg(PZ_PASSCODE_EVT, req) != SUCCESS)
        {
          ICall_free(req);
        }
    }
    ;
}

/*********************************************************************
 * @fn      ProjectZero_paramUpdClockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - app message pointer
 */
static void ProjectZero_paramUpdClockHandler(UArg arg)
{
    pzSendParamReq_t *req =
        (pzSendParamReq_t *)ICall_malloc(sizeof(pzSendParamReq_t));
    if(req)
    {
        req->connHandle = (uint16_t)arg;
        if(ProjectZero_enqueueMsg(PZ_SEND_PARAM_UPD_EVT, req) != SUCCESS)
        {
          ICall_free(req);
        }
    }
}

/*********************************************************************
 * @fn     interruptCallbackFxn
 *
 * @brief  Callback from PIN driver on interrupt X or Y
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void interruptCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    // Send the right message queue
    switch(pinId)
    {
    case Board_PIN_INTX:
        ProjectZero_enqueueMsg(INT_X_TRIGGERED, NULL);
        break;
    case Board_PIN_INTY:
        ProjectZero_enqueueMsg(INT_Y_TRIGGERED, NULL);
        break;
    }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn      I2C_start_TMP
 *
 * @brief   Start a conversion of temperature, must be call before I2C_start_TMP
 */
static void I2C_start_TMP(void)
{
    I2C_writeRegister(TMP1075_ADDR, TMP1075_OS_REG, 0x85); //Start conversion
}

/*********************************************************************
 * @fn      I2C_get_TMP
 *
 * @brief   Get a conversion of temperature
 */
static void I2C_get_TMP(void)
{
    bool ret;
    I2C_Transaction i2cTransaction;

    /* Common I2C transaction setup */
    txBuffer[0] = TMP1075_RES_REG;
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 2;

    /* Address the read register */
    i2cTransaction.slaveAddress = TMP1075_ADDR;
    ret = I2C_transfer(i2c, &i2cTransaction);
    if (!ret) {
      while(1);
    }

    /* Take 1 sample */
    tempVal = (rxBuffer[0] << 4) | (rxBuffer[1] >> 4);
}

/*********************************************************************
 * @fn      I2C_reset_BQ
 *
 * @brief   Reset the BQ25125 to default
 */
static void I2C_reset_BQ(void)
{
    I2C_writeRegister(BQ25125_ADDR, BQ25125_ILIM_REG, 0x80); //RESET
}

/*********************************************************************
 * @fn      I2C_get_status_BQ
 *
 * @brief   Get the BQ25125 status
 * 
 * @return  Value of the register as a byte
 */
static uint8_t I2C_get_status_BQ(void)
{
    return (I2C_readRegister(BQ25125_ADDR, BQ25125_STATUS_REG));
}

/*********************************************************************
 * @fn      I2C_start_SoC_BQ
 *
 * @brief   Start SoC for BQ25125, must be called at least 2 ms before I2C_get_SoC_BG
 */
static void I2C_start_SoC_BQ(void)
{
    I2C_writeRegister(BQ25125_ADDR, BQ25125_VB_REG, 0x80);
}

/*********************************************************************
 * @fn      I2C_get_SoC_BQ
 *
 * @brief   Get SoC for BQ25125
 */
static void I2C_get_SoC_BQ(void)
{
    uint8_t val;
    uint8_t tens;
    uint8_t ones;

    val = I2C_readRegister(BQ25125_ADDR, BQ25125_VB_REG);

    switch (val >> 5) {
        case 0b00:     // 6x%
            tens = 60;
        case 0b01:     // 7x%
            tens = 70;
        case 0b10:     // 8x%
            tens = 80;
        case 0b11:     // 9x%
            tens = 90;
        default:      // Error not in range
            tens = 255;
    }

    switch (val >> 2 & 0b111) {
        case 0b000:   // Impossible case
            ones = 255;
        case 0b001:   // x0%
            ones = 0;
        case 0b010:   // x2%
            ones = 2;
        case 0b011:   // x4%
            ones = 4;
        case 0b100:   // Impossible case
            ones = 255;
        case 0b101:   // Impossible case
            ones = 255;
        case 0b110:   // x6%
            ones = 6;
        case 0b111:   // x8%
            ones = 8;
        default:      // Error not in range
            ones = 255;
  }

  // If problem
  if (ones == 255 || tens == 255){
    batteryVal = 255;
  }

  batteryVal = tens + ones;
}

/*********************************************************************
 * @fn      I2C_start_ADPD
 *
 * @brief   Start time slots for ADPD4101
 */
static void I2C_start_ADPD(void)
{
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_TS_REG, 0x0401); //Time slots ABCDE enabled and START
}

/*********************************************************************
 * @fn      I2C_clear_FIFO_ADPD
 *
 * @brief   Clear FIFO of ADPD4101, called on interrupt X
 */
static void I2C_clear_FIFO_ADPD(void)
{
    I2C_writeLongRegister(ADPD4101_ADDR, ADPD4101_FIFO_REG, 0x8000); //Clear FIFO
}

/*********************************************************************
 * @fn      I2C_read_ADPD
 *
 * @brief   Read ADPD sensors value, called on interrupt Y
 */
static void I2C_read_ADPD(void)
{
    uint16_t LSB;
    uint16_t MSB;
    
    // Read data bytes for time slot A = PPG G
    LSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_AL_REG); //16LSB bits
    MSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_AH_REG); //16MSB bits
    ppgVals[0] = (MSB << 16) | (LSB);
    
    // Read data bytes for time slot B = PPG R
    LSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_BL_REG); //16LSB bits
    MSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_BH_REG); //16MSB bits
    ppgVals[1] = (MSB << 16) | (LSB);
    
    // Read data bytes for time slot C = PPG IR
    LSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_CL_REG); //16LSB bits
    MSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_CH_REG); //16MSB bits
    ppgVals[2] = (MSB << 16) | (LSB);
    
    // Read data bytes for time slot D = ECG
    LSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_DL_REG); //16LSB bits
    MSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_DH_REG); //16MSB bits
    ecgVal = (MSB << 16) | (LSB);
    
    // Read data bytes for time slot E = ECG Lead Off
    LSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_EL_REG); //16LSB bits
    MSB = I2C_readLongRegister(ADPD4101_ADDR, ADPD4101_SIG_EH_REG); //16MSB bits
    ecgLeadOff = (MSB << 16) | (LSB);
}

/*********************************************************************
 * @fn     I2C_writeRegister
 *
 * @brief  Utility function that writes a register of a slave
 *
 * @param  slave_addr   I2C address of the slave
 * @param  reg          Address of the register
 * @param  val          Value that needs to be written
 */
static void I2C_writeRegister(const uint8_t slave_addr, const uint8_t reg, const uint8_t val)
{
  bool ret;
  I2C_Transaction i2cTransaction;

  /* Common I2C transaction setup */
  txBuffer[0] = reg;
  txBuffer[1] = val;
  i2cTransaction.writeBuf = txBuffer;
  i2cTransaction.writeCount = 2;
  i2cTransaction.readBuf = rxBuffer;
  i2cTransaction.readCount = 0;
  
  /* I2C data transfer */
  i2cTransaction.slaveAddress = slave_addr;
  ret = I2C_transfer(i2c, &i2cTransaction);
  if (!ret) {
      Log_error0("Unsuccessful I2C transfer");
      while(1);
  }
}

/*********************************************************************
 * @fn     I2C_readRegister
 *
 * @brief  Utility function that reads a register of a slave
 *
 * @param  slave_addr   I2C address of the slave
 * @param  reg          Address of the register
 * 
 * @return  Value of the register as a byte
 */
static uint8_t I2C_readRegister(const uint8_t slave_addr, const uint8_t reg)
{
  bool ret;
  I2C_Transaction i2cTransaction;
  
  /* Common I2C transaction setup */
  txBuffer[0] = reg;
  i2cTransaction.writeBuf   = txBuffer;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf    = rxBuffer;
  i2cTransaction.readCount  = 1;

  /* I2C data transfer */
  i2cTransaction.slaveAddress = slave_addr;
  ret = I2C_transfer(i2c, &i2cTransaction);
  if (!ret) {
      Log_error0("Unsuccessful I2C transfer");
      while(1);
  }

  return (rxBuffer[0]);
}

/*********************************************************************
 * @fn     I2C_writeLongRegister
 *
 * @brief  Utility function that writes a long (16 bits) register of a slave
 *
 * @param  slave_addr   I2C address of the slave
 * @param  reg          Address of the register
 * @param  val          Value that needs to be written
 */
static void I2C_writeLongRegister(const uint8_t slave_addr, const uint16_t reg, const uint16_t val)
{
  bool ret;
  I2C_Transaction i2cTransaction;
  
  /* Common I2C transaction setup */
  txBuffer[0] = reg >> 8 + 0x80;
  txBuffer[1] = reg;
  txBuffer[2] = val >> 8;
  txBuffer[3] = val;
  i2cTransaction.writeBuf   = txBuffer;
  i2cTransaction.writeCount = 4;
  i2cTransaction.readBuf    = rxBuffer;
  i2cTransaction.readCount  = 0;

  /* I2C data transfer */
  i2cTransaction.slaveAddress = slave_addr;
  ret = I2C_transfer(i2c, &i2cTransaction);
  if (!ret) {
      Log_error0("Unsuccessful I2C transfer");
      while(1);
  }
}

/*********************************************************************
 * @fn     I2C_readLongRegister
 *
 * @brief  Utility function that reads a long (16 bits) register of a slave
 *
 * @param  slave_addr   I2C address of the slave
 * @param  reg          Address of the register
 * 
 * @return  Value of the register as 2 bytes
 */
static uint16_t I2C_readLongRegister(const uint8_t slave_addr, const uint16_t reg)
{
  bool ret;
  I2C_Transaction i2cTransaction;
  uint16_t value;
  
  /* Common I2C transaction setup */
  txBuffer[0] = reg >> 8 + 0x80;
  txBuffer[1] = reg;
  i2cTransaction.writeBuf   = txBuffer;
  i2cTransaction.writeCount = 2;
  i2cTransaction.readBuf    = rxBuffer;
  i2cTransaction.readCount  = 2;

  /* I2C data transfer */
  i2cTransaction.slaveAddress = slave_addr;
  ret = I2C_transfer(i2c, &i2cTransaction);
  if (!ret) {
      Log_error0("Unsuccessful I2C transfer");
      while(1);
  }

  value = (rxBuffer[0] << 8) | (rxBuffer[1]);
  return value;
}

/*********************************************************************
 * @fn     ProjectZero_enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
static status_t ProjectZero_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    pzMsg_t *pMsg = ICall_malloc(sizeof(pzMsg_t));

    if(pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

        success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return(bleMemAllocError);
}

/*********************************************************************
 * @fn     util_arrtohex
 *
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
char * util_arrtohex(uint8_t const *src, uint8_t src_len,
                     uint8_t *dst, uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if(reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while(src_len && avail > 3)
    {
        if(avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if(src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return((char *)dst);
}

/*********************************************************************
 * @fn     util_getLocalNameStr
 *
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 * @param   len  - Length of advertisment or scan repsonse data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char * util_getLocalNameStr(const uint8_t *data, uint8_t len)
{
    uint8_t nuggetLen = 0;
    uint8_t nuggetType = 0;
    uint8_t advIdx = 0;

    static char localNameStr[32] = { 0 };
    memset(localNameStr, 0, sizeof(localNameStr));

    for(advIdx = 0; advIdx < len; )
    {
        nuggetLen = data[advIdx++];
        nuggetType = data[advIdx];
        if((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
            nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) )
        {
            uint8_t len_temp = nuggetLen < (sizeof(localNameStr)-1)? (nuggetLen - 1):(sizeof(localNameStr)-2);
            // Only copy the first 31 characters, if name bigger than 31.
            memcpy(localNameStr, &data[advIdx + 1], len_temp);
            break;
        }
        else
        {
            advIdx += nuggetLen;
        }
    }

    return(localNameStr);
}

/*********************************************************************
*********************************************************************/
