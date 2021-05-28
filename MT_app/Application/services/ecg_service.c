/**********************************************************************************************
 * Filename:       ecg_service.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "ecg_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// ecg_service Service UUID
CONST uint8_t ecg_serviceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ECG_SERVICE_SERV_UUID), HI_UINT16(ECG_SERVICE_SERV_UUID)
};

// ecgValue UUID
CONST uint8_t ecg_service_EcgValueUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(ECG_SERVICE_ECGVALUE_UUID)
};
// ecgLeadOff UUID
CONST uint8_t ecg_service_EcgLeadOffUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(ECG_SERVICE_ECGLEADOFF_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static ecg_serviceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t ecg_serviceDecl = { ATT_BT_UUID_SIZE, ecg_serviceUUID };

// Characteristic "EcgValue" Properties (for declaration)
static uint8_t ecg_service_EcgValueProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "EcgValue" Value variable
static uint8_t ecg_service_EcgValueVal[ECG_SERVICE_ECGVALUE_LEN] = {0};

// Characteristic "EcgValue" CCCD
static gattCharCfg_t *ecg_service_EcgValueConfig;
// Characteristic "EcgLeadOff" Properties (for declaration)
static uint8_t ecg_service_EcgLeadOffProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "EcgLeadOff" Value variable
static uint8_t ecg_service_EcgLeadOffVal[ECG_SERVICE_ECGLEADOFF_LEN] = {0};

// Characteristic "EcgLeadOff" CCCD
static gattCharCfg_t *ecg_service_EcgLeadOffConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t ecg_serviceAttrTbl[] =
{
  // ecg_service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&ecg_serviceDecl
  },
    // EcgValue Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ecg_service_EcgValueProps
    },
      // EcgValue Characteristic Value
      {
        { ATT_UUID_SIZE, ecg_service_EcgValueUUID },
        GATT_PERMIT_READ,
        0,
        ecg_service_EcgValueVal
      },
      // EcgValue CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&ecg_service_EcgValueConfig
      },
    // EcgLeadOff Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ecg_service_EcgLeadOffProps
    },
      // EcgLeadOff Characteristic Value
      {
        { ATT_UUID_SIZE, ecg_service_EcgLeadOffUUID },
        GATT_PERMIT_READ,
        0,
        ecg_service_EcgLeadOffVal
      },
      // EcgLeadOff CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&ecg_service_EcgLeadOffConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t ecg_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t ecg_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t ecg_serviceCBs =
{
  ecg_service_ReadAttrCB,  // Read callback function pointer
  ecg_service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Ecg_service_AddService- Initializes the Ecg_service service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Ecg_service_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  ecg_service_EcgValueConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( ecg_service_EcgValueConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, ecg_service_EcgValueConfig );
  // Allocate Client Characteristic Configuration table
  ecg_service_EcgLeadOffConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( ecg_service_EcgLeadOffConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, ecg_service_EcgLeadOffConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( ecg_serviceAttrTbl,
                                        GATT_NUM_ATTRS( ecg_serviceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &ecg_serviceCBs );

  return ( status );
}

/*
 * Ecg_service_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Ecg_service_RegisterAppCBs( ecg_serviceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * Ecg_service_SetParameter - Set a Ecg_service parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Ecg_service_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ECG_SERVICE_ECGVALUE_ID:
      if ( len == ECG_SERVICE_ECGVALUE_LEN )
      {
        memcpy(ecg_service_EcgValueVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( ecg_service_EcgValueConfig, (uint8_t *)&ecg_service_EcgValueVal, FALSE,
                                    ecg_serviceAttrTbl, GATT_NUM_ATTRS( ecg_serviceAttrTbl ),
                                    INVALID_TASK_ID,  ecg_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case ECG_SERVICE_ECGLEADOFF_ID:
      if ( len == ECG_SERVICE_ECGLEADOFF_LEN )
      {
        memcpy(ecg_service_EcgLeadOffVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( ecg_service_EcgLeadOffConfig, (uint8_t *)&ecg_service_EcgLeadOffVal, FALSE,
                                    ecg_serviceAttrTbl, GATT_NUM_ATTRS( ecg_serviceAttrTbl ),
                                    INVALID_TASK_ID,  ecg_service_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * Ecg_service_GetParameter - Get a Ecg_service parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Ecg_service_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          ecg_service_ReadAttrCB
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
static bStatus_t ecg_service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the EcgValue Characteristic Value
if ( ! memcmp(pAttr->type.uuid, ecg_service_EcgValueUUID, pAttr->type.len) )
  {
    if ( offset > ECG_SERVICE_ECGVALUE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, ECG_SERVICE_ECGVALUE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the EcgLeadOff Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, ecg_service_EcgLeadOffUUID, pAttr->type.len) )
  {
    if ( offset > ECG_SERVICE_ECGLEADOFF_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, ECG_SERVICE_ECGLEADOFF_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      ecg_service_WriteAttrCB
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
static bStatus_t ecg_service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
    {
      uint16_t svcUuid = ECG_SERVICE_SERV_UUID;
      pAppCBs->pfnChangeCb(connHandle, svcUuid, paramID, len, pValue); // Call app function from stack task context.
    }
  return status;
}
