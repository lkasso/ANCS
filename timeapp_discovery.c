/**************************************************************************************************
  Filename:       timeapp_discovery.c
  Revised:        Laura Kassovic @ MbientLab 
  Revision:       1/21/2014

  Description:    Time App service and characteristic discovery routines.

  Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "timeapp.h"
#include "battservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

// Attribute handle cache
uint16 timeAppHdlCache[HDL_CACHE_LEN];

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Attribute handles used during discovery
static uint16 timeAppSvcStartHdl;
static uint16 timeAppSvcEndHdl;
static uint8 timeAppEndHdlIdx;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 TimeAppDiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg );

/*********************************************************************
 * @fn      timeAppDiscStart()
 *
 * @brief   Start service discovery. 
 *
 *
 * @return  New discovery state.
 */
uint8 timeAppDiscStart( void )
{
  // Clear handle cache
  osal_memset( timeAppHdlCache, 0, sizeof(timeAppHdlCache) );
  
  // Start discovery with first service
  return timeAppDiscGattMsg( DISC_ANCS_START, NULL );
}

/*********************************************************************
 * @fn      timeAppDiscGattMsg()
 *
 * @brief   Handle GATT messages for characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
uint8 timeAppDiscGattMsg( uint8 state, gattMsgEvent_t *pMsg )
{
  // Execute discovery function for service
  do
  {
    switch ( state & 0xF0 )
    {
      // Alert notification service
      case DISC_ANCS_START:
        state = TimeAppDiscAlertNtf( state, pMsg );
        if ( state == DISC_FAILED )
        {
          state = DISC_IDLE;
        }
        break;

      default:
        break;
    }
  } while ( (state != 0) && ((state & 0x0F) == 0) );
  
  return state;    
}

/*********************************************************************
 * @fn      TimeAppDiscAlertNtf()
 *
 * @brief   Alert notification service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_ANCS_START:  
      {
        uint8 uuid[ATT_UUID_SIZE] = ANCS_SVC_UUID;

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_ANCS_SVC;
      } 
      break;

    case DISC_ANCS_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_ANCS_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_ANCS_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID128_LEN )
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                           
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case ANCS_NOTIF_CHAR_UUID:
                timeAppHdlCache[HDL_ANCS_NTF_NOTIF_START] = handle;
                timeAppEndHdlIdx = HDL_ANCS_NTF_NOTIF_END;
                break;

              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID128_LEN;
          }
          
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          // If didn't find mandatory characteristic
          if ( timeAppHdlCache[HDL_ANCS_NTF_NOTIF_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_ANCS_NTF_NOTIF_START] <
                    timeAppHdlCache[HDL_ANCS_NTF_NOTIF_END] )
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_ANCS_NTF_NOTIF_START] + 1,
                                   timeAppHdlCache[HDL_ANCS_NTF_NOTIF_END],
                                   timeAppTaskId );
                                        
            newState = DISC_ANCS_CCCD;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;
      
      case DISC_ANCS_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_ANCS_NTF_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
*********************************************************************/
