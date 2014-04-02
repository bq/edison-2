/****************************************************************************
 *
 *      Copyright (c) DiBcom SA.  All rights reserved.
 *
 *      THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *      KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *      IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
 *      PURPOSE.
 *
 ****************************************************************************/

/**************************************************************************************************
* @file "I2c.c"
* @brief Target Specific.
*
***************************************************************************************************/
#include "DibBridgeCommon.h"
#include "DibBridgeTargetDebug.h"
#include "DibBridgeTarget.h"
#include "DibBridgeTargetCommon.h"
#include "DibBridge.h"

#if (DIBINTERF_PHY == DIBINTERF_PHY_I2C)

#if (DIBCOM_TEST_MODE == TEST_MODE_SPP)
#include "SppI2C.h"
#endif

extern struct DibBridgeContext *pLocalContext;
/****************************************************************************
* I2CInit
****************************************************************************/
int32_t I2CInit(void)
{
#if (DIBCOM_TEST_MODE == TEST_MODE_SPP)
   int32_t version;

   if (ExistPort(1)==0)
   {
      printf(CRB "NO PORT " CRA);

      return -1;
   }
   else
   {
      printf(CRB "PORT FIND " CRA);
   }

   return SppInit(1,10,&version);
#elif (DIBCOM_TEST_MODE == TEST_MODE_HOOK)
   DIBSTATUS retval = DIBSTATUS_ERROR;

   retval = HookInit(pLocalContext, 1, pLocalContext->BoardHdl);
#else
   return 0;
#endif
}

/****************************************************************************
* I2CWrite
****************************************************************************/
int32_t I2CWrite(uint32_t dev_addr, uint8_t *txdata, uint32_t txlen, uint32_t byteMode)
{
#if (DIBCOM_TEST_MODE == TEST_MODE_SPP)
   return SppI2CWrite(dev_addr, txdata, txlen );
#elif (DIBCOM_TEST_MODE == TEST_MODE_HOOK)
   return HookI2CWrite(pLocalContext, dev_addr, byteMode, txlen, txdata);
#else
   return 0;
#endif
}
/****************************************************************************
* I2CRead
****************************************************************************/
int32_t I2CRead(uint32_t dev_addr, uint8_t *txdata, uint32_t txlen, uint8_t *rxdata, uint32_t rxlen, uint32_t byteMode )
{
#if (DIBCOM_TEST_MODE == TEST_MODE_SPP)
   return SppI2CWriteRead(dev_addr, txdata, txlen, rxdata, rxlen); 
#elif (DIBCOM_TEST_MODE == TEST_MODE_HOOK)
    return HookI2CRead(pLocalContext, dev_addr, txdata, txlen, rxdata, rxlen, byteMode );
#else
   return 0;
#endif
}
#endif
