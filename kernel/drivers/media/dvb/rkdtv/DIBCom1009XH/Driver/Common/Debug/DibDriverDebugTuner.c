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

#include "DibDriverCommon.h"
#include "DibDriverMessages.h"

#include "DibMsgDebug.h"

#define CHIP_ID 0

#define dbgp(args...) fprintf(stderr, args)

static void waitForMessage(struct DibDriverDebugInstance *i, int id)
{
   i->Platform.expectedId = id;
   pthread_cond_wait(&i->Platform.msgResponseCondition, &i->Platform.msgBufferLock);
}

static int regAccess(struct DibDriverDebugInstance *i, uint32_t address, uint32_t value, uint32_t bits, uint8_t rw)
{
   char result[25];
   struct MsgRegisterAccess access = {
      .Head = {
         .MsgSize = GetWords(MsgRegisterAccessBits, 32),
         .ChipId  = CHIP_ID,
         .MsgId   = OUT_MSG_REGISTER_ACCESS,
         .Sender  = 0,
         .Type    = MSG_TYPE_DEBUG,
      },

      .ReadWrite  = rw,
      .Address    = address,
      .Offset     = 0,
      .Bits       = bits,
      .AccessType = bits,
      .Msb        = 0,
      .Lsb        = value,
   };

   pthread_mutex_lock(&i->Platform.msgBufferLock);
   MsgRegisterAccessPackInit(&access, &i->Platform.SerialBuf);
   DibDriverDebugOutMessageCollector(i, i->Platform.Buffer, &access.Head);

   waitForMessage(i, IN_MSG_REGISTER_ACCESS_ACK);

   struct MsgRegisterAccessAck ack;
   MsgRegisterAccessAckUnpackInit(&i->Platform.SerialBuf, &ack);
   if (rw)
      snprintf(result, sizeof(result), "OK");
   else
      snprintf(result, sizeof(result), "%u", ack.Lsb);


   pthread_mutex_unlock(&i->Platform.msgBufferLock);

   DibDriverDebugPlatformInstanceWriteRaw(i, result, strlen(result));
   return 1;
}

static int readI2C(struct DibDriverDebugInstance *i, const char *p1, const char *p2, const char *p3)
{
   int address = strtol(p1, NULL, 10);
   return regAccess(i, 0x10000000 + address, 0, 16, 0);
}

static int writeI2C(struct DibDriverDebugInstance *i, const char *p1, const char *p2, const char *p3)
{
   int address = strtol(p1, NULL, 10);
   int value   = strtol(p2, NULL, 10);
   return regAccess(i, 0x10000000 + address, value, 16, 1);
}

static int read65Nm(struct DibDriverDebugInstance *i, const char *p1, const char *p2, const char *p3)
{
   int bus   = strtol(p1, NULL, 10);
   int address = strtol(p2, NULL, 10);
   return regAccess(i, 0x10000000 + (bus * 500) + address , 0, 32, 0);
}

static int write65Nm(struct DibDriverDebugInstance *i, const char *p1, const char *p2, const char *p3)
{
   int bus     = strtol(p1, NULL, 10);
   int address = strtol(p2, NULL, 10);
   int value   = strtol(p3, NULL, 10);
   return regAccess(i, 0x10000000 + (bus * 500) + address, value, 32, 1);
}

static int getChannel(struct DibDriverDebugInstance *i, const char *p1, const char *p2, const char *p3)
{
//Launch a scan {	//Reinitialize the Channel description....  cd = m_Fe->current_adapter->current_channel; temp=cd->RF_kHz; INIT_CHANNEL(cd,STANDARD_DVBT); cd->RF_kHz=atoi(Par1);; tune_digital(0,cd); //sprintf(d,"CD=%d",cd->RF_kHz); sprintf(d,"OK");

   DibDriverDebugPlatformInstanceWriteRaw(i, "OK", 2);
   return 1;
}

static const struct {
   const char *cmd;
   int (*handle)(struct DibDriverDebugInstance *, const char *, const char *, const char *);
} command[] = {
   { "READI2C",    readI2C },
   { "WRITEI2C",   writeI2C },
   { "READ_65NM",  read65Nm },
   { "WRITE_65NM", write65Nm },
   { "WRITE0WIR",  NULL },

   { "IADC",       NULL }, //sprintf(d,"%f",m.I_adc_power);
   { "QADC",       NULL }, //sprintf(d,"%f",m.Q_adc_power);
   { "BER",        NULL }, //sprintf(d,"%f",m.ber);
   { "SNR",        NULL }, //sprintf(d,"%f",m.CoN);
   { "MPEG",       NULL }, //sprintf(d,"%d",m.locks.fec_mpeg);
   { "QUAD",       NULL }, //sprintf(d,"%f",m.iq_phase_mismatch);
   { "UQMIS",      NULL }, // sprintf(d,"%f",m.iq_gain_mismatch);
   { "PACKET",     NULL }, // if (m.CoN>0) sprintf(d,"%f", (double) m.PacketErrorCount); else sprintf(d,"%f", 1000.); //Force the packet error to be high
   { "CARR",       NULL }, // sprintf(d,"%f", (double) m.locks.fec_frm);
   { "CHAN",       NULL }, //Tune other Channel {   cd = m_Fe->current_adapter->current_channel; cd->RF_kHz=atoi(Par1); tuner_tune(m_Fe->demod,cd); sprintf(d,"OK");
   { "SCAN",       getChannel },
};

/* this is the 'SocketHandler' */
int DibDriverDebugTunerEmulatorAccess(struct DibDriverDebugInstance *instance, char *b, uint32_t size)
{
   const char *Par0, *Par1, *Par2, *Par3;
   char *pos;
   int i;

   dbgp("received '%s' in %d chars: ", b, size);
	Par0=strtok_r(b," ", &pos);
	Par1=strtok_r(NULL," ", &pos);
	Par2=strtok_r(NULL," ", &pos);
	Par3=strtok_r(NULL," ", &pos);

   dbgp("after parsing: '%s' '%s' '%s' '%s'\n", Par0, Par1, Par2, Par3);
   for (i = 0; i < sizeof(command)/sizeof(command[0]); i++) {
      if (strcasecmp(command[i].cmd, Par0) == 0) {
         if (command[i].handle)
            command[i].handle(instance, Par1, Par2, Par3);
         else
            dbgp("unhandled command received: '%s' (with '%s' and '%s' as parameter)\n", Par0, Par1, Par2);

         break;
      }
   }
   if (i == sizeof(command)/sizeof(command[0])) {
      dbgp("unknown command received: '%s' (with '%s' and '%s' as parameter)\n", Par0, Par1, Par2);
      DibDriverDebugPlatformInstanceWriteRaw(instance, "NOK", 3);
   }

   return 1;
}

void DibDriverDebugTunerEmulatorAccessMsgResponse(struct DibDriverDebugInstance *i, const uint32_t *data, uint32_t size)
{
   struct MsgHeader head;

   pthread_mutex_lock(&i->Platform.msgBufferLock);

   memcpy(i->Platform.Buffer, data, size * 4);
   MsgHeaderUnpackInit(&i->Platform.SerialBuf, &head);

   if (head.MsgId == i->Platform.expectedId)
      pthread_cond_signal(&i->Platform.msgResponseCondition);
   else
      dbgp("unexpected message has arrived (ID: %d) -> dropped\n", head.MsgId);

   pthread_mutex_unlock(&i->Platform.msgBufferLock);
}
