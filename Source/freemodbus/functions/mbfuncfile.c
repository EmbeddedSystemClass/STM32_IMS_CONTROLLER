/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbfuncinput.c,v 1.10 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
//#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_BYTECNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_FILECNT_MAX        ( 0xA )

#define MB_PDU_FUNC_READ_RSP_BYTECNT_OFF    ( MB_PDU_DATA_OFF )

#define MB_PDU_FUNC_READ_SUBQUERY_LEN		( 7 )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/
#if MB_FUNC_READ_FILE_ENABLED > 0

eMBException
eMBFuncReadFile( UCHAR * pucFrame, USHORT * usLen )
{
	UCHAR			chByteCount;
	UCHAR           usFileCount;
    UCHAR          *pucFrameCur;
    UCHAR i=0;

    xMBReadFileRequest xReadFileRequest[16];

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eFileStatus;

    if( *usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
    	chByteCount = ( USHORT )( pucFrame[MB_PDU_FUNC_READ_BYTECNT_OFF] );
    	usFileCount = chByteCount/MB_PDU_FUNC_READ_SUBQUERY_LEN;

    	for(i=0;i<usFileCount;i++)
    	{
    		xReadFileRequest[i]=*( xMBReadFileRequest* )( &pucFrame[(MB_PDU_FUNC_READ_BYTECNT_OFF+1)+i*MB_PDU_FUNC_READ_SUBQUERY_LEN+1] );
    	}

        if( ( usFileCount >= 1 )&& ( usFileCount < MB_PDU_FUNC_READ_FILECNT_MAX ) )
        {
            pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
            *usLen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pucFrameCur++ = MB_FUNC_READ_FILE;
            *usLen += 1;

            chByteCount=0;
            for(i=0;i<usFileCount;i++)
            {
            	chByteCount+=(xReadFileRequest[i].usRegNum*2);
            }

			*pucFrameCur++ = chByteCount;
			*usLen += 1;

            eFileStatus =eMBFileCB( pucFrameCur, xReadFileRequest, usFileCount );


            if( eFileStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eFileStatus );
            }
            else
            {
                *usLen += usFileCount * 6;
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read file request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
