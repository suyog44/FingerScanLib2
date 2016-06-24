
#include <iostream>

#include <malloc.h>
#include <string.h>
#include <ctime>

#include <unistd.h>		//UART
#include <fcntl.h>		//UART
#include <termios.h>	//UART

#include "FingerScan.h"

using namespace std;

int hComm;
char bRet;
unsigned char responseBuffer[12];
unsigned char datapacketBuffer[2048];

BYTE	gbyImg8bit[IMG8BIT_SIZE];
BYTE	gbyImgRaw[320*240];
BYTE	gbyTemplate[FP_TEMPLATE_SIZE];
int		gnPassedTime = 0;

static	BYTE	gbyImg256_2[216*240];
static	BYTE	gbyImg256_tmp[240*216];
static  BYTE	gbyImgRaw2[240*320/4];

// firmware date
int year;
int month;
int day;

// function delay() as Arduino
void delay(int millisecs) {
	usleep(millisecs*1000);
}

// function GetTickCount() as VisualC++
long double GetTickCount(){
    return time(0)*1000;
}

FingerScan::FingerScan()
{}

FingerScan::FingerScan(const char* sDevice, int dBaudrate)
{
	status = false;

	//Open in non blocking read/write mode
	hComm = open(sDevice, O_RDWR | O_NOCTTY | O_NDELAY);
	if (hComm == -1)
	{
		cout << "Error - Unable to open UART.\n" << endl;
		return;
	}
	status = true;

	struct termios options;

	tcgetattr(hComm, &options);

	options.c_cflag = B9600|CS8|CLOCAL|CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	tcflush(hComm, TCIFLUSH);
	tcsetattr(hComm, TCSANOW, &options);

	uwDevID = DEVICE_ID;
    CommTimeOut = COMM_DEF_TIMEOUT;
    gwLastAck = 0;
    gwLastAckParam = 0;

    openPort();
}

FingerScan::~FingerScan()
{
	close(hComm);
}

int FingerScan::openPort()
{
	if (executeCmd( CMD_OPEN, 1 ) < 0 )
		return COMM_ERR;

	delay(100);

	BYTE Data[6];

	if (ReceiveData( uwDevID, (BYTE*)&Data, 6, 100) < 0 )
		return COMM_ERR;

	firmwareYear = Data[0]*100+Data[1];
	firmwareMonth = Data[2];
	firmwareDay = Data[3];

	return 0;
}

int FingerScan::cmosLed(BOOL bOn)
{
	return executeCmd( CMD_CMOSLED, bOn ? 1 : 0 );
}

int FingerScan::getEnrollCount()
{
	return executeCmd( CMD_GETENROLLCOUNT, 0 );
}

int FingerScan::checkEnrolled(int nId)
{
	return executeCmd( CMD_CHECKENROLLED, nId );
}

int FingerScan::enroll_start(int nId)
{
	return executeCmd( CMD_ENROLLSTART, nId );
}

int FingerScan::enroll_num(int nTurn)
{
	return executeCmd( CMD_ENROLLSTART+nTurn, 0 );
}

int FingerScan::isPressFinger()
{
	return executeCmd( CMD_ISPRESSFINGER, 0 );
}

int FingerScan::deleteId(int nId)
{
	return executeCmd( CMD_DELETEID, nId );
}

int FingerScan::deleteAll()
{
	return executeCmd( CMD_DELETEALL, 0 );
}

int FingerScan::verify(int nId)
{
	return executeCmd(CMD_VERIFY, nId);
}

int FingerScan::identify()
{
	return executeCmd( CMD_IDENTIFY, 0 );
}

int FingerScan::verifyTemplate(int nId)
{
	if( executeCmd( CMD_VERIFYTEMPLATE, nId ) < 0 )
		return COMM_ERR;

	if(gwLastAck == CMD_ACK)
	{
		if( SendData( uwDevID, &gbyTemplate[0], FP_TEMPLATE_SIZE ) < 0 )
			return COMM_ERR;

		gnPassedTime = GetTickCount();
		if( ReceiveCmd( uwDevID, &gwLastAck, &gwLastAckParam ) < 0 )
			return COMM_ERR;
		gnPassedTime = GetTickCount() - gnPassedTime;
	}

	return 0;
}

int FingerScan::identifyTemplate()
{
	if( executeCmd( CMD_IDENTIFYTEMPLATE, 0 ) < 0 )
		return COMM_ERR;

	if(gwLastAck == CMD_ACK)
	{
		if( SendData(uwDevID, &gbyTemplate[0], FP_TEMPLATE_SIZE ) < 0 )
			return COMM_ERR;

		gnPassedTime = GetTickCount();
		if( ReceiveCmd(uwDevID, &gwLastAck, &gwLastAckParam ) < 0 )
			return COMM_ERR;
		gnPassedTime = GetTickCount() - gnPassedTime;
	}

	return 0;
}

int FingerScan::captureFinger(BOOL bBest)
{
	return executeCmd( CMD_CAPTUREFINGER, bBest );
}

int FingerScan::makeTemplate()
{
	if( executeCmd(CMD_MAKETEMPLATE, 0) < 0 )
		return COMM_ERR;

	if(gwLastAck == CMD_ACK)
	{
		if( ReceiveData( uwDevID, &byTemplate[0], FP_TEMPLATE_SIZE, 3000 ) < 0 )
			return COMM_ERR;
	}

	return 0;
}

int FingerScan::getImage()
{
	if( executeCmd( CMD_GETIMAGE, 0 ) < 0 )
		return COMM_ERR;

	if( ReceiveData( uwDevID, gbyImg256_tmp, sizeof gbyImg256_tmp, 3000 ) < 0 )
		return COMM_ERR;

	// image rotate
	int i, j;

	for( i=0; i<216; i++)
	{
		for( j=0; j<240; j++)
		{
			gbyImg256_2[i*240+j] = gbyImg256_tmp[j*216+i];
		}
	}

	memset(gbyImg8bit, 161, sizeof(gbyImg8bit));

	for (i=0; i<202; i++)
	{
		memcpy(&gbyImg8bit[256*(27 + i) + 0], &gbyImg256_2[i * 258 + 1], 256);
	}

	return 0;
}

int FingerScan::getRawImage()
{
	if( executeCmd( CMD_GETRAWIMAGE, 0 ) < 0 )
		return COMM_ERR;

	if( ReceiveData( uwDevID, gbyImgRaw2, sizeof gbyImgRaw2, 3000 ) < 0 )
		return COMM_ERR;

	/*AVW*/
	memset(gbyImgRaw, 66, sizeof gbyImgRaw);
	int i, j;
	for (i=0; i<120; i++)
	{
		for(j=0; j< 160; j++)
		{
			gbyImgRaw[320*(2*i+0)+(2*j+0)] = gbyImgRaw2[i*160+j];
			gbyImgRaw[320*(2*i+0)+(2*j+1)] = gbyImgRaw2[i*160+j];
			gbyImgRaw[320*(2*i+1)+(2*j+0)] = gbyImgRaw2[i*160+j];
			gbyImgRaw[320*(2*i+1)+(2*j+1)] = gbyImgRaw2[i*160+j];
		}
	}

	return 0;
}

int FingerScan::getTemplate(int nPos)
{
	if( executeCmd(CMD_GETTEMPLATE, nPos) < 0 )
		return COMM_ERR;

	if(gwLastAck == CMD_ACK)
	{
		if( ReceiveData( uwDevID, &byTemplate[0], FP_TEMPLATE_SIZE, 3000 ) < 0 )
			return COMM_ERR;
	}

	return 0;
}

int FingerScan::executeCmd(WORD wCmd, int nCmdParam)
{
	if( SendCmd(uwDevID, wCmd, nCmdParam ) < 0 )
		return COMM_ERR;
    delay(900);
	if( ReceiveCmd(uwDevID, &gwLastAck, &gwLastAckParam ) < 0 )
		return COMM_ERR;
	return 0;
}

int FingerScan::SendCmd( WORD wDevID, WORD wCmdOrAck, int nParam )
{
	CMD_PKT pkt;
	int nSentBytes;

	pkt.Head1 = (BYTE)COMMAND_START_CODE1;
	pkt.Head2 = (BYTE)COMMAND_START_CODE2;
	pkt.wDevId0 = wDevID;
	pkt.wDevId1 = wDevID>>8;
	pkt.nParam0 = nParam & 0xFF;
	pkt.nParam1 = (nParam & 0xFF00) >> 8 ;
	pkt.nParam2 = (nParam & 0xFF0000) >> 16;
	pkt.nParam3 = (nParam & 0xFF000000) >> 24;
	pkt.wCmd0 = wCmdOrAck;
	pkt.wCmd1 = 0x00;
	WORD chksum = CalcChkSumOfCmd( &pkt );
	pkt.wChkSum0 = (BYTE)(chksum % 256);
	pkt.wChkSum1 = (BYTE)(chksum >> 8);

    nSentBytes = write(hComm, (BYTE*)&pkt, PKT_SIZE);
    if( nSentBytes != PKT_SIZE )
		return PKT_COMM_ERR;
	return 0;
}

int FingerScan::ReceiveCmd( WORD wDevID, WORD* pwCmd, int* pnParam )
{
	CMD_PKT pkt;
	int nReceivedBytes;

	if( ( pwCmd == NULL ) ||
		( pnParam == NULL ) )
	{
		return PKT_PARAM_ERR;
	}

	nReceivedBytes = read(hComm, (BYTE*)&pkt, PKT_SIZE);
	if( nReceivedBytes != PKT_SIZE )
		return PKT_COMM_ERR;

	if( ( pkt.Head1 != COMMAND_START_CODE1 ) ||
		( pkt.Head2 != COMMAND_START_CODE2 ) )
		return PKT_HDR_ERR;

	WORD chksum = CalcChkSumOfCmd(&pkt);

	pkt.wChkSum0 = (BYTE)(chksum % 256);
	pkt.wChkSum1 = (BYTE)(chksum >> 8);

	if( (pkt.wChkSum0 !=(BYTE)(chksum % 256)) ||
		(pkt.wChkSum1 !=(BYTE)(chksum >> 8) ))
		return PKT_CHK_SUM_ERR;

	*pwCmd = pkt.wCmd1<<8|pkt.wCmd0;
	*pnParam = pkt.nParam1<<8|pkt.nParam0;

	return 0;
}

int FingerScan::SendData( WORD wDevID, BYTE* pBuf, int nSize )
{
	WORD wChkSum = 0;
	BYTE Buf[4], *pCommBuf;
	int nSentBytes;

	if( pBuf == NULL )
		return PKT_PARAM_ERR;

	Buf[0] = (BYTE)DATA_START_CODE1;
	Buf[1] = (BYTE)DATA_START_CODE2;
	*((WORD*)(&Buf[HEADER_SIZE])) = wDevID;

	wChkSum = CalcChkSumOfDataPkt( Buf, HEADER_SIZE+DEV_ID_SIZE  );
	wChkSum += CalcChkSumOfDataPkt( pBuf, nSize );

	pCommBuf = new BYTE[nSize+HEADER_SIZE+DEV_ID_SIZE+CHK_SUM_SIZE];
	memcpy(pCommBuf, Buf, HEADER_SIZE+DEV_ID_SIZE);
	memcpy(pCommBuf+HEADER_SIZE+DEV_ID_SIZE, pBuf, nSize);
	*(WORD*)(pCommBuf+nSize+HEADER_SIZE+DEV_ID_SIZE) = wChkSum;

	nSentBytes = write(hComm, pCommBuf, nSize+HEADER_SIZE+DEV_ID_SIZE+CHK_SUM_SIZE);
	if( nSentBytes != nSize+HEADER_SIZE+DEV_ID_SIZE+CHK_SUM_SIZE )
	{
		if(pCommBuf)
			delete pCommBuf;
		return PKT_COMM_ERR;
	}

	if(pCommBuf)
		delete pCommBuf;

	return 0;
}

int FingerScan::ReceiveData(WORD wDevID, BYTE* pBuf, int nSize, WORD timeout )
{
	WORD wReceivedChkSum, wChkSum;
	BYTE Buf[4],*pCommBuf;
	int nReceivedBytes;

	if( pBuf == NULL )
		return PKT_PARAM_ERR;

	pCommBuf = new BYTE[nSize+HEADER_SIZE+nSize+CHK_SUM_SIZE];
	nReceivedBytes = read(hComm, pCommBuf, nSize+HEADER_SIZE+DEV_ID_SIZE+CHK_SUM_SIZE);
	if( nReceivedBytes != nSize+HEADER_SIZE+DEV_ID_SIZE+CHK_SUM_SIZE )
	{
		if(pCommBuf)
			delete pCommBuf;
		return PKT_COMM_ERR;
	}
	memcpy(Buf, pCommBuf, HEADER_SIZE+DEV_ID_SIZE);
	memcpy(pBuf, pCommBuf + HEADER_SIZE + DEV_ID_SIZE, nSize);
	wReceivedChkSum = *(WORD*)(pCommBuf+nSize+HEADER_SIZE+DEV_ID_SIZE);
	if(pCommBuf)
		delete pCommBuf;
	////////////// pc end ///////////////

	if( ( Buf[0] != DATA_START_CODE1 ) ||
		( Buf[1] != DATA_START_CODE2 ) )
	{
		return PKT_HDR_ERR;
	}

	wChkSum = CalcChkSumOfDataPkt( Buf, HEADER_SIZE+DEV_ID_SIZE  );
	wChkSum += CalcChkSumOfDataPkt( pBuf, nSize );

	if( wChkSum != wReceivedChkSum )
		return PKT_CHK_SUM_ERR;

	return 0;
}

WORD FingerScan::CalcChkSumOfCmd( CMD_PKT* pPkt )
{
	WORD wChkSum = 0;
	BYTE* pBuf = (BYTE*)pPkt;
	int i;

	for(i=0;i<(PKT_SIZE-CHK_SUM_SIZE);i++)
		wChkSum += pBuf[i];
	return wChkSum;
}

WORD FingerScan::CalcChkSumOfDataPkt( BYTE* pDataPkt, int nSize )
{
	int i;
	WORD wChkSum = 0;
	BYTE* pBuf = (BYTE*)pDataPkt;

	for(i=0;i<nSize;i++)
		wChkSum += pBuf[i];
	return wChkSum;
}

