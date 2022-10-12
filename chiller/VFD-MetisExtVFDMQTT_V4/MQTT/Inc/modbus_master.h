/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#ifndef __MODBUS_MASTER_H__
#define __MODBUS_MASTER_H__

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#define	INLINE
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal_tim.h"

#define ENTER_CRITICAL_SECTION()	EnterCriticalSection()
#define EXIT_CRITICAL_SECTION()    ExitCriticalSection()

#define FALSE	0
#define TRUE	!FALSE


typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

typedef enum
{
  MB_ENOERR,                  /*!< no error. */
  MB_ENOREG,                  /*!< illegal register address. */
  MB_EINVAL,                  /*!< illegal argument. */
  MB_EPORTERR,                /*!< porting layer error. */
  MB_ENORES,                  /*!< insufficient resources. */
  MB_EIO,                     /*!< I/O error. */
  MB_EILLSTATE,               /*!< protocol stack in illegal state. */
  MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;

typedef enum
{
  MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
  MB_REG_WRITE                /*!< Update register values. */
} eMBRegisterMode;

typedef enum
{
  MB_RTU,                     /*!< RTU transmission mode. */
  MB_ASCII,                   /*!< ASCII transmission mode. */
  MB_TCP                      /*!< TCP mode. */
} eMBMode;

typedef enum
{
  STATE_M_RX_INIT,              /*!< Receiver is in initial state. */
  STATE_M_RX_IDLE,              /*!< Receiver is in idle state. */
  STATE_M_RX_RCV,               /*!< Frame is beeing received. */
  STATE_M_RX_ERROR,              /*!< If the frame is invalid. */
  STATE_M_TX_PROCESS,              /*!< If the frame is invalid. */
} eMBMasterRcvState;

typedef enum
{
  STATE_M_TX_IDLE,              /*!< Transmitter is in idle state. */
  STATE_M_TX_XMIT,              /*!< Transmitter is in transfer state. */
  STATE_M_TX_XFWR,              /*!< Transmitter is in transfer finish and wait receive state. */
} eMBMasterSndState;

typedef enum
{
  EV_READY            = 1<<0,         /*!< Startup finished. */
  EV_FRAME_RECEIVED   = 1<<1,         /*!< Frame received. */
  EV_EXECUTE          = 1<<2,         /*!< Execute function. */
  EV_FRAME_SENT       = 1<<3          /*!< Frame sent. */
} eMBEventType;

typedef enum
{
  EV_MASTER_READY                    = 1<<0,  /*!< Startup finished. */
  EV_MASTER_FRAME_RECEIVED           = 1<<1,  /*!< Frame received. */
  EV_MASTER_EXECUTE                  = 1<<2,  /*!< Execute function. */
  EV_MASTER_FRAME_SENT               = 1<<3,  /*!< Frame sent. */
  EV_MASTER_ERROR_PROCESS            = 1<<4,  /*!< Frame error process. */
  EV_MASTER_PROCESS_SUCESS           = 1<<5,  /*!< Request process success. */
  EV_MASTER_ERROR_RESPOND_TIMEOUT    = 1<<6,  /*!< Request respond timeout. */
  EV_MASTER_ERROR_RECEIVE_DATA       = 1<<7,  /*!< Request receive data error. */
  EV_MASTER_ERROR_EXECUTE_FUNCTION   = 1<<8,  /*!< Request execute function error. */
} eMBMasterEventType;

typedef enum
{
  EV_ERROR_RESPOND_TIMEOUT,         /*!< Slave respond timeout. */
  EV_ERROR_RECEIVE_DATA,            /*!< Receive frame data erroe. */
  EV_ERROR_EXECUTE_FUNCTION,        /*!< Execute function error. */
} eMBMasterErrorEventType;

static enum
{
  STATE_ENABLED,
  STATE_DISABLED,
  STATE_NOT_INITIALIZED,
  STATE_ESTABLISHED,
} eMBState = STATE_NOT_INITIALIZED;

typedef enum
{
    MB_EX_NONE = 0x00,
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;

typedef enum
{
  MB_FC_0 = 0x00,
  MB_FC_1 = 0x01,
  MB_FC_2 = 0x02, 
  MB_FC_3 = 0x03,
  MB_FC_4 = 0x04,
  MB_FC_5 = 0x05,
  MB_FC_6 = 0x06,
  MB_FC_15 = 0x0F,
  MB_FC_16 = 0x10,
  MB_FC_23 = 0x17,
  MB_FC_ER1 = 0x81,
  MB_FC_ER2 = 0x82, 
  MB_FC_ER3 = 0x83,
  MB_FC_ER4 = 0x84,
  MB_FC_ER5 = 0x85,
  MB_FC_ER6 = 0x86,
  MB_FC_ER15 = 0x8F,
  MB_FC_ER16 = 0x90,
  MB_FC_ER23 = 0x97
} eMBFunctionCall;

typedef enum
{
    MB_MRE_NO_ERR,                  /*!< no error. */
    MB_MRE_NO_REG,                  /*!< illegal register address. */
    MB_MRE_ILL_ARG,                 /*!< illegal argument. */
    MB_MRE_REV_DATA,                /*!< receive data error. */
    MB_MRE_TIMEDOUT,                /*!< timeout error occurred. */
    MB_MRE_MASTER_BUSY,             /*!< master is busy now. */
    MB_MRE_EXE_FUN                  /*!< execute function error. */
} eMBMasterReqErrCode;

typedef enum
{
	MB_TMODE_T35,                   /*!< Master receive frame T3.5 timeout. */
	MB_TMODE_RESPOND_TIMEOUT,       /*!< Master wait respond for slave. */
	MB_TMODE_CONVERT_DELAY          /*!< Master sent broadcast ,then delay sometime.*/
}eMBMasterTimerMode;


typedef eMBException( *pxMBFunctionHandler ) ( UCHAR * pucFrame, USHORT * pusLength );
typedef eMBFunctionCall( *rxMBFunctionHandler ) ( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data  );

typedef struct
{
    UCHAR           ucFunctionCode;
    pxMBFunctionHandler pxHandler;
} xMBFunctionHandler;

typedef struct
{
    uint8_t   ucFuncCode;
    rxMBFunctionHandler rxHandler;
} xMBFuncHandler;


#define MB_MASTER_DELAY_MS_CONVERT              200
#define MB_MASTER_TIMEOUT_MS_RESPOND            100
#define MB_MASTER_TOTAL_SLAVE_NUM               255


#define MB_SER_PDU_SIZE_MIN                     4 /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX                     20/*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC                     2 /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF                     0 /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF                      1 /*!< Offset of Modbus-PDU in Ser-PDU. */

#define M_DISCRETE_INPUT_START                  0
#define M_DISCRETE_INPUT_NDISCRETES             16
#define M_COIL_START                            0
#define M_COIL_NCOILS                           64
#define M_REG_INPUT_START                       0
#define M_REG_INPUT_NREGS                       120
#define M_REG_HOLDING_START                     0
#define M_REG_HOLDING_NREGS                     12


#define MB_FUNC_HANDLERS_MAX                    16
#define MB_FUNC_OTHER_REP_SLAVEID_BUF           32



#define MB_ADDRESS_BROADCAST    ( 0 )   /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN          ( 1 )   /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX          ( 247 ) /*! Biggest possible slave address. */
#define MB_FUNC_NONE                            0
#define MB_FUNC_READ_COILS                      1
#define MB_FUNC_READ_DISCRETE_INPUTS            2
#define MB_FUNC_WRITE_SINGLE_COIL               5
#define MB_FUNC_WRITE_MULTIPLE_COILS           15
#define MB_FUNC_READ_HOLDING_REGISTER           3
#define MB_FUNC_READ_INPUT_REGISTER             4
#define MB_FUNC_WRITE_REGISTER                  6
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS       16
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS   23
#define MB_FUNC_DIAG_READ_EXCEPTION             7       
#define MB_FUNC_DIAG_DIAGNOSTIC                 8
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT         11
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG         12
#define MB_FUNC_OTHER_REPORT_SLAVEID           17
#define MB_FUNC_ERROR                          128


#define MB_PDU_SIZE_MAX     20 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN     1   /*!< Function Code */
#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */

#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_DISCCNT_MAX        ( 0x07D0 )



#define MB_PDU_REQ_READ_ADDR_OFF            ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READ_COILCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READ_SIZE                ( 4 )
#define MB_PDU_FUNC_READ_COILCNT_OFF        ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READ_VALUES_OFF         ( MB_PDU_DATA_OFF + 1 )
#define MB_PDU_FUNC_READ_SIZE_MIN           ( 1 )

#define MB_PDU_REQ_WRITE_VALUE_OFF          ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_SIZE               ( 4 )
#define MB_PDU_FUNC_WRITE_ADDR_OFF          ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_WRITE_VALUE_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE              ( 4 )

#define MB_PDU_REQ_WRITE_MUL_ADDR_OFF       ( MB_PDU_DATA_OFF )
#define MB_PDU_REQ_WRITE_MUL_COILCNT_OFF    ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_REQ_WRITE_MUL_VALUES_OFF     ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_REQ_WRITE_MUL_SIZE_MIN       ( 5 )
#define MB_PDU_REQ_WRITE_MUL_COILCNT_MAX    ( 0x07B0 )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE          ( 5 )



#define MB_PDU_REQ_READ_ADDR_OFF            ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READ_DISCCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READ_SIZE                ( 4 )
#define MB_PDU_FUNC_READ_DISCCNT_OFF        ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READ_VALUES_OFF         ( MB_PDU_DATA_OFF + 1 )
#define MB_PDU_FUNC_READ_SIZE_MIN           ( 1 )

#define MB_PDU_REQ_READ_ADDR_OFF                ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READ_REGCNT_OFF              ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READ_SIZE                    ( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX             ( 0x007D )
#define MB_PDU_FUNC_READ_BYTECNT_OFF            ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READ_VALUES_OFF             ( MB_PDU_DATA_OFF + 1 )
#define MB_PDU_FUNC_READ_SIZE_MIN               ( 1 )

#define MB_PDU_REQ_WRITE_ADDR_OFF               ( MB_PDU_DATA_OFF + 0)
#define MB_PDU_REQ_WRITE_VALUE_OFF              ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_SIZE                   ( 4 )
#define MB_PDU_FUNC_WRITE_VALUE_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE                  ( 4 )

#define MB_PDU_REQ_WRITE_MUL_REGCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF        ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_REQ_WRITE_MUL_VALUES_OFF         ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_REQ_WRITE_MUL_SIZE_MIN           ( 5 )
#define MB_PDU_REQ_WRITE_MUL_REGCNT_MAX         ( 0x0078 )
#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF          ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF        ( MB_PDU_DATA_OFF + 2 )

#define MB_PDU_REQ_READWRITE_READ_ADDR_OFF      ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READWRITE_READ_REGCNT_OFF    ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF     ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF   ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_REQ_READWRITE_WRITE_BYTECNT_OFF  ( MB_PDU_DATA_OFF + 8 )
#define MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF   ( MB_PDU_DATA_OFF + 9 )
#define MB_PDU_REQ_READWRITE_SIZE_MIN           ( 9 )
#define MB_PDU_FUNC_READWRITE_READ_BYTECNT_OFF  ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READWRITE_READ_VALUES_OFF   ( MB_PDU_DATA_OFF + 1 )

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF     ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 8 )
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  ( MB_PDU_DATA_OFF + 9 )
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          ( 9 )


/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   16
#define S_COIL_START                  0
#define S_COIL_NCOILS                 64
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             100
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           100


/* Tick per Second */
#define RT_TICK_PER_SECOND	    10000   //0.1ms

#define BITS_UCHAR      8U

/**
 * IPC flags and control command definitions
 */
#define RT_IPC_FLAG_FIFO                0x00            /**< FIFOed IPC. @ref IPC. */
#define RT_IPC_FLAG_PRIO                0x01            /**< PRIOed IPC. @ref IPC. */

#define RT_IPC_CMD_UNKNOWN              0x00            /**< unknown IPC command */
#define RT_IPC_CMD_RESET                0x01            /**< reset IPC object */

#define RT_WAITING_FOREVER              -1              /**< Block forever until get resource. */
#define RT_WAITING_NO                   0               /**< Non-block. */


void
modbus_master_init();

uint16_t get_received_value();
void EnterCriticalSection(void);
void ExitCriticalSection(void);

void fvModbusWriteFrq_ALL(float fFreq);
void fvModbusWriteFrq_pump(uint8_t id ,float fFreq);

void fvModbusWriteMinFrq_All(float fFreq);

void fvModbusWriteMinFrq_pump(uint8_t id ,float fFreq);

void fvModbusWriteMaxFrq_All(float fFreq);
void fvModbusWriteMaxFrq_pump(uint8_t id ,float fFreq);

void fvModbusWriteVfd_pump(uint8_t slave_id ,uint8_t address,uint16_t vfd);
void fvModbusWriteVfd_All(uint16_t vfd);

void fvModbusWriteSlaveId_pump (uint8_t id ,uint16_t slaveid);

uint8_t Get_Cur_Error_Type();
eMBErrorCode    eMBMasterPoll( void );

eMBErrorCode    eMBMasterRTUInit();
void            eMBMasterRTUStart( void );
void            eMBMasterRTUStop( void );
eMBErrorCode    eMBMasterRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength );
eMBErrorCode    eMBMasterRTUSend( UCHAR slaveAddress, const UCHAR * pucFrame, USHORT usLength );
BOOL            xMBMasterRTUReceiveFSM( void );
BOOL            xMBMasterRTUTransmitFSM( void );
BOOL            xMBMasterRTUTimerExpired( void );

eMBMasterErrorEventType eMBMasterGetErrorType( void );

eMBErrorCode    eMBRegisterCB( UCHAR ucFunctionCode, 
                               pxMBFunctionHandler pxHandler );

eMBErrorCode eMBMasterRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs, eMBRegisterMode eMode );

eMBException    eMBFuncReportSlaveID( unsigned char * pucFrame, uint16_t * usLen );

eMBException
eMBMasterFuncReadInputRegister( unsigned char * pucFrame, uint16_t * usLen );

eMBException
eMBMasterFuncReportSlaveID( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncReadInputRegister( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncReadHoldingRegister( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncWriteHoldingRegister( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncWriteMultipleHoldingRegister( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncReadCoils( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncWriteCoil( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncWriteMultipleCoils( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncReadDiscreteInputs( unsigned char * pucFrame, uint16_t * usLen );
eMBException
eMBMasterFuncReadWriteMultipleHoldingRegister( unsigned char * pucFrame, uint16_t * usLen );

eMBErrorCode eMBMasterRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNDiscrete );

eMBMasterReqErrCode
eMBMasterReqReadDiscreteInputs( UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut );

eMBErrorCode eMBMasterRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNCoils, eMBRegisterMode eMode );
eMBErrorCode eMBMasterRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs );

void vMBMasterSetCurTimerMode( eMBMasterTimerMode eMBTimerMode );

eMBMasterReqErrCode
eMBMasterReqWriteCoil( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usCoilData, LONG lTimeOut );

eMBMasterReqErrCode
eMBMasterReqReadCoils( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usNCoils ,LONG lTimeOut );

eMBMasterReqErrCode
eMBMasterReqWriteHoldingRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usRegData, LONG lTimeOut );

eMBMasterReqErrCode
eMBMasterReqReadInputRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut );

eMBMasterReqErrCode
eMBMasterReqWriteMultipleCoils( UCHAR ucSndAddr,
		USHORT usCoilAddr, USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut );
eMBMasterReqErrCode
eMBMasterReqReadWriteMultipleHoldingRegister( UCHAR ucSndAddr,
		USHORT usReadRegAddr, USHORT usNReadRegs, USHORT * pusDataBuffer,
		USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut );
                
eMBMasterReqErrCode
eMBMasterReqReadHoldingRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut );


eMBMasterReqErrCode
eMBMasterReqWriteMultipleHoldingRegister( UCHAR ucSndAddr, USHORT usRegAddr,
		USHORT usNRegs, USHORT * pusDataBuffer, LONG lTimeOut );


bool
Timer5_init(uint16_t timer_35us);

void vMBMasterPortTimersT35Enable();

void vMBMasterPortTimersT35Disable();

void vMBMasterPortTimersDisable();

void vMBMasterPortTimersEnable();

INLINE void vMBMasterPortTimersRespondTimeoutEnable( void );

uint8_t
serial_init();

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable);

void transmitenable(void);

INLINE BOOL     xMBMasterPortSerialGetByte( uint8_t * pucByte );
INLINE BOOL     xMBMasterPortSerialPutByte(CHAR ucByte);



void
modbus_process_init();

eMBFunctionCall
eMBMaster_Function0( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data);

eMBFunctionCall
eMBMaster_Function1( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function2( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function3( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function4( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function5( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function6( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function15( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function16( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);

eMBFunctionCall
eMBMaster_Function23( uint8_t slvId , uint32_t Addr , uint8_t size, uint16_t *data);



bool
xMBMasterPortEventInit( void );

bool
xMBMasterPortEventPost( eMBMasterEventType eEvent );

bool
xMBMasterPortEventGet( eMBMasterEventType * eEvent );

BOOL xMBMasterRunResTake( int32_t time );

void vMBMasterRunResRelease( void );

eMBMasterReqErrCode eMBMasterWaitRequestFinish( void );



void vMBMasterErrorCBRespondTimeout( UCHAR ucDestAddress, const UCHAR* pucPDUData,
                                               USHORT ucPDULength );

void vMBMasterErrorCBReceiveData( UCHAR ucDestAddress, const UCHAR* pucPDUData,
                                            USHORT ucPDULength );

void vMBMasterErrorCBExecuteFunction( UCHAR ucDestAddress, const UCHAR* pucPDUData,
                                                USHORT ucPDULength );

void vMBMasterCBRequestScuuess( void );
void fvReadModbusParamsItem(struct stReceivedModbusData *revModbusData,uint8_t index);

void fvReadRelayModbusParams(struct stRelayModbusdata *rModbusData,uint8_t index);

//void fvReadChillerModbusParams(struct stChillerModbusdata *ModbusData,uint8_t index);
void fvReadChillerHoldParams(struct stChillerholdregister *holdData,uint8_t index);
void fvReadChillerCoilParams(struct stChillerCoildata *coilData,uint8_t index);
#endif