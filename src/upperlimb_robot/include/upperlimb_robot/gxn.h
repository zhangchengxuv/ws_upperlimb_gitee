#pragma once

#ifdef __linux__
// Linux平台下GT_API的定义
#define GT_API extern "C" short

/*
Windows平台下，部分函数使用了long数据类型的函数参数
在linux平台下，那些函数的long类型参数改为int32_t类型
Linux平台使用系统提供的stdint.h头文件来定义int32_t
*/
#include <stdint.h>

typedef int64_t			__int64;
#else
// Windows平台下GT_API的定义
#define GT_API extern "C" short __stdcall
#endif

/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        标准发布功能函数		                          */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/


/*-----------------------------------------------------------*/
/* Channel of Command                                        */
/*-----------------------------------------------------------*/
#define CHANNEL_HOST                   (0)
#define CHANNEL_UART                   (1)
#define CHANNEL_SIM                    (2)
#define CHANNEL_ETHER                  (3)
#define CHANNEL_RS232                  (4)
#define CHANNEL_PCIE                   (5)
#define CHANNEL_RINGNET                (6)

/*-----------------------------------------------------------*/
/* Error Code                                                */
/*-----------------------------------------------------------*/
#define CMD_SUCCESS                     (0)

#define CMD_ERROR_READ_LEN              (-2)     /* 读取数据长度错误 */
#define CMD_ERROR_READ_CHECKSUM         (-3)     /* 读取数据校验和错误 */

#define CMD_ERROR_WRITE_BLOCK           (-4)     /* 写入数据块错误 */
#define CMD_ERROR_READ_BLOCK            (-5)     /* 读取数据块错误 */

#define CMD_ERROR_OPEN                  (-6)     /* 打开设备错误 */
#define CMD_ERROR_CLOSE                 (-6)     /* 关闭设备错误 */
#define CMD_ERROR_DSP_BUSY              (-7)     /* DSP忙 */

#define CMD_LOCK_ERROR                  (-8)     /* 多线程资源忙 */
#define CMD_DMA_ERROR                   (-9)     /* DMA传输错误 */
#define CMD_COMM_ERROR                  (-10)    /* pcie通讯失败 */
#define CMD_LOAD_RINGNET_DLL_ERROR      (-11)    /* 等环网库加载失败 */
#define CMD_RINGNET_STIME_ERROR         (-12)    /* 等环网库加载失败 */

#define CMD_RINGNET_ENC0_ERROR          (-13)    /* core1编码器初始化失败 */

#define CMD_RINGNET_ENC1_ERROR          (-14)    /* core2编码器初始化失败 */

#define CMD_LOAD_RINGNET_ERROR          (-17)    /* 等环网库API加载失败 */
#define CMD_MCVERSION_MATCH_ERROR       (-15)    /* 等环网匹配失败，需要更新库 */
#define CMD_LOCK_NULL				    (-20)    /* 多线程保护打开句柄失败 */
#define CMD_MCVERSION_MATCH_WARNING     (15)     /* 运控版本匹配失败，某些功能不具备 */
#define CMD_DSPVERSION_MATCH_WARNING    (16)     /* 运控DSP版本较老,不具备版本匹配功能 */

#define CMD_FILE_MATCH_WARNING          (17)     /* 配置文件格式或日期不匹配 */

#define CMD_ERROR_EXECUTE               (1)
#define CMD_ERROR_VERSION_NOT_MATCH     (3)
#define CMD_ERROR_PARAMETER             (7)
#define CMD_ERROR_UNKNOWN               (8)      /* unspported command */

#define MC_NONE                         (-1)

#define MC_LIMIT_POSITIVE               (0)
#define MC_LIMIT_NEGATIVE               (1)
#define MC_ALARM                        (2)
#define MC_HOME                         (3)
#define MC_GPI                          (4)
#define MC_ARRIVE                       (5)
#define MC_EGPI0                        (6)
// #define MC_EGPI1                     (7)
// #define MC_EGPI2                     (8)
#define MC_MPG                          (9)

#define MC_ENABLE                       (10)
#define MC_CLEAR                        (11)
#define MC_GPO                          (12)
// #define MC_EGPO0                     (13)
// #define MC_EGPO1                     (14)
#define MC_AU_ADC						(17)
#define MC_HSO                          (18)
#define MC_AU_DAC						(19)


#define MC_DAC                          (20)
#define MC_STEP                         (21)
#define MC_PULSE                        (22)
#define MC_ENCODER                      (23)
#define MC_ADC                          (24)

#define MC_AU_ENCODER                   (26)

#define MC_ABS_ENCODER                  (29)

#define MC_AXIS                         (30)
#define MC_PROFILE                      (31)
#define MC_CONTROL                      (32)

#define MC_TRIGGER                      (40)

#define MC_AU_TRIGGER			        (44)

#define MC_TERMINAL                     (50)                                   // 从站模块类型资源
#define MC_NET_PORT	                    (51)
#define MC_AU_ENCODER_EX								(52)
#define MC_MPG_ENCODER									(53)
#define MC_SERIAL_NUMBER                (55)                                   // 模块的序列号资源标识
#define MC_SLOTS_NUMBER                 (56)                                   // 模块支持的槽数信息资源标识
#define MC_SUB_TERMINAL                 (57)                                   // 从站模块子板类型资源
#define MC_EXT_MODULE                   (60)
#define MC_EXT_DI                       (61)
#define MC_EXT_DO                       (62)
#define MC_EXT_AI                       (63)
#define MC_EXT_AO                       (64)
#define MC_EHMI_DI                      (65)
#define MC_EHMI_DO                      (66)
#define MC_LOCAL_TERMINAL_NO_RESOURCE		(67)
#define MC_AXIS_CONNECT				   				(68)

#define MC_SCAN_CRD                     (70)
#define MC_LASER		                (71)

#define MC_POS_COMPARE                  (80)

#define MC_WATCH_VAR                    (200)
#define MC_WATCH_EVENT                  (201)

typedef struct
{
	short year;
	short month;
	short day;
	short version;
	short user;
	short reserve1;
	short reserve2;
	short chip;
} TVersion;

#define CORE_MODE_TIMER					(0)
#define CORE_MODE_SYNCH					(1)
#define CORE_MODE_EXTERNAL				(2)

#define CORE_TASK_DEFAULT				(0)
#define CORE_TASK_DLM					(1)

#define SKIP_MODULE_SCAN				(0x001)
#define SKIP_MODULE_POS_COMPARE			(0x002)
#define SKIP_MODULE_CRD					(0x004)

#define SKIP_MODULE_PLC					(0x010)
#define SKIP_MODULE_DLM					(0x020)

#define SKIP_MODULE_AXIS_CALCULATE		(0x100)

#define SKIP_MODULE_WATCH				(0x800)

typedef enum
{
	TIME_ELAPSE_PROFILE = 1000,

	TIME_ELAPSE_INTERRUPT_ERROR_COUNT = 1100,

	TIME_ELAPSE_HOST_COMMAND_EXECUTE = 1220,
	TIME_ELAPSE_ETHER_COMMAND_EXECUTE,

	TIME_ELAPSE_PROFILE_CALCULATE = 6000,

	TIME_ELAPSE_SCAN = 18000,

	TIME_ELAPSE_AXIS_CHECK = 20000,
	TIME_ELAPSE_AXIS_CALCULATE,
	TIME_ELAPSE_AXIS_FILTER,

	TIME_ELAPSE_ENCODER = 30000,

	TIME_ELAPSE_DI = 31000,

	TIME_ELAPSE_DO = 32000,

	TIME_ELAPSE_AI = 33000,

	TIME_ELAPSE_AO = 34000,

	TIME_ELAPSE_TRIGGER = 38000,

	TIME_ELAPSE_CONTROL = 40000,

	TIME_ELAPSE_WATCH = 52000,

	TIME_ELAPSE_TERMINAL = 53000,

	TIME_ELAPSE_TERMINAL_OPERATION = 53001,

	TIME_ELAPSE_READ_ADC = 54000,

} ETimeElapse;

typedef struct Pid
{
	double kp;
	double ki;
	double kd;
	double kvff;
	double kaff;
	int32_t   integralLimit;
	int32_t   derivativeLimit;
	short  limit;
} TPid;

#define RESET_TYPE_NONE			(-1)
#define RESET_TYPE_IO			(0)

typedef struct
{
	short mode;		// mode为1表示保持，为0表示复位到初始状态
}TResetIo;

typedef union
{
	TResetIo resetIo;
	double data[7];
}TResetModePrmUnion;

typedef struct
{
	short axisIndex[2];
	short reserve1[2];
	double radius;
	double center[2];
}TAxisCircularSafetyZone;

GT_API GTN_Open(short channel=5,short param=1);
GT_API GTN_Close(void);

GT_API GTN_OpenRingNet(short channel=5,short param=1,char *pFile=NULL,short index=1,int32_t count=1);
GT_API GTN_RingNetInit(short mode,char *pFile,short index);

GT_API GTN_GetChannel(short *pChannel);
GT_API GTN_GetVersion(short core,char **pVersion);
GT_API GTN_GetVersionEx(short core,short type,TVersion *pVersion);
GT_API GTN_SetVersion(short core,short type,TVersion *pVersion);
GT_API GTN_Reset(short core);
GT_API GTN_SetResetMode(short core,short type,TResetModePrmUnion *pResetPrm);
GT_API GTN_GetClock(short core,uint32_t *pClock,uint32_t *pLoop=NULL);
GT_API GTN_GetClockHighPrecision(short core,uint32_t *pClock);
GT_API GTN_ClearTime(short core,ETimeElapse item);
GT_API GTN_GetTime(short core,ETimeElapse item,uint32_t *pTime,uint32_t *pTimeMax,uint32_t *pValue=NULL);
GT_API GTN_SetCoreMode(short core,short mode);
GT_API GTN_GetCoreMode(short core,short *pMode);
GT_API GTN_SetCoreShare(short core,short type,short index,short count);
GT_API GTN_GetCoreShare(short core,short type,short *pIndex,short *pCount);
GT_API GTN_SetCoreTask(short core,short task);
GT_API GTN_GetCoreTask(short core,short *pTask);
GT_API GTN_GetResMax(short core,short type,short *pCount);
GT_API GTN_SetResCount(short core,short type,short count);
GT_API GTN_GetResCount(short core,short type,short *pCount);
GT_API GTN_SetDeviceShareMax(short core,short count);
GT_API GTN_GetDeviceShareMax(short core,short *pCount);
GT_API GTN_GetChipTemperature(double *pTempValue,double *pMinValue,double *pMaxValue);
GT_API GTN_SetTemperatureEnable(short core,short enable);
GT_API GTN_GetTemperature(short core,double *pTemperature);

GT_API GTN_GetSts(short core,short axis,int32_t *pSts,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetStsEx(short core,short axis,int32_t *pSts,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetPrfSts(short core,short profile,int32_t *pSts,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetDriverSts(short core,short axis,int32_t *pSts,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetDriverPosErr(short core,short station,short axis,int32_t *posError);
GT_API GTN_SetDryRunVel(short core, short crd, double vel, double velG0);
GT_API GTN_GetDryRunVel(short core, short crd, double *pVel, double *pVelG0);
GT_API GTN_SetDryRunMode(short core, short crd, short mode);
GT_API GTN_GetDryRunMode(short core, short crd, short *pMode);
GT_API GTN_SetDrvPrfVel(short core,short axis,int32_t *pPrfVel,short count);
GT_API GTN_GetDrvAtlVel(short core,short axis,int32_t *pPrfVel,short count);
GT_API GTN_ClrSts(short core,short axis,short count=1);
GT_API GTN_AxisOn(short core,short axis);
GT_API GTN_AxisOff(short core,short axis);
GT_API GTN_MultiAxisOn(short core,uint32_t mask);
GT_API GTN_MultiAxisOff(short core,uint32_t mask);
GT_API GTN_SetAxisOnDelayTime(short core,unsigned short delayTime);
GT_API GTN_GetAxisOnDelayTime(short core,unsigned short *pDelayTime);
GT_API GTN_Stop(short core,int32_t mask,int32_t option);
GT_API GTN_SetPrfPos(short core,short profile,int32_t prfPos);
GT_API GTN_SetPrfPosEx(short core,short profile,double pos);
GT_API GTN_SynchAxisPos(short core,int32_t mask);
GT_API GTN_ZeroPos(short core,short axis,short count=1);
GT_API GTN_GetLimitStatus(short core,short axis,short *pLimitPositive,short *pLimitNegative);
GT_API GTN_SetSoftLimitMode(short core,short axis,short mode);
GT_API GTN_GetSoftLimitMode(short core,short axis,short *pMode);
GT_API GTN_SetSoftLimit(short core,short axis,int32_t positive,int32_t negative);
GT_API GTN_GetSoftLimit(short core,short axis,int32_t *pPositive,int32_t *pNegative);
GT_API GTN_SetSoftLimitEx(short core,short axis,double positive,double negative);
GT_API GTN_GetSoftLimitEx(short core,short axis,double *pPositive,double *pNegative);
GT_API GTN_SetSoftLimitTrimMode(short core,short enable);
GT_API GTN_SetAxisBand(short core,short axis,int32_t band,int32_t time);
GT_API GTN_GetAxisBand(short core,short axis,int32_t *pBand,int32_t *pTime);

GT_API GTN_GetPrfPos(short core,short profile,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetPrfVel(short core,short profile,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetPrfAcc(short core,short profile,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetPrfMode(short core,short profile,int32_t *pValue, short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisPrfPos(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisPrfPosCompensate(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisPrfVel(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisPrfAcc(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisEncPos(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisEncVel(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisEncAcc(short core,short axis,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAxisError(short core,short axis,double *pValue, short count=1,uint32_t *pClock=NULL);
GT_API GTN_SetControlFilter(short core,short control,short index);
GT_API GTN_GetControlFilter(short core,short control,short *pIndex);
GT_API GTN_SetControlSuperimposed(short core,short control,short superimposedType,short superimposedIndex);
GT_API GTN_GetControlSuperimposed(short core,short control,short *pSuperimposedType,short *pSuperimposedIndex);
GT_API GTN_SetPid(short core,short control,short index,TPid *pPid);
GT_API GTN_GetPid(short core,short control,short index,TPid *pPid);
GT_API GTN_SetKvffFilter(short core,short control,short index,short kvffFilterExp,double accMax);
GT_API GTN_GetKvffFilter(short core,short control,short index,short *pKvffFilterExp,double *pAccMax);
GT_API GTN_Delay(short core,unsigned short ms);
GT_API GTN_DelayHighPrecision(short core,unsigned short profile);
GT_API GTN_SetLimit(short core,short axis,short type,void *pPrm);
GT_API GTN_GetLimit(short core,short axis,short type,void *pPrm);


#define STEP_DIR                        (0)
#define STEP_PULSE                      (1)
#define STEP_ORTHOGONAL                 (2)

GT_API GTN_LoadConfig(short core,char *pFile);
GT_API GTN_AlarmOn(short core,short axis);
GT_API GTN_AlarmOff(short core,short axis);
GT_API GTN_ClearAlarm(short core,short axis,short count,unsigned short delayTime);

#define MC_LIMIT_POSITIVE               (0)
#define MC_LIMIT_NEGATIVE               (1)
#define LIMIT_MODE_EXTERMAL     				(0)
#define LIMIT_MODE_SOFT         				(1)
#define LIMIT_MODE_ALL          				(-1)

GT_API GTN_LmtsOn(short core,short axis,short limitType=-1);
GT_API GTN_LmtsOnEx(short core,short axis,short limitType,short limitMode);
GT_API GTN_LmtsOff(short core,short axis,short limitType=-1);
GT_API GTN_LmtsOffEx(short core,short axis,short limitType,short limitMode);
GT_API GTN_StepDir(short core,short step);
GT_API GTN_StepPulse(short core,short step);
GT_API GTN_StepOrthogonal(short core,short step);
GT_API GTN_SetMtrBias(short core,short dac,short bias);
GT_API GTN_GetMtrBias(short core,short dac,short *pBias);
GT_API GTN_SetMtrLmt(short core,short dac,short limit);
GT_API GTN_GetMtrLmt(short core,short dac,short *pLimit);
GT_API GTN_SetSense(short core,short dataType,short dataIndex,short value);
GT_API GTN_GetSense(short core,short dataType,short dataIndex,short *pValue);
GT_API GTN_EncOn(short core,short encoder);
GT_API GTN_EncOff(short core,short encoder);
GT_API GTN_SetPosErr(short core,short control,int32_t error);
GT_API GTN_GetPosErr(short core,short control,int32_t *pError);
GT_API GTN_SetAxisPosErrLimit(short core,short axis,int32_t errorLimit);
GT_API GTN_GetAxisPosErrLimit(short core,short axis,int32_t *pErrorLimit);
GT_API GTN_SetStopDec(short core,short profile,double decSmoothStop,double decAbruptStop);
GT_API GTN_GetStopDec(short core,short profile,double *pDecSmoothStop,double *pDecAbruptStop);
GT_API GTN_CtrlMode(short core,short axis,short mode);
GT_API GTN_SetStopIo(short core,short axis,short stopType,short inputType,short inputIndex);
GT_API GTN_SetProbeCaptureStopAxis(short core,short encoder,short stopAxis,short stopType,short enable);
GT_API GTN_SetAxisPrfVelFilter(short core,short axis,short filterNumExp);
GT_API GTN_GetAxisPrfVelFilter(short core,short axis,short *pFilterNumExp);
GT_API GTN_SetAxisEncVelFilter(short core,short axis,short filterNumExp);
GT_API GTN_GetAxisEncVelFilter(short core,short axis,short *pFilterNumExp);

GT_API GTN_SetAxisFilterArrivalBand(short core,short axis,double band,int32_t time);
GT_API GTN_GetAxisFilterArrivalBand(short core,short axis,double *pBand,int32_t *pTime);
GT_API GTN_SetProfileScale(short core,short i,int32_t alpha,int32_t beta);
GT_API GTN_GetProfileScale(short core,short i,int32_t *pAlhpa,int32_t *pBeta);
GT_API GTN_SetEncoderScale(short core,short i,int32_t alpha,int32_t beta);
GT_API GTN_GetEncoderScale(short core,short i,int32_t *pAlhpa,int32_t *pBeta);
GT_API GTN_SetAuEncoderScale(short core,short i,int32_t alpha,int32_t beta);
GT_API GTN_GetAuEncoderScale(short core,short i,int32_t *pAlhpa,int32_t *pBeta);
GT_API GTN_SetEncoderMapRelation(short core,short masterEncType,short masterEncIndex,short mapEncType,short mapEncIndex);
GT_API GTN_GetEncoderMapRelation(short core,short masterEncType,short masterEncIndex,short *pMapEncType,short *pMapEncIndex);


/*-----------------------------------------------------------*/
/* Capture and Triggr                                        */
/*-----------------------------------------------------------*/
#define CAPTURE_HOME                    (1)
#define CAPTURE_INDEX                   (2)
#define CAPTURE_PROBE                   (3)
#define CAPTURE_HSIO0                   (6)
#define CAPTURE_HSIO1                   (7)
#define CAPTURE_POSCOMPARE              (6)

typedef struct Trigger
{
	short encoder;
	short probeType;
	short probeIndex;
	short sense;
	int32_t  offset;
	uint32_t loop;
	short windowOnly;
	int32_t firstPosition;
	int32_t lastPosition;
}TTrigger;

typedef struct TriggerEx
{
	short latchType;
	short latchIndex;
	short probeType;
	short probeIndex;
	short sense;
	int32_t  offset;
	uint32_t loop;
	short windowOnly;
	int32_t firstPosition;
	int32_t lastPosition;
}TTriggerEx;

typedef struct TriggerAlign
{
	short encoder;
	short probeType;
	short probeIndex;
	short sense;
	int32_t offset;
	uint32_t loop;
	short windowOnly;
	short pad2;
	int32_t firstPosition;
	int32_t lastPosition;
}TTriggerAlign;

typedef struct TriggerStatus
{
	short execute;
	short done;
	int32_t position;
}TTriggerStatus;

typedef struct TriggerStatusEx
{
	short execute;
	short done;
	int32_t position;
	uint32_t clock;
	uint32_t loopCount;
}TTriggerStatusEx;

typedef struct TriggerPrm
{
	short latchType;
	short latchIndex;
	short probeType;
	short probeIndex;
	short sense;
	short loopType;
	int32_t  offset;
	uint32_t loop;
	short windowOnly;
	short pad1;
	int32_t firstPosition;
	int32_t lastPosition;
	short fifoMode;
	short pad2[3];
	double pad3;
}TTriggerPrm;

typedef struct
{
	short fifoFull;
	short pad1[3];
	double pad2[2];
}TLatchValueInfo;

GT_API GTN_SetAuTrigger(short core,short index,TTriggerEx *pTrigger);
GT_API GTN_GetAuTrigger(short core,short index,TTriggerEx *pTrigger);
GT_API GTN_ClearAuTriggerStatus(short core,short index);
GT_API GTN_GetAuTriggerStatus(short core,short index,TTriggerStatusEx *pTriggerStatusEx,short count=1);

GT_API GTN_SetTriggerPrm(short core,short index,TTriggerPrm *pTriggerPrm);
GT_API GTN_GetTriggerPrm(short core,short index,TTriggerPrm *pTriggerPrm);
GT_API GTN_GetTriggerLatchValue(short core,short index,int32_t count,int32_t *pValue,int32_t *pCount,TLatchValueInfo *pInfo);

GT_API GTN_SetTrigger(short core,short index,TTrigger *pTrigger);
GT_API GTN_GetTrigger(short core,short index,TTrigger *pTrigger);
GT_API GTN_SetTriggerEx(short core,short index,TTriggerEx *pTrigger);
GT_API GTN_GetTriggerEx(short core,short index,TTriggerEx *pTrigger);
GT_API GTN_GetTriggerStatus(short core,short index,TTriggerStatus *pTriggerStatus,short count=1);
GT_API GTN_GetTriggerStatusEx(short core,short index,TTriggerStatusEx *pTriggerStatusEx,short count=1);
GT_API GTN_ClearTriggerStatus(short core,short index);
GT_API GTN_DisableTrigger(short core,short index);


#define TRIGGER_DELTA_MODE_DEFAULT            (0)

#define TRIGGER_DELTA_CHECKPOINT_MODE_AUTO    (0)
#define TRIGGER_DELTA_CHECKPOINT_MODE_MANUAL  (1)

typedef struct Checkpoint
{
	short mode;
	int32_t offset;
	short fifoIndex;
	uint32_t crossCount;
	short fifoDataCount;
	short dataReady;
	int32_t data;
	short dataIndex;
} TCheckpoint;

typedef struct TriggerDeltaPrm
{
	short mode;
	short dir;
	short triggerIndex[2];
} TTriggerDeltaPrm;

typedef struct TriggerDelta
{
	short enable;
	short checkpointCount;
	short fifoDataCount;
	short lostCount;
}TTriggerDeltaInfo;

GT_API GTN_ClearTriggerDelta(short core,short index,short mode=0);
GT_API GTN_AddTriggerDeltaCheckpoint(short core,short index,short mode,int32_t offset,short fifo=0,short *pIndex=NULL);
GT_API GTN_ReadTriggerDeltaCheckpointData(short core,short index,short checkpointIndex,int32_t *pBuf,short count,short *pReadCount);
GT_API GTN_WriteTriggerDeltaCheckpointData(short core,short index,short checkpointIndex,int32_t *pBuf,short count,short *pWriteCount);
GT_API GTN_SetTriggerDeltaPrm(short core,short index,TTriggerDeltaPrm *pPrm);
GT_API GTN_GetTriggerDeltaPrm(short core,short index,TTriggerDeltaPrm *pPrm);
GT_API GTN_GetTriggerDeltaCheckpoint(short core,short index,short checkpointIndex,TCheckpoint *pCheckpoint);
GT_API GTN_GetTriggerDeltaInfo(short core,short index,TTriggerDeltaInfo *pTriggerDelta);
GT_API GTN_TriggerDeltaOn(short core,short index);
GT_API GTN_TriggerDeltaOff(short core,short index);

GT_API GTN_SetCaptureMode(short core,short encoder,short mode);
GT_API GTN_GetCaptureMode(short core,short encoder,short *pMode,short count=1);
GT_API GTN_GetCaptureStatus(short core,short encoder,short *pStatus,int32_t *pValue,short count,uint32_t *pClock);
GT_API GTN_SetCaptureSense(short core,short encoder,short mode,short sense);
GT_API GTN_ClearCaptureStatus(short core,short encoder);
GT_API GTN_SetCaptureRepeat(short core,short encoder,short count);
GT_API GTN_GetCaptureRepeatStatus(short core,short encoder,short *pCount);
GT_API GTN_GetCaptureRepeatPos(short core,short encoder,int32_t *pValue,short startNum,short count);
GT_API GTN_SetCaptureRepeatFifoMode(short core,short encoder,short mode);
GT_API GTN_GetCaptureRepeatFifoMode(short core,short encoder,short *pMode);

GT_API GTN_LinkCaptureOffset(short core,short encoder,short source);
GT_API GTN_SetCaptureOffset(short core,short encoder,int32_t *pOffset,short count,int32_t loop);
GT_API GTN_GetCaptureOffset(short core,short encoder,int32_t *pOffset,short *pCount,int32_t *pLoop);
GT_API GTN_GetCaptureOffsetStatus(short core,short encoder,short *pCount,int32_t *pLoop,int32_t *pCapturePos);

GT_API GTN_AutoCaptureOn(short core,short encoder);
GT_API GTN_AutoCaptureOff(short core,short encoder);

/*-----------------------------------------------------------*/
/* Basic Motion                                              */
/*-----------------------------------------------------------*/
typedef struct TrapPrm
{
	double acc;
	double dec;
	double velStart;
	short  smoothTime;
} TTrapPrm;

typedef struct TTrapTime
{
	double totalTime;
	double remainderTime;
	double pad[2];
} TTrapTime;

GT_API GTN_Update(short core,int32_t mask);
GT_API GTN_SetPos(short core,short profile,int32_t pos);
GT_API GTN_GetPos(short core,short profile,int32_t *pPos);
GT_API GTN_SetVel(short core,short profile,double vel);
GT_API GTN_GetVel(short core,short profile,double *pVel);
GT_API GTN_SetPosArray(short core,short profile,double *pPos,short count);
GT_API GTN_GetPosArray(short core,short profile,double *pPos,short count);

GT_API GTN_PrfTrap(short core,short profile);
GT_API GTN_SetTrapPrm(short core,short profile,TTrapPrm *pPrm);
GT_API GTN_GetTrapPrm(short core,short profile,TTrapPrm *pPrm);
GT_API GTN_GetTrapTime(short core,short profile,TTrapTime *pTime);
GT_API GTN_GetTrapSts(short core,short profile,short *prfsts);
GT_API GTN_ClearTrapSts(short core,short profile);

GT_API GTN_SetTrapOverride(short core,short profile,double velRatio);
GT_API GTN_GetTrapOverride(short core,short profile,double *pVelRatio);

typedef struct JogPrm
{
	double acc;
	double dec;
	double smooth;
} TJogPrm;

GT_API GTN_PrfJog(short core,short profile);
GT_API GTN_SetJogPrm(short core,short profile,TJogPrm *pPrm);
GT_API GTN_GetJogPrm(short core,short profile,TJogPrm *pPrm);

#define PT_MODE_STATIC                  (0)
#define PT_MODE_DYNAMIC                 (1)

#define PT_SEGMENT_NORMAL               (0)
#define PT_SEGMENT_EVEN                 (1)
#define PT_SEGMENT_STOP                 (2)

typedef struct
{
	double prfPos;
	int32_t loop;
	short mode;
	short fifoUse;
	short fifoPlace;
	short segmentNumber;
	uint32_t segmentReceive[2];
	uint32_t segmentExecute[2];
	uint32_t bufferReceive[2];
	uint32_t bufferExecute[2];
} TPtInfo;

//GT_API GTN_PosCurrFeedForward(short core,short profile,double pos,int32_t time,short torque,short type,short fifo=0);

GT_API GTN_PrfPt(short core,short profile,short mode=PT_MODE_STATIC);
GT_API GTN_SetPtLoop(short core,short profile,int32_t loop);
GT_API GTN_GetPtLoop(short core,short profile,int32_t *pLoop);
GT_API GTN_PtSpace(short core,short profile,short *pSpace,short fifo=0);
GT_API GTN_PtSpaceEx(short core,short profile,short *pSpace,short *pListSpace,short fifo=0);
GT_API GTN_PtData(short core,short profile,double pos,int32_t time,short type=PT_SEGMENT_NORMAL,short fifo=0);
GT_API GTN_PtClear(short core,short profile,short fifo=0);
GT_API GTN_PtStart(short core,int32_t mask,int32_t option=0);
GT_API GTN_SetPtMemory(short core,short profile,short memory);
GT_API GTN_GetPtMemory(short core,short profile,short *pMemory);
GT_API GTN_SetPtPrecisionMode(short core,short profile,short precisionMode);
GT_API GTN_GetPtPrecisionMode(short core,short profile,short *pPrecisionMode);
GT_API GTN_GetPtInfo(short core,short profile,TPtInfo *pPtInfo);
GT_API GTN_SetPtLink(short core,short profile,short fifo,short list);
GT_API GTN_GetPtLink(short core,short profile,short fifo,short *pList);
GT_API GTN_PtDoBit(short core,short profile,short doType,short index,short value,short fifo=0);
GT_API GTN_PtAo(short core,short profile,short aoType,short index,double value,short fifo=0);
GT_API GTN_PtSpaceDmaType(short core,short profile,short mode);
GT_API GTN_PtSpaceArray(short core,short profile,short *pSpace,short fifo,short count);
GT_API GTN_GetPtRemainder(short core,short profile,int32_t *pRemainder);
GT_API GTN_PtDataArray(short core,short profile,double *pos,int32_t time,short *type,short fifo,short count);
GT_API GTN_PtSmoothBegin(short core,short profile,int32_t deltaTime,int32_t degree,short fifo);
GT_API GTN_PtSmoothEnd(short core,short profile,short fifo);


typedef struct TPvtTableMovePrm
{
	short tableId;
	int32_t distance;
	double vm;
	double am;
	double jm;
	double time;
} TPvtTableMovePrm;

GT_API GTN_PrfPvt(short core,short profile);
GT_API GTN_SetPvtLoop(short core,short profile,int32_t loop);
GT_API GTN_GetPvtLoop(short core,short profile,int32_t *pLoopCount,int32_t *pLoop);
GT_API GTN_PvtStatus(short core,short profile,short *pTableId,double *pTime,short count=1);
GT_API GTN_PvtStart(short core,int32_t mask);
GT_API GTN_PvtTableSelect(short core,short profile,short tableId);

GT_API GTN_PvtTable(short core,short tableId,int32_t count,double *pTime,double *pPos,double *pVel);
GT_API GTN_PvtTableEx(short core,short tableId,int32_t count,double *pTime,double *pPos,double *pVelBegin,double *pVelEnd);
GT_API GTN_PvtTableComplete(short core,short tableId,int32_t count,double *pTime,double *pPos,double *pA,double *pB,double *pC,double velBegin=0,double velEnd=0);
GT_API GTN_PvtTablePercent(short core,short tableId,int32_t count,double *pTime,double *pPos,double *pPercent,double velBegin=0);
GT_API GTN_PvtPercentCalculate(short core,int32_t n,double *pTime,double *pPos,double *pPercent,double velBegin,double *pVel);
GT_API GTN_PvtTableContinuous(short core,short tableId,int32_t count,double *pPos,double *pVel,double *pPercent,double *pVelMax,double *pAcc,double *pDec,double timeBegin);
GT_API GTN_PvtContinuousCalculate(short core,int32_t n,double *pPos,double *pVel,double *pPercent,double *pVelMax,double *pAcc,double *pDec,double *pTime);

GT_API GTN_PvtTableMove(short core,short tableId,int32_t distance,double vm,double am,double jm,double *pTime=NULL);
GT_API GTN_PvtTableMoveCalculate(short core,int32_t distance,double vm,double am,double *pVm,double *pJm);
GT_API GTN_PvtTableMove2(short core,short tableId,int32_t distance,double vm,double am,double jm,double *pTime=NULL);
GT_API GTN_PvtTableMove2Calculate(short core,int32_t distance,double vm,double am,double *pVm,double *pJm);
GT_API GTN_PvtTableMovePercent(short core,short tableId,int32_t distance,double vm,
							   double acc,double pa1,double pa2,
							   double dec,double pd1,double pd2,
							   double *pVel=NULL,double *pAcc=NULL,double *pDec=NULL,double *pTime=NULL);
GT_API GTN_PvtTableMovePercentEx(short core,short tableId,int32_t distance,double vm,
								 double acc,double pa1,double pa2,double ma,
								 double dec,double pd1,double pd2,double md,
								 double *pVel=NULL,double *pAcc=NULL,double *pDec=NULL,double *pTime=NULL);

GT_API GTN_PvtTableMoveTogether(short core,short tableCount,TPvtTableMovePrm *pPvtTableMovePrm);

GT_API GTN_SetPvtTableMoveAccList(short core,short tableId,short count,int32_t *pDistance,double *pAccMax);
GT_API GTN_SetPvtTableLoop(short core,short profile,short tableId,int32_t loop,short nextTableId);
GT_API GTN_GetPvtTableLoop(short core,short profile,short tableId,int32_t *pLoopCount,int32_t *pLoop,short *pNextTableId);

GT_API GTN_PvtTableMoveAbsolute(short core,short profile,double *pDistance,double *pVm,
	double *pAcc,double *pPa1,double *pPa2,
	double *pDec,double *pPd1,double *pPd2,
	double *pVelM,double *pAccM,double *pDecM,double *pTime,short count);


#define GEAR_MASTER_ENCODER             (1)
#define GEAR_MASTER_PROFILE             (2)
#define GEAR_MASTER_AXIS                (3)
#define GEAR_MASTER_AU_ENCODER          (4)
#define GEAR_MASTER_MPG_ENCODER					(5)

#define GEAR_MASTER_ENCODER_OTHER       (101)
#define GEAR_MASTER_AXIS_OTHER          (103)
#define GEAR_MASTER_AU_ENCODER_OTHER		(104)
#define GEAR_MASTER_MPG_ENCODER_OTHER		(105)

#define GEAR_EVENT_START                (1)
#define GEAR_EVENT_PASS                 (2)
#define GEAR_EVENT_AREA                 (5)

GT_API GTN_PrfGear(short core,short profile,short dir=0);
GT_API GTN_SetGearMaster(short core,short profile,short masterIndex,short masterType=GEAR_MASTER_PROFILE,short masterItem=0);
GT_API GTN_GetGearMaster(short core,short profile,short *pMasterIndex,short *pMasterType=NULL,short *pMasterItem=NULL);
GT_API GTN_SetGearRatio(short core,short profile,int32_t masterEven,int32_t slaveEven,int32_t masterSlope=0);
GT_API GTN_GetGearRatio(short core,short profile,int32_t *pMasterEven,int32_t *pSlaveEven,int32_t *pMasterSlope=NULL);
GT_API GTN_GearStart(short core,int32_t mask);
GT_API GTN_SetGearEvent(short core,short profile,short event,int32_t startPara0,int32_t startPara1);
GT_API GTN_GetGearEvent(short core,short profile,short *pEvent,int32_t *pStartPara0,int32_t *pStartPara1);

#define FOLLOW_SWITCH_SEGMENT			(1)
#define FOLLOW_SWITCH_TABLE				(2)

#define FOLLOW_MASTER_ENCODER           (1)
#define FOLLOW_MASTER_PROFILE           (2)
#define FOLLOW_MASTER_AXIS              (3)
#define FOLLOW_MASTER_AU_ENCODER        (4)

#define FOLLOW_MASTER_ENCODER_OTHER     (101)
#define FOLLOW_MASTER_AXIS_OTHER        (103)

#define FOLLOW_EVENT_START              (1)
#define FOLLOW_EVENT_PASS               (2)

#define FOLLOW_SEGMENT_NORMAL           (0)
#define FOLLOW_SEGMENT_EVEN             (1)
#define FOLLOW_SEGMENT_STOP             (2)
#define FOLLOW_SEGMENT_CONTINUE         (3)

typedef struct
{
	short fifoNum;
	short switchStatus;
	short segNum;
	short pad1;
	double masterPosDelta;
	double slavePosDelta;
	double virtualMasterError;
	double pad2;
	double pad3;
} TFollowInfo;

GT_API GTN_PrfFollow(short core,short profile,short dir=0);
GT_API GTN_SetFollowMaster(short core,short profile,short masterIndex,short masterType=FOLLOW_MASTER_PROFILE,short masterItem=0);
GT_API GTN_GetFollowMaster(short core,short profile,short *pMasterIndex,short *pMasterType=NULL,short *pMasterItem=NULL);
GT_API GTN_SetFollowLoop(short core,short profile,int32_t loop);
GT_API GTN_GetFollowLoop(short core,short profile,int32_t *pLoop);
GT_API GTN_SetFollowEvent(short core,short profile,short event,short masterDir,int32_t pos=0);
GT_API GTN_GetFollowEvent(short core,short profile,short *pEvent,short *pMasterDir,int32_t *pPos=NULL);
GT_API GTN_FollowSpace(short core,short profile,short *pSpace,short fifo=0);
GT_API GTN_FollowData(short core,short profile,int32_t masterSegment,double slaveSegment,short type=FOLLOW_SEGMENT_NORMAL,short fifo=0);
GT_API GTN_FollowClear(short core,short profile,short fifo=0);
GT_API GTN_FollowStart(short core,int32_t mask,int32_t option=0);
GT_API GTN_FollowSwitch(short core,int32_t mask);
GT_API GTN_SetFollowMemory(short core,short profile,short memory);
GT_API GTN_GetFollowMemory(short core,short profile,short *pMemory);
GT_API GTN_GetFollowStatus(short core,short profile,short *pFifoNum,short *pSwitchStatus);
GT_API GTN_SetFollowPhasing(short core,short profile,short profilePhasing);
GT_API GTN_GetFollowPhasing(short core,short profile,short *pProfilePhasing);
GT_API GTN_GetFollowInfo(short core,short profile,TFollowInfo *pFollowInfo);
GT_API GTN_SetFollowVirtualSeg(short core,short profile,short seg,short axis,short fifo);
GT_API GTN_GetFollowVirtualSeg(short core,short profile,short *pSegment,short *pAxis,short fifo);
GT_API GTN_GetFollowVirtualErr(short core,short profile,double *pVirtualErr);
GT_API GTN_ClearFollowVirtualErr(short core,short profile);


GT_API GTN_PrfFollowEx(short core,short profile,short dir=0);
GT_API GTN_SetFollowMasterEx(short core,short profile,short masterIndex,short masterType=FOLLOW_MASTER_PROFILE,short masterItem=0);
GT_API GTN_GetFollowMasterEx(short core,short profile,short *pMasterIndex,short *pMasterType=NULL,short *pMasterItem=NULL);
GT_API GTN_SetFollowLoopEx(short core,short profile,int32_t loop);
GT_API GTN_GetFollowLoopEx(short core,short profile,int32_t *pLoop);
GT_API GTN_SetFollowEventEx(short core,short profile,short event,short masterDir,int32_t pos=0);
GT_API GTN_GetFollowEventEx(short core,short profile,short *pEvent,short *pMasterDir,int32_t *pPos=NULL);
GT_API GTN_FollowSpaceEx(short core,short profile,short *pSpace,short fifo=0);
GT_API GTN_FollowDataPercentEx(short core,short profile,double masterSegment,double slaveSegment,short type=FOLLOW_SEGMENT_NORMAL,short percent=0,short fifo=0);
GT_API GTN_FollowClearEx(short core,short profile,short fifo=0);
GT_API GTN_FollowStartEx(short core,int32_t mask,int32_t option=0);
GT_API GTN_FollowSwitchEx(short core,int32_t mask);
GT_API GTN_SetFollowMemoryEx(short core,short profile,short memory);
GT_API GTN_GetFollowMemoryEx(short core,short profile,short *pMemory);
GT_API GTN_GetFollowStatusEx(short core,short profile,short *pFifoNum,short *pSwitchStatus);
GT_API GTN_SetFollowPhasingEx(short core,short profile,short profilePhasing);
GT_API GTN_GetFollowPhasingEx(short core,short profile,short *pProfilePhasing);
GT_API GTN_FollowSwitchNowEx(short core,short profile,short method,short buffer=0,short fifo=0);
GT_API GTN_FollowDataPercent2Ex(short core,short profile,double masterSegment,double slaveSegment,double velBeginRatio,double velEndRatio,short percent=100,short *pPercent1=NULL,short fifo=0);
GT_API GTN_GetFollowDataPercent2Ex(short core,double masterPos,double v1,double v2,double p,double p1,double *pSlavePos);
GT_API GTN_FollowDoBitEx(short core,short profile,short doType,short index,short value,short fifo=0);
GT_API GTN_FollowDelayEx(short core,short profile,uint32_t delayTime,short fifo=0);
GT_API GTN_FollowDiBitEx(short core,short profile,short diType,short index,short value,uint32_t time=0,short fifo=0);

GT_API GTN_SetAxisFollowErrorMode(short core,short axis,short followErrorMode);
GT_API GTN_GetAxisFollowErrorMode(short core,short axis,short *pFollowErrorMode);

GT_API GTN_SetFollowRegist(short core,short profile,short segment,short fifo);
GT_API GTN_GetFollowRegist(short core,short profile,short *pSegment,short fifo);
GT_API GTN_GetFollowRegistCount(short core,short profile,uint32_t *pCount);

typedef struct
{
	int32_t pos;
	double vel;
	double acc;
	double dec;
	short percent;
} TMoveAbsolutePrm;

typedef struct
{
	int32_t pos;
	double vel;
	double acc;
	double dec;
	short percent;
	double velStart;
	double velEnd;
	short accStartPercent;
	short decEndPercent;
} TMoveAbsolutePrmEx;

GT_API GTN_MoveAbsolute(short core,short profile,TMoveAbsolutePrm *pPrm);
GT_API GTN_GetMoveAbsolute(short core,short profile,TMoveAbsolutePrm *pPrm);

GT_API GTN_MoveAbsoluteEx(short core,short profile,TMoveAbsolutePrmEx *pPrm);
GT_API GTN_GetMoveAbsoluteEx(short core,short profile,TMoveAbsolutePrmEx *pPrm);

typedef struct
{
	double vel;
	double acc;
	double dec;
	double jerkBegin;
	double jerkEnd;
	short direction;
} TMoveVelocityPrm;

GT_API GTN_MoveVelocity(short core,short profile,TMoveVelocityPrm *pPrm);
GT_API GTN_GetMoveVelocity(short core,short profile,TMoveVelocityPrm *pPrm);

typedef struct
{
	short list;
	short reserve1[2];
	short modal;
	int32_t segNum;
	int32_t reserve2[3];
	double reserve3[4];
}TListInfo;

typedef struct
{
	double pos;
	double vel;
	double velEnd;
	double acc;
	double dec;
	short  direction;
	short overrideSelect;
	short reserve[2];
} TMoveContinuousAbsolutePrm;

typedef struct
{
	double distance;
	double vel;
	double velEnd;
	double acc;
	double dec;
	short overrideSelect;
	short reserve[3];
} TMoveContinuousRelativePrm;

GT_API GTN_MoveContinuousAbsolute(short core,short profile,TMoveContinuousAbsolutePrm *pPrm,TListInfo *pListInfo=NULL,short group=0);
GT_API GTN_MoveContinuousRelative(short core,short profile,TMoveContinuousRelativePrm *pPrm,TListInfo *pListInfo,short group);

typedef struct
{
	short overrideSelect;
	short reserve1[3];
	double vel;
	double acc;
	double dec;
	short reserve[44];
}TJogParameter;

typedef union
{
	TJogParameter jog;
	short reserve[60];
}TMoveJogPrmUnion;

typedef struct
{
	short mode;
	short reserve[3];
	TMoveJogPrmUnion data;
}TMoveJogPrm;

GT_API GTN_MoveJog(short core,short index,TMoveJogPrm *pPrm,TListInfo *pListInfo,short group);
GT_API GTN_MoveTrap(short core,short profile,double pos,double vel,TTrapPrm *pPrm,TListInfo *pList=NULL);

/*-----------------------------------------------------------*/
/* Command Array			                                       */
/*-----------------------------------------------------------*/
GT_API GTN_SetVelArray(short core,short profile,double *pVel,short count);
GT_API GTN_SetTrapPrmArray(short core,short profile,TTrapPrm *pPrm,short count);
GT_API GTN_ClearTriggerStatusArray(short core,short trigger,short count);


/*-----------------------------------------------------------*/
/* MovePos                                                   */
/*-----------------------------------------------------------*/
typedef struct
{
	short value;
}TMovePosPercent;

typedef struct
{
	short value;
}TMovePosSmoothTime;

typedef union
{
	TMovePosPercent percent;
	TMovePosSmoothTime smoothTime;
	double value[2];
}TMovePosUnion;

typedef struct
{
	double pos;
	double vel;
	double acc;
	double dec;
	short direction;
	short overrideSelect;
	short pad;
	short mode;
	TMovePosUnion data;
}TMovePosParameter;

typedef struct
{
	double pos[2];
	double vel[2];
	double acc;
	double dec;
	short direction;
	short overrideSelect;
	short pad;
	short mode;
	TMovePosUnion data;
}TMovePosTwoSegmentParameter;

typedef struct
{
	short profile;
	short pad1[3];
	double pos;
	double vel;
	double acc;
	double dec;
	short direction;
	short overrideSelect;
	short pad2;
	short mode;
	TMovePosUnion data;
}TMultiMovePosParameter;

typedef struct
{
	short profile;
	short pad1[3];
	double pos[2];
	double vel[2];
	double acc;
	double dec;
	short direction;
	short overrideSelect;
	short pad2;
	short mode;
	TMovePosUnion data;
}TMultiMovePosTwoSegmentParameter;

GT_API GTN_MovePos(short core,short profile,TMovePosParameter *pMovePos,TListInfo *pListInfo=NULL,short group=0);
GT_API GTN_MovePosTwoSegment(short core,short profile,TMovePosTwoSegmentParameter *pMovePos,TListInfo *pListInfo=NULL,short group=0);
GT_API GTN_MultiMovePos(short core,TMultiMovePosParameter *pMovePos,short count,TListInfo *pListInfo,short group);
GT_API GTN_MultiMovePosTwoSegment(short core,TMultiMovePosTwoSegmentParameter *pMovePos,short count,TListInfo *pListInfo,short group);

#define MOTION_RESTRICT_PROFILE_MAX						(8)

#define MOTION_TIME_RESTRICT_MODE_DIRECT				(1)
#define MOTION_TIME_RESTRICT_MODE_ATTENTION_PROFILE		(2)

typedef struct
{
	short condition[MOTION_RESTRICT_PROFILE_MAX];
	double time[MOTION_RESTRICT_PROFILE_MAX];
}TMotionTimeRestrictDirect;

typedef struct
{
	short condition;
	short attentionProfileCount;
	short attentionProfile[MOTION_RESTRICT_PROFILE_MAX];
	short reserve1[6];
	double settlingTime;
}TMotionTimeRestrictAttentionProfile;

typedef union
{
	TMotionTimeRestrictDirect direct;
	TMotionTimeRestrictAttentionProfile attentionProfile;
	double data[32];
}TMotionTimeRestrictUnion;

typedef struct
{
	short profileCount;
	short profile[MOTION_RESTRICT_PROFILE_MAX];
	short pad1[7];
	double profilePos[MOTION_RESTRICT_PROFILE_MAX];

	short mode;
	short pad2[3];
	TMotionTimeRestrictUnion parameter;
}TMotionTimeRestrict;
GT_API GTN_SetMotionTimeRestrict(short core,short restrictIndex,TMotionTimeRestrict *pRestrict,TListInfo *pListInfo);
GT_API GTN_GetMotionTimeRestrict(short core,short restrictIndex,TMotionTimeRestrict *pRestrict);

typedef struct
{
	short type;     // 类型
	short index;    //索引
	short subIndex; // 二级索引
	short reserve[5];
}TProfileItem;

typedef struct
{
	double masterEven;          //传动比主轴，单位是pulse
	double slaveEven;            //传动比从轴，单位是mm
	double masterSlope;    // 主轴离合区位移
	short reserve[48];
}TGearParameter;

typedef struct
{
	double masterEven;          //传动比主轴，单位是pulse
	double slaveEven;            //传动比从轴，单位是mm
	double masterFilterTime;// 主轴滤波时间
	double sampleTime;    // 主轴采样时间
	double stopWaitTime;   // 停止等待时间
	double acceleration;    // 从轴运动加速度
	double deceleration;    // 从轴运动减速度
	double maxVel;        // 从轴运动最大速度
	short reserve[28];
}TMpgParameter;

typedef struct
{
	double masterEven;          //传动比主轴，单位是pulse
	double slaveEven;            //传动比从轴，单位是mm
	short reserve[52];
}TFollowParameter;

typedef struct
{
	double pos;            // 从轴终点位置
	double masterLength;   // 主轴位移
	short reserve[52];
}TGearPosParameter;

typedef union
{
	TGearParameter gear;
	TMpgParameter mpg;
	TFollowParameter follow;
	TGearPosParameter gearPos;
	short reserve[60];
}TMoveSynchronizationUnion;

typedef struct
{
	short mode;          // 同步运动模式
	short enable;        // 是否使能同步运动功能
	short softDirCheck;  //0:关闭方向趋势判断功能 1：开启方向趋势判断功能
	short reserve;
	TMoveSynchronizationUnion prm;
}TMoveSynchronizationPrm;

GT_API GTN_MoveSynchronization(short core,TProfileItem slave,TProfileItem master,TMoveSynchronizationPrm *pPrm,TListInfo *pListInfo);

typedef struct
{
	short source;
	short enable;
	short x;
	short y;
	double theta;		// degree
} TTransformOrthogonal;

GT_API GTN_SetTransformOrthogonal(short core,short index,TTransformOrthogonal *pOrthogonal);
GT_API GTN_GetTransformOrthogonal(short core,short index,TTransformOrthogonal *pOrthogonal);
GT_API GTN_GetTransformOrthogonalPosition(short core,short index,double *pPositionX,double *pPositionY);

typedef struct
{
	short source;
	short enable;
	short x;
	short y;
	short z;
	double alpha;		// Z轴和XY平面的夹角
	double beta;		// Z轴在XY平面的投影和X轴的夹角
	double gama;		// XY轴之间的夹角
} TTransformPerpendicularity;

GT_API GTN_SetTransformPerpendicularity(short core,short index,TTransformPerpendicularity *pPerpendicularity);
GT_API GTN_GetTransformPerpendicularity(short core,short index,TTransformPerpendicularity *pPerpendicularity);
GT_API GTN_GetTransformPerpendicularityPosition(short core,short index,double *pPositionX,double *pPositionY,double *pPositionZ);

/*-----------------------------------------------------------*/
/* Comp                                                      */
/*-----------------------------------------------------------*/

#define COMP_TYPE_COORD_SYNC	(0)

typedef struct
{
	double translateion[2];	//工件坐标系相对原始坐标系的平移
	double theta;	//工件坐标系相对原始坐标系的旋转
}TCoordTransformPrm;

typedef struct
{
	short enable;	//是否使能坐标系转换功能
	short refType;	//参考类型，可以设置为MC_CRD、MC_GROUP、MC_PROFILE等
	short index;	//如果参考类型为MC_CRD，写入坐标系号
	short refIndex[2];	//参考轴号，指定坐标系中的某两个轴，或者profile的两个轴
	double offset[2];	//补偿轴X2、Y2零点相对参考坐标系X1、Y1零点的偏移
	TCoordTransformPrm refTrans;	//参考工件坐标系相对原坐标系的平移和旋转
	TCoordTransformPrm syncTrans;	//同步工件坐标系相对原坐标系的平移和旋转
	short reserve1[4];
	double reserve2[4];
}TCoordSyncCompPrm;

typedef struct
{
	short type;	//补偿类型，例如同步补偿等
	short index;	//一级索引，例如同步补偿支持最多允许四套坐标系，该索引表示第几套同步补偿
	short subIndex;	//二级索引，例如第一套同步补偿的第一个轴
	short reserve1[5];
	double reserve2[4];
}TPrfComp;

GT_API GTN_PrfComp(short core,short profile,TPrfComp *pComp);
GT_API GTN_PrfCompEnable(short core,short profile,short enable,short enableType = 0);
GT_API GTN_BufPrfCompEnable(short core,short crd,short fifo,short profile,short enable,short enableType = 0);
GT_API GTN_BufPrfCompEnableEx(short core,short crd,short fifo,short profile,short enable,short enableType = 0);
GT_API GTN_SetCoordSyncCompPrm(short core,short index,TCoordSyncCompPrm *pPrm);
GT_API GTN_GetCoordSyncCompPrm(short core,short index,TCoordSyncCompPrm *pPrm);
GT_API GTN_GetCoordSyncCompValue(short core,short index,double x,double y,double *pCompX,double *pCompY);

/*-----------------------------------------------------------*/
/* Home                                                      */
/*-----------------------------------------------------------*/
#define HOME_STAGE_IDLE                           (0)
#define HOME_STAGE_START                          (1)
#define HOME_STAGE_ON_HOME_LIMIT_ESCAPE           (2)
#define HOME_STAGE_ON_HOME_ESCAPE                 (3)

#define HOME_STAGE_SEARCH_LIMIT                   (10)
#define HOME_STAGE_SEARCH_LIMIT_STOP              (11)

#define HOME_STAGE_SEARCH_LIMIT_ESCAPE            (13)

#define HOME_STAGE_SEARCH_LIMIT_RETURN            (15)
#define HOME_STAGE_SEARCH_LIMIT_RETURN_STOP       (16)

#define HOME_STAGE_SEARCH_HOME                    (20)

#define HOME_STAGE_SERCH_HOME_STOP                (22)
#define HOME_STAGE_SEARCH_HOME_OPPOSITE           (23)
#define HOME_STAGE_SEARCH_HOME_AGAIN              (24)
#define HOME_STAGE_SEARCH_HOME_RETURN             (25)

#define HOME_STAGE_SEARCH_INDEX                   (30)

#define HOME_STAGE_SEARCH_GPI                     (40)

#define HOME_STAGE_SEARCH_GPI_RETURN              (45)

#define HOME_STAGE_GO_HOME                        (80)

#define HOME_STAGE_END                            (100)

#define HOME_ERROR_NONE                           (0)
#define HOME_ERROR_NOT_TRAP_MODE                  (1)
#define HOME_ERROR_DISABLE                        (2)
#define HOME_ERROR_ALARM                          (3)
#define HOME_ERROR_STOP                           (4)
#define HOME_ERROR_STAGE                          (5)
#define HOME_ERROR_HOME_MODE                      (6)
#define HOME_ERROR_SET_CAPTURE_HOME               (7)
#define HOME_ERROR_NO_HOME                        (8)
#define HOME_ERROR_SET_CAPTURE_INDEX              (9)
#define HOME_ERROR_NO_INDEX                       (10)
#define HOME_ERROR_LIMIT                          (11)
#define HOME_ERROR_ESCAPE                         (12)

#define HOME_MODE_LIMIT                           (10)
#define HOME_MODE_LIMIT_HOME                      (11)
#define HOME_MODE_LIMIT_INDEX                     (12)
#define HOME_MODE_LIMIT_HOME_INDEX                (13)

#define HOME_MODE_HOME                            (20)

#define HOME_MODE_HOME_INDEX                      (22)

#define HOME_MODE_HOME_LEVEL                      (24)                               // 使用HOME电平回零(适用于没有HOME捕获的情况下)

#define HOME_MODE_INDEX                           (30)
#define HOME_MODE_DRIVER_HOME                     (42)
#define HOME_MODE_FORCED_HOME                     (50)
#define HOME_MODE_FORCED_HOME_INDEX               (51)
typedef struct
{
	short mode;
	short moveDir;
	short indexDir;
	short edge;
	short triggerIndex;
	short pad1[3];
	double velHigh;
	double velLow;
	double acc;
	double dec;
	short smoothTime;
	short pad2[3];
	int32_t homeOffset;
	int32_t searchHomeDistance;
	int32_t searchIndexDistance;
	int32_t escapeStep;
	int32_t pad3[2];
} THomePrm;

typedef struct
{
	short run;
	short stage;
	short error;
	short pad1;
	int32_t capturePos;
	int32_t targetPos;
} THomeStatus;

GT_API GTN_GoHome(short core,short axis,THomePrm *pHomePrm);
GT_API GTN_GetHomePrm(short core,short axis,THomePrm *pHomePrm);
GT_API GTN_GetHomeStatus(short core,short axis,THomeStatus *pHomeStatus);

GT_API GTN_HandwheelInit(short core,short mode=0);
GT_API GTN_SetHandwheelStopDec(short core,short slave,double decSmoothStop,double decAbruptStop);
GT_API GTN_StartHandwheel(short core,short slave,short master,short masterEven,short slaveEven,short intervalTime,double acc,double dec,double vel,short stopWaitTime);
GT_API GTN_EndHandwheel(short core,short slave);

/*-----------------------------------------------------------*/
/* PLC                                                       */
/*-----------------------------------------------------------*/
#define PLC_THREAD_MAX					(32)
#define PLC_PAGE_MAX					(32)
#define PLC_LOCAL_VAR_MAX				(1024)
#define PLC_ACCESS_VAR_COUNT_MAX        (8)

#define PLC_TIMER_TT					(0)
#define PLC_TIMER_TF					(1)
#define PLC_TIMER_TTF					(2)

#define PLC_COUNTER_EQ					(0)
#define PLC_COUNTER_LE					(1)
#define PLC_COUNTER_GE					(2)

#define PLC_COUNTER_EDGE_UP				(0)
#define PLC_COUNTER_EDGE_DOWN			(1)
#define PLC_COUNTER_EDGE_UP_DOWN		(2)

#define PLC_FLANK_UP					(0)
#define PLC_FLANK_DOWN					(1)
#define PLC_FLANK_UP_DOWN				(2)

typedef enum
{
	PLC_BIND_NONE,
	PLC_BIND_DI,
	PLC_BIND_DO,
	PLC_BIND_TIMER,
	PLC_BIND_COUNTER,
	PLC_BIND_FLANK,
	PLC_BIND_SRFF,
} EPlcBind;

typedef struct VarInfo
{
	short id;
	short dataType;
	char  name[32];
} TVarInfo;

typedef struct
{
	short diType;
	short index;
	short reverse;
} TBindDi;

typedef struct
{
	short doType;
	short index;
	short reverse;
} TBindDo;

typedef struct
{
	short timerType;
	int32_t delay;
	short inputVarId;
} TBindTimer;

typedef struct
{
	short counterType;
	short edge;
	int32_t init;
	int32_t target;
	int32_t begin;
	int32_t end;
	short dir;
	int32_t unit;
	short inputVarId;
	short resetVarId;
} TBindCounter;

typedef struct
{
	short flankType;
	short inputVarId;
} TBindFlank;

typedef struct
{
	short setVarId;
	short resetVarId;
} TBindSrff;

typedef struct CompileInfo
{
	char  *pFileName;
	short *pLineNo;
	char  *pMessage;
} TCompileInfo;

typedef struct ThreadSts
{
	short  run;
	short  error;
	double result;
	short  line;
} TThreadSts;

GT_API GTN_Compile(char *pFileName,TCompileInfo *pWrongInfo);
GT_API GTN_Download(short core,char *pFileName);
GT_API GTN_GetFunId(char *pFunName,short *pFunId);
GT_API GTN_GetVarId(char *pFunName,char *pVarName,TVarInfo *pVarInfo);
GT_API GTN_Bind(short core,short thread,short funId,short page);
GT_API GTN_RunThread(short core,short thread);
GT_API GTN_RunThreadPeriod(short core,short thread,short ms,short priority=4);
GT_API GTN_StepThread(short core,short thread);
GT_API GTN_StopThread(short core,short thread);
GT_API GTN_PauseThread(short core,short thread);
GT_API GTN_GetThreadSts(short core,short thread,TThreadSts *pThreadSts);
GT_API GTN_GetThreadTime(short core,short thread,short *pPeriod,double *pExecuteTime,double *pExecuteTimeMax);
GT_API GTN_RunThreadToBreakpoint(short core,short thread,short line);
GT_API GTN_SetVarValue(short core,short page,TVarInfo *pVarInfo,double *pValue,short count=1);
GT_API GTN_GetVarValue(short core,short page,TVarInfo *pVarInfo,double *pValue,short count=1);


GT_API GTN_UnbindVar(short core,short thread);
GT_API GTN_BindDi(short core,short thread,TVarInfo *pVarInfo,TBindDi *pBindDi);
GT_API GTN_BindDo(short core,short thread,TVarInfo *pVarInfo,TBindDo *pBindDo);
GT_API GTN_BindTimer(short core,short thread,TVarInfo *pVarInfo,TBindTimer *pBindTimer);
GT_API GTN_BindCounter(short core,short thread,TVarInfo *pVarInfo,TBindCounter *pBindCounter);
GT_API GTN_BindFlank(short core,short thread,TVarInfo *pVarInfo,TBindFlank *pBindFlank);
GT_API GTN_BindSrff(short core,short thread,TVarInfo *pVarInfo,TBindSrff *pBindSrff);

GT_API GTN_GetBindDi(short core,TVarInfo *pVarInfo,TBindDi *pBindDi);
GT_API GTN_GetBindDo(short core,TVarInfo *pVarInfo,TBindDo *pBindDo);
GT_API GTN_GetBindTimer(short core,TVarInfo *pVarInfo,TBindTimer *pBindTimer,int32_t *pCount);
GT_API GTN_GetBindCounter(short core,TVarInfo *pVarInfo,TBindCounter *pBindCounter,int32_t *pUnitCount,int32_t *pCount);
GT_API GTN_GetBindFlank(short core,TVarInfo *pVarInfo,TBindFlank *pBindFlank);
GT_API GTN_GetBindSrff(short core,TVarInfo *pVarInfo,TBindSrff *pBindSrff);

GT_API GTN_GetBindDiCount(short core,short thread,short *pCount);
GT_API GTN_GetBindDoCount(short core,short thread,short *pCount);
GT_API GTN_GetBindTimerCount(short core,short thread,short *pCount);
GT_API GTN_GetBindCounterCount(short core,short thread,short *pCount);
GT_API GTN_GetBindFlankCount(short core,short thread,short *pCount);
GT_API GTN_GetBindSrffCount(short core,short thread,short *pCount);

GT_API GTN_GetBindDiInfo(short core,short thread,short index,short *pVar,TBindDi *pBindDi);
GT_API GTN_GetBindDoInfo(short core,short thread,short index,short *pVar,TBindDo *pBindDo);
GT_API GTN_GetBindTimerInfo(short core,short thread,short index,short *pVar,TBindTimer *pBindTimer);
GT_API GTN_GetBindCounterInfo(short core,short thread,short index,short *pVar,TBindCounter *pBindCounter);
GT_API GTN_GetBindFlankInfo(short core,short thread,short index,short *pVar,TBindFlank *pBindFlank);
GT_API GTN_GetBindSrffInfo(short core,short thread,short index,short *pVar,TBindSrff *pBindSrff);

typedef struct ThreadStatus
{
	short link;
	uint32_t  address;
	short size;
	uint32_t  page;
	short delay;
	short priority;
	short ptr;
	short status;
	short error;
	short result[4];
	short resultType;
	short breakpoint;
	short period;
	short count;
	short function;
} TThreadStatus;
GT_API GTN_GetThread(short core,short thread,TThreadStatus *pThread);

GT_API GTN_ClearPlc(short core);
GT_API GTN_LoadPlc(short core,short id,short returnType);
GT_API GTN_LoadPlcCommand(short core,short id,short count,short *pData);

/*-----------------------------------------------------------*/
/* Interpolation                                             */
/*-----------------------------------------------------------*/
#define INTERPOLATION_AXIS_MAX                   (8)

#define CRD_OPERATION_DATA_EXT_MAX               (2)


#define CRD_BUFFER_MODE_DYNAMIC_DEFAULT          (0)
#define CRD_BUFFER_MODE_DYNAMIC_KEEP             (1)

#define CRD_BUFFER_MODE_STATIC_INPUT             (11)
#define CRD_BUFFER_MODE_STATIC_READY             (12)
#define CRD_BUFFER_MODE_STATIC_START             (13)

#define INTERPOLATION_CIRCLE_PLAT_XY             (0)
#define INTERPOLATION_CIRCLE_PLAT_YZ             (1)
#define INTERPOLATION_CIRCLE_PLAT_ZX             (2)

#define INTERPOLATION_HELIX_CIRCLE_XY_LINE_Z     (0)
#define INTERPOLATION_HELIX_CIRCLE_YZ_LINE_X     (1)
#define INTERPOLATION_HELIX_CIRCLE_ZX_LINE_Y     (2)

#define INTERPOLATION_CIRCLE_DIR_CW              (0)
#define INTERPOLATION_CIRCLE_DIR_CCW             (1)

typedef struct CrdPrm
{
	short dimension;
	short profile[8];
	double synVelMax;
	double synAccMax;
	short evenTime;
	short setOriginFlag;
	int32_t originPos[8];
}TCrdPrm;

typedef struct CrdBufOperation
{
	short flag;
	unsigned short delay;
	short doType;
	unsigned short doMask;
	unsigned short doValue;
	unsigned short dataExt[CRD_OPERATION_DATA_EXT_MAX];
}TCrdBufOperation;

typedef struct CrdData
{
	short motionType;
	short circlePlat;
	int32_t pos[INTERPOLATION_AXIS_MAX];
	double radius;
	short circleDir;
	double center[3];
	double vel;
	double acc;
	short velEndZero;
	TCrdBufOperation operation;

	double cos[INTERPOLATION_AXIS_MAX];
	double velEnd;
	double velEndAdjust;
	double r;
}TCrdData;

typedef struct
{
	double time;
	int32_t segmentUsed;
	int32_t segmentHead;
	int32_t segmentTail;
} TCrdTime;

typedef struct
{
	short crdAxis;
	short masterIndex;
	short masterType;
} TBufFollowMaster;

typedef struct
{
	int32_t masterPos;
	int32_t pad;
} TBufFollowEventCross;

typedef struct
{
	short triggerIndex;
	int32_t triggerOffset;
	int32_t pad;
} TBufFollowEventTrigger;

typedef struct
{
	double velRatioMax;
	double accRatioMax;
	int32_t masterLead;
	int32_t masterEven;
	int32_t slaveEven;
	short dir;
	short smoothPercent;
	short synchAlign;
} TCrdFollowPrm;

typedef struct
{
	short stage;
	double slavePos;
	double slaveVel;
	int32_t masterFrameWidth;
	int32_t masterFrameIndex;
	uint32_t loopCount;
} TCrdFollowStatus;

GT_API GTN_SetCrdPrm(short core,short crd,TCrdPrm *pCrdPrm);
GT_API GTN_GetCrdPrm(short core,short crd,TCrdPrm *pCrdPrm);
GT_API GTN_CrdSpace(short core,short crd,int32_t *pSpace,short fifo=0);
GT_API GTN_CrdData(short core,short crd,TCrdData *pCrdData,short fifo=0);

GT_API GTN_LnXY(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYOverride2(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYWN(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYOverride2WN(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYG0(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYG0Override2(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYG0WN(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYG0Override2WN(short core,short crd,int32_t x,int32_t y,double synVel,double synAcc,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZ(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZWN(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYZOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZG0(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYZG0Override2(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYZG0WN(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYZG0Override2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double synVel,double synAcc,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZA(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZAOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZAWN(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYZAOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZAG0(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYZAG0Override2(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,short fifo=0);
GT_API GTN_LnXYZAG0WN(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYZAG0Override2WN(short core,short crd,int32_t x,int32_t y,int32_t z,int32_t a,double synVel,double synAcc,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZACUVW(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZACUVWOverride2(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_LnXYZACUVWWN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_LnXYZACUVWOverride2WN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_LnXYZAC(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd,short fifo);
GT_API GTN_LnXYZACOverride2(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd,short fifo);
GT_API GTN_LnXYZACWN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo);
GT_API GTN_LnXYZACOverride2WN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo);

GT_API GTN_LnXYZACG0(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,short fifo);
GT_API GTN_LnXYZACG0Override2(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,short fifo);
GT_API GTN_LnXYZACG0WN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short fifo);
GT_API GTN_LnXYZACG0Override2WN(short core,short crd,int32_t *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short fifo);


GT_API GTN_ArcXYR(short core,short crd,int32_t x,int32_t y,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcXYROverride2(short core,short crd,int32_t x,int32_t y,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcXYRWN(short core,short crd,int32_t x,int32_t y,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcXYROverride2WN(short core,short crd,int32_t x,int32_t y,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcXYC(short core,short crd,int32_t x,int32_t y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcXYCOverride2(short core,short crd,int32_t x,int32_t y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcXYCWN(short core,short crd,int32_t x,int32_t y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcXYCOverride2WN(short core,short crd,int32_t x,int32_t y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcYZR(short core,short crd,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcYZROverride2(short core,short crd,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcYZRWN(short core,short crd,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcYZROverride2WN(short core,short crd,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcYZC(short core,short crd,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcYZCOverride2(short core,short crd,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcYZCWN(short core,short crd,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcYZCOverride2WN(short core,short crd,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcZXR(short core,short crd,int32_t z,int32_t x,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcZXROverride2(short core,short crd,int32_t z,int32_t x,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcZXRWN(short core,short crd,int32_t z,int32_t x,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcZXROverride2WN(short core,short crd,int32_t z,int32_t x,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcZXC(short core,short crd,int32_t z,int32_t x,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcZXCOverride2(short core,short crd,int32_t z,int32_t x,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_ArcZXCWN(short core,short crd,int32_t z,int32_t x,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_ArcZXCOverride2WN(short core,short crd,int32_t z,int32_t x,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_ArcXYZ(short core,short crd,int32_t x,int32_t y,int32_t z,double interX,double interY,double interZ,double synVel,double synAcc,double velEnd,short fifo=0);
GT_API GTN_ArcXYZWN(short core,short crd,int32_t x,int32_t y,int32_t z,double interX,double interY,double interZ,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);
GT_API GTN_ArcXYZOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double interX,double interY,double interZ,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);



GT_API GTN_HelixXYRZ(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd,short fifo=0);
GT_API GTN_HelixXYRZOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixXYRZWN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixXYRZOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixXYCZ(short core,short crd,int32_t x,int32_t y,int32_t z,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixXYCZOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixXYCZWN(short core,short crd,int32_t x,int32_t y,int32_t z,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixXYCZOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixYZRX(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixYZRXOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixYZRXWN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixYZRXOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixYZCX(short core,short crd,int32_t x,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixYZCXOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixYZCXWN(short core,short crd,int32_t x,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixYZCXOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixZXRY(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixZXRYOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixZXRYWN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixZXRYOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixZXCY(short core,short crd,int32_t x,int32_t y,int32_t z,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixZXCYOverride2(short core,short crd,int32_t x,int32_t y,int32_t z,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short fifo=0);
GT_API GTN_HelixZXCYWN(short core,short crd,int32_t x,int32_t y,int32_t z,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);
GT_API GTN_HelixZXCYOverride2WN(short core,short crd,int32_t x,int32_t y,int32_t z,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd=0,int32_t segNum=0,short fifo=0);

GT_API GTN_HelixXYRMultiZ(short core,short crd,int32_t *pPos,double radius,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixYZRMultiX(short core,short crd,int32_t *pPos,double radius,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixZXRMultiY(short core,short crd,int32_t *pPos,double radius,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);

GT_API GTN_HelixXYCMultiZ(short core,short crd,int32_t *pPos,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixYZCMultiX(short core,short crd,int32_t *pPos,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixZXCMultiY(short core,short crd,int32_t *pPos,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,double velEnd,int32_t segNum,short override2,short fifo);

GT_API GTN_HelixXYRMultiZEx(short core,short crd,double *pPos,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixYZRMultiXEx(short core,short crd,double *pPos,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixZXRMultiYEx(short core,short crd,double *pPos,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);

GT_API GTN_HelixXYCMultiZEx(short core,short crd,double *pPos,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixYZCMultiXEx(short core,short crd,double *pPos,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_HelixZXCMultiYEx(short core,short crd,double *pPos,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo);


GT_API GTN_BufIO(short core,short crd,unsigned short doType,unsigned short doMask,unsigned short doValue,short fifo=0);
GT_API GTN_BufDelay(short core,short crd,unsigned short delayTime,short fifo=0);
GT_API GTN_BufDA(short core,short crd,short chn,short daValue,short fifo=0);
GT_API GTN_BufLmtsOn(short core,short crd,short axis,short limitType,short fifo=0);
GT_API GTN_BufLmtsOff(short core,short crd,short axis,short limitType,short fifo=0);
GT_API GTN_BufSetStopIo(short core,short crd,short axis,short stopType,short inputType,short inputIndex,short fifo=0);
GT_API GTN_BufMove(short core,short crd,short moveAxis,int32_t pos,double vel,double acc,short modal,short fifo=0);
GT_API GTN_BufGear(short core,short crd,short gearAxis,int32_t pos,short fifo=0);
GT_API GTN_BufGearPercent(short core,short crd,short gearAxis,int32_t pos,short accPercent,short decPercent,short fifo=0);
GT_API GTN_BufStop(short core,short crd,int32_t mask,int32_t option,short fifo);
GT_API GTN_BufStopEx(short core,short crd,int32_t mask,int32_t option,short fifo);
GT_API GTN_BufMoveJog(short core,short crd,short moveAxis,double vel,double acc,short modal,short fifo);
GT_API GTN_BufMoveJogEx(short core,short crd,short moveAxis,double vel,double acc,short modal,short fifo);
GT_API GTN_BufStopMotion(short core,short crd,short fifo=0);
GT_API GTN_BufSetVarValue(short core,short crd,short pageId,TVarInfo *pVarInfo,double value,short fifo=0);
GT_API GTN_BufJumpNextSeg(short core,short crd,short axis,short limitType,short fifo=0);
GT_API GTN_BufSynchPrfPos(short core,short crd,short encoder,short profile,short fifo=0);
GT_API GTN_BufVirtualToActual(short core,short crd,short fifo=0);
GT_API GTN_BufEnableDoBitPulse(short core,short crd,short doType,short doIndex,unsigned short highLevelTime,unsigned short lowLevelTime,int32_t pulseNum,short firstLevel,short fifo);
GT_API GTN_BufDisableDoBitPulse(short core,short crd,short doType,short doIndex,short fifo);
GT_API GTN_BufTrend(short core,short crd,uint32_t trendSegNum,double trendDistance,double trendVelEnd,short fifo);
GT_API GTN_BufExtIO(short core,short crd,unsigned short doIndex,unsigned short doMask,unsigned short doValue,short fifo);
GT_API GTN_BufExtDA(short core,short crd,short chn,short daValue,short fifo);
GT_API GTN_BufGearListBegin(short core,short crd,short gearAxis,double pos,short fifo);
GT_API GTN_BufGearListAdd(short core,short crd,float k,float percent,short fifo);
GT_API GTN_BufGearListEnd(short core,short crd,short fifo);

typedef struct
{
	double pos;
	double vel;
	double acc;
	double velMax;
	double reserve1[4];
	short smoothTime;
	short enableRatio;
	short modal;
	short fifo;
	short dir;
	short reserve2[3];
	int32_t segNum;
	int32_t reserve3[3];
}TBufMoveAbsPrm;
GT_API GTN_BufMoveAbsoluteEx(short core,short crd,short moveAxis,TBufMoveAbsPrm *pPrm);

typedef struct
{
	short time;
	short aheadOfTime;
	short shape;
	short reverseSmooth;
	double velDelta;
} TCrdBlend;
GT_API GTN_BufSetCrdBlend(short core,short crd,TCrdBlend *pCrdBlend,short fifo);
GT_API GTN_BufCrdBlendOn(short core,short crd,short fifo);
GT_API GTN_BufCrdBlendOff(short core,short crd,short fifo);

GT_API GTN_CrdStart(short core,short mask,short option);
GT_API GTN_CrdStartStep(short core,short mask,short option);
GT_API GTN_CrdStepMode(short core,short mask,short option);
GT_API GTN_SetOverride(short core,short crd,double synVelRatio);
GT_API GTN_SetOverride2(short core,short crd,double synVelRatio);
GT_API GTN_InitLookAhead(short core,short crd,short fifo,double T,double accMax,short n,TCrdData *pLookAheadBuf);
GT_API GTN_GetLookAheadSpace(short core,short crd,int32_t *pSpace,short fifo=0);
GT_API GTN_GetLookAheadSegCount(short core,short crd,int32_t *pSegCount,short fifo=0);
GT_API GTN_CrdClear(short core,short crd,short fifo);
GT_API GTN_CrdStatus(short core,short crd,short *pRun,int32_t *pSegment,short fifo=0);
GT_API GTN_SetUserSegNum(short core,short crd,int32_t segNum,short fifo=0);
GT_API GTN_GetUserSegNum(short core,short crd,int32_t *pSegment,short fifo=0);
GT_API GTN_GetUserSegNumWN(short core,short crd,int32_t *pSegment,short fifo=0);
GT_API GTN_GetRemainderSegNum(short core,short crd,int32_t *pSegment,short fifo=0);
GT_API GTN_GetRemainderSegNumEx(short core,short crd,int32_t *pSegment,short fifo);
GT_API GTN_SetCrdStopDec(short core,short crd,double decSmoothStop,double decAbruptStop);
GT_API GTN_GetCrdStopDec(short core,short crd,double *pDecSmoothStop,double *pDecAbruptStop);
GT_API GTN_SetCrdLmtStopMode(short core,short crd,short lmtStopMode);
GT_API GTN_GetCrdLmtStopMode(short core,short crd,short *pLmtStopMode);
GT_API GTN_GetUserTargetVel(short core,short crd,double *pTargetVel);
GT_API GTN_GetSegTargetPos(short core,short crd,int32_t *pTargetPos);
GT_API GTN_GetCrdPos(short core,short crd,double *pPos);
GT_API GTN_GetCrdVel(short core,short crd,double *pSynVel);
GT_API GTN_SetCrdSingleMaxVel(short core,short crd,double *pMaxVel);
GT_API GTN_GetCrdSingleMaxVel(short core,short crd,double *pMaxVel);

#define DIMENSION_MAX                   (8)                 // 每个坐标系的最大维数
typedef struct
{
	int32_t segNumUserSet;
	int32_t segNumInCmd;
	double synPrfVel;
	double prfPosCrd[DIMENSION_MAX];
}TCrdSegSyncInfo;
GT_API GTN_GetCrdSegSyncInfo(short core,short crd,short fifo,TCrdSegSyncInfo *pSegInfo);

typedef struct
{
	short runEmpty;
	int32_t segUseCount;
	int32_t segReceiveCount;
	short pad1[2];
	int32_t pad2[2];
	double pad3[2];
}TCrdStatusEx;
GT_API GTN_CrdStatusEx(short core,short crd,TCrdStatusEx *pCrdStatus,short fifo);

typedef struct
{
	double receive;
	double travel;
	double left;
} TCrdSegmentLength;
GT_API GTN_GetCrdSegmentLength(short core,short crd,TCrdSegmentLength *pSegmentLength,short fifo);

GT_API GTN_SetArcAllowError(short core,short crd, double error);
GT_API GTN_SetArcAllowErrorLa(short core,short crd,double error);

/*-----------------------------------------------------------*/
/* Laser Follow	                                            */
/*-----------------------------------------------------------*/
typedef struct
{
	short laserFollowMode;
	double maxFrq;
	double minFrq;
	double maxDuty;
	double minDuty;
	short pad1[7];
	double pad2[8];
}TLaserFollowPrm;

typedef struct
{
	uint32_t onCount;
	uint32_t offCount;
	uint32_t onCountInFpga;
	uint32_t offCountInFpga;
	uint32_t pad[4];
}TLaserOnOffCount;

GT_API GTN_SetLaserFollowMode(short core,TLaserFollowPrm *pPrm,short channel);
GT_API GTN_LaserFollowOff(short core,short crd,short fifo,short channel);
GT_API GTN_SetLaserFollowTable(short core,short tableId,int32_t n,double *pVel,double *pPower,short channel);
GT_API GTN_GetLaserFollowTable(short core,short tableId,int32_t n,double *pVel,double *pPower,int32_t *pCount,short channel);
GT_API GTN_SetLaserFollowSpline(short core,short tableId,int32_t n,double *pX,double *pY,double beginValue,double endValue,short channel);
GT_API GTN_GetLaserFollowSpline(short core,short tableId,int32_t n,double *pX,double *pY,double *pA,double *pB,double *pC,int32_t *pCount,short channel);
GT_API GTN_GetLaserOnOffCount(short core,short channel,TLaserOnOffCount *pLaserCount);
GT_API GTN_ClearLaserOnOffCount(short core,short channel);

GT_API GTN_BufLaserOn(short core,short crd,short fifo=0,short channel=0);
GT_API GTN_BufLaserOff(short core,short crd,short fifo=0,short channel=0);
GT_API GTN_BufLaserPrfCmd(short core,short crd,double laserPower,short fifo=0,short channel=0);
GT_API GTN_BufLaserFollowMode(short core,short crd,short source,short fifo,short channel,double startPower);
GT_API GTN_BufLaserFollowRatio(short core,short crd,double ratio,double minPower,double maxPower,short fifo,short channel);
GT_API GTN_BufLaserFollowOff(short core,short crd,short fifo,short channel);
GT_API GTN_BufLaserFollowSpline(short core,short crd,short tableId,double minPower,double maxPower,short fifo,short channel);
GT_API GTN_BufLaserFollowTable(short core,short crd,short tableId,double minPower,double maxPower,short fifo,short channel);


GT_API GTN_BufFollowMaster(short core,short crd,TBufFollowMaster *pBufFollowMaster,short fifo=0);
GT_API GTN_BufFollowEventCross(short core,short crd,TBufFollowEventCross *pEventCross,short fifo=0);
GT_API GTN_BufFollowEventTrigger(short core,short crd,TBufFollowEventTrigger *pEventTrigger,short fifo=0);
GT_API GTN_BufFollowStart(short core,short crd,int32_t masterSegment,int32_t slaveSegment,int32_t masterFrameWidth,short fifo=0);
GT_API GTN_BufFollowNext(short core,short crd,int32_t width,short fifo=0);
GT_API GTN_BufFollowReturn(short core,short crd,double vel,double acc,short smoothPercent,short fifo=0);

GT_API GTN_BufLaserOnEx(short core,short crd,short fifo=0,short channel=0);
GT_API GTN_BufLaserOffEx(short core,short crd,short fifo=0,short channel=0);
GT_API GTN_BufLaserFollowModeEx(short core,short crd,short source,short fifo,short channel,double startPower=0);
GT_API GTN_BufLaserFollowTableEx(short core,short crd,short tableId,double minPower,double maxPower,short fifo,short channel);
GT_API GTN_BufLaserFollowOffEx(short core,short crd,short fifo,short channel);
GT_API GTN_BufLaserPrfCmdEx(short core,short crd,double laserPower,short fifo=0,short channel=0);
GT_API GTN_BufLaserFollowRatioEx(short core,short crd,double ratio,double minPower,double maxPower,short fifo,short channel);

typedef struct
{
	short frqTableId;
	short dutyTableId;
	double pad1[8];
	short pad2[8];
}TLaserFollowDuoTablePrm;
GT_API GTN_BufLaserFollowDuoTableEx(short core,short crd,TLaserFollowDuoTablePrm *pPrm,short fifo,short channel);


/*-----------------------------------------------------------*/
/* Crd					                                              */
/*-----------------------------------------------------------*/
GT_API GTN_SetG0Mode(short core,short crd,short mode);
GT_API GTN_GetG0Mode(short core,short crd,short *pMode);

GT_API GTN_SetCrdMapBase(short core,short crd,short base);
GT_API GTN_GetCrdMapBase(short core,short crd,short *pBase);
GT_API GTN_SetCrdBufferMode(short core,short crd,  short bufferMode,short fifo=0);
GT_API GTN_GetCrdBufferMode(short core,short crd,short *pBufferMode,short fifo=0);
GT_API GTN_GetCrdSegmentTime(short core,short crd,int32_t segmentIndex,double *pSegmentTime,int32_t *pSegmentNumber,short fifo=0);
GT_API GTN_GetCrdTime(short core,short crd,TCrdTime *pTime,short fifo=0);
GT_API GTN_SetCrdFollowLoop(short core,short crd,uint32_t loop);
GT_API GTN_GetCrdFollowLoop(short core,short crd,uint32_t *pLoop);
GT_API GTN_SetCrdFollowPrm(short core,short crd,TCrdFollowPrm *pPrm);
GT_API GTN_GetCrdFollowPrm(short core,short crd,TCrdFollowPrm *pPrm);
GT_API GTN_GetCrdFollowStatus(short core,short crd,TCrdFollowStatus *pStatus);


GT_API GTN_SetCrdBlend(short core,short crd,TCrdBlend *pCrdBlend);
GT_API GTN_GetCrdBlend(short core,short crd,TCrdBlend *pCrdBlend);
GT_API GTN_CrdBlendOn(short core,short crd);
GT_API GTN_CrdBlendOff(short core,short crd);
GT_API GTN_CrdEnableVariableCircle(short core,short crd,short fifo);
GT_API GTN_CrdDisableVariableCircle(short core,short crd,short fifo);



GT_API GTN_SetCrdContourErrorControl(short core,short crd,short enable,double percent);
GT_API GTN_GetCrdContourErrorControl(short core,short crd,short *pEnable,double *pPercent);

//////////////////////////////////////////////////////////////////////////
//Smooth
//////////////////////////////////////////////////////////////////////////
GT_API GTN_SetCrdJerk(short core,short crd,double jerkMax);
GT_API GTN_GetCrdJerk(short core,short crd,double *pJerkMax);
GT_API GTN_SetCrdJerkTime(short core,short crd,double jerkTime,double coef);
GT_API GTN_GetCrdJerkTime(short core,short crd,double *pJerkTime,double *pCoef);
GT_API GTN_SetCrdJerkEx(short core,short crd,double jerkMax,double acc);
GT_API GTN_GetCrdJerkEx(short core,short crd,double *pJerkMax,double *pAcc);
GT_API GTN_SetCrdAccTime(short core,short crd,short time,short k);
GT_API GTN_GetCrdAccTime(short core,short crd,short *pTime,short *pK);

typedef struct
{
	short percent;
	short accStartPercent;
	short decEndPercent;
	double reserve;
} TCrdSmooth;
GT_API GTN_SetCrdSmooth(short core,short crd,TCrdSmooth *pCrdSmooth);
GT_API GTN_GetCrdSmooth(short core,short crd,TCrdSmooth *pCrdSmooth);
GT_API GTN_SetCrdSmoothTime(short core,short crd,short smoothType,double *pPrm);
GT_API GTN_GetCrdSmoothTime(short core,short crd,short *pSmoothType,double *pPrm);

GT_API GTN_SetAxisPrfVelSmooth(short core,short index,short type,short k);
GT_API GTN_GetAxisPrfVelSmooth(short core,short index,short *pType,short *pK);
GT_API GTN_SetAxisPrfVelSmoothEx(short core,short index,double w,double t,double n);
GT_API GTN_GetAxisPrfVelSmoothEx(short core,short index,double *pW,double *pT,double *pN);
GT_API GTN_SetMotionSmoothWorkMode(short core,short type,short index,short mode);
GT_API GTN_GetMotionSmoothWorkMode(short core,short type,short index,short *pMode);
GT_API GTN_SetAxisMotionSmooth(short core,short axis,double time,double k);
GT_API GTN_GetAxisMotionSmooth(short core,short axis,double *pTime,double *pK);
GT_API GTN_SetPathOptimizePrmLa(short core,short crd,short enable,short mode,void *pPrm);

typedef struct
{
	short mode;                  //平滑模式
	short reserve[3];            //保留值
	double prm[6];               //参数
}TMotionSmooth;
GT_API GTN_SetMotionSmooth(short core,short index,TMotionSmooth *pSmooth,short count);
GT_API GTN_GetMotionSmooth(short core,short index, TMotionSmooth *pSmooth,short count,short *pCountReturn);

//设置本核的硬件平滑资源数量，另外一个核为总数量减去本核数量
GT_API GTN_SetMotionSmoothFpgaResCount(short core,short count);
GT_API GTN_GetMotionSmoothFpgaResCount(short core,short *pCount);

// Input shaping
GT_API GTN_SetAxisInputShaping(short core,short axis,short enable,short count,double k);
GT_API GTN_SetAxisInputShapingPro(short core,short axis,short enable,double freq,double z);

GT_API GTN_SetPathShaping(short core, short axis, short enable, double t1, double t2);
GT_API GTN_GetPathShaping(short core, short axis, short *pEnable, double *pT1, double *pT2);
GT_API GTN_SetPathShapingPara(short core, short axis, double coef, short level);

/*-----------------------------------------------------------*/
/* Compensate                                                */
/*-----------------------------------------------------------*/
typedef struct
{
	short enable;
	int32_t compValue;
	int32_t compTime;
	short dir;
}TBacklashComp;

typedef struct
{
	short run;
	short pad1[3];
	int32_t  pad2[2];
	double pad3[2];
}TLeadScrewCompStatus;

GT_API GTN_SetBacklash(short core,short axis,int32_t value,double changeValue,int32_t dir);
GT_API GTN_GetBacklash(short core,short axis,int32_t *pValue,double *pChangeValue,int32_t *pDir);
GT_API GTN_SetBacklashComp(short core,short axis,TBacklashComp *pBacklash);
GT_API GTN_GetBacklashComp(short core,short axis,TBacklashComp *pBacklash);
GT_API GTN_SetLeadScrewComp(short core,short axis,short n,int32_t startPos,int32_t lenPos,int32_t *pPositive,int32_t *pNegative);
GT_API GTN_EnableLeadScrewComp(short core,short axis,short mode);
GT_API GTN_SetLeadScrewCrossComp(short core,short axis,short n,int32_t startPos,int32_t lenPos,int32_t *pPositive,int32_t *pNegative,short link);
GT_API GTN_EnableLeadScrewCrossComp(short core,short axis,short mode);
GT_API GTN_SetLeadScrewLink(short core,short axis,short link);
GT_API GTN_GetLeadScrewLink(short core,short axis,short *pLink);
GT_API GTN_GetLeadScrewCompStatus(short core,short axis,TLeadScrewCompStatus *pSts);
GT_API GTN_GetCompensate(short core,short axis,double *pPitchError,double *pCrossError,double *pBacklashError,double *pEncPos,double *pPrfPos);
GT_API GTN_SetLeadScrewCompMode(short core,short axis,short cycleMode,short revCompCycle);
GT_API GTN_GetLeadScrewCompMode(short core,short axis,short *pCycleMode,short *pRevCompCycle);
GT_API GTN_SetFriction(short core,short axis,short gain,double compTime);
GT_API GTN_GetFriction(short core,short axis,short *pGain,double *pCompTime);
GT_API GTN_EnableFriction(short core,short axis);
GT_API GTN_DisableFriction(short core,short axis);
GT_API GTN_SetZeroVelThreshold(short core,short axis,double zeroVelThreshold);
GT_API GTN_GetZeroVelThreshold(short core,short axis,double *pZeroVelThreshold);


typedef struct
{
	short n;
	int32_t startPos;
	int32_t lenPos;
	int32_t *pCompPos;
	int32_t *pCompNeg;
} TLeadScrewPrm;

GT_API GTN_SetLeadScrewTable(short core,short axis,TLeadScrewPrm *pPrm);
GT_API GTN_EnableLeadScrewTable(short core,short axis,int32_t error=0);
GT_API GTN_DisableLeadScrewTable(short core,short axis);
GT_API GTN_GetLeadScrewTablePrfPosCount(short core,int32_t encPos,TLeadScrewPrm *pPrm,short *pCountPositive,short *pCountNegative);
GT_API GTN_GetLeadScrewTablePrfPosPositive(short core,int32_t encPos,TLeadScrewPrm *pPrm,short index,int32_t *pPrfPosPositive);
GT_API GTN_GetLeadScrewTablePrfPosNegative(short core,int32_t encPos,TLeadScrewPrm *pPrm,short index,int32_t *pPrfPosNegative);

typedef struct
{
	short count[2];
	int32_t posBegin[2];
	int32_t step[2];
} TCompensate2DTable;

typedef struct
{
	short enable;
	short tableIndex;
	short axisType[2];
	short axisIndex[2];
} TCompensate2D;

GT_API GTN_SetCompensate2DTable(short core,short tableIndex,TCompensate2DTable *pTable,int32_t *pData,short extend);
GT_API GTN_GetCompensate2DTable(short core,short tableIndex,TCompensate2DTable *pTable,short *pExtend);
GT_API GTN_SetCompensate2DTableRotationAngle(short core,short tableIndex,short enable,double rotationAngle);
GT_API GTN_GetCompensate2DTableRotationAngle(short core,short tableIndex,short *pEnable,double *pRotationAngle);
GT_API GTN_SetCompensate2D(short core,short axis,TCompensate2D *pComp2d);
GT_API GTN_GetCompensate2D(short core,short axis,TCompensate2D *pComp2d);
GT_API GTN_GetCompensate2DValue(short core,short axis,double *pValue);
GT_API GTN_SetCompensate2DMode(short core,short axis,short mode);
GT_API GTN_GetCompensate2DMode(short core,short axis,short *pMode);

/*-----------------------------------------------------------*/
/* IO and Encoder                                            */
/*-----------------------------------------------------------*/
typedef struct
{
	short pad1[2];
	int32_t reverseTime;
	int32_t pad2[3];
	double pad3[2];
}TDoBit;

GT_API GTN_SetDo(short core,short doType,int32_t value);
GT_API GTN_SetDoEx(short core,short doType,int32_t *pValue,short count);
GT_API GTN_SetDoBit(short core,short doType,short doIndex,short value);
GT_API GTN_GetDo(short core,short doType,int32_t *pValue);
GT_API GTN_GetDoEx(short core,short doType,int32_t *pValue,short count);
GT_API GTN_SetDoBitReverse(short core,short doType,short doIndex,short value,short reverseTime);
GT_API GTN_SetDoBitReverseEx(short core,short doType,short doIndex,short value,TDoBit *pDoBit);
GT_API GTN_EnableDoBitPulse(short core,short doType,short doIndex,unsigned short highLevelTime,unsigned short lowLevelTime,int32_t pulseNum,short firstLevel);
GT_API GTN_DisableDoBitPulse(short core,short doType,short doIndex);
GT_API GTN_GetDi(short core,short diType,int32_t *pValue);
GT_API GTN_GetDiBit(short core,short diType,short diIndex,short *pValue);
GT_API GTN_GetDiReverseCount(short core,short diType,short diIndex,uint32_t *pReverseCount,short count=1);
GT_API GTN_SetDiReverseCount(short core,short diType,short diIndex,uint32_t *pReverseCount,short count=1);
GT_API GTN_GetDiRaw(short core,short diType,int32_t *pValue);
GT_API GTN_GetDiEx(short core,short diType,int32_t *pValue,short count);

GT_API GTN_SetGtmDoBit(short core,short station,short slot,short doIndex,short *pValue,short count);
GT_API GTN_GetGtmDoBit(short core,short station,short slot,short doIndex,short *pValue,short count);
GT_API GTN_GetGtmDiBit(short core,short station,short slot,short doIndex,short *pValue,short count);

GT_API GTN_SetDac(short core,short dac,short *pValue,short count=1);
GT_API GTN_GetDac(short core,short dac,short *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_SetAuDac(short core,short dac,short *pValue,short count=1);
GT_API GTN_GetAuDac(short core,short dac,short *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAdc(short core,short adc,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAdcValue(short core,short adc,short *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAuAdc(short core,short adc,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAuAdcValue(short core,short adc,short *pValue,short count=1,uint32_t *pClock=NULL);

GT_API GTN_SetAdcFilter(short core,short adc,short filterTime);
GT_API GTN_SetAdcFilterPrm(short core,short adc,double k);
GT_API GTN_GetAdcFilterPrm(short core,short adc,double *pk);
GT_API GTN_SetAuAdcFilterPrm(short core,short adc,double k);
GT_API GTN_GetAuAdcFilterPrm(short core,short adc,double *pk);

GT_API GTN_SetAdcBias(short core,short adc,short bias);
GT_API GTN_GetAdcBias(short core,short adc,short *pBias);
GT_API GTN_SetAuAdcBias(short core,short auAdc,short bias);
GT_API GTN_GetAuAdcBias(short core,short auAdc,short *pBias);


#define DIGITAL_OUTPUT_MODE_NORMAL                    (0)
#define DIGITAL_OUTPUT_MODE_REVERSE_TIME              (10)

typedef struct
{
	double time;
	double reserve[9];
}TDoReverseParameter;

typedef union
{
	TDoReverseParameter doReverse;
	double data[10];
}TDigitalOutputMode;

typedef struct
{
	double distance;
	double delayTime;
	double reserve[8];
}TAtParameter;

typedef struct
{
	double distance;
	double reserve[9];
}TPsoParameter;

typedef struct
{
	double time;
	double reserve[9];
}TTsoParameter;

typedef struct
{
	TAtParameter atPrm;
	TPsoParameter psoPrm;
	TTsoParameter tsoPrm;
	TDoReverseParameter doReverse;
	double data[10];
}TWriteDigitalOutputMode;

typedef struct
{
	short mode;              //Do输出模式
	short doType;
	short doIndex;
	short doCount;
	short *pValue;
	short reverse1[2];
	int32_t  reserve2[2];
	TWriteDigitalOutputMode prm;
}TDigitalOutput;

typedef struct
{
	short mode;              //Do输出模式
	short doType;
	short doIndex;
	short doValue;
	short reverse1[4];
	int32_t  reserve2[2];
	TDigitalOutputMode prm;
}TDigitalOutputBit;

typedef struct
{
	double delayTime;
	short reserve1[4];
	int32_t reserve2[2];
	double reserve3[2];
}TDelay;

typedef struct
{
	short type;					// type=0:距离起点distance后Do输出；type=1:距离终点distance时Do输出
	short motionType;			// 保留，必须为0，目前只支持插补运动指令，
	int32_t delayTime;				// 保留，必须为0

	double distance;			// 运动distance后Do输出，单位mm
}TDigitalOutputProByMoveDistance;

typedef struct
{
	short type;					// type=0:开始运动后延时delayTime后Do输出；type1:运动结束前提前delayTimeDo输出
	short motionType;			// 保留，必须为0，目前只支持插补运动指令，
	short reserve1[2];			// 保留，必须为0
	double delayTime;			// 时间,单位ms
}TDigitalOutputProByMoveTime;

typedef struct
{
	double time;				// 延时输出的时间，单位ms
}TDigitalOutputProDelay;

typedef union
{
	double data[30];
	TDigitalOutputProDelay delay;					// 纯延时输出
	TDigitalOutputProByMoveTime moveTime;			// 开始运动后，根据运动时间输出。
	TDigitalOutputProByMoveDistance moveDistance;  // 开始运动后，根据运动距离输出。
}TWriteDigitalOutputProPrmUnion;

typedef struct
{
	// Do输出模式:
	// 模式1：延时输出;
	// 模式2：执行下一条插补指令时，按照设定的距离段前延迟输出或者段末提前输出;
	// 模式3：执行下一条插补指令时，按照设定的时间段前延迟输出或者段末提前输出;
	short mode;              // Do输出模式
	short doType;		     // Do类型
	short doIndex;			 // Do索引
	short doCount;			 // Do输出个数
	short *pValue;			 // Do输出值，
	short reserve1[2];		 // 保留，必须为0
	int32_t  reserve2[2];		 // 保留，必须为0
	TWriteDigitalOutputProPrmUnion prm;
}TDigitalOutputPro;

typedef struct
{
	short aoType;
	short aoIndex;
	short count;
	short reserve1;
	double *pValue;
	int32_t reserve2[3];
}TAnalogOutput;

typedef struct
{
	short diType;
	short diIndex;
	short diCount;
	short *pValue;
	short reverse1[3];
	int32_t  reserve2[2];
}TDigitalInput;

GT_API GTN_ReadDigitalInput(short core,short diType,short diIndex,short *pDi,short diCount);
GT_API GTN_WaitDigitalInput(short core,TDigitalInput *pDi,TListInfo *pListInfo);
GT_API GTN_WriteDigitalOutput(short core,TDigitalOutput *pDo,TListInfo *pListInfo);
GT_API GTN_WriteDigitalOutputBit(short core,TDigitalOutputBit *pDoBit,TListInfo *pListInfo=NULL);
GT_API GTN_ReadDigitalOutput(short core,short doType,short doIndex,short *pValue,short doCount);
GT_API GTN_ReadAnalogInput(short core,short adcType,short adcIndex,double *pValue,short adcCount);
GT_API GTN_WriteAnalogOutput(short core,TAnalogOutput *pAo,TListInfo *pListInfo);
GT_API GTN_SetDelay(short core,TDelay *pDelay,TListInfo *pListInfo);

GT_API GTN_SetAuMtrBias(short core,short dac,short bias);
GT_API GTN_GetAuMtrBias(short core,short dac,short *pBias);
GT_API GTN_SetAuMtrLmt(short core,short dac,short limit);
GT_API GTN_GetAuMtrLmt(short core,short dac,short *pLimit);

GT_API GTN_SetEncPrm(short core,short encoder,int32_t lineNum,short type);
GT_API GTN_GetEncPrm(short core,short encoder,int32_t *pLineNum,short *pType);
GT_API GTN_GetEncLineNum(short core,short encoder,int32_t *pLineNum,short count,uint32_t *pClock);
GT_API GTN_GetEncType(short core,short encoder,short *pType,short count,uint32_t *pClock);
GT_API GTN_SetGratingScale(short core,short index,short *pScale,short count=1);
GT_API GTN_GetGratingScale(short core,short index,short *pScale,short count=1);
GT_API GTN_SetEncPos(short core,short encoder,int32_t encPos);
GT_API GTN_SetEncPosEx(short core,short encoder,double pos);
GT_API GTN_GetEncPos(short core,short encoder,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetEncPosPre(short core,short encoder,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetEncPosRegist(short core,short encoder,double *pValue,short count,uint32_t *pClock);
GT_API GTN_GetEncVel(short core,short encoder,double *pValue,short count=1,uint32_t *pClock=NULL);

GT_API GTN_EncSns(short core,unsigned short sense);
GT_API GTN_LmtSns(short core,unsigned short sense);
GT_API GTN_GpiSns(short core,unsigned short sense);


GT_API GTN_SetPlsPos(short core,short encoder,int32_t encPos);
GT_API GTN_GetPlsPos(short core,short pulse,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetPlsVel(short core,short pulse,double *pValue,short count=1,uint32_t *pClock=NULL);

GT_API GTN_SetAuEncPos(short core,short encoder,int32_t encPos);
GT_API GTN_GetAuEncPos(short core,short encoder,double *pValue,short count=1,uint32_t *pClock=NULL);
GT_API GTN_GetAuEncVel(short core,short encoder,double *pValue,short count=1,uint32_t *pClock=NULL);

GT_API GTN_GetAbsEncPos(short core,short encoder,int32_t *pValue,short mode=0,short param=0);
GT_API GTN_GetAbsEncPosEx(short core,short encoder,short mode,__int64 *pValue);

typedef struct
{
	short type;
	short index;
}TEncoderSource;
GT_API GTN_SetEncoderSource(short core,short encoder,TEncoderSource *pEncoderSource);
GT_API GTN_GetEncoderSource(short core,short encoder,TEncoderSource *pEncoderSource);

typedef struct
{
	short probeType;			// 捕获类型
	short probeIndex;			// 捕获索引
	short latchType;			// 锁存类型
	short latchIndex;			// 锁存索引

	short sense;				// 捕获电平
	short pad[3];
	double filterWidth;			// 捕获最小宽度
}TSignalDetect;

typedef struct
{
	short status;				// 完成标志,0，未触发，1计数中，2计数完成
	short pad[3];
	double latchWidth;			// 锁存脉宽值
}TSignalDetectStatus;

GT_API GTN_SetSignalTimeFilter(short core,short type,short index,double filterWidth);
GT_API GTN_GetSignalTimeFilter(short core,short type,short index,double *pFilterWidth);
GT_API GTN_ClearSignalDetect(short core,short index);
GT_API GTN_SetSignalDetect(short core,short index,short enable,TSignalDetect *pPrm);
GT_API GTN_GetSignalDetect(short core,short index,short *pEnable,TSignalDetect *pPrm);
GT_API GTN_GetSignalDetectStatus(short core,short index,TSignalDetectStatus *pPrm);


/*-----------------------------------------------------------*/
/* ExtModule                                                 */
/*-----------------------------------------------------------*/
GT_API GTN_ExtModuleInit(short core,short method=1);
GT_API GTN_SetILinkManuMode(short core,short station);
GT_API GTN_GetILinkManuId(short core,short station,short autoId,short *manuId);
GT_API GTN_GetILinkAutoId(short core,short station,short manuId,short *autoId);
GT_API GTN_SetExtDoBit(short core,short doIndex,short value);
GT_API GTN_GetExtDoBit(short core,short doIndex,short *pValue);
GT_API GTN_SetExtDo(short core,short doIndex,int32_t value,int32_t mask);
GT_API GTN_GetExtDo(short core,short doIndex,int32_t *pValue);
GT_API GTN_GetExtDi(short core,short diIndex,int32_t *pValue);
GT_API GTN_GetExtDiBit(short core,short diIndex,short *pValue);
GT_API GTN_SetExtAoValue(short core,short index,short *pValue,short count=1);
GT_API GTN_GetExtAiValue(short core,short index,short *pValue,short count=1);
GT_API GTN_SetExtAo(short core,short index,double *pValue,short count=1);
GT_API GTN_GetExtAo(short core,short index,double *pValue,short count=1);
GT_API GTN_GetExtAi(short core,short index,double *pValue,short count=1);


GT_API GTN_SetILinkDo(short core,short station,short module,unsigned short data,unsigned short mask);
GT_API GTN_GetILinkDi(short core,short station,short module,unsigned short *data);
GT_API GTN_SetILinkAo(short core,short station,short module,short channel,double data);
GT_API GTN_GetILinkAi(short core,short station,short module,short channel,double *data);

GT_API GTN_SetEHMIType(short core,int32_t length);
GT_API GTN_InitialEHMIIO(short core,char* pCfgFile, char cfgType);
GT_API GTN_InitialEHMIPrm(short core,short inputCount,short outputCount);
GT_API GTN_GetEHMIDo(short core,unsigned char *data,unsigned short offset,unsigned short count);
GT_API GTN_SetEHMIDo(short core,unsigned char *data,unsigned short offset,unsigned short count);
GT_API GTN_GetEHMIDi(short core,unsigned char *data,unsigned short offset,unsigned short count);

GT_API GTN_OpenExtMdl(short core,char *pDllName);
GT_API GTN_CloseExtMdl(short core);
GT_API GTN_ResetExtMdl(short core);

/*-----------------------------------------------------------*/
/* Config of Ext-Module                                      */
/*-----------------------------------------------------------*/
typedef struct
{
	short active;
	short checkError;
	short linkError;
	short packageErrorCount;
	short pad[8];
} TExtModuleStatus;

typedef struct
{
	short type;
	short input;
	short output;
} TExtModuleType;

typedef struct
{
	short station;
	short module;
	short index;
} TExtIoMap;

GT_API GTN_LoadExtModuleConfig(short core,char *pFile);
GT_API GTN_SaveExtModuleConfig(short core,char *pFile);
GT_API GTN_ExtModuleOn(short core,short station);
GT_API GTN_ExtModuleOff(short core,short station);
GT_API GTN_GetExtModuleStatus(short core,short station,TExtModuleStatus *pStatus);
GT_API GTN_SetExtModuleId(short core,short station,short count,short *pId);
GT_API GTN_GetExtModuleId(short core,short station,short count,short *pId);
GT_API GTN_SetExtModuleReverse(short core,short station,short module,short inputCount,short *pInputReverse,short outputCount,short *pOutputReverse);
GT_API GTN_GetExtModuleReverse(short core,short station,short module,short inputCount,short *pInputReverse,short outputCount,short *pOutputReverse);
GT_API GTN_GetExtModuleCount(short core,short station,short *pCount);
GT_API GTN_GetExtModuleType(short core,short station,short module,TExtModuleType *pModuleType);
GT_API GTN_SetExtIoMap(short core,short type,short index,TExtIoMap *pMap);
GT_API GTN_GetExtIoMap(short core,short type,short index,TExtIoMap *pMap);
GT_API GTN_ClearExtIoMap(short core,short type);
GT_API GTN_SetExtAoRange(short core,short index,double max,double min);
GT_API GTN_GetExtAoRange(short core,short index,double *pMax,double *pMin);
GT_API GTN_SetExtAiRange(short core,short index,double max,double min);
GT_API GTN_GetExtAiRange(short core,short index,double *pMax,double *pMin);

GT_API GTN_LoadExtConfig(short core,char *pFileName);

/*-----------------------------------------------------------*/
/* Pos Compare                                               */
/*-----------------------------------------------------------*/
#define POS_COMPARE_MODE_FIFO                       (0)
#define POS_COMPARE_MODE_LINEAR                     (1)
#define POS_COMPARE_MODE_EQUIDISTANT                (2)

#define POS_COMPARE_OUTPUT_PULSE                    (0)
#define POS_COMPARE_OUTPUT_LEVEL                    (1)
#define POS_COMPARE_OUTPUT_AUTO                     (2)

#define POS_COMPARE_SOURCE_ENCODER                  (0)
#define POS_COMPARE_SOURCE_PULSE                    (1)


typedef struct
{
	short mode;
	short dimension;
	short sourceMode;
	short sourceX;
	short sourceY;
	short outputMode;
	short outputCounter;
	unsigned short outputPulseWidth;
	unsigned short errorBand;
} TPosCompareMode;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;

	int32_t startPos;
	int32_t interval;
} TPosCompareLinear;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;

	int32_t startPosX;
	int32_t startPosY;
	int32_t intervalX;
	int32_t intervalY;
} TPosCompareLinear2D;


typedef struct
{
	int32_t pos;
	unsigned short hso;
	unsigned short gpo;
	uint32_t segmentNumber;
} TPosCompareData;

typedef struct
{
	int32_t posX;
	int32_t posY;
	unsigned short hso;
	unsigned short gpo;
	uint32_t segmentNumber;
} TPosCompareData2D;

typedef struct
{
	unsigned short mode;
	unsigned short run;
	unsigned short space;
	uint32_t pulseCount;
	unsigned short hso;
	unsigned short gpo;
	uint32_t segmentNumber;
} TPosCompareStatus;

typedef struct
{
	unsigned short config;
	unsigned short fifoEmpty;
	unsigned short head;
	unsigned short tail;
	uint32_t commandReceive;
	uint32_t commandSend;
	int32_t posX;
	int32_t posY;
} TPosCompareInfo;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;
	int32_t startPosX;
	int32_t startPosY;
	int32_t syncPos;
	int32_t time;
	short reserve[20];
} TPosComparePsoPrm;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;
	int32_t startPosX;
	int32_t startPosY;
	int32_t syncPos;
	int32_t time;
	short pulseWidth;
	short pad;
} TPosComparePsoPrmPro;

typedef struct
{
	short mode;
	unsigned short count;
	short highLevel;
	short lowLevel;
	short resver[20];
} TPosCompareContinueMode;

typedef struct
{
	short	referenceX;
	short	referenceY;
	short	pad1[2];
	double	pad2[2];
} TPosCompareReferencePrm;

typedef struct
{
	short mode;          // 0：不输出，1：按默认设置输出，2：按照当前指令配置信息输出
	short timeScale;     // 时间精度：0：1us，1:0.1us
	short pad1[2];
	double pulseWidth;
	double pad2[3];
}THsoPulsePrm;

typedef struct
{
	short outputMode;
	short level;
	short reserve1[2];
	double highLevelTime;
	double lowLevelTime;
	double reserve2[4];
}TPosComparePulse;

typedef struct
{
	int32_t count;
	short reserve1[2];
	double reserve2[4];
}TPosComparePulseStatus;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;
	int32_t startPosX;
	int32_t startPosY;
	int32_t time;
	short multiNumber;
	short pad1;
	int32_t syncPosArray[256];
} TPosCompareMultiPsoPrm;


GT_API GTN_PosCompareStart(short core,short posCompareIndex);
GT_API GTN_PosCompareStop(short core,short posCompareIndex);
GT_API GTN_PosCompareClear(short core,short posCompareIndex);
GT_API GTN_PosCompareStatus(short core,short posCompareIndex,TPosCompareStatus *pStatus);
GT_API GTN_PosCompareData(short core,short posCompareIndex,TPosCompareData *pData);
GT_API GTN_PosCompareData2D(short core,short posCompareIndex,TPosCompareData2D *pData);
GT_API GTN_PosComparePulse(short core,short posCompareIndex,short outputMode,short level,unsigned short outputPulseWidth);
GT_API GTN_SetPosComparePulseCount(short core,short posCompareIndex,int32_t count);
GT_API GTN_PosComparePulseEx(short core,short posCompareIndex,TPosComparePulse *pPosComparePulse);
GT_API GTN_GetPosComparePulseStatus(short core,short posCompareIndex,TPosComparePulseStatus *pPosComparePulseStatus);
GT_API GTN_SetPosCompareMode(short core,short posCompareIndex,TPosCompareMode *pMode);
GT_API GTN_GetPosCompareMode(short core,short posCompareIndex,TPosCompareMode *pMode);
GT_API GTN_SetPosCompareContinueMode(short core,short posCompareIndex,TPosCompareContinueMode *pMode);
GT_API GTN_GetPosCompareContinueMode(short core,short posCompareIndex,TPosCompareContinueMode *pMode);
GT_API GTN_SetPosCompareLinear(short core,short posCompareIndex,TPosCompareLinear *pLinear);
GT_API GTN_GetPosCompareLinear(short core,short posCompareIndex,TPosCompareLinear *pLinear);
GT_API GTN_SetPosCompareLinear2D(short core,short posCompareIndex,TPosCompareLinear2D *pLinear);
GT_API GTN_GetPosCompareLinear2D(short core,short posCompareIndex,TPosCompareLinear2D *pLinear);
GT_API GTN_PosCompareInfo(short core,short posCompareIndex,TPosCompareInfo *pInfo);
GT_API GTN_PosCompareHsOn(short core,short posCompareIndex,short link,unsigned short threshold);
GT_API GTN_PosCompareHsOff(short core,short posCompareIndex);
GT_API GTN_PosCompareSpace(short core,short posCompareIndex,unsigned short *pSpace);
GT_API GTN_SetPosComparePsoPrm(short core,short posCompareIndex,TPosComparePsoPrm *pPrm);
GT_API GTN_GetPosComparePsoPrm(short core,short posCompareIndex,TPosComparePsoPrm *pPrm);
GT_API GTN_SetPosCompareMultiPsoPrm(short core,short index,TPosCompareMultiPsoPrm *pPrm);
GT_API GTN_GetPosCompareMultiPsoPrm(short core,short index,TPosCompareMultiPsoPrm *pPrm);
GT_API GTN_SetPosComparePsoOffDistance(short core,short index,int32_t distance);
GT_API GTN_SetPosCompareStartLevel(short core,short posCompareIndex,short type,short startLevel);
GT_API GTN_SetPosCompareReference(short core,short posCompareIndex,TPosCompareReferencePrm *pPrm);
GT_API GTN_GetPosCompareReference(short core,short posCompareIndex,TPosCompareReferencePrm *pPrm);
GT_API GTN_SetHsoPulsePrm(short core,short station,short hsoIndex,THsoPulsePrm *pPem,short hsoCount=1);
GT_API GTN_GetHsoPulsePrm(short core,short station,short hsoIndex,THsoPulsePrm *pPem,short hsoCount=1);
GT_API GTN_SetPosComparePulseDuty(short core,short index,double duty);
GT_API GTN_PosCompareVariablePsoData(short core,short index,TPosCompareData *pData);


typedef struct
{
	short			mode;							// 0：FIFO模式，1：Linear模式 2：等间距输出模式
	short			dimension;					//1：1D，2：2D
	short			sourceMode;					// 0：编码器； 1：脉冲计数器
	short			source[8];					//编码器比较源
	short			outputMode;					/*输出模式：0:脉冲 1:电平2：电平自
										动翻转(电平模式并且输出电平自动反转，不受posCompareData电平影响)*/
	short			outputCounter;				// 保留
	unsigned short	outputPulseWidth;	/*输出脉冲宽度,单位为1us，电平模式该参数无效*/
	unsigned short	errorBand;			// 二维位置比较输出误差带
	short			reserve1[2];
	double			reserve2[16];
} TPosCompareModeEx;

typedef struct
{
	uint32_t count;
	unsigned short hso;
	unsigned short gpo;
	int32_t startPosX;
	int32_t startPosY;
	int32_t time;

	short pad1;
	short multiNumber;
	int32_t scale[6];
	double syncPosArray[256];
} TPosComparePsoPrmEx;

GT_API GTN_SetPosCompareModeEx(short core,short index,TPosCompareModeEx *pMode);
GT_API GTN_GetPosCompareModeEx(short core,short index,TPosCompareModeEx *pMode);
GT_API GTN_SetPosComparePsoPrmEx(short core,short index,TPosComparePsoPrmEx *pPrm);
GT_API GTN_GetPosComparePsoPrmEx(short core,short index,TPosComparePsoPrmEx *pPrm);

GT_API GTN_ComparePulse(short core,short level,short outputType,short time);
GT_API GTN_CompareStop(short core);
GT_API GTN_CompareStatus(short core,short *pStatus,int32_t *pCount);
GT_API GTN_CompareData(short core,short encoder,short source,short pulseType,short startLevel,short time, int32_t *pBuf1,short count1,int32_t *pBuf2,short count2);
GT_API GTN_CompareLinear(short core,short encoder,short channel,int32_t startPos,int32_t repeatTimes,int32_t interval,short time,short source);

/*-----------------------------------------------------------*/
/* Config of Position Compare                                */
/*-----------------------------------------------------------*/
typedef struct
{
	short module;
	short fifo;
} TPosCompareMap;

GT_API GTN_SetPosCompareMap(short core,short index,TPosCompareMap *pMap);
GT_API GTN_GetPosCompareMap(short core,short index,TPosCompareMap *pMap);
GT_API GTN_ClearPosCompareMap(short core);

#define POS_COMPARE_MODE_PSO_SYNC_EXTERNAL_SIGNAL	(1)
#define POS_COMPARE_MODE_PSO_SYNC_INTERNAL_SIGNAL	(2)

GT_API GTN_SetPosComparePsoSyncPrm(short core,short posCompareIndex,short psoSyncMode,double frq=100);

#define COMPARE_SEND_DATA_MAX			(30)
#define COMPARE_DATA_MAX				(4096)
#define COMPARE_STEP_MAX				(0x1fff)
#define COMPARE_MAX_NUM					(0x3fffffff)


/*-----------------------------------------------------------*/
/* Laser and Scan                                            */
/*-----------------------------------------------------------*/
#define FIFO_MODE_STATIC                  (0)
#define FIFO_MODE_DYNAMIC                 (1)

#define SCAN_STATUS_WAIT                  (0)
#define SCAN_STATUS_RUN	                  (1)
#define SCAN_STATUS_DONE                  (2)

struct TScanInit
{
	int lookAheadNum;
	double time;
	double radiusRatio;
};

typedef struct
{
	uint32_t segmentNumber;
	unsigned short commandNumber;
	unsigned short prfVel;
	unsigned short fifoEmpty;
	unsigned short head;
	unsigned short tail;
	uint32_t commandReceive;
	uint32_t commandSend;
	uint32_t reserve[6];
} TScanInfo;

typedef struct
{
	short enable;
	short superposeSrc;
	short superposeAxisX;
	short superposeAxisY;
	double xCoefficient;
	double yCoefficient;
	double xVelCoefficient;
	double yVelCoefficient;
}TScanPosSuperposeParameter;

typedef struct
{
	unsigned short hso;
	unsigned short powerMode;
	unsigned short power;
	unsigned short powerMax;
	unsigned short powerMin;
	unsigned short frequency;
	unsigned short pulseWidth;
} TLaserInfo;

typedef struct
{
	short n;
	double startVel;
	double power;
}TLaserPowerPrm;

typedef struct
{
	short n;
	double startVel;
	double velStep;
	double *power;
}TLaserPowerTable;

typedef struct
{
	short corrX[65][65];
	short corrY[65][65];
}TScanCorrectionTableData;

GT_API GTN_ScanInit(short core,TScanInit *pScanInit=NULL,double jumpAcc=0,double markAcc=0,short scan=1);
GT_API GTN_ScanCrdDataEnd(short core,short scan=1);
GT_API GTN_SetScanLaserLink(short core,short link,short scan=1);
GT_API GTN_GetScanLaserLink(short core,short *pLink,short scan=1);
GT_API GTN_SetScanMode(short core,short mode,short scan=1);
GT_API GTN_GetScanMode(short core,short *pMode,short scan=1);
GT_API GTN_ClearScanStatus(short core,short scan=1);
GT_API GTN_ScanGetCrdPos(short core,short *pPos,short scan=1);
GT_API GTN_ScanJump(short core,short x,short y,double vel,short scan=1);
GT_API GTN_ScanJumpPoint(short core,short x,short y,double vel,int32_t motionDelayTime,int32_t laserDelayTime,short scan=1);
GT_API GTN_ScanTimeJump(short core,short x,short y,unsigned short time,short scan=1);
GT_API GTN_ScanTimeJumpPoint(short core,short x,short y,unsigned short time,int32_t motionDelayTime,int32_t laserDelayTime,short scan=1);
GT_API GTN_ScanMark(short core,short x,short y,double vel,short scan=1);
GT_API GTN_ScanTimeMark(short core,short x,short y,unsigned short time,short scan=1);
GT_API GTN_ScanBufLaserPrfCmd(short core,double laserPower,short scan=1);
GT_API GTN_ScanBufIO(short core,unsigned short doType,unsigned short doMask,unsigned short doValue,short scan=1);
GT_API GTN_ScanBufDelay(short core,int32_t time,short scan=1);
GT_API GTN_ScanBufDA(short core,unsigned short chn,short value,short scan=1);
GT_API GTN_ScanBufAO(short core,unsigned short chn,double voltage,short scan=1);
GT_API GTN_ScanBufLaserDelay(short core,short laserOnDelay,short laserOffDelay,short scan=1);
GT_API GTN_ScanBufLaserDelayLong(short core,int32_t laserOnDelay,int32_t laserOffDelay,short scan=1);
GT_API GTN_ScanBufLaserOutFrq(short core,double outFrq,short scan=1);
GT_API GTN_ScanBufSetPulseWidth(short core,unsigned short width,short scan=1);
GT_API GTN_ScanBufLaserOn(short core,short scan=1);
GT_API GTN_ScanBufLaserOff(short core,short scan=1);
GT_API GTN_ScanBufStop(short core,short scan=1);
GT_API GTN_ScanLaserIntervalOnList(short core,int32_t time,short scan=1);
GT_API GTN_SetScanDelayTime(short core,unsigned short maxJumpDelay,unsigned short markDelay,unsigned short multiMarkDelayConst,short scan=1);
GT_API GTN_SetScanDelayMode(short core,short multiMarkDelayMode,unsigned short multiMarkLaserOffDelay,unsigned short minJumpDelay,unsigned short jumpDelayLengthLimit,short scan=1);
GT_API GTN_ScanStop(short core,short stopType,short scan=1);
GT_API GTN_ScanCrdSpace(short core,short *pSpace,short scan=1);
GT_API GTN_ScanCrdStart(short core,short scan=1);
GT_API GTN_ScanCrdClear(short core,short scan=1);
GT_API GTN_ScanCrdStatus(short core,short *pRun,short *pCmdId,short scan=1);

GT_API GTN_SetScanPosSuperposeParameter(short core,short scan,TScanPosSuperposeParameter param);
GT_API GTN_GetScanPosSuperposeParameter(short core,short scan,TScanPosSuperposeParameter *pParam);
GT_API GTN_ScanSetCorrectionTable(short core,short scan,TScanCorrectionTableData *pParam);
GT_API GTN_ScanCorrectionOn(short core,short scan);
GT_API GTN_ScanCorrectionOff(short core,short scan);
GT_API GTN_ScanGenerateCorrectionTable(short core,double paraX,double paraY,short rangeX,short rangeY,TScanCorrectionTableData *pParam);
GT_API GTN_GetScanPosSuperposeEncPos(short core,short scan,double *pXPos,double *pYPos);


GT_API GTN_SetScanDaType(short core,short type,short scan);

GT_API GTN_ScanInfo(short core,TScanInfo *pScanInfo,short scan=1);
GT_API GTN_GetScanExecuteTime(short core,short scan,double *pExecuteTime);
GT_API GTN_ClearScanExecuteTime(short core,short scan=1);
GT_API GTN_ScanHsOn(short core,short scan=1,short link=1,unsigned short threshold=200);
GT_API GTN_ScanHsOff(short core,short scan=1);

GT_API GTN_LaserOn(short core,short laserChannel=0);
GT_API GTN_LaserOff(short core,short laserChannel=0);
GT_API GTN_LaserOnStatus(short core,unsigned short *pValue,short laserChannel=0);
GT_API GTN_LaserPowerMode(short core,short laserPowerMode,double maxValue,double minValue,short laserChannel=0);
GT_API GTN_LaserPrfCmd(short core,double power,short laserChannel=0);
GT_API GTN_LaserOutFrq(short core,double outFrq,short laserChannel=0);
GT_API GTN_SetPulseWidth(short core,unsigned short width,short laserChannel=0);
GT_API GTN_SetLevelDelay(short core,uint32_t highLevelDelay,uint32_t lowLevelDelay,short laserChannel=0);
GT_API GTN_LaserInfo(short core,TLaserInfo *pLaserInfo,short laserChannel=0);
GT_API GTN_WriteLaserPrfCmd(short core,double power,short channel);

typedef struct
{
	short run;
	short mode;
	double power;
	double frequency;
	double pulseWidth;
	double pad1[9];
	short pad2[8];
} TLaserStatus;
GT_API GTN_GetLaserStatus(short core,short channel,TLaserStatus *pStatus);

GT_API GTN_ScanLaserOn(short core,short scan=1);
GT_API GTN_ScanLaserOff(short core,short scan=1);
GT_API GTN_ScanLaserOnStatus(short core,unsigned short *pValue,short scan=1);
GT_API GTN_ScanSetLaserMode(short core,unsigned short laserMode, short scan=1);
GT_API GTN_ScanLaserPowerMode(short core,short laserPowerMode,double maxValue,double minValue, short scan=1);
GT_API GTN_ScanLaserPrfCmd(short core,double power, short scan=1);
GT_API GTN_ScanLaserOutFrq(short core,double outFrq, short scan=1);
GT_API GTN_ScanSetPulseWidth(short core,unsigned short width, short scan=1);
GT_API GTN_ScanSetLevelDelay(short core,unsigned short highLevelDelay,unsigned short lowLevelDelay, short scan=1);
GT_API GTN_ScanLaserInfo(short core,TLaserInfo *pLaserInfo, short scan=1);

GT_API GTN_SetHsoPwmLink(short core,short station,short hsoIndex);
GT_API GTN_GetHsoPwmLink(short core,short station,short *pHsoIndex);

GT_API GTN_ScanSynchOn(short core);
GT_API GTN_ScanSynchOff(short core);
GT_API GTN_SetScanSynchModePrm(short core,short masterScan,short count,short *synchScan,short synchSignalSource);
GT_API GTN_GetScanWorkMode(short core,short scan,short *pMode);
/*-----------------------------------------------------------*/
/* DLM                                                       */
/*-----------------------------------------------------------*/
#define DLM_FUNCTION_EVENT							(0)
#define DLM_FUNCTION_TIMER							(1)
#define DLM_FUNCTION_BACKGROUND						(2)
#define DLM_FUNCTION_COMMAND						(3)

#define DLM_FUNCTION_PROCEDURE						(7)

#define DLM_FUNCTION_PROFILE_EVENT					(8)
#define DLM_FUNCTION_PROFILE						(9)
#define DLM_FUNCTION_PROFILE_SUPERIMPOSED			(10)
#define DLM_FUNCTION_PROFILE_FILTER					(11)

#define DLM_FUNCTION_SERVO_EVENT					(16)
#define DLM_FUNCTION_SERVO							(17)
#define DLM_FUNCTION_SERVO_SUPERIMPOSED				(18)
#define DLM_FUNCTION_SERVO_FILTER					(19)

#define DLM_LOAD_MODE_NONE							(0)
#define DLM_LOAD_MODE_COMMAND						(1)
#define DLM_LOAD_MODE_BOOT							(2)
#define DLM_LOAD_MODE_RUN							(3)

typedef struct TDlmStatus
{
	int32_t version;
	int32_t date;
	short enable;
	int32_t function;
} TDlmStatus;

typedef struct TDlmFunction
{
	short function;
	short enable;
	int32_t value;
} TDlmFunction;

GT_API GTN_LoadDlm(short core,int32_t vender,int32_t module,char *fileName,short *pId);
GT_API GTN_ProgramDlm(short core,short id,short loadMode);
GT_API GTN_GetDlmLoadMode(short core,short id,short *pLoadMode);
GT_API GTN_RunDlm(short core,short id);
GT_API GTN_StopDlm(short core,short id);
GT_API GTN_GetDlmStatus(short core,short id,TDlmStatus *pStatus);
GT_API GTN_SetDlmFunction(short core,short id,TDlmFunction *pFunction);
GT_API GTN_GetDlmFunction(short core,short id,TDlmFunction *pFunction);

GT_API GTN_DlmCommandInit(short core,short code,int32_t index);
GT_API GTN_DlmCommandAdd16(short core,short value);
GT_API GTN_DlmCommandAdd32(short core,int32_t value);
GT_API GTN_DlmCommandAddFloat(short core,float value);
GT_API GTN_DlmCommandAddDouble(short core,double value);
GT_API GTN_SendDlmCommand(short core,short id,short *pReturnValue);
GT_API GTN_DlmCommandGet16(short core,short *pValue);
GT_API GTN_DlmCommandGet32(short core,int32_t *pValue);
GT_API GTN_DlmCommandGetFloat(short core,float *pValue);
GT_API GTN_DlmCommandGetDouble(short core,double *pValue);
/*-----------------------------------------------------------*/
/* Event-Task                                                */
/*-----------------------------------------------------------*/
#define TASK_SET_DO_BIT							(0x1101)
#define TASK_SET_DAC							(0x1120)

#define TASK_STOP								(0x1303)

#define TASK_UPDATE_POS							(0x2002)
#define TASK_UPDATE_VEL							(0x2004)
#define TASK_UPDATE_DISTANCE					(0x2022)

#define TASK_PT_START							(0x2306)
#define TASK_PVT_START							(0x2346)
#define TASK_MOVE_ABSOLUTE						(0x2500)

#define TASK_GEAR_START							(0x3005)

#define TASK_FOLLOW_START						(0x310A)
#define TASK_FOLLOW_SWITCH						(0x310B)

#define TASK_CRD_START							(0x4004)
#define TASK_CRD_OVERRIDE						(0x4006)
#define TASK_SCAN_START							(0x4102)

#define TASK_MOVE_ESCAPE						(0x5000)

#define TASK_SET_DO_BIT_MODE_NONE				(0)
#define TASK_SET_DO_BIT_MODE_TIME				(10)
#define TASK_SET_DO_BIT_MODE_DISTANCE			(20)

//保存运控变量任务，以及保存变量的最大个数
#define TASK_SAVE_MC_VAR_MAX                    (5)
#define TASK_SAVE_MC_VAR                        (50)

typedef struct
{
	unsigned short type;
	unsigned short index;
	unsigned short id;
} TWatchVar;

typedef struct
{
	short doType;
	short doIndex;
	short doValue;
	short mode;
	int32_t parameter[8];
} TTaskSetDoBit;

typedef struct
{
	short dac;
	short value;
} TTaskSetDac;

typedef struct
{
	int32_t mask;
	int32_t option;
} TTaskStop;

typedef struct
{
	short type;
	short index;
	short operation;
	short data[20];
} TTaskFifoOperation;

typedef struct
{
	short profile;
	int32_t pos;
} TTaskUpdatePos;

typedef struct
{
	short profile;
	short triggerIndex;
	int32_t distance;
} TTaskUpdateDistance;

typedef struct
{
	short profile;
	double vel;
} TTaskUpdateVel;

typedef struct
{
	int32_t mask;
	int32_t option;
} TTaskPtStart;

typedef struct
{
	int32_t mask;
} TTaskPvtStart;

typedef struct
{
	int32_t mask;
} TTaskGearStart;

typedef struct
{
	int32_t mask;
	int32_t option;
} TTaskFollowStart;

typedef struct
{
	int32_t mask;
} TTaskFollowSwitch;

typedef struct
{
	short profile;
	int32_t pos;
	double vel;
	double acc;
	double dec;
	short percent;
} TTaskMoveAbsolute;

typedef struct
{
	short mask;
	short option;
} TTaskCrdStart;

typedef struct
{
	short crd;
	double synVelOverride;
} TTaskCrdOverride;

typedef struct
{
	short port;
	short index;
	short count;
} TTaskScanStart;

typedef struct
{
	short count;
	TWatchVar var[TASK_SAVE_MC_VAR_MAX];
}TTaskSaveMcVar;

typedef struct
{
	uint32_t loop;
	TWatchVar var;
	unsigned short condition;
	double value;
} TEvent;

GT_API GTN_ClearEvent(short core);
GT_API GTN_ClearTask(short core);
GT_API GTN_ClearEventTaskLink(short core);
GT_API GTN_DeleteEvent(short core,short eventIndex);
GT_API GTN_DeleteTask(short core,short taskIndex);
GT_API GTN_DeleteEventTaskLink(short core,short linkIndex);
GT_API GTN_AddEvent(short core,TEvent *pEvent,short *pEventIndex);
GT_API GTN_AddTask(short core,short taskType,void *pTaskData,short *pTaskIndex);
GT_API GTN_AddEventTaskLink(short core,short eventIndex,short taskIndex,short *pLinkIndex);
GT_API GTN_SetEvent(short core,TEvent *pEvent,short eventIndex);
GT_API GTN_SetTask(short core,short taskType,void *pTaskData,short taskIndex);
GT_API GTN_SetEventTaskLink(short core,short eventIndex,short taskIndex,short linkIndex);
GT_API GTN_GetEventCount(short core,short *pCount);
GT_API GTN_GetEvent(short core,short eventIndex,TEvent *pEvent);
GT_API GTN_GetEventLoop(short core,short eventIndex,uint32_t *pEventLoop);
GT_API GTN_GetTaskCount(short core,short *pCount);
GT_API GTN_GetTask(short core,short taskIndex,short *pTaskType,void *pTaskData);
GT_API GTN_GetEventTaskLinkCount(short core,short *pCount);
GT_API GTN_GetEventTaskLink(short core,short linkIndex,short *pEventIndex,short *pTaskIndex);
GT_API GTN_EventOn(short core,short eventIndex,short count);
GT_API GTN_EventOff(short core,short eventIndex,short count);
GT_API GTN_GetTaskSaveMcVarResult(short core,short taskIndex,TWatchVar *pVar,double *pValue,short count,short *pReadCount);

GT_API GTN_SetEventWaitTime(short core,double waitTime);


#define WAIT_TIMEOUT_MODE_INFINITY          (0)
#define WAIT_TIMEOUT_MODE_SKIP              (1)
#define WAIT_TIMEOUT_MODE_STOP              (2)

typedef struct
{
	TWatchVar var;
	unsigned short condition;
	double value;
} TWatchCondition;

typedef struct
{
	unsigned short operation;
	unsigned short varType;
	unsigned short result;
	unsigned short lhs;
	unsigned short rhs;
} TVarCalculate;

typedef struct
{
	short varIndex;
	short reserve[3];
	TWatchCondition watchCondition;
} TVarCondition;

typedef struct
{
	int32_t time;				// 超时时间
	short mode;		        // 超时后的行为，0：无限等待，1：跳过当前等待操作继续执行指令流，2：停止指令流
	short reserve1;
	double reserve2[4];
} TWaitTimeout;

typedef struct
{
	short enable;
	short loop;
	short reserve[2];
} TConditionTaskEnable;

typedef struct
{
	short halt;
	short reserve1[15];
} TCommandListStopParameter;

//停止指令流
typedef struct
{
	short stopList;
	short stopMode;
	short triggerSource;
	short reserve[5];
	TCommandListStopParameter prm;
} TTaskStopCommandList;

GT_API GTN_SetVarBoolCondition(short core,short varIndex,TWatchCondition *pWatchCondition);
GT_API GTN_GetVarBoolCondition(short core,short varIndex,TWatchCondition *pWatchCondition);
GT_API GTN_LoadVarCalculate(short core,TVarCalculate *pVarCalculate,short count,TListInfo *pListInfo=NULL);
GT_API GTN_AddVarCalculate(short core,TVarCalculate *pVarCalculate,short *pIndex);
GT_API GTN_GetVarCalculate(short core,short index,TVarCalculate *pVarCalculate);
GT_API GTN_GetVarCalculateCount(short core,short *pCount);
GT_API GTN_SetVarCondition(short core,short varIndex,TWatchCondition *pWatchCondition,short conditionCount,short operation,TListInfo *pListInfo);
GT_API GTN_GetVarCondition(short core,short varIndex,TWatchCondition *pWatchCondition,short *pConditionCount,short *pOperation);
GT_API GTN_ClearVar(short core);

GT_API GTN_GetMcVar(short core,const TWatchVar *pVar,double *pValue);
GT_API GTN_SetMcVar(short core,const TWatchVar *pVar,double value,TListInfo *pListInfo);
GT_API GTN_GetMcVarEx(short core,const TWatchVar *pVar,double *pValue,short count);
GT_API GTN_SetMcVarEx(short core,const TWatchVar *pVar,double *pValue,short count,TListInfo *pListInfo);

GT_API GTN_WaitForCondition(short core,TWatchCondition *pWatchCondition,short conditionCount,short operation,TWaitTimeout *pTimeout,TListInfo *pListInfo=NULL);
GT_API GTN_GetWaitForCondition(short core,short list,TWatchCondition *pWatchCondition,short *pConditionCount,short *pOperation,TWaitTimeout *pTimeout,short *pConditionResult,short *pConditionDone);

GT_API GTN_SetConditionTask(short core,short conditionTaskindex,TWatchCondition *pWatchCondition,short conditionCount,short operation,short taskType,void *pTaskData,TConditionTaskEnable *pEnable,TListInfo *pListInfo);
GT_API GTN_GetConditionTask(short core,short conditionTaskIndex,TWatchCondition *pWatchCondition,short *pConditionCount,short *pOperation,short *pTaskType,void *pTaskData,TConditionTaskEnable *pEnable);
GT_API GTN_ConditionTaskEnable(short core,short conditionTaskIndex,TConditionTaskEnable *pEnable,TListInfo *pListInfo);
GT_API GTN_ConditionStopCommandList(short core,TWatchCondition *pWatchCondition,short conditionCount,short operation,TTaskStopCommandList *pTask,TListInfo *pListInfo);

GT_API GTN_GetGroupConditionTriggerPos(short core,short group,short index,double *pValue,short count,short coordSystem,short oriMode);
GT_API GTN_GetAxisConditionTriggerPos(short core,short type,short axis,double *pValue,short count);

/*--------- -------------------------------------------------*/
/* Gantry                                                    */
/*-----------------------------------------------------------*/
#define GANTRY_MODE_NONE					 			(-1)
#define GANTRY_MODE_OPEN_LOOP_GANTRY					(1)
#define GANTRY_MODE_DECOUPLE_POSITION_LOOP				(2)
#define GANTRY_MODE_RINGNET_DRIVER						(10)

GT_API GTN_SetGantryMode(short core,short group,short master,short slave,short mode,int32_t syncErrorLimit=1000);
GT_API GTN_GetGantryMode(short core,short group,short *pMaster,short *pSlave,short *pMode,int32_t *pSyncErrorLimit);
GT_API GTN_SetGantryPid(short core,short group,TPid *pGantryPid,TPid *pYawPid);
GT_API GTN_GetGantryPid(short core,short group,TPid *pGantryPid,TPid *pYawPid);
GT_API GTN_GantryAxisOn(short core,short group);
GT_API GTN_GantryAxisOff(short core,short group);

GT_API GTN_SetAxisInputShapeVerI(short core,short axis, short enable, short freq, double z);

typedef struct
{
	short windowSize;										// 窗口大小
	short lostSize;											// 使能后多少拍以后开始检查
	int32_t torqueLimitMax;									// 最大值
	int32_t torqueLimitMin;									// 最小值

	short continuousProtectCount;							// 窗宽范围内，实际电流连续这么多拍超过最大值或者最小值则保护触发
	short oscillationPositiveProtectCount;					// 窗宽范围内，实际电流总共这么多拍大于最大值则保护触发
	short oscillationNegativeProtectCount;					// 窗宽范围内，实际电流总共这么多拍小于最小值则保护触发
	short reserve[15];
} TGantryTorqueProtectPrm;
GT_API GTN_SetGantryProtectPrm(short core,short group,short torqueProtectType,TGantryTorqueProtectPrm *pPrm);
GT_API GTN_GetGantryProtectPrm(short core,short group,short torqueProtectType,TGantryTorqueProtectPrm *pPrm);
GT_API GTN_GantryProtectEnable(short core,short group,short enable,short protectType,short subProtectType);

typedef struct
{
	int32_t torqueProtectSts;												// 触发状态
	int32_t torqueProtectEnable;											// 触发使能状态
	short reserve[20];
} TGantryInf;
GT_API GTN_GetGantryInf(short core,short group,TGantryInf *pPrm);

GT_API GTN_SetGantrySynchErrorCompensate2DTable(short core,short tableIndex,TCompensate2DTable *pTable,int32_t *pData,short extend);
GT_API GTN_GetGantrySynchErrorCompensate2DTable(short core,short tableIndex,TCompensate2DTable *pTable,short *pExtend);
GT_API GTN_SetGantrySynchErrorCompensate2D(short core,short group,TCompensate2D *pComp2d);
GT_API GTN_GetGantrySynchErrorCompensate2D(short core,short group,TCompensate2D *pComp2d);
GT_API GTN_GetGantrySynchErrorCompensate2DValue(short core,short group,double *pValue);

typedef struct SecondOrderFilterPara
{
	short active;

	double a0;
	double a1;
	double a2;

	double b1;
	double b2;

} TSecondOrderFilterPara;

#define TYPE_CONTROL_OUTPUT_FILTER			(1)
#define TYPE_YAW_CONTROL_OUTPUT_FILTER		(2)
#define FILTER_COUNT_MAX					(2)

GT_API GTN_SetSecondOrderFilterPara(short core,short control,short type,short index,TSecondOrderFilterPara *pFilterPrm);
GT_API GTN_GetSecondOrderFilterPara(short core,short control,short type,short index,TSecondOrderFilterPara *pFilterPrm);

typedef struct _OCA_CTRL_GEN_
{
	short  s16Gain;
	short  s16Offset;
	double dFrq;
	int32_t   u32RunPeriod;
	short  u16SigType;
} OCA_CTRL_GEN;

typedef struct ExcitationPara
{
	short gain;
	short offset;
	short frq;		// hz
	double runTime;	// s
} TExcitationPara;

typedef struct ExcitationTrapPara
{
	double vel;
	double acc;
	double dec;
	double runTime;	// s
} TExcitationTrpPara;

#define NO_EXCIT						(0xff)
#define OPEN_LOOP_EXCIT					(1)
#define SIGNAL_TYPE_STEP				(3)
#define SIGNAL_TYPE_SIN					(0x10)
#define SIGNAL_TYPE_TRAP				(0x11)
#define SIGNAL_TYPE_NOTHING				(0xFF)

GT_API GTN_SetGenerator(short core,OCA_CTRL_GEN *pGenStr);
GT_API GTN_GetGenerator(short core,OCA_CTRL_GEN *pGenStr);
GT_API GTN_StepResponse(short core,short control,short gain,double time);
GT_API GTN_SetExctLoopMode(short core,short control,short exciteLoopMode);
GT_API GTN_GetExctLoopMode(short core,short control,short *pExciteLoopMode);


/*-----------------------------------------------------------*/
/* DMA                                                       */
/*-----------------------------------------------------------*/
GT_API GTN_CrdHsOn(short core,short crd,short fifo,short dmaBuf=1,unsigned short threshold=200,short lookAheadInMc=0);
GT_API GTN_CrdHsOff(short core,short crd,short fifo);
GT_API GTN_GetCrdHsPrm(short core,short crd,short fifo,short *pEnable,short *pLink,unsigned short *pThreshold,short *pLookAheadInMc);


/*-----------------------------------------------------------*/
/* Others                                                    */
/*-----------------------------------------------------------*/
GT_API GTN_SetRetainValue(short core,uint32_t address,short count,short *pData);
GT_API GTN_GetRetainValue(short core,uint32_t address,short count,short *pData);

typedef struct
{
	short direction;
	short reserve[3];
} TGPIOConfig;
GT_API GTN_SetGPIOConfig(short core,short station,int32_t effectiveLevel,int32_t direction);
GT_API GTN_SetGPIOConfigEx(short core,short station,short index,TGPIOConfig *pGPIOConfig,short count);
GT_API GTN_GetGPIOConfigEx(short core,short station,short index,TGPIOConfig *pGPIOConfig,short count);
GT_API GTN_GetGPIOStatus(short core,short station,short type,short *pStatus,short index,short count);

GT_API GTN_PosCurrFeedForward(short core,short profile,double pos,int32_t gtime,short torque,short gtype,short fifo=0);
GT_API GTN_SetMotionMode(short core,short axis,short motionMode);
GT_API GTN_GetMotionMode(short core,short axis,short *motionMode);
GT_API GTN_PosCurrFeedForwardArray(short core,short profile,double *pPos,int32_t time,short *pTorque,short *pType,short fifo,short count);
GT_API GTN_GetNetStatus(short core,short *pNetSts);

GT_API GTN_GetPowerSts(short core,int32_t *pSts);
GT_API GTN_SynchPrfPosWithEncPos(short core,short profile,short encoder);


GT_API GTN_SetProfileTime(short core,int32_t profileTime,int32_t delay,int32_t stepCoef);
GT_API GTN_GetProfileTime(short core,int32_t *pProfileTime,int32_t *pDelay,int32_t *pStepCoef);
GT_API GTN_SetCoreTime(short core,int32_t profileTime,int32_t delay,int32_t stepCoef);
GT_API GTN_GetCoreTime(short core,int32_t *pProfileTime,int32_t *pDelay,int32_t *pStepCoef);

typedef struct ControlInfo
{
	double refPos;
	double refPosFilter;
	double refPosFilter2;
	double cntPos;
	double cntPosFilter;

	double error;
	double refVel;
	double refAcc;

	short value;
	short valueFilter;

	short offset;
} TControlInfo;

typedef struct
{
	uint32_t notify;
	uint32_t receive;
	uint32_t execute;
	uint32_t retry;
	uint32_t receiveError;
	uint32_t echo;
} TCommandCount;

GT_API GTN_ClearCommandCount(short core);
GT_API GTN_GetCommandCount(short core,TCommandCount *pCommandCount);


GT_API GTN_SetLongVar(short core,short index,int32_t value);
GT_API GTN_GetLongVar(short core,short index,int32_t *pValue);
GT_API GTN_SetDoubleVar(short core,short index,double value);
GT_API GTN_GetDoubleVar(short core,short index,double *pValue);

GT_API GTN_GetBufWaitDiStatus(short core,short crd,short *pDiType,unsigned short *pDiIndex,unsigned short *pLevel, short *pContinueTime,int32_t *pOverTime,short *pFlagMode,int32_t *pSegNum,short *pEnable,int32_t *pCount,short fifo);
GT_API GTN_GetBufWaitLongVarStatus(short core,short crd,short *pIndex,int32_t *pValue,short *pFlagMode,int32_t *pSegNum,short *pEnable,short *pStatus,short fifo);
GT_API GTN_ClearBufWaitStatus(short core,short crd,short fifo);
GT_API GTN_BufWaitDi(short core,short crd, short diType, unsigned short diIndex,unsigned short level, short continueTime, int32_t overTime, short flagMode,int32_t segNum, short fifo);
GT_API GTN_BufWaitLongVar(short core,short crd, short index,int32_t value,int32_t overTime,short flagMode,int32_t segNum,short fifo);

GT_API GTN_BufDoBit(short core,short crd,unsigned short doType,unsigned short index,short value,short fifo);
GT_API GTN_BufDoBitDelay(short core,short crd,unsigned short doType,unsigned short index,short value,int32_t delayTime,short fifo);


typedef struct
{
	short type;
	short enable;
	short flagMode;
	short diType;
	short diIndex;
	short diValue;
	unsigned short continueTime;
	unsigned short trigDelay;
	uint32_t overTime;
	uint32_t counter;
	int32_t longVar;
	int32_t segNum;
	short stop;
	short overTimeStop;
	short pad1[2];
}TBufWaitDiStatusEx;
GT_API GTN_GetBufWaitDiStatusEx(short core,short crd,short fifo,TBufWaitDiStatusEx *pStatus);



//////////////////////////////////////////////////////////////////////////
//Standard Home
//////////////////////////////////////////////////////////////////////////

#define STANDARD_HOME_STAGE_IDLE              (0)   //未启动回原点
#define STANDARD_HOME_STAGE_START             (1)   //启动回原点
#define HOME_STAGE_ON_HOME_LIMIT_ESCAPE		  (2)
#define STANDARD_HOME_STAGE_SEARCH_HOME       (20)  //正在搜索Home
#define STANDARD_HOME_STAGE_SEARCH_INDEX      (30)  //正在搜索Index
#define STANDARD_HOME_STAGE_GO_HOME           (80)  //正在运动到原点
#define STANDARD_HOME_STAGE_END               (100) //回原点结束
#define STANDARD_HOME_STAGE_START_CHECK       (-1)  //启动回原点前自检
#define STANDARD_HOME_STAGE_CHECKING          (-2)  //自检中

#define STANDARD_HOME_ERROR_NONE		      (0)   //未发生错误
#define STANDARD_HOME_ERROR_DISABLE	          (10)  //执行回原点的轴未使能
#define STANDARD_HOME_ERROR_ALARM		      (20)  //执行回原点的轴报警
#define STANDARD_HOME_ERROR_STOP		      (30)  //未完成回原点，被停止运动
#define STANDARD_HOME_ERROR_ON_LIMIT          (40)  //触发了限位无法继续
#define STANDARD_HOME_ERROR_NO_HOME	          (50)  //未找到Home
#define STANDARD_HOME_ERROR_NO_INDEX          (60)  //未找到Index
#define STANDARD_HOME_ERROR_NO_LIMIT          (70)  //未找到限位
#define STANDARD_HOME_ERROR_ENCODER_DIR_SCALE (-1)  //规划器与编码器方向方向相反或者当量不一致


typedef struct
{
	short mode;		      // 回原点模式取值范围1~36
	double highSpeed;     // 搜索Home的速度，单位pulse/ms
	double lowSpeed;	  // 搜索Index的速度，单位pulse/ms
	double acc;		      // 回零加速度，单位pulse/ms^2
	int32_t offset;          // 回零偏移量，单位pulse
	short check;          // 是否启用自检功能，1-启用，其它值-不启用
	short autoZeroPos;    // 回零完毕是否自动清零，1-自动清零，其它值-不清零
	int32_t motorStopDelay;  //电机到位延时，单位：控制周期
	short pad1[3];	      // 保留（不需要设置）
} TStandardHomePrm;

typedef struct
{
	short run;           // 是正在进行回原点，0—已停止运动，1-正在回原点
	short stage;         // 回原点运动的阶段
	short error;         // 回原点过程的发生的错误
	short pad1[3];       // 保留（无具体含义）
	int32_t capturePos;     // 捕获到Home或Index时刻的编码器位置
	int32_t targetPos;      // 需要运动到的目标位置（原点位置或者原点位置+偏移量），在搜索Limit时或者搜索Home或Index时，设置的搜索距离为0，那么该值显示为805306368
} TStandardHomeStatus;

GT_API GTN_ExecuteStandardHome(short core,short axis,TStandardHomePrm *pHomePrm);
GT_API GTN_GetStandardHomePrm(short core,short axis,TStandardHomePrm *pHomePrm);
GT_API GTN_GetStandardHomeStatus(short core,short axis,TStandardHomeStatus *pHomeStatus);



GT_API GTN_SetPosCompareFifoMode(short core,short index,short mode);
GT_API GTN_GetPosCompareFifoMode(short core,short index,short *pMode);
GT_API GTN_GetPosCompareLatchValue(short core,short index,int32_t count,int32_t *pDataX,int32_t *pDataY,int32_t *pCount,TLatchValueInfo *pInfo);



typedef struct
{
	uint32_t profileMask;
	int32_t offset;
	double vel;
	double acc;
}TTaskMoveEscape;

typedef struct
{
	short eventHit;
	short pad1[3];
	int32_t pad2[2];
	double pad3[2];
}TEventStatus;

typedef struct
{
	short start;
	short execute;
	short pad1[2];
	int32_t pad2[2];
	double pad3[2];
}TaskStatus;

GT_API GTN_GetEventStatus(short core,short index,TEventStatus *pStatus);
GT_API GTN_GetTaskStatus(short core,short index,TaskStatus *pStatus);
GT_API GTN_SetTriggerMoveEscape(short core,short trigger,TTaskMoveEscape *pPrm);
GT_API GTN_GetPrfRemainderPos(short core,short profile,double *pValue,short count,uint32_t *pClock);


/*-----------------------------------------------------------*/
/* MPG							                                        */
/*-----------------------------------------------------------*/
typedef struct
{
	double pos;
	double vel;
	double reserve[2];
	int32_t di;
	int32_t reserve1[3];
}TMpgInfo;
GT_API GTN_ReadMpgInfo(short core,short mpg,TMpgInfo *pMpgInfo);
GT_API GTN_WriteMpgPos(short core,short mpg,double *pPos,short count=1);
GT_API GTN_ReadAuEncPos(short core,short encoder,double *pPos,short count=1);
GT_API GTN_WriteAuEncPos(short core,short encoder,double *pPos,short count=1);


GT_API GTN_ClearMcStatus(short core);
GT_API GTN_SetMcInfo(short core,int32_t info,int32_t index,uint32_t data);
GT_API GTN_GetMcInfo(short core,int32_t info,int32_t index,uint32_t *pData);
GT_API GTN_SetMcMode(short core,short mode,short value);
GT_API GTN_GetMcMode(short core,short mode,short *pValue);



GT_API GTN_GetRNMasterInfo(short core,unsigned short *pPhyId,unsigned short *pType,unsigned short *pInfo);
GT_API GTN_SetRNMasterInfo(short core,unsigned short phyId,unsigned short type,unsigned short info);
GT_API GTN_DmaWriteBlock(short core,uint64_t offsetBytes, void *pBuf, uint32_t bytes,int wait);

GT_API GTN_RN_MltPcPduRd(short core,unsigned char* pData, unsigned char des_id,unsigned short byte_start_offset,unsigned short byte_num);
GT_API GTN_RN_MltPcPduRdUpdate(short core,unsigned char des_id);
GT_API GTN_RN_MltPcPduWr(short core,unsigned char* pData, unsigned char des_id, unsigned short byte_start_offset,unsigned short byte_num);
GT_API GTN_RN_MltPcPduWrUpdate(short core,unsigned char des_id);



//////////////////////////////////////////////////////////////////////////
//Command List
//////////////////////////////////////////////////////////////////////////
typedef struct
{
	short execute;
	short empty;
	short stopInfo;
	short reserve1;
	short motionDone;
	short commandType;
	short command;
	short direction;
	int32_t executeSegNum;
	int32_t remainderSegCount;
	int32_t reserve2[6];
} TCommandListStatus;

typedef struct
{
	short mode;
	short elementSize;
	short reserve1[6];
	int32_t forwardSpace;
	int32_t reverseSpace;
	int32_t reserve2[6];
} TCommandListConfig;

typedef struct
{
	short command;
	short index[2];
	short reserve[5];
} TCommandListLink;

typedef struct
{
	short startMode;
	short reserve1[15];
} TCommandListStartParameter;

GT_API GTN_GetCommandListSpace(short core,short list,int32_t *pSpace);
GT_API GTN_GetCommandListStatus(short core,short list,TCommandListStatus *pStatus);
GT_API GTN_ClearCommandListData(short core,short list,TListInfo *pListInfo=NULL);
GT_API GTN_CommandListDataEnd(short core,short list);
GT_API GTN_StartCommandList(short core,short list,TListInfo *pListInfo=NULL);
GT_API GTN_StartMultiCommandList(short core,uint32_t mask,TListInfo *pListInfo=NULL);
GT_API GTN_StopCommandList(short core,short index,short stopMode,TListInfo *pListInfo=NULL);
GT_API GTN_SetCommandListStartParameter(short core,short list,TCommandListStartParameter *pPrm,TListInfo *pListInfo);
GT_API GTN_GetCommandListStartParameter(short core,short list,TCommandListStartParameter *pPrm);
GT_API GTN_StopMultiCommandList(short core,uint32_t mask,short stopMode,TListInfo *pListInfo=NULL);
GT_API GTN_ClearCommandListStatus(short core,short index,TListInfo *pListInfo);
GT_API GTN_SetCommandListConfig(short core,short index,TCommandListConfig *pConfig);
GT_API GTN_GetCommandListConfig(short core,short index,TCommandListConfig *pConfig);
GT_API GTN_SetCommandListLink(short core,short linkId,short list,TCommandListLink *pLink);
GT_API GTN_GetCommandListLink(short core,short linkId,short *pList,TCommandListLink *pLink);
GT_API GTN_ClearCommandListLink(short core,short index);
GT_API GTN_DeleteCommandListLink(short core,short linkId);
GT_API GTN_SetCommandListStopParameter(short core,short list,TCommandListStopParameter *pPrm,TListInfo *pListInfo);
GT_API GTN_GetCommandListStopParameter(short core,short list,TCommandListStopParameter *pPrm);
GT_API GTN_SetCommandListDirection(short core,short index,short direction,TListInfo *pListInfo);
GT_API GTN_GetCommandListDirection(short core,short index,short *pDirection);
GT_API GTN_CommandListDataPush(short core,short list);
GT_API GTN_SetCommandListLinkGroup(short core,short list,uint32_t linkGroupMask);
GT_API GTN_GetCommandListLinkGroup(short core,short list,uint32_t *pLinkGroupMask);


typedef struct
{
	short  count;
	short  reverse1[3];
	double alpha[4];
	double beta[4];
}TProfileScale;
GT_API GTN_SetAxisScale(short core,short profile,TProfileScale *pScale,TListInfo *pListInfo=NULL);
GT_API GTN_GetAxisScale(short core,short profile,TProfileScale *pScale);

typedef struct
{
	double velMax;
	double accMax;
	double decMax;
	double jerkMax;
	double dvMax;
	short reverseLimitMode;
	short reserve1[3];
	double reserve2[8];
} TAxisMotionConstraint;
GT_API GTN_SetAxisMotionConstraint(short core,short axis,TAxisMotionConstraint *pPrm,TListInfo *pListInfo);
GT_API GTN_GetAxisMotionConstraint(short core,short axis,TAxisMotionConstraint *pPrm);

typedef struct
{
	short  count;
	short  reserve[3];
	double alpha[4];
	double beta[4];
}TScaleParameter;
GT_API GTN_SetScaleParameter(short core,short type,short index,TScaleParameter *pScalePrm,TListInfo *pListInfo);
GT_API GTN_GetScaleParameter(short core,short type,short index,TScaleParameter *pScalePrm);

GT_API GTN_SetAxisGearRatio(short core,short profile,double *pGearRatio,short count,TListInfo *pListInfo);
GT_API GTN_GetAxisGearRatio(short core,short profile,double *pGearRatio,short count);

typedef struct
{
	double currentToTorqueCoef;
	double ratedCurrent;
	double reserve[4];
} TAxisMotorParameter;
GT_API GTN_SetAxisMotorParameter(short core,short profile,TAxisMotorParameter *pPrm,TListInfo *pListInfo);
GT_API GTN_GetAxisMotorParameter(short core,short profile,TAxisMotorParameter *pPrm);

typedef struct
{
	double deceleration;
	double jerk;
	short reserve1[4];
	double reserve2[5];
} TAxisStopParameter;
GT_API GTN_SetAxisStopParameter(short core,short profile,short type,TAxisStopParameter *pPrm,TListInfo *pListInfo);
GT_API GTN_GetAxisStopParameter(short core,short profile,short type,TAxisStopParameter *pPrm);
GT_API GTN_GetProfileGroupInfo(short core,short profile,short *pGroup,short *pIndentInGroup);

GT_API GTN_SetVelOverride(short core,short index,double velRatio,TListInfo *pListInfo);
GT_API GTN_GetVelOverride(short core,short index,double *pVelRatio);

/*-----------------------------------------------------------*/
/* New Watch  Code                                           */
/*-----------------------------------------------------------*/
#define WATCH_GROUP_TIMER					(0)
#define WATCH_GROUP_BACKGROUND				(1)

#define WATCH_MODE_STATIC					(0)
#define WATCH_MODE_LOOP						(1)
#define WATCH_MODE_DYNAMIC					(2)

#define WATCH_EVENT_MAX						(8)

#define WATCH_EVENT_RUN						(1)
#define WATCH_EVENT_START					(10)
#define WATCH_EVENT_STOP					(20)
#define WATCH_EVENT_OFF						(30)

#define WATCH_CONDITION_NONE				(0)

#define WATCH_CONDITION_EQ					(1)
#define WATCH_CONDITION_NE					(2)
#define WATCH_CONDITION_GE					(3)
#define WATCH_CONDITION_LE					(4)

#define WATCH_CONDITION_CHANGE_TO			(11)
#define WATCH_CONDITION_CHANGE				(12)
#define WATCH_CONDITION_UP					(13)
#define WATCH_CONDITION_DOWN				(14)
#define WATCH_CONDITION_CHANGE_BEYOND		(15)
#define WATCH_CONDITION_CHANGE_MAX			(16)

#define WATCH_CONDITION_REMAIN_AT			(21)
#define WATCH_CONDITION_REMAIN				(22)
#define WATCH_CONDITION_CROSS_POSITIVE		(23)
#define WATCH_CONDITION_CROSS_NEGATIVE		(24)

#define WATCH_CONDITION_NEAREST				(31)
#define WATCH_CONDITION_DELTA				(32)


#define WATCH_VAR_NONE						(0)

#define WATCH_VAR_UUID						(1000)

#define WATCH_VAR_POWER_ON_COUNT			(1060)

#define WATCH_VAR_DSP_PROGRAM_COUNT			(1070)

#define WATCH_VAR_DSP_TEMPERATURE			(1072)

#define WATCH_VAR_FPGA_PROGRAM_COUNT		(1080)

#define WATCH_VAR_CLOCK						(1200)
#define WATCH_VAR_PRF_LOOP					(1201)
#define WATCH_VAR_PRF_PERIOD				(1205)
#define WATCH_VAR_PRF_TIME					(1206)

#define WATCH_VAR_SKIP_MODULE				(1211)
#define WATCH_VAR_TEST_MODE					(1212)

#define WATCH_VAR_COMMAND_CODE				(1220)
#define WATCH_VAR_COMMAND_DATA				(1221)
#define WATCH_VAR_COMMAND_COUNT				(1222)
#define WATCH_VAR_COMMAND_READ_FLAG			(1223)

#define WATCH_VAR_LINK_PORT_TX_USE			(1660)
#define WATCH_VAR_LINK_PORT_TX_REQUIRE		(1661)
#define WATCH_VAR_LINKPORT_TX_STATUS		(1662)
#define WATCH_VAR_LINKPORT_TX_ERROR_COUNT	(1663)
#define WATCH_VAR_LINKPORT_TX_SKIP_MAX		(1664)

#define WATCH_VAR_SPORT_STATUS				(1682)
#define WATCH_VAR_SPORT_ERROR_COUNT			(1683)


#define WATCH_VAR_PRF_POS					(6000)
#define WATCH_VAR_PRF_VEL					(6001)
#define WATCH_VAR_PRF_ACC					(6002)
#define WATCH_VAR_PRF_POS_RAW				(6003)
#define WATCH_VAR_PRF_VEL_RAW				(6004)

#define WATCH_VAR_PRF_VEL_F32				(6011)
#define WATCH_VAR_PRF_ACC_F32				(6012)
#define WATCH_VAR_PRF_JERK_F32				(6013)

#define WATCH_VAR_PRF_VEL_64				(6021)

#define WATCH_VAR_PRF_RUN					(6200)
#define WATCH_VAR_LIMIT_POSITIVE_STATUS     (6201)
#define WATCH_VAR_LIMIT_NEGATIVE_STATUS     (6202)

// 6600 Trap

#define WATCH_VAR_PT_PRF_POS_F64			(6650)
#define WATCH_VAR_PT_SPACE					(6660)
#define WATCH_VAR_PT_RECEIVE				(6662)
#define WATCH_VAR_PT_EXECUTE				(6663)

#define WATCH_VAR_TRAP_TOTAL_TIME           (6670)
#define WATCH_VAR_TRAP_REMAIN_TIME          (6671)

// 6700 PVT

// 6750 Follow

// 7000 MoveAbsolute
#define WATCH_VAR_MOVE_ABSOLUTE_DONE		(7008)

// 7050 MoveVelocity
#define WATCH_VAR_MOVE_VELOCITY_IN_VELOCITY	(7058)

#define WATCH_VAR_CRD_PRF_POS				(8000)
#define WATCH_VAR_CRD_PRF_VEL				(8001)
#define WATCH_VAR_CRD_PRF_ACC				(8002)
#define WATCH_VAR_CRD_PRF_VEL_RAW			(8003)

#define WATCH_VAR_CRD_PRF_TEMP_DATA         (8010)

#define WATCH_VAR_CRD_RUN					(8200)

#define WATCH_VAR_CRD_SEGMENT_NUMBER		(8202)
#define WATCH_VAR_CRD_SEGMENT_NUMBER_USER	(8203)
#define WATCH_VAR_CRD_COMMAND_RECEIVE		(8204)
#define WATCH_VAR_CRD_COMMAND_EXECUTE		(8205)

#define WATCH_VAR_CRD_FOLLOW_SLAVE_POS		(8600)
#define WATCH_VAR_CRD_FOLLOW_SLAVE_VEL		(8601)

#define WATCH_VAR_CRD_FOLLOW_STAGE			(8610)

#define WATCH_VAR_CONTOUR_PEDAL_POS			(8800)
#define WATCH_VAR_CONTOUR_PRF_POS			(8801)
#define WATCH_VAR_CONTOUR_PRF_POS_SHAPING	(8802)
#define WATCH_VAR_CONTOUR_ENC_POS			(8803)
#define WATCH_VAR_CONTOUR_DAC               (8804)

#define WATCH_VAR_CONTOUR_FIFO_POINT_COUNT	(8808)
#define WATCH_VAR_CONTOUR_FIFO_POINT_MAX	(8809)

#define WATCH_VAR_LIST_EXECUTE				(9240)
#define WATCH_VAR_LIST_SEG_NUM              (9250)

#define WATCH_VAR_CATCH_UP_STATE                           (12000)
#define WATCH_VAR_CATCH_UP_COMMAND                         (12001)
#define WATCH_VAR_CATCH_UP_MASTER_POS                      (12002)
#define WATCH_VAR_CATCH_UP_MASTER_VEL                      (12003)
#define WATCH_VAR_CATCH_UP_SLAVE_POS                       (12004)
#define WATCH_VAR_CATCH_UP_SLAVE_VEL                       (12005)
#define WATCH_VAR_CATCH_UP_PIECE_POS                       (12006)
#define WATCH_VAR_CATCH_UP_PIECE_ID                        (12007)
#define WATCH_VAR_CATCH_UP_PIECE_COUNT                     (12008)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_RECEIVE      (12009)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_SEND         (12010)
#define WATCH_VAR_CATCH_UP_SYNCH_DISTANCE                  (12011)
#define WATCH_VAR_CATCH_UP_SYNCN_TIME                      (12012)
#define WATCH_VAR_CATCH_UP_PIECE_START_POS                 (12013)
#define WATCH_VAR_CATCH_UP_PIECE_SYNCH_POS                 (12014)
#define WATCH_VAR_CATCH_UP_SLAVE_START_POS                 (12015)
#define WATCH_VAR_CATCH_UP_SLAVE_SYNCH_POS                 (12016)

#define WATCH_VAR_POS_COMPARE_COMMAND_RECEIVE	(17422)
#define WATCH_VAR_POS_COMPARE_COMMAND_SEND		(17423)
#define WATCH_VAR_POS_COMPARE_COMMAND_LEFT		(17424)
#define WATCH_VAR_POS_COMPARE_COMMAND_TX		(17425)
#define WATCH_VAR_POS_COMPARE_PULSE_COUNT		(17426)
#define WATCH_VAR_POS_COMPARE_PSO_ON_OFF		(17427)
#define WATCH_VAR_POS_COMPARE_PSO_REG			(17430)
#define	WATCH_VAR_POS_COMPARE_PSO_REG_EXT		(17431)
#define	WATCH_VAR_POS_COMPARE_ENC_PSO_ON_OFF	(17432)

#define WATCH_VAR_SCAN_PRF_POS				(18000)
#define WATCH_VAR_SCAN_PRF_VEL				(18001)
#define WATCH_VAR_SCAN_PRF_ACC				(18002)

#define WATCH_VAR_SCAN_PRF_POS_X			(18010)
#define WATCH_VAR_SCAN_PRF_POS_Y			(18020)
#define WATCH_VAR_SCAN_PRF_POS_Z			(18030)

#define WATCH_VAR_SCAN_RUN					(18200)

#define WATCH_VAR_SCAN_SEGMENT_NUMBER		(18202)

#define WATCH_VAR_SCAN_COMMAND_RECEIVE		(18422)
#define WATCH_VAR_SCAN_COMMAND_SEND			(18423)
#define WATCH_VAR_SCAN_COMMAND_LEFT			(18424)
#define WATCH_VAR_SCAN_COMMAND_TX			(18425)

#define WATCH_VAR_LASER_HSIO				(18600)
#define WATCH_VAR_LASER_POWER				(18601)
#define WATCH_VAR_LASER_WORK_RATIO			(18602)

#define WATCH_VAR_AXIS_PRF_POS				(20000)
#define WATCH_VAR_AXIS_PRF_VEL				(20001)
#define WATCH_VAR_AXIS_PRF_ACC				(20002)

#define WATCH_VAR_AXIS_PRF_POS_OTHER		(20004)
#define WATCH_VAR_AXIS_PRF_VEL_OTHER		(20005)

#define WATCH_VAR_AXIS_PRF_POS_FILTER		(20010)
#define WATCH_VAR_AXIS_PRF_VEL_FILTER		(20011)

#define WATCH_VAR_AXIS_TEMP_PRF_POS_FILTER  (20012)
#define WATCH_VAR_AXIS_PRF_POS_FILTER_BEFORE_COMP (20013)

#define WATCH_VAR_AXIS_PRF_VEL_64			(20021)

#define WATCH_VAR_AXIS_STATUS		        (20100)

#define WATCH_VAR_AXIS_RUN		            (20200)

#define WATCH_VAR_AXIS_ENC_POS_FILTER		(20310)
#define WATCH_VAR_AXIS_ENC_VEL_FILTER		(20311)

#define WATCH_VAR_AXIS_ENC_VEL_64_FILTER	(20321)

#define WATCH_VAR_AXIS_PREDICTION_POS_IN    (20350)
#define WATCH_VAR_AXIS_PREDICTION_POS_IN_2  (20351)
#define WATCH_VAR_AXIS_PREDICTION_POS_OUT   (20352)

#define WATCH_VAR_AXIS_PREDICTION_VEL_IN    (20355)
#define WATCH_VAR_AXIS_PREDICTION_VEL_IN_2  (20356)
#define WATCH_VAR_AXIS_PREDICTION_VEL_OUT   (20357)

#define WATCH_VAR_AXIS_PREDICTION_OUT       (20360)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_POS                    (20361)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_VEL                    (20362)

#define WATCH_VAR_POSCOMPARE_PREDICTION_ENABLE                  (20370)
#define WATCH_VAR_POSCOMPARE_PREDICTION_WORK                    (20371)
#define WATCH_VAR_POSCOMPARE_PREDICTION_OUTPUT_COUNT_TOTAL      (20372)

#define WATCH_VAR_DRIVER_ACT_VEL			(20400)
#define WATCH_VAR_STIMULATE_DATA			(20401)
#define WATCH_VAR_DRIVER_CURRENT_KVFF		(20402)

#define WATCH_VAR_CONTROL_ERROR				(20500)

#define WATCH_VAR_BACKLASH_VALUE			(21000)
#define WATCH_VAR_LEADSCREW_VALUE			(21001)
#define WATCH_VAR_FRICTION_VALUE			(21003)

#define WATCH_VAR_ENC_POS					(30000)
#define WATCH_VAR_ENC_VEL					(30001)
#define WATCH_VAR_AU_ENC_POS				(30002)
#define WATCH_VAR_AU_ENC_VEL				(30003)

#define WATCH_VAR_PULSE_POS					(30010)

#define WATCH_VAR_ENC_VEL_64				(30021)
#define WATCH_VAR_AU_ENC_VEL_64				(30022)
#define WATCH_VAR_ENC_VEL_64_1MS			(30023)
#define WATCH_VAR_AU_ENC_VEL_64_1MS			(30024)

#define WATCH_VAR_MPG_INFO					(30200)
#define WATCH_VAR_AU_ENCODER_EX_POS         (30201)
#define WATCH_VAR_AU_ENCODER_EX_VEL         (30202)
#define WATCH_VAR_MPG_ENCODER_POS           (30203)
#define WATCH_VAR_MPG_ENCODER_VEL           (30204)

#define WATCH_VAR_GPI						(31000)
#define WATCH_VAR_LIMIT_POSITIVE			(31010)
#define WATCH_VAR_LIMIT_NEGATIVE			(31020)
#define WATCH_VAR_ALARM   		        	(31030)
#define WATCH_VAR_HOME             			(31040)
#define WATCH_VAR_ARRIVE           			(31050)
#define WATCH_VAR_BANK_GPI					(31060)

#define WATCH_VAR_GPO						(32000)
#define WATCH_VAR_ENABLE					(32010)
#define WATCH_VAR_CLEAR 					(32020)

#define WATCH_VAR_AI     					(33000)

#define WATCH_VAR_AO     					(34000)

#define WATCH_VAR_TRIGGER_EXECUTE			(38000)
#define WATCH_VAR_TRIGGER_STATUS			(38001)
#define WATCH_VAR_TRIGGER_POSITION			(38002)

#define WATCH_VAR_TRIGGER_COUNT				(38010)

#define WATCH_VAR_TRIGGER_NOTIFY_ENABLE			(38020)
#define WATCH_VAR_TRIGGER_NOTIFY_STATUS_ECHO	(38021)
#define WATCH_VAR_TRIGGER_NOTIFY_CLEAR_WAIT		(38022)

#define WATCH_VAR_TRIGGER_DELTA_CROSS_COUNT		(38110)


#define WATCH_VAR_POS_LOOP_ERROR			(40000)
#define WATCH_VAR_POS_LOOP_REF_POS			(40001)
#define WATCH_VAR_POS_LOOP_FILTER_VALUE		(40002)

#define WATCH_VAR_FIR_POS_IN				(48000)
#define WATCH_VAR_FIR_POS_OUT				(48001)

#define WATCH_VAR_WATCH_ENABLE				(52000)
#define WATCH_VAR_WATCH_TIME				(52001)
#define WATCH_VAR_WATCH_EVENT_VALUE_WORK	(52002)

#define WATCH_VAR_INT32						(52020)
#define WATCH_VAR_INT64						(52021)
#define WATCH_VAR_FLOAT						(52022)
#define WATCH_VAR_DOUBLE					(52023)
#define WATCH_VAR_BOOL						(52024)


#define WATCH_VAR_TERMINAL_LIMIT_POSITIVE		(53000)
#define WATCH_VAR_TERMINAL_LIMIT_NEGATIVE		(53001)
#define WATCH_VAR_TERMINAL_ALARM				(53002)
#define WATCH_VAR_TERMINAL_HOME					(53003)
#define WATCH_VAR_TERMINAL_GPI					(53004)
#define WATCH_VAR_TERMINAL_ARRIVE				(53005)
#define WATCH_VAR_TERMINAL_MPG					(53009)

#define WATCH_VAR_TERMINAL_ENABLE				(53010)
#define WATCH_VAR_TERMINAL_CLEAR				(53011)
#define WATCH_VAR_TERMINAL_GPO					(53012)

#define WATCH_VAR_TERMINAL_DAC					(53020)

#define WATCH_VAR_TERMINAL_PULSE				(53022)
#define WATCH_VAR_TERMINAL_ENCODER				(53023)
#define WATCH_VAR_TERMINAL_ADC					(53024)

#define WATCH_VAR_TERMINAL_AU_ENCODER			(53026)

#define WATCH_VAR_TERMINAL_PRF_POS				(53030)

#define WATCH_VAR_TERMINAL_TRIGGER_POSITION	    (53040)
#define WATCH_VAR_TERMINAL_TRIGGER_STATUS	    (53041)

#define WATCH_VAR_TERMINAL_COMMAND				(53100)
#define WATCH_VAR_TERMINAL_STATUS				(53101)

#define WATCH_VAR_EVENT_HIT						(60000)
#define WATCH_VAR_TASK_START					(61000)
#define WATCH_VAR_TASK_WORK						(61001)


#define VAR_FORMAT_INT						(1)
#define VAR_FORMAT_FLOAT					(2)
#define VAR_FORMAT_DOUBLE					(3)

#define WATCH_LOAD_MODE_NONE				(0)
#define WATCH_LOAD_MODE_BOOT				(2)
#define WATCH_LOAD_MODE_RUN					(3)



#define VAR_CALCULATE_MAX					(32)

#define VAR_CALCULATE_NONE					(0)
#define VAR_CALCULATE_OR					(1)
#define VAR_CALCULATE_AND					(3)
#define VAR_CALCULATE_NOT					(5)

#define VAR_CALCULATE_ADD					(11)
#define VAR_CALCULATE_SUB					(12)
#define VAR_CALCULATE_MUL					(13)
#define VAR_CALCULATE_DIV					(14)

// 用户接口高速读元素结构体
typedef struct
{
	int32_t code;                                      // 需要读取的高速读元素编码,参见WATCH变量编码
	short index;                                    // 需要读取的高速读元素索引,从1开始
	short subIndex;                                 // 需要读取的高速读元素子索引,从1开始
	short pad1[2];                                  // 预留
}TReadHsCommand;
GT_API GTN_AddReadHs(short core,TReadHsCommand *pCommand);
GT_API GTN_LoadReadHsConfig(short core,char *pFile);
GT_API GTN_ReadHsOn(short core,short enable,short mode,double interval);
GT_API GTN_ClearReadHs(short core);
GT_API GTN_SetReadHs(short core,short enable,short mode,short interval);
GT_API GTN_ReadHsReadBuffer(short core,short *pData,int32_t count);
GT_API GTN_FlushReadHs(short core,short wait);


typedef struct
{
	short mode;
	short pad0;
	int32_t band;		//控制器判断到位误差带
	int32_t time;		//控制器判断到位时间
	int32_t pad1[2];
}TAxisArrivePrm;
GT_API GTN_SetAxisArriveMode(short core,short axis,TAxisArrivePrm *pPrm);
GT_API GTN_GetAxisArriveMode(short core,short axis,TAxisArrivePrm *pPrm);


#define MAX_CRDMPG_FIXEDMPGVEL 8
typedef struct
{
	short enable;                                                        //使能标志
	short master;                                                        //手轮主轴号
	short filterTime;                                                    //手轮主轴滤波时间
	short mode;                                                          //手轮的模式
	short fixedMpgVelCount;                                              //手轮引导的挡位个数
	short pad1[3];
	int32_t masterEven;                                                     //手轮主轴比例
	int32_t slaveEven;                                                      //手轮从轴比例
	int32_t pad2[2];
	double fixedMpgVel[MAX_CRDMPG_FIXEDMPGVEL];                          //手轮引导的挡位Ratio值
	double ratioUpdateTime;                                              //手轮引导倍率刷新时间
	double pad3[3];
}TCrdMpgPrm;
GT_API GTN_SetCrdMPGMode(short core,short crd,short enable,short master,int32_t masterEven,int32_t slaveEven,short filterTime,short mode);
GT_API GTN_GetCrdMPGMode(short core,short crd,short *pEnable,short *pMaster,int32_t *pMasterEven,int32_t *pSlaveEven,short *pFilterTime,short *pMode,short *pFifoEnd);
GT_API GTN_SetCrdMPGModeEx(short core,short crd,TCrdMpgPrm *pMpgPrm);
GT_API GTN_GetCrdMPGModeEx(short core,short crd,short *pFifoEnd,TCrdMpgPrm *pCrdMpgPrm);


typedef struct
{
	short mode;	    //运动类型,0-点位，6-PVT，其它类型暂时不支持
	short enable;   //是否使能，0-不使能，1-使能
	short trigger;	//trigger索引，取值从1开始
	short pad1;		//保留
	int32_t distance;	//触发时偏移量，可正可负，单位：脉冲
	int32_t posLimit;  //重新规划后，触发位置+偏移量不能超过该值
	double vel;     //目标速度，重新规划的目标速度，单位：脉冲/ms
	double acc;     //需要提速时的加速度，单位：脉冲/ms
	double dec;		//运动到触发偏移量的减速度，单位：脉冲/ms
	double percent;	//减速段S型曲线时间百分比，例如60表示60%
	double reserve[4];
}TTriggerProfilePrm;

typedef struct
{
	short mode;
	short enable;
	short execute;			//是否执行中,0-未执行，1-执行
	short status;			//执行过程中的状态，正常为0，异常则返回错误码
	int32_t endPos;			//终点位置（捕获+偏移量）
	int32_t reserve[7];
}TTriggerProfileStatus;

GT_API GTN_SetTriggerProfilePrm(short core,short profile,TTriggerProfilePrm *pPrm);
GT_API GTN_GetTriggerProfileStatus(short core,short profile,TTriggerProfileStatus *pSts);


/*-----------------------------------------------------------*/
/* DEBUG INFO                                                */
/*-----------------------------------------------------------*/
GT_API GTN_PrintLogInfo(short core,const char *pFileName,int32_t start,int32_t count);
GT_API GTN_PrintLogInfoDebug(short core,const char *pFileName,int32_t start,int32_t count);
GT_API GTN_PrintMcStsInfo(short core,const char *pFileName,short type,short index,short count);
GT_API GTN_PrintCommandInfo(short core,const char *pFileName,int32_t start,int32_t count);

#define COMMANDINFO_DATA_MAX	     (225)
typedef struct
{
	uint32_t   commandCode;                   //日志的指令字
	short   commandRtn;							   //2word对齐
	short   errorCode;
	uint32_t  clockTime;                      //控制器的时钟
	int32_t    segmentNum;                            //段号
	int32_t    userTag;                               //用户标签
	short   dataLength;
	unsigned short  data16[COMMANDINFO_DATA_MAX];  //
} TCommandInfoData;
//获取指令错误信息
GT_API GTN_GetLastCommandError(short core,TCommandInfoData *getCommandInfoData,int32_t start,int32_t count);

/*-----------------------------------------------------------*/
/* ILC                                                       */
/*-----------------------------------------------------------*/
struct TIlcResult
{
	double ErrorMax;
	double ErrorAvg;
	double ErrorRms;
	double pad1[9];
	short  pad2[10];
};

GT_API GTN_InitIlc(short core);
GT_API GTN_StartIlc(short core,short crd);
GT_API GTN_StopIlc(short core,short crd,TIlcResult *pPrm);
GT_API GTN_SaveIlcFile(short core,char *pIlcFile);
GT_API GTN_LoadIlcFile(short core,short crd,char *filePath);

GT_API GTN_SetIterationLearnData(short core,short axis,double *pData,uint32_t count);
GT_API GTN_EnableIterationLearn(short core,short axis,short mode,double k);

GT_API GTN_SetIteraionData(short core,int32_t count,double xPrf,double yPrf,double xEnc,double yEnc,short pso);
GT_API GTN_GetIteraionData(short core,int32_t count,double *xPrf,double *yPrf,double *xEnc,double *yEnc,double *xRef,double *yRef,double *error,double *xErr,double *yErr, short *pso,int32_t *index);

/*----------------*/
/*振镜激光相关指令*/
/*----------------*/
#define    SCAN_LASER_MODE_DUTY_RATIO          (0)
#define    SCAN_LASER_MODE_FREQUENCY           (1)
#define    SCAN_LASER_MODE_ANALOG              (2)
#define    SCAN_LASER_MODE_PARALLEL            (4)
#define    SCAN_LASER_MODE_NONE               (10)

/*占空比模式固定参数*/
typedef struct
{
	double minDutyRatio;
	double maxDutyRatio;
	double frequency;
}TLaserDutyRatioModeParameterPro;
/*频率模式固定参数*/
typedef struct
{
	double minFrequency;
	double maxFrequency;
	double pulseWidth;
}TLaserFrequencyModeParameterPro;
/*模拟量模式固定参数*/
typedef struct
{
	double minVoltage;
	double maxVoltage;
}TlaserAnalogModeParameterPro;
/*激光各个模式固定参数*/
typedef union
{
	TLaserDutyRatioModeParameterPro dutyRatioModePrm;
	TLaserFrequencyModeParameterPro frequencyModePrm;
	TlaserAnalogModeParameterPro analogModePrm;
	double data[8];
}TLaserParameterUnionPro;
/*激光信息参数*/
typedef struct
{
	unsigned short laserOn;   //激光开关状态
	unsigned short laserMode; //PWM输出模式，宏定义
	short pad[2];             //对齐
	double power;             //激光能量
	TLaserParameterUnionPro laserPrm;
}TLaserInfoPro;

typedef struct
{
	unsigned short laserMode; //PWM输出模式，定义宏
	short pad[3];             //对齐
	TLaserParameterUnionPro laserPrm;
}TLaserParameterPro;

GT_API GTN_GetScanLaserLinkPro(short core,short scanCrd,short *pLaserChannel);
GT_API GTN_SetScanLaserLinkPro(short core,short scanCrd,short laserChannel,TListInfo *pListInfo=NULL);//振镜激光绑定,laserChannel为0则是解绑
GT_API GTN_GetScanLaserInfoPro(short core,short scanCrd,TLaserInfoPro *pPrm);
GT_API GTN_SetScanLaserEnablePro(short core,short scanCrd,short laserEnable,TListInfo *pListInfo=NULL);//激光开关光
GT_API GTN_SetScanLaserPrmPro(short core,short scanCrd,TLaserParameterPro *pLaserPrm,TListInfo *pListInfo=NULL);//激光模式和固定参数设置
GT_API GTN_SetScanLaserPowerPro(short core,short scanCrd,double power,TListInfo *pListInfo=NULL);//激光能量设置
GT_API GTN_SetScanLaserDelayPro(short core,short scanCrd,double laserOnDelay,double laserOffDelay,TListInfo *pListInfo);

/*----------------*/
/*振镜相关指令    */
/*----------------*/
#define    SCAN_MOTION_MODE_NONE                (0)
#define    SCAN_MOTION_MODE_JUMP                (1)
#define    SCAN_MOTION_MODE_JUMP_POINT          (2)
#define    SCAN_MOTION_MODE_JUMP_TIME           (3)
#define    SCAN_MOTION_MODE_JUMP_TIME_POINT     (4)
#define    SCAN_MOTION_MODE_MARK                (5)
#define    SCAN_MOTION_MODE_MARK_TIME           (6)

#define    SCAN_MOTION_CIRCLE_DIR_CW            (0)
#define    SCAN_MOTION_CIRCLE_DIR_CCW           (1)
/*振镜前瞻参数*/
typedef struct
{
	short lookAheadNum;   //前瞻段数
	short highSpeedMode;
	short pad[2];         //对齐
	double time;          //时间常数
	double radiusRatio;   //曲率限制调节参数
	double reserve[2];    //保留
}TScanLookAheadParameterPro;

/*振镜状态*/
typedef struct
{
	short run;                    //振镜运动标志
	short space;
	unsigned short fifoEmpty;     //跑空标志
	short pad1;                   //对齐
	uint32_t segmentNumber;  //执行段号
	uint32_t commandReceive; //接收到的指令数
	uint32_t commandSend;    //已发送的指令数
	int32_t pad2;                    //对齐
	double prfVel;                //合成规划速度
}TScanStatusPro;

//Jump运动、Mark运动
typedef struct
{
	double acc;
	double dec;
	double vel;
}TVelModePro;

typedef struct
{
	double acc;
	double dec;
	uint32_t time;
	int32_t pad;//对齐
}TTimeModePro;

typedef struct
{
	double acc;
	double dec;
	double vel;
	uint32_t motionDelayTime;
	uint32_t laserDelayTime;
}TVelPointModePro;

typedef struct
{
	double acc;
	double dec;
	uint32_t time;
	uint32_t motionDelayTime;
	uint32_t laserDelayTime;
	int32_t pad;//对齐
}TTimePointModePro;

typedef union
{
	TVelModePro velMode;
	TTimeModePro timeMode;
	TVelPointModePro velPointMode;
	TTimePointModePro timePointMode;
	double data[8];
}TScanMotionPrmUnionPro;

typedef struct
{
	double pos[3];
	double reserve;
	TScanMotionPrmUnionPro motionPrm;
}TScanLinearMotionPro;

typedef struct
{
	double endPos[3];
	double radius;//半径正负区分优弧劣弧
	short dir;
	short pad[3];//对齐
	TScanMotionPrmUnionPro motionPrm;
}TScanCircularMotionPro;

typedef struct
{
	short multiMarkDelayMode;
	unsigned short jumpDelayLengthLimit;
	short pad[2];//对齐
	double multiMarkLaserOffDelay;
	double multiMarkDelayConst;
	double markDelay;
	double minJumpDelay;
	double maxJumpDelay;
}TScanDelayParameterPro;

GT_API GTN_ScanInitPro(short core,short scanCrd,TScanLookAheadParameterPro *pPrm,TListInfo *pListInfo);
GT_API GTN_GetScanCrdPosPro(short core,short scanCrd,double *pPos);
GT_API GTN_GetScanStatusPro(short core,short scanCrd,TScanStatusPro *pPrm);
GT_API GTN_ScanLinearPro(short core,short scanCrd,short motionMode,TScanLinearMotionPro *pPrm,TListInfo *pListInfo);
GT_API GTN_ScanCircularPro(short core,short scanCrd,short motionMode,TScanCircularMotionPro *pPrm,TListInfo *pListInfo);
GT_API GTN_SetScanDelayPrmPro(short core,short scanCrd,TScanDelayParameterPro *pPrm,TListInfo *pListInfo=NULL);
GT_API GTN_SetScanMotionDelayPro(short core,short scanCrd,double delayTime,TListInfo *pListInfo);
GT_API GTN_GetScanExecuteTimePro(short core,short scanCrd,double *pExecuteTime);
GT_API GTN_ClearScanExecuteTimePro(short core,short scanCrd);

//20200713同步GTS的冰海定制，编码器断线检测
GT_API GTN_SetEncResponseCheck(short core,short control,short dacThreshold,double minEncVel,int32_t time);
GT_API GTN_GetEncResponseCheck(short core,short control,short *pDacThreshold,double *pMinEncVel,int32_t *pTime);
GT_API GTN_EnableEncResponseCheck(short core,short control);
GT_API GTN_DisableEncResponseCheck(short core,short control);

/************************************************************************/
/* 旋转轴功能                                                           */
/************************************************************************/
#define ROTARY_DIRECTION_SELECT_MODE_DEFAULT                  (0)
#define ROTARY_DIRECTION_SELECT_MODE_SMART                    (1)

typedef struct
{
	short rotary;
	short pad[3];
	double start;
	double length;
	double reserve;
	double pulse;
} TRotaryConfig;

typedef struct
{
	short fifo;
	short overrideNum;
	short g0Mode;
	short reserve1[5];
	int32_t segNum;
	int32_t reserve2[3];
	double vel;
	double acc;
	double velEnd;
	double velLimit;
	double reserve3[8];
} TLineAbsolutePrm;

GT_API GTN_GetAxisPrfPosRotary(short core,short axis,double *pTheta,double *pRound,short count=1);
GT_API GTN_GetProfileRotaryConfig(short core,short profile,TRotaryConfig *pConfig);
GT_API GTN_GetPrfPosRotary(short core,short profile,double *pTheta,double *pRound,short count=1);
GT_API GTN_SetCrdScale(short core,short crd,short dimension,double alpha,double beta);
GT_API GTN_GetCrdScale(short core,short crd,short dimension,double *pAlpha,double *pBeta);
GT_API GTN_GetCrdRotaryConfig(short core,short crd,short dimension,TRotaryConfig *pConfig);
GT_API GTN_GetCrdPosRotary(short core,short crd,short dimension,double *pTheta,double *pRound,short count=1);
GT_API GTN_GetEncoderRotaryConfig(short core,short encoder,TRotaryConfig *pConfig);
GT_API GTN_GetEncPosRotary(short core,short encoder,double *pTheta,double *pRound,short count=1);
GT_API GTN_SetAxisRotaryConfig(short core,short axis,TRotaryConfig *pConfig);
GT_API GTN_GetAxisRotaryConfig(short core,short axis,TRotaryConfig *pConfig);
GT_API GTN_ZeroAxisRotaryRound(short core,short axis,short count=1);
GT_API GTN_LineAbsoluteEx(short core,short crd,double *pPos,short *pDir,TLineAbsolutePrm *pPrm);
GT_API GTN_SetAxisRotaryDirectionSelectMode(short core,short axis,short mode);
GT_API GTN_GetAxisRotaryDirectionSelectMode(short core,short axis,short *pMode);


/*-----------------------------------------------------------*/
//原config.h：配置功能，包括主卡和模块                       */
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* conifg of controller                                      */
/*-----------------------------------------------------------*/
#define RES_LIMIT                       (8)
#define RES_ALARM                       (8)
#define RES_HOME                        (8)
#define RES_GPI                         (16)
#define RES_ARRIVE                      (8)
#define RES_MPG                         (7)
#define RES_ENABLE                      (8)
#define RES_CLEAR                       (8)
#define RES_GPO                         (16)
#define RES_DAC                         (12)
#define RES_STEP                        (8)
#define RES_PULSE                       (8)
#define RES_ENCODER                     (11)
#define RES_LASER                       (2)
#define AXIS_MAX                        (8)
#define PROFILE_MAX                     (8)
#define CONTROL_MAX                     (8)
#define PRF_MAP_MAX                     (2)
#define ENC_MAP_MAX                     (2)

typedef struct DiConfig
{
	short active;
	short reverse;
	short filterTime;
} TDiConfig;

typedef struct CountConfig
{
	short active;
	short reverse;
	short filterType;

	short captureSource;
	short captureHomeSense;
	short captureIndexSense;
} TCountConfig;

typedef struct DoConfig
{
	short active;
	short axis;
	short axisItem;
	short reverse;
} TDoConfig;

typedef struct StepConfig
{
	short active;
	short axis;
	short mode;
	short parameter;
	short reverse;
} TStepConfig;

typedef struct DacConfig
{
	short active;
	short control;
	short reverse;
	short bias;
	short limit;
} TDacConfig;

typedef struct AdcConfig
{
	short active;
	short reverse;
	double a;
	double b;
	short filterMode;
} TAdcConfig;

typedef struct ControlConfig
{
	short active;
	short axis;
	short encoder1;
	short encoder2;
	int32_t  errorLimit;
	short filterType[3];
	short encoderSmooth;
	short controlSmooth;
} TControlConfig;

typedef struct ControlConfigEx
{
	short refType;
	short refIndex;

	short feedbackType;
	short feedbackIndex;

	int32_t  errorLimit;
	short feedbackSmooth;
	short controlSmooth;
} TControlConfigEx;

typedef struct ProfileConfig
{
	short  active;
	double decSmoothStop;
	double decAbruptStop;
} TProfileConfig;

typedef struct AxisConfig
{
	short active;
	short alarmType;
	short alarmIndex;
	short limitPositiveType;
	short limitPositiveIndex;
	short limitNegativeType;
	short limitNegativeIndex;
	short smoothStopType;
	short smoothStopIndex;
	short abruptStopType;
	short abruptStopIndex;
	int32_t  prfMap;
	int32_t  encMap;
	short prfMapAlpha[PRF_MAP_MAX];
	short prfMapBeta[PRF_MAP_MAX];
	short encMapAlpha[ENC_MAP_MAX];
	short encMapBeta[ENC_MAP_MAX];
} TAxisConfig;


GT_API GTN_SaveConfig(short core,char *pFile);
GT_API GTN_SetDiConfig(short core,short diType,short diIndex,TDiConfig *pDi);
GT_API GTN_GetDiConfig(short core,short diType,short diIndex,TDiConfig *pDi);
GT_API GTN_SetDoConfig(short core,short doType,short doIndex,TDoConfig *pDo);
GT_API GTN_GetDoConfig(short core,short doType,short doIndex,TDoConfig *pDo);
GT_API GTN_SetStepConfig(short core,short step,TStepConfig *pStep);
GT_API GTN_GetStepConfig(short core,short step,TStepConfig *pStep);
GT_API GTN_SetDacConfig(short core,short dac,TDacConfig *pDac);
GT_API GTN_GetDacConfig(short core,short dac,TDacConfig *pDac);
GT_API GTN_SetAdcConfig(short core,short adc,TAdcConfig *pAdc);
GT_API GTN_GetAdcConfig(short core,short adc,TAdcConfig *pAdc);
GT_API GTN_SetCountConfig(short core,short countType,short countIndex,TCountConfig *pCount);
GT_API GTN_GetCountConfig(short core,short countType,short countIndex,TCountConfig *pCount);
GT_API GTN_SetControlConfig(short core,short control,TControlConfig *pControl);
GT_API GTN_GetControlConfig(short core,short control,TControlConfig *pControl);
GT_API GTN_SetControlConfigEx(short core,short control,TControlConfigEx *pControl);
GT_API GTN_GetControlConfigEx(short core,short control,TControlConfigEx *pControl);
GT_API GTN_SetProfileConfig(short core,short profile,TProfileConfig *pProfile);
GT_API GTN_GetProfileConfig(short core,short profile,TProfileConfig *pProfile);
GT_API GTN_SetAxisConfig(short core,short axis,TAxisConfig *pAxis);
GT_API GTN_GetAxisConfig(short core,short axis,TAxisConfig *pAxis);
GT_API GTN_ProfileScale(short core,short axis,short alpha,short beta);
GT_API GTN_EncScale(short core,short axis,short alpha,short beta);

/*-----------------------------------------------------------*/
/* Config of Laser and Scan                                  */
/*-----------------------------------------------------------*/
struct TScanCommandMotion
{
	int32_t segmentNumber;
	short x;
	short y;
	int32_t deltaX;
	int32_t deltaY;
	int32_t vel;
	int32_t acc;
};

struct TScanCommandMotionDelay
{
	int32_t delay;
};

struct TScanCommandDo
{
	short doType;
	short doMask;
	short doValue;
};

struct TScanCommandDoDelay
{
	int32_t delay;
};

struct TScanCommandLaser
{
	short mask;
	short value;
} ;

struct TScanCommandLaserDelay
{
	int32_t laserOnDelay;
	int32_t laserOffDelay;
} ;

struct TScanCommandLaserPower
{
	int32_t power;
} ;

struct TScanCommandLaserFrequency
{
	int32_t frequency;
} ;

struct TScanCommandLaserPulseWidth
{
	int32_t pulseWidth;
} ;

struct TScanCommandDa
{
	short daIndex;
	short daValue;
} ;

typedef struct
{
	short module;
	short fifo;
} TScanMap;

GT_API GTN_SetScanMap(short core,short scan,TScanMap *pMap);
GT_API GTN_GetScanMap(short core,short scan,TScanMap *pMap);
GT_API GTN_ClearScanMap(short core);
GT_API GTN_UpdateScanMap(short core);

/*-----------------------------------------------------------*/
//原ringnet.h：和等环网网络相关功能的指令                    */
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* Ringnet                                                  */
/*-----------------------------------------------------------*/
#define RTN_SUCCESS				                    (0)
#define RTN_MALLOC_FAIL						            (-100) /* malloc memory fail */
#define RTN_FREE_FAIL							       			(-101) /* free memory or delete the object fail */
#define RTN_NULL_POINT						            (-102) /* the param point input is null */
#define RTN_ERR_ORDER						            	(-103) /* call the function order is wrong, some msg isn't validable */
#define RTN_PCI_NULL							        		(-104) /* the pci address is empty, can't access the pci device*/
#define RTN_PARAM_OVERFLOW			              (-105) /* the param input is too larget*/
#define RTN_LINK_FAIL							       			(-106) /* the two ports both link fail*/
#define RTN_IMPOSSIBLE_ERR				            (-107) /* it means the system or same function work wrong*/
#define RTN_TOPOLOGY_CONFLICT			            (-108) /* the id conflict*/
#define RTN_TOPOLOGY_ABNORMAL		              (-109) /* scan the net abnormal*/
#define RTN_STATION_ALONE				              (-110) /* the device no id, it means the device id is 0xF0 */
#define RTN_WAIT_OBJECT_OVERTIME	            (-111) /* multi thread wait for object overtime */
#define RTN_ACCESS_OVERFLOW			              (-112) /* data[i];  i is larger than the define */
#define RTN_NO_STATION						            (-113) /* the station accessed not existent */
#define RTN_OBJECT_UNCREATED			            (-114) /* the object not created yet*/
#define RTN_PARAM_ERR						            	(-115) /* the param input is wrong*/
#define RTN_PDU_CFG_ERR                       (-116) /*Pdu DMA Cfg Err*/
#define RTN_PCI_FPGA_ERR					            (-117) /*PCI op err or FPGA op err*/
#define RTN_CHECK_RW_ERR					            (-118) /*data write to reg, then rd out, and check err */
#define RTN_REMOTE_UNEABLE				            (-119) /*the device which will be ctrl by net can't be ctrl by net function*/

#define RTN_NET_REQ_DATA_NUM_ZERO		          (-120) /*mail op or pdu op req data num can't be 0*/
#define RTN_WAIT_NET_OBJECT_OVERTIME	        (-121) /* net op multi thread wait for object overtime */
#define RTN_WAIT_NET_RESP_OVERTIME		        (-122) /* Can't wait for resp */
#define RTN_WAIT_NET_RESP_ERR				          (-123) /*wait mailbox op err*/
#define RTN_INITIAL_ERR								    		(-124) /*initial the device err*/
#define RTN_PC_NO_READY							        	(-125) /*find the station'pc isn't work*/
#define RTN_STATION_NO_EXIST					        (-126)
#define RTN_MASTER_FUNCTION					          (-127) /* this funciton only used by master */

#define RTN_NOT_ALL_RETURN							    	(-128) /* the GT_RN_PtProcessData funciton fail return */
#define RTN_NUM_NOT_EQUAL							   			(-129) /* the station number of RingNet do not equal  the station number of CFG */

#define RTN_CHECK_STATION_ONLINE_NUM_ERR		  (-130) /*Check no slave*/
#define RTN_FILE_ERR_OPEN											(-131) /*open file error*/
#define RTN_FILE_ERR_FORMAT							    	(-132) /*parse file error*/
#define RTN_FILE_ERR_MISSMATCH					      (-133) /*file info is not match with the actual ones*/
#define RTN_DMALIST_ERR_MISSMATCH			        (-134) /*can't find the slave*/

#define RTN_REQUSET_MAIL_BUS_OVERTIME		      (-150) /*Requset Mail Bus Err*/
#define RTN_INSTRCTION_ERR							    	(-151) /*instrctions err*/
#define RTN_MAIL_RESP_REQ_ERR						    	(-152) /*RN_MailRespReq  err*/
#define RTN_CTRL_SRC_ERR											(-153) /* the controlled source  is error */
#define RTN_PACKET_ERR												(-154) /*packet is error*/
#define RTN_STATION_ID_ERR							    	(-155) /*the device id is not in the right rang*/
#define RTN_WAIT_NET_PDU_RESP_OVERTIME	      (-156) /*net pdu op wait overtime*/
#define RTN_ETHCAT_ENC_POS_ERR					      (-157) /**/

#define RTN_IDLINK_PACKET_ERR			            (-200) /*ilink master  decode err! packet_length is not match*/
#define RTN_IDLINK_PACKET_END_ERR		          (-201) /* the ending of ilink packet is not 0xFF*/
#define RTN_IDLINK_TYPER_ERR					        (-202) /* the type of ilink module is error*/
#define RTN_IDLINK_LOST_CNT 					        (-203) /* the ilink module has lost connection*/
#define RTN_IDLINK_CTRL_SRC_ERR			          (-204) /* the controlled source of ilink module is error */
#define RTN_IDLINK_UPDATA_ERR				          (-205) /* the ilink module updata error*/
#define RTN_IDLINK_NUM_ERR					         	(-206) /* the ilink num larger the IDLINK_MAX_NUM(30) */
#define RTN_IDLINK_NUM_ZERO					          (-207) /* the ilink num is zero */

#define RTN_NO_PACKET							        		(301)  /* no valid packet */
#define RTN_RX_ERR_PDU_PACKET				          (-302) /* ERR PDU PACKET */
#define RTN_STATE_MECHINE_ERR				          (-303)
#define RTN_PCI_DSP_UN_FINISH				          (304)
#define RTN_SEND_ALL_FAIL						        	(-305)
#define RTN_STATION_CLOSE					            (310)
#define RTN_STATION_RESP_FAIL				          (311)

#define RTN_UPDATA_MODAL_ERR			            (-330) /* update the modal in normal way fail*/

#define RTN_NO_MAIL_DATA						        	(340) /*There is no mail data*/
#define RTN_NO_PDU_DATA						            (341) /*There is no pdu data*/

#define RTN_FILE_PARAM_NUM_ERR					      (-500)
#define RTN_FILE_PARAM_LEN_ERR					      (-501)
#define RTN_FILE_MALLOC_FAIL									(-502)
#define RTN_FILE_FREE_FAIL										(-503)
#define RTN_FILE_PARAM_ERR							   		(-504)
#define RTN_FILE_NOT_EXSITS						    		(505)
#define RTN_FILE_CREATE_FAIL									(510)
#define RTN_FILE_DELETE_FAIL									(511)
#define RTN_FIFE_CRC_CHECK_ERR								(-512)
#define RTN_FIFE_FUNCTION_ID_RETURN_ERR				(-600)
#define RTN_DLL_WINCE													(-800)
#define RTN_DLL_WIN32													(-801)
#define RTN_XML_STATION_ERR										(-900)//dma config file confilit with slave type


GT_API GTN_LoadRingNetConfig(short core,char*pFile);
GT_API GTN_SaveRingNetConfig(short core,char *pFile);
GT_API GTN_CheckRingNetStructure(short core,char *pFile,unsigned short *pStatus);


/*-----------------------------------------------------------*/
/* 读取SPORT包数据                                           */
/*-----------------------------------------------------------*/
#define SPORT_COUNT                     (256)

typedef struct
{
	uint32_t terminalTxBuf[SPORT_COUNT];
	uint32_t terminalRxBuf[SPORT_COUNT];
}TTerminalData;

GT_API GTN_ReadTerminalData(short core,TTerminalData *pTerminalData);

/*-----------------------------------------------------------*/
/* Config of module                                          */
/*-----------------------------------------------------------*/
#define TERMINAL_OPERATION_NONE             (0)
#define TERMINAL_OPERATION_SKIP             (1)
#define TERMINAL_OPERATION_CLEAR            (2)
#define TERMINAL_OPERATION_RESET_MODULE     (3)

#define TERMINAL_OPERATION_PROGRAM          (11)

typedef struct
{
	uint32_t portACrcOkCnt;
	unsigned short portACrcErrorCnt;
	uint32_t portBCrcOkCnt;
	unsigned short portBCrcErrorCnt;
	uint32_t reserve;//目前用于读取FLASH总数据长度
} TRingNetCrcStatus;

typedef struct
{
	unsigned short errorCountReceive;
	unsigned short errorCountPackageDown;
	unsigned short errorCountPackageUp;
	unsigned short reserve[13];
} TTerminalError;

typedef struct
{
	short moduleDataType;
	short moduleDataIndex;
	short dataIndex;
	short dataCount;
} TTerminalMap;

GT_API GTN_LoadTerminalConfig(short core,char *pFile);
GT_API GTN_SaveTerminalConfig(short core,char *pFile);
GT_API GTN_TerminalOn(short core,short index);
GT_API GTN_TerminalSynch(short core,short index);
GT_API GTN_TerminalSynchEx(short core,short index,short synchEnable,short synchSource);
GT_API GTN_GetRingNetCrcStatus(short core,short index,TRingNetCrcStatus *pRingNetCrcStatus);
GT_API GTN_GetTerminalError(short core,short index,TTerminalError *pTerminalError);

GT_API GTN_GetTerminalPhyId(short core,short count,short *pPhyId);
GT_API GTN_GetTerminalLinkStatus(short core,short count,short *ringNetType,short *pLinkStatus);
GT_API GTN_SetTerminalMap(short core,short dataType,short moduleIndex,TTerminalMap *pMap);
GT_API GTN_GetTerminalMap(short core,short dataType,short moduleIndex,TTerminalMap *pMap);
GT_API GTN_ClearTerminalMap(short core,short dataType);
GT_API GTN_SetTerminalMode(short core,short station,unsigned short mode);
GT_API GTN_GetTerminalMode(short core,short station,unsigned short *pMode);
GT_API GTN_SetTerminalTest(short core,short station,short index,unsigned short value);
GT_API GTN_GetTerminalTest(short core,short station,short index,unsigned short *pValue);
GT_API GTN_SetTerminalOperation(short core,short operation);
GT_API GTN_GetTerminalOperation(short core,short *pOperation);



#define TERMINAL_LOAD_MODE_NONE             (0)
#define TERMINAL_LOAD_MODE_BOOT             (2)

typedef struct
{
	unsigned short type;
	short id;
	int32_t status;
	uint32_t synchCount;
	uint32_t ringNetType;
	uint32_t portStatus;
	uint32_t sportDropCount;
	uint32_t reserve[7];
} TTerminalStatus;

GT_API GTN_TerminalInit(short core,short detect=1);
GT_API GTN_GetTerminalVersion(short core,short index,TVersion *pTerminalVersion);
GT_API GTN_SetTerminalPermit(short core,short index,short dataType,unsigned short permit);
GT_API GTN_GetTerminalPermit(short core,short index,short dataType,unsigned short *pPermit);
GT_API GTN_SetTerminalPermitEx(short core,short station,short dataType,short *permit,short index,short count);
GT_API GTN_GetTerminalPermitEx(short core,short station,short dataType,short *pPermit,short index,short count);

GT_API GTN_SetTerminalSafeMode(short core,short index,short safeMode);
GT_API GTN_GetTerminalSafeMode(short core,short index,short *pSafeMode);
GT_API GTN_ClearTerminalSafeMode(short core,short index);
GT_API GTN_GetTerminalStatus(short core,short index,TTerminalStatus *pTerminalStatus);
GT_API GTN_GetTerminalType(short core,short count,unsigned short *pType,short *pTypeConnect=NULL);
GT_API GTN_SetTerminalType(short core,short count,short *pType);
GT_API GTN_GetTerminalMapCount(short core,short dataType,short moduleIndex,short *pMapCount);
GT_API GTN_GetTerminalMapData(short core,short dataType,short moduleIndex,short mapIndex,TTerminalMap *pMap);

GT_API GTN_FindStation(short core,short station,uint32_t time);

GT_API GTN_ProgramTerminalConfig(short core,short loadMode);
GT_API GTN_GetTerminalConfigLoadMode(short core,short *pLoadMode);




GT_API GTN_SetMailbox(short core,short station,unsigned short byteAddress,unsigned short *pData,unsigned short wordCount,unsigned short dataMode,unsigned short desId,unsigned short type);
GT_API GTN_GetMailbox(short core,short station,unsigned short byteAddress,unsigned short *pData,unsigned short wordCount,unsigned short dataMode,unsigned short desId,unsigned short type);

/*-----------------------------------------------------------*/
/* UUID					                                            */
/*-----------------------------------------------------------*/
GT_API GTN_SetUuid(short core,char *pUuid,short count);
GT_API GTN_GetUuid(short core,char *pCode,short count);

/*-----------------------------------------------------------*/
/* FLASH					                                            */
/*-----------------------------------------------------------*/
GT_API GTN_SetFlashValue(short core,uint32_t address,short count,short *pData);
GT_API GTN_GetFlashValue(short core,uint32_t address,short count,short *pData);

/*-----------------------------------------------------------*/
/* 网络恢复指令                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_RingNetRecover(short core,short station);
/*-----------------------------------------------------------*/
/* GSHD最大最小力矩设置                                      */
/*-----------------------------------------------------------*/
typedef struct
{
	unsigned short torqueMax;
	unsigned short torquePostive;
	unsigned short torqueNegitive;
	short reserve1;
	double reserve2[4];
}TTorqueLimit;
GT_API GTN_RN_SetMaxTorqueLimit(short core,unsigned short station, short axis, unsigned short torqueMax, unsigned short torquePostive,unsigned short torqueNegitive);
GT_API GTN_RN_GetMaxTorqueLimit(short core,unsigned short station,short axis, unsigned short* torqueMax, unsigned short* torquePostive,  unsigned short* torqueNegitive);
GT_API GTN_RN_SetTorqueLimit(short core,short axis,TTorqueLimit *pTorqueLimit);
GT_API GTN_RN_GetTorqueLimit(short core,short axis, TTorqueLimit *pTorqueLimit);

GT_API GTN_SetPrfTorque(short core,short axis,short prfTorque);
GT_API GTN_GetAtlTorque(short core,short axis,short *atlTorque);
GT_API GTN_SetPrfTorqueArray(short core,short axis,short *prfTorque,short count);
GT_API GTN_GetAtlTorqueArray(short core,short axis,short *pAtlTorque,short count);
GT_API GTN_GetSafeTorqueOffStatus(short core,int32_t *pSts);

typedef struct TorqueTriggerPrm
{
	short	enable;
	short	latchType;
	short	latchIndex;
	short	probeType;
	short	probeIndex;
	short	windowOnly;
	int32_t	firstPosition;
	int32_t	lastPosition;
	int32_t	continueTime;
	double	currentValue;
	double	pad2[3];
}TTorqueTriggerPrm;
GT_API GTN_SetTorqueTriggerPrm(short core,short index,TTorqueTriggerPrm *pPrm);
GT_API GTN_GetTorqueTriggerPrm(short core,short index,TTorqueTriggerPrm *pPrm);





/*-----------------------------------------------------------*/
/* GSHD闭环参数设置                                          */
/*-----------------------------------------------------------*/
typedef struct
{
	double value;
	double reverse[4];
}TServoPosLoopPidMode0;

typedef union
{
	TServoPosLoopPidMode0 servoPosLoopPidMode0;
}TServoPosLoopPidUnion;

typedef struct
{
	short mode;
	TServoPosLoopPidUnion servoPosLoopPidPrm;
}TServoPosLoopPid;

GT_API GTN_RN_SetServoPosLoopPid(short core,short axis,TServoPosLoopPid *pServoPosLoopPid);
GT_API GTN_RN_GetServoPosLoopPid(short core,short axis,TServoPosLoopPid *pServoPosLoopPid);

typedef struct
{
	double value;
	double reverse[4];
}TServoSpdLoopPidMode0;

typedef union
{
	TServoSpdLoopPidMode0 servoSpdLoopPidMode0;
}TServoSpdLoopPidUnion;

typedef struct
{
	short mode;
	short reverse[3];
	TServoSpdLoopPidUnion servoSpdLoopPidPrm;
}TServoSpdLoopPid;

GT_API GTN_RN_SetServoSpdLoopPid(short core,short axis,TServoSpdLoopPid *pServoSpdLoopPid);
GT_API GTN_RN_GetServoSpdLoopPid(short core,short axis,TServoSpdLoopPid *pServoSpdLoopPid);

#define SERBVO_POS_LOOP                   (0)		// 驱动器位置环
#define SERBVO_SPD_LOOP                   (1)   // 驱动器速度环

GT_API GTN_RN_GetServoPid(short core,short axis,short loop,short mode,void *pServoPid);
GT_API GTN_RN_SetServoPid(short core,short axis,short loop,short mode,void *pServoPid);
GT_API GTN_RN_SetServoPidRatio(short core,short axis,double ratio);
GT_API GTN_RN_GetServoPidRatio(short core,short axis,double *pRatio);


GT_API GTN_SetServoTime(short core,int32_t servoTime,int32_t delay,int32_t stepCoef);
GT_API GTN_GetServoTime(short core,int32_t *pServoTime,int32_t *pDelay,int32_t *pStepCoef);
GT_API GTN_GetControlInfo(short core,short control,TControlInfo *pControlInfo);
/*-----------------------------------------------------------*/
/* 安全模式设置                                              */
/*-----------------------------------------------------------*/
GT_API GTN_RN_ClearStationSafeModeStatus(short cardIndex,short stationPhyId);
GT_API GTN_RN_IlinkSetSafeModeControl(short cardIndex,short stationPhyId,short modulePhyId,short enable,short clearMode);
GT_API GTN_RN_IlinkClearSafeModeStatus(short cardIndex,short stationPhyId,short modulePhyId);
GT_API GTN_RN_IlinkSetSafeModeOut(short cardIndex,short stationPhyId,short modulePhyId,short type,short index,short *pEnable,double *pValue,short count);
GT_API GTN_RN_GetStationOfflineCount(short cardIndex,int32_t *pStationPhyId,short count,short *pStationOfflineCount);

/*-----------------------------------------------------------*/
//原LookAheadEx.h：和前瞻相关功能的指令                      */
/*-----------------------------------------------------------*/
#define LA_AXIS_NUM    8
#define LA_WORK_AXIS_NUM  6
#define LA_MACHINE_AXIS_NUM   5

//轴的参数信息（各轴最大速度，各轴最大加速度，各轴最大速度变化量）是否限制速度模式
#define AXIS_LIMIT_NONE       0       //轴无限制
#define AXIS_LIMIT_MAX_VEL    1       //轴最大速度限制
#define AXIS_LIMIT_MAX_ACC    2       //轴最大加速度限制
#define AXIS_LIMIT_MAX_DV     4       //轴最大速度跳变量限制

#define KIN_MSG_BUFFER_SIZE   32

//速度规划模式
enum EVelMode
{
	T_CURVE=0,
	S_CURVE,
	S_CURVE_NEW,                  //根据加加速度、最大加速度进行S曲线速度前瞻，2015.11.16
	S_CURVE_SMOOTH,

	VEL_MODE_MAX=0x10000,         //确保长度为4Byte
};

//工件坐标系下轨迹是否限制速度模式
enum EWorkLimitMode
{
	WORK_LIMIT_INVALID=0,       //工件坐标系信息不限制
	WORK_LIMIT_VALID,           //工件坐标系限制生效

	WORK_LIMIT_MODE_MAX=0x10000,//确保长度为4Byte
};

//设置的速度定义规则
enum EVelSettingDef
{
	NORMAL_DEF_VEL=0,             //输入为轴坐标系所有轴的合成速度
	NUM_DEF_VEL,                  //以NUM系统的规则定义
	CUT_DEF_VEL,                  //速度为切削速度

	VEL_SETTING_DEF_MAX=0x10000,  //确保长度为4Byte
};

//设置的加速度定义规则
enum EAccSettingDef
{
	NORMAL_DEF_ACC=0,             //输入即输出
	LONG_AXIS_ACC,                //长轴最大速度

	ACC_SETTING_DEF_MAX=0x10000,  //确保长度为4Byte
};

//机床类型
enum EMachineMode
{
	NORMAL_THREE_AXIS=0,      //标准三轴机床模式
	MULTI_AXES,               //多轴联动模式
	FIVE_AXIS,                //五轴机床模式,轴坐标系为主，工件坐标系为辅
	FIVE_AXIS_WORK,           //五轴机床模式，工件坐标系为主，轴坐标系为辅
	ROBOT,                    //机器人模式，轴坐标系为主，工件坐标系为辅，2014.12.3
	ROBOT_WORK,

	MACHINE_MODE_MAX=0x10000, //确保长度为4Byte
};

//前瞻参数结构体
struct TLookAheadParameter
{
	int lookAheadNum;					//前瞻段数
	double time;						//时间常数
	double radiusRatio;					//曲率限制调节参数
	double vMax[LA_AXIS_NUM];			//各轴的最大速度
	double aMax[LA_AXIS_NUM];			//各轴的最大加速度
	double DVMax[LA_AXIS_NUM];			//各轴的最大速度变化量（在时间常数内）
	double scale[LA_AXIS_NUM];			//各轴的脉冲当量
	short axisRelation[LA_AXIS_NUM];	//输入坐标和内部坐标的对应关系
	char machineCfgFileName[128];		//机床配置文件名
};

//////////////////////////////////////
typedef struct
{
	short RobotType;
	short reserved1;

	short KinParUse[18];
	double KinPar[18];
	short KinLimitUse[12];
	double KinLimitMin[12];
	double KinLimitMax[12];
	double KinLimitMinShift[12];
	double KinLimitMaxShift[12];

	short AxisUse[8];
	char AxisPosSignSwitch[8];
	double AxisPosOffset[8];

	short CartUnitUse[6];
	char CartPosKCSSignSwitch[6];
	short reserved2[3];
	double CartPosKCSOffset[6];
}RC_KIN_CONFIG;

typedef struct
{
	char Error;
	short ErrorID;
	char Message[129];
}RC_ERROR_INTERFACE;

typedef struct
{
	short ErrorID;
	char Message[129];
	char LogTime[32];
	int32_t InternalID;
}RC_MSG_BUFFER_ELEMENT;

typedef struct
{
	short LastMsgIndex;
	RC_MSG_BUFFER_ELEMENT MsgElement[KIN_MSG_BUFFER_SIZE];
	int32_t LastMsgID;
}RC_MSG_BUFFER;

//正逆解方向
enum ETransDir
{
	FORWARD_TRANS=0,            //正解
	INVERSE_TRANS,              //逆解

	TRANS_DIR_MAX = 0x10000,	// 确保长度为4Byte
};

//旋转轴范围设置
struct TRotationAxisRange
{
	int primaryAxisRangeOn;              //第一旋转轴限定范围是否生效，0：不生效，1：生效
	int slaveAxisRangeOn;                //第二旋转轴限定范围是否生效，0：不生效，1：生效
	double maxPrimaryAngle;              //第一旋转轴最大值
	double minPrimaryAngle;              //第一旋转轴最小值
	double maxSlaveAngle;                //第二旋转轴最大值
	double minSlaveAgnle;                //第二旋转轴最小值
};

//选解参数
enum EGroupSelect
{
	Continuous=0,
	Group_1,
	Group_2,
};

enum OptimizeState
{
	OPT_OFF=0,
	OPT_ON,
};

enum OptimizeMethod
{
	NO_OPT=0,
	OPT_BLENDING,
	OPT_CIRCLEFITTING,
	OPT_CUBICSPLINE,
	OPT_BSPLINE,
};

enum ErrorID
{
	INIT_ERROR=1,		                    //没有进行参数初始化
	PASSWORD_ERROR,		                    //密码错误，请在固高运动控制平台上运行
	INDATA_ERROR,		                    //输入数据错误（检查圆弧数据是否正确）
	PRE_PROCESS_ERROR,
	TOOL_RADIUS_COMPENSATE_ERROR_INOUT,		//刀具半径补偿错误：进入/结束刀补处不能是圆弧
	TOOL_RADIUS_COMPENSATE_ERROR_NOCROSS,	//刀具半径补偿错误：数据不合理，无法计算交点
	USERDATA_ERROR,
};

//轨迹优化参数结构体
typedef struct OptimizeParamUser
{
	OptimizeState usePathOptimize;	//是否使用路径优化：OPT_OFF:不使用	OPT_ON:使用

	float tolerance;				//公差(suggest: rough:0.1, pre-finish:0.05, finish:0.01)

	OptimizeMethod optimizeMethod;	//选择曲线优化方式

	OptimizeState keepLargeArc;		//是否保留大圆弧：OPT_OFF：不保留， OPT_ON：保留

	float blendingMinError;			//blending的最小设定误差

	float blendingMaxAngle;			//blending的最大角度限制（即当线段向量角度大于该角度时，不做blending，单位：度）

}TOptimizeParamUser;

struct TErrorInfo
{
	ErrorID errorID;		//错误号(INIT_ERROR:未初始化参数；PRE_PROCESS_ERROR:预处理模块错误；
	//TOOL_RADIUS_COMPENSATE_ERROR:刀具半径补偿错误；)
	int32_t errorRowNum;		//错误行号
};

struct TPreStartPos
{
	double Pos[LA_AXIS_NUM];
};

enum EMachineType
{
	RW_C_ON_B=0,//B 为第一旋转轴，C 为第二旋转轴
	RW_B_ON_A,	//A 为第一旋转轴，B 为第二旋转轴
	RW_A_ON_B,	//B 为第一旋转轴，A 为第二旋转轴
	RW_C_ON_A,	//A 为第一旋转轴，C 为第二旋转轴
	DT_B_ON_A,	//A 为第一旋转轴，B 为第二旋转轴
	DT_A_ON_B,	//B 为第一旋转轴，A 为第二旋转轴
	DT_A_ON_C,	//C 为第一旋转轴，A 为第二旋转轴
	DT_B_ON_C,	//C 为第一旋转轴，B 为第二旋转轴
	T_A_W_B,	//A 为第一旋转轴，B 为第二旋转轴
	T_B_W_A,	//B 为第一旋转轴，A 为第二旋转轴
	T_A_W_C,	//A 为第一旋转轴，C 为第二旋转轴
	T_B_W_C,	//B 为第一旋转轴，C 为第二旋转轴
};
struct TMachCfgInfo
{
	EMachineType machineType;            //机床类型
	short reserve1[2];                   //保留参数
	double primaryAxisPoint[3];          //第一旋转轴中心在MCS的坐标
	double slaveAxisPoint[3];            //第二旋转轴中心在MCS的坐标
	double toolLocationPoint[3];         //刀具坐标系中心在MCS的坐标
	short dirMode;                       //方向描述模式
	short reserve2[2];                   //保留参数
	short dir[5];                        //各轴方向
	double axisVector[5][3];             //各轴轴线方向
};

struct TPos
{
	double machinePos[LA_MACHINE_AXIS_NUM];
	double workPos[LA_WORK_AXIS_NUM];
};

GT_API GTN_SetupLookAheadCrd(short core,short crd,EMachineMode machineMode);
GT_API GTN_SetAxisFollowModeLa(short core,short crd,int *pFollowMode);
GT_API GTN_SetMinEvenVelTimeLa(short core,short crd, double evenTime);
GT_API GTN_SetMinDccAngle(short core,short crd,double dccAngle);
GT_API GTN_SetMinDccAngleLa(short core,short crd,double dccAngle);
GT_API GTN_SetVelDefineModeLa(short core,short crd,EVelSettingDef velDefMode);
GT_API GTN_SetAccDefineModeLa(short core,short crd,EAccSettingDef accDefMode);
GT_API GTN_SetMaxOverrideLa(short core,short crd,double maxSynVelRatio);
GT_API GTN_SetAxisLimitModeLa(short core,short crd,int *pAxisLimitMode);
GT_API GTN_SetWorkLimitModeLa(short core,short crd,EWorkLimitMode workLimitMode);
GT_API GTN_SetAxisVelValidModeLa(short core,short crd,int velValidAxis);
GT_API GTN_SetAngleCoefTableLa(short core,short crd,short count,double *pAngle,double *pCoef);
GT_API GTN_SetVelModeLa(short core,short crd,EVelMode velMode);
GT_API GTN_SetVelSmoothModeLa(short core,short crd,short smoothMode);
GT_API GTN_SetVelLimitAngleThresholdLa(short core,short crd,double angleThreshold);
GT_API GTN_SetRadiusRatioTableLa(short core,short crd,short count,double *pRadius,double *pRatio);
GT_API GTN_PrintLACmdLa(short core,short crd,int printFlag, int clearFile);
GT_API GTN_UpdateMachineBuildingFileLa(short core,short crd, int update);
GT_API GTN_InitialMachineBuilding(short core,short crd, char *pMachineCfgFileName,double *machineCoordCenter, double *workCoordCenter, double toolLength);
GT_API GTN_InitialMachineBuildingEx(short core,short crd, char *pMachineCfgFileName,double *machineCoordCenter, double *workCoordCenter, double toolLength);
GT_API GTN_InitialMachineBuildingPara(short core,short crd, TMachCfgInfo *pMachCfgInfo,double *machineCoordCenter, double *workCoordCenter, double toolLength);
GT_API GTN_GetLookAheadParaEx(short core,short crd,short fifo,TLookAheadParameter *pLookAheadPrm,short *pMotionMode);

// 五轴
GT_API GTN_CrdRTCPOn(short core,short crd,short fifo=0);
GT_API GTN_CrdRTCPOff(short core,short crd,short fifo=0);
GT_API GTN_SetNonlinearErrorControl(short core,short crd, int enable, double nonlinearError);
GT_API GTN_EnableDiscreateArc(short core,short crd,short enable,double arcError);
GT_API GTN_MachineForwardTrans(short core,short crd, double *pMachinePos, double *pWorkPos);
GT_API GTN_MachineRTCPTrans(short core,short crd, double *pInputPos, double *pMachinePos, double *pWorkPos);
GT_API GTN_SetCompToolLength(short core,short crd, double compToolLength);
GT_API GTN_SetCompWorkCoordOffset(short core,short crd, double *pCompWorkOffset);
GT_API GTN_UpdateMachineBuildingFile(short core,short crd, int update);
GT_API GTN_SetRotationAxisRange(short core,short crd, TRotationAxisRange *pRotationAxisRange);
GT_API GTN_SetInverseSolutionSelectPara(short core,short crd, EGroupSelect groupSelect, int priorAxisSet);
GT_API GTN_SetPowerOffDo(short core,short mode,short doIndex,short doValue);
GT_API GTN_MachineTransformation(short core,short crd, int posType, double *pPrePos, double *pPos, int *pPosNum, TPos **pReturnPos);
GT_API GTN_SetStartPointProcessMode(short core,short crd,short enable, double z,short fifo);
GT_API GTN_SetWorkCrdLaserFollowMode(short core,short crd,short enbale,short fifo,short chn);

GT_API GTN_SetAxisVelValidCompModeLa(short core,short crd, int enable, int *pCompAxis);
GT_API GTN_SetFollowAxisProcessModeLa(short core,short crd,int AxisFollowMode);
GT_API GTN_SetCmdVelLimitLa(short core,short crd, int enable, int n1, int n2, int mode);
GT_API GTN_InitLookAheadEx(short core,short crd,TLookAheadParameter *pLookAheadParameter,short fifo, short motionMode=0,TPreStartPos *pPreStartPos=NULL);
GT_API GTN_InitLookAheadPara(short core,short crd,int32_t lookAheadNum,double time,double radiusRatio,double scale,short fifo);
GT_API GTN_SetFollowAxisParaLa(short core,short crd,int *pAxisLimitMode,double *pVmax,double *pAmax,double *pDVmax);

GT_API GTN_SetBlendingParaLa(short core,short crd, short paraType, double para, double minAngle, double maxAngle);


GT_API GTN_GetLookAheadSegCountEx(short core,short crd,int32_t *pSegCount,short fifo=0);
GT_API GTN_GetMotionTimeEx(short core,short crd,double *pTime,short fifo);
GT_API GTN_SetUserSegNumEx(short core,short crd,int32_t segNum,short fifo);
GT_API GTN_CrdDataEx(short core,short crd,TCrdData *pCrdData,short fifo=0);

GT_API GTN_LnXYEx(short core,short crd,double x,double y,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYG0Ex(short core,short crd,double x,double y,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYZEx(short core,short crd,double x,double y,double z,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYZG0Ex(short core,short crd,double x,double y,double z,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYZAEx(short core,short crd,double x,double y,double z,double a,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYZAG0Ex(short core,short crd,double x,double y,double z,double a,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_LnXYZACEx(short core,short crd,double *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_LnXYZACG0Ex(short core,short crd,double *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_LnXYZACUVWEx(short core,short crd,double *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_LnXYZACUVWG0Ex(short core,short crd,double *pPos,short posMask,double synVel,double synAcc,int32_t segNum,short override2,short fifo);
GT_API GTN_ArcXYREx(short core,short crd,double x,double y,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcYZREx(short core,short crd,double y,double z,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcZXREx(short core,short crd,double z,double x,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcXYCEx(short core,short crd,double x,double y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcYZCEx(short core,short crd,double y,double z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcZXCEx(short core,short crd,double z,double x,double zCenter,double xCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcXYZEx(short core,short crd,double x,double y,double z,double interX,double interY,double interZ,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcXYRACEx(short core,short crd,double x,double y,double a,double c,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcXYCACEx(short core,short crd,double x,double y,double a,double c,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_ArcXYZACEx(short core,short crd,double x,double y,double z,double a,double c,double interX,double interY,double interZ,double interA,double interC,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_HelixXYRZEx(short core,short crd,double x,double y,double z,double radius,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_HelixXYCZEx(short core,short crd,double x,double y,double z,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,int32_t segNum,short override2,short fifo=0);
GT_API GTN_BufDelayEx(short core,short crd,unsigned short delayTime,short fifo=0);
GT_API GTN_BufGearEx(short core,short crd,short gearAxis,double deltaPos,short fifo=0);
GT_API GTN_BufMoveEx(short core,short crd,short moveAxis,double pos,double vel,double acc,short modal,short fifo=0);
GT_API GTN_BufIOEx(short core,short crd,unsigned short doType,unsigned short doMask,unsigned short doValue,short fifo=0);
GT_API GTN_BufEnableDoBitPulseEx(short core,short crd,short doType,short doIndex,unsigned short highLevelTime,unsigned short lowLevelTime,int32_t pulseNum,short firstLevel,short fifo);
GT_API GTN_BufDisableDoBitPulseEx(short core,short crd,short doType,short doIndex,short fifo);
GT_API GTN_BufDAEx(short core,short crd,short chn,short daValue,short fifo=0);

GT_API GTN_BufFollowMasterEx(short core,short crd,TBufFollowMaster *pBufFollowMaster,short fifo=0);
GT_API GTN_BufFollowEventCrossEx(short core,short crd,TBufFollowEventCross *pEventCross,short fifo=0);
GT_API GTN_BufFollowEventTriggerEx(short core,short crd,TBufFollowEventTrigger *pEventTrigger,short fifo=0);
GT_API GTN_BufFollowStartEx(short core,short crd,int32_t masterSegment,int32_t slaveSegment,int32_t masterFrameWidth,short fifo=0);
GT_API GTN_BufFollowNextEx(short core,short crd,int32_t width,short fifo=0);
GT_API GTN_BufFollowReturnEx(short core,short crd,double vel,double acc,short smoothPercent,short fifo=0);


// Smart Cutter
typedef struct
{
	short x;						// X轴对应的规划轴
	short y;						// Y轴对应的规划轴
	short c;						// C轴对应的规划轴

	short tableRadiusIndex;			// 半径补偿表索引
	short tableAngleIndex;			// C轴补偿表索引

	short directionReverse;			// 默认规划位置增大时角度也是增大的

	int32_t offset;		            // C轴旋转角度为0时的规划位置
	int32_t resolution;				// C轴每转脉冲数

	short adcIndex;					// ADC索引
	double adcThreshold;			// ADC触发补偿的阈值
} TSmartCutterPrm;

typedef struct
{
	short enable;
	short execute;
	double radiusValue;
	double angleValue;
} TSmartCutterInfo;

GT_API GTN_SmartCutterOn(short core,short index);
GT_API GTN_SmartCutterOff(short core,short index);
GT_API GTN_SetSmartCutterPrm(short core,short index,TSmartCutterPrm *pPrm);
GT_API GTN_GetSmartCutterPrm(short core,short index,TSmartCutterPrm *pPrm);
GT_API GTN_BufSmartCutterOn(short core,short crd,short smartCutterIndex,short fifo);
GT_API GTN_BufSmartCutterOff(short core,short crd,short smartCutterIndex,short fifo);
GT_API GTN_BufSmartCutterOnEx(short core,short crd,short index,short fifo=0);
GT_API GTN_BufSmartCutterOffEx(short core,short crd,short index,short fifo=0);
GT_API GTN_GetSmartCutterInfo(short core,short index,TSmartCutterInfo *pInfo);
GT_API GTN_SetSmartCutterValue(short core,short index,double radiusValue,double angleValue);
GT_API GTN_SmartCutterStart(short core,short index);


GT_API GTN_BufWaitDiEx(short core,short crd, short diType, unsigned short diIndex,unsigned short level, short continueTime, int32_t overTime, short flagMode,int32_t segNum, short fifo);
GT_API GTN_BufWaitLongVarEx(short core,short crd, short index,int32_t value,int32_t overTime,short flagMode,int32_t segNum,short fifo);

GT_API GTN_BufDoBitEx(short core,short crd,unsigned short doType,unsigned short index,short value,short fifo);
GT_API GTN_BufDoBitDelayEx(short core,short crd,unsigned short doType,unsigned short index,short value,int32_t delayTime,short fifo);

// 缓存区位置比较
GT_API GTN_BufPosComparePsoPrm(short core,short crd,short index,TPosComparePsoPrm *pPrm,short fifo);
GT_API GTN_BufPosComparePsoPrmPro(short core,short crd,short index,TPosComparePsoPrmPro *pPrmPro,short fifo);
GT_API GTN_BufPosCompareStart(short core,short crd,short fifo,short index);
GT_API GTN_BufPosCompareStop(short core,short crd,short fifo,short index);

GT_API GTN_BufPosComparePsoPrmEx(short core,short crd,short index,TPosComparePsoPrm *pPrm,short fifo);
GT_API GTN_BufPosComparePsoPrmProEx(short core,short crd,short index,TPosComparePsoPrmPro *pPrmPro,short fifo);
GT_API GTN_BufPosCompareStartEx(short core,short crd,short fifo,short index);
GT_API GTN_BufPosCompareStopEx(short core,short crd,short fifo,short index);

GT_API GTN_BufPosComparePulse(short core,short crd,short index,short outputMode,short level,unsigned short outputPulseWidth,short fifo);
GT_API GTN_BufPosComparePulseEx(short core,short crd,short index,short outputMode,short level,unsigned short outputPulseWidth,short fifo);

GT_API GTN_BufPsoStart(short core,short crd,short fifo,short index,int32_t x,int32_t y);
GT_API GTN_BufPsoStop(short core,short crd,short fifo,short index,int32_t x,int32_t y);

GT_API GTN_BufPsoStartEx(short core,short crd,short fifo,short index,double x,double y);
GT_API GTN_BufPsoStopEx(short core,short crd,short fifo,short index,double x,double y);

/*-----------------------------------------------------------*/
/* 压力补偿                                            */
/*-----------------------------------------------------------*/
typedef struct
{
	double kp;
	double ki;
	double kd;
	double integralLimit;	//积分极限
	double derivativeLimit;	//微分极限
	double limit;			//调节限制（力或电压）
	double pad1[4];
}TAxisPressPid;

#define PRESS_COMPENSATE_MODE_LINEAR                       (0)                           // 线性补偿
#define PRESS_COMPENSATE_MODE_TABLE	                       (1)                           // 查表补偿
#define PRESS_COMPENSATE_MODE_REGION_LEARN                 (2)                           // 区域内自学习补偿

typedef struct
{
	short enable;		           //是否使能，0-关闭，1-使能
	short type;			           //输入类型，电压或网络模块数据
	short dimension;	           //补偿输入的维度，最大3维
	short index[3];		           //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
	short subIndex[3];             //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始
	short mode;			           //模式，线性还是查表
	short revolveAxis;             //用来计算旋转角度的轴号
	short regionAxisIndex;         //补偿功能区间有效的参考轴
	short relatedMasterIndex;      //随动主轴的索引
	short pad1[3];
	double target;			       //目标力或电压
	double thredshold;		       //thredshold，什么时候开始补偿
	double deadZone;		       //死区力或电压，死区内不补偿
	double factor;			       //力和位移的转化系数
	double revolveOffset;	       //初始旋转的脉冲数，默认合成方向和ingdex[0]的方向重合
	double revolveScale;	       //旋转轴的一圈脉冲数
	TAxisPressPid pid;
	int32_t compPosMaxP;              //输出补偿位置区间[N,P]的端点P
	int32_t compPosMaxN;              //输出补偿位置区间[N,P]的端点N
	double k;	                   //补偿量滤波系数 0-1 数值越大滤波越强
	double pad2[4];
	int32_t activeRegionP;            //补偿功能有效的规划位置区间[N,P]的端点P
	int32_t activeRegionN;            //补偿功能有效的规划位置区间[N,P]的端点N
	int32_t activeRegionInterval;     //补偿功能有效的规划位置区间内的自学习间隔，当mode为自学习PRESS_COMPENSATE_MODE_REGION_LEARN时有效
	int32_t relatedMasterEven;        //随动主轴的比例
	int32_t relatedSlaveEven;         //随动从轴的比例
	int32_t pad3[3];
} TAxisPressCompensate;

GT_API GTN_SetAxisPressCompensate(short core,short axis,TAxisPressCompensate *pPressComp);
GT_API GTN_GetAxisPressCompensate(short core,short axis,TAxisPressCompensate *pPressComp);
GT_API GTN_SetAxisPressCompensateTable(short core,short axis,short index,int32_t count,double *pPressData,double *pPosData);
GT_API GTN_SelectAxisPressCompensateTable(short core,short axis,short index);

typedef struct
{
	short type;			      //输入类型，电压或网络模块数据
	short dimension;	      //补偿输入的维度，最大3维
	short index[3];		      //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
	short subIndex[3];        //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始

	double targetMax;	      //标定的力最大值，达到或超过该值时标定结束
	double targetMin;	      //标定的力最小值，达到或小于该值时标定结束
	double factor;		      //力和位移的转化系数，启动时传入0,获取状态时得到标定值。

	int32_t fixRegionP;          //标定规划位置区间[N,P]的端点P，启动位置的相对位置
	int32_t fixRegionN;          //标定规划位置区间[N,P]的端点N，启动位置的相对位置
	int32_t fixRegionInterval;   //标定规划位置区间内的标定间隔

	int32_t tmp[16];
} TAxisPressCompensateFixFactor;

GT_API GTN_StartAxisPressCompensateFixFactor(short core,short axis,TAxisPressCompensateFixFactor *pPressComp);
GT_API GTN_GetAxisPressCompensateFixFactorStatus(short core,short axis,short *pFixFactorSts,TAxisPressCompensateFixFactor *pPressComp);

typedef struct
{
	short type;			 //输入类型，电压或网络模块数据
	short dimension;     //补偿输入的维度，最大3维
	short index[3];		 //输入的索引,DAC从1开始，ECAT IO模块站号从0开始
	short subIndex[3];   //输入的子索引，,DAC该参数无效，ECAT IO模块的IOMAP从0开始

	double target;		 //目标力或电压
	double factor;		 //力和位移的转化系数
	TAxisPressPid pid;

	int32_t fixPosLimitP;   //标定位置区间[N,P]的端点P
	int32_t fixPosLimitN;   //标定补偿位置区间[N,P]的端点N
	double thredshold;   //thredshold，什么时候开始补偿
	double deadZone;	 //死区力或电压，死区内不补偿

	//自整定内部参数，调试使用，后续用户接口没有该部分
	double Knp;			 //压力环全局增益
	double K1;			 //全局增益递增系数1
	double K2;			 //全局增益递增系数2
	double K3;			 //全局增益递增系数3
	double Krise;		 //上升系数
	double Kpeak;		 //峰值系数
	double Tset;		 //目标响应时间
	double Td;			 //保守响应时间

	double tmp[8];
} TAxisPressCompensateFixPid;

GT_API GTN_StartAxisPressCompensateFixPid(short core,short axis,TAxisPressCompensateFixPid *pPressComp);
GT_API GTN_GetAxisPressCompensateFixPidStatus(short core,short axis,short *pFixPidSts,TAxisPressCompensateFixPid *pPressComp);

GT_API GTN_AxisPressCompensateEnable(short core,short axis,short enable,double deadZone,double factor,TListInfo *pListInfo);
GT_API GTN_SetAxisPressCompensateTarget(short core,short axis,double target,double thredshold,TListInfo *pListInfo);

GT_API GTN_BufAxisPressCompensatePrmEx(short core,short crd,short axis,double target,double thredshold,short fifo=0);
GT_API GTN_BufAxisPressCompensateEx(short core,short crd,short axis,short enable,double deadZone,double factor,short fifo=0);

GT_API GTN_SetCrdUserDataEndVelLa(short core,short crd, short crdUserDataType,double endVel);

/*-----------------------------------------------------------*/
/* PVT模式缓冲区激光操作                                      */
/*-----------------------------------------------------------*/
#define PVT_OPERATION_TYPE_LASER_ON					(2)
#define PVT_OPERATION_TYPE_BUF_DA					(4)
#define PVT_OPERATION_TYPE_LASER_CMD				(5)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_RATIO		(6)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_MODE		(25)
#define PVT_OPERATION_TYPE_LASER_FOLLOW_OFF			(26)
#define PVT_OPERATION_TYPE_BUF_DO_BIT				(94)

typedef struct
{
	short doType;
	short index;
	short value;
}TPVTDoBit;

typedef struct
{
	short chn;
	short daValue;
}TPVTDA;

typedef struct
{
	short chn;
	short enable;
}TPVTLaserSwitch;

typedef struct
{
	short chn;
	double power;
}TPVTLaserPrfCmd;

typedef struct
{
	short  chn;
	double ratio;
	double minPower;
	double maxPower;
}TPVTLaserFollowRatio;

typedef struct
{
	short  chn;
}TPVTLaserFollowOff;

typedef struct
{
	short  chn;
	short  source;
	double startPower;
}TPVTLaserFollowMode;


/*-----------------------------------------------------------*/
/* 绝对值编码器	                                            */
/*-----------------------------------------------------------*/
GT_API GTN_RN_GetAbsEncPosEx(short core,short encoder, int64_t *pEncPos);
GT_API GTN_RN_SetEncMultiLinesEx(short core, short encoder,uint64_t multiLines);

GT_API GTN_RN_SetEncMultiLines(short core, short stationId, short axis,uint64_t multiLines);
/*-----------------------------------------------------------*/
/* 加密功能	                                            		*/
/*-----------------------------------------------------------*/
typedef struct
{
	char uuid[32];		// 原始密钥
	char mcInf[32];		//
}TEncryptConfigPrm;

typedef struct
{
	char keyValue[32];		// 密钥
	char data[32];			// 读取到的或者写下去的新的数据
}TEncryptData;

typedef struct
{
	short type;					// 芯片类型
	short userKeySlotOffset;		// 秘钥槽偏置
	short userDataSlotOffset;		// 数据槽偏置
	short cardType;		// 控制卡类型
	short reverse1[8];
}TEncryptChipInf;

typedef struct
{
	char sn[9];				// 芯片序列号
	char random[32];		// 返回的随机数
	TEncryptChipInf chipInf;	// 芯片其他信息
}TEncryptAuxInf;

typedef struct
{
	char data[32];			// 读取到的加密数据
	char hMac[32];			//
}TEncryptDataPro;

GT_API GTN_SetEncryptConfig(short core,TEncryptConfigPrm *pConfigPrm, short *pSts);
// 基础版本加密功能
//  加密读写数据
GT_API GTN_GetEncryptData(short core,TEncryptData *pReadPrm);
GT_API GTN_SetEncryptData(short core,TEncryptData *pWritePrm);
// 晋级版本，开放解密函数，用户根据读取的密文、设置的密钥解密
GT_API GTN_GetEncryptAuxInf(short core,TEncryptAuxInf *pAuxInf,short *pSts);
GT_API GTN_SetEncryptDataPro(short core,short slotIndex,short block,TEncryptDataPro *pWritePrm,short *pSts);
GT_API GTN_GetEncryptDataPro(short core,short slotIndex,short block,TEncryptDataPro *pReadPrm,short *pSts);

// 晋级版本，更新秘钥
typedef struct
{
	char oldKey[32];		// 原始密钥
	char newKey[32];		// 新密钥
}TEncryptKey;

GT_API GTN_UpdateEncryptKey(short core,short keyIndex,TEncryptKey *pKeyInf,short *pSts);

/*-----------------------------------------------------------*/
/* press				                                           	 */
/*-----------------------------------------------------------*/
#define LOOP_MODE_POSITION               (0)
#define LOOP_MODE_PRESS                  (1)
#define LOOP_MODE_NONE                   (2)

#define LIMIT_TYPE_PRESS				(7)
#define LIMIT_TYPE_TORQUE				(8)

#define MC_PRESS                        (600)
#define MC_TORQUE                       (601)

typedef struct
{
	short loopMode;
	short pressProfileMode;
	short pad[2];
} TLoopMode;

typedef struct
{
	short active;			// 功能使能
	short pad1;
	int32_t scale;				// 当量，物理量1牛（N）转换到脉冲的当量。
	short linkAxis;			// 压力规划器与那个轴相关联。
	short pad2[3];
} TPressPrm;

typedef struct
{
	double acc;				// 力矩加速度
	double dec;				// 力矩加速度
	double pressStart;		// 起跳力矩
	short  smoothTime;		// 平滑时间
	short pad[3];
} TPressTargetPrm;

typedef struct
{
	double pressTarget;		// 目标压力
	int32_t pressTime;			// 爬升时间 ms单位
	int32_t holdTime;			// 保持时间
} TPressArrayData;

typedef struct
{
	short count;
	short pad1[3];
	TPressArrayData buffer[4];
	short exit;				// 规划完成后是否切换到位置闭环模式
	short pad2[3];
} TPressArray;

typedef struct
{
	double kp;
	double ki;
	double kd;
	double kvff;
	double kaff;
	int32_t integralLimit;
	int32_t derivativeLimit;
	short limitMax;
	short limitMin;
	short pad[2];
} TPressPid;

typedef struct
{
	short limit1;
	short limit2;
	short time;
	short triggerCondition;        // 压力急停触发条件，PRESS_GREATER_THAN_LIMIT,PRESS_LESS_THAN_LIMIT,
	short loopMode;					// 触发后切换到什么模式，什么规划。
	short pressProfileMode;
	short pad[2];
} TPressAutoSwitchPrm;

typedef struct
{
	int32_t distance;		// 回退距离
	short pad[2];
	double vel;			// 回速度
	double acc;			// 回退加速度
} TStopOffset;

GT_API GTN_SetLoopMode(short core,short axis,TLoopMode *pMode);
GT_API GTN_GetLoopMode(short core,short axis,TLoopMode *pMode);
GT_API GTN_SetPressPrm(short core,short pressAxis,TPressPrm *pPressPrm);
GT_API GTN_GetPressPrm(short core,short pressAxis,TPressPrm *pPressPrm);
GT_API GTN_SetPressTarget(short core,short pressProfile,double value,TPressTargetPrm *pPressTargetPrm);
GT_API GTN_GetPressTarget(short core,short pressProfile,double *pValue,TPressTargetPrm *pPressTargetPrm);
GT_API GTN_SetPressArray(short core,short pressProfile,TPressArray *pPressArray);
GT_API GTN_GetPressArray(short core,short pressProfile,TPressArray *pPressArray);
GT_API GTN_SetPressPid(short core,short pressControl,TPressPid *pPid);
GT_API GTN_GetPressPid(short core,short pressControl,TPressPid *pPid);
GT_API GTN_GetPrfPress(short core,short pressProfile,double *pValue,double *pValueFilter);
GT_API GTN_GetAtlPress(short core,short pressAxis,double *pValue);
GT_API GTN_GetPressStatus(short core,short pressAxis,int32_t *pStatus);
GT_API GTN_SetPressAutoSwitchPrm(short core,short pressAxis,TPressAutoSwitchPrm *pPrm);
GT_API GTN_GetPressAutoSwitchPrm(short core,short pressAxis,TPressAutoSwitchPrm *pPrm);
GT_API GTN_PressAutoSwitchEnable(short core,short pressAxis,short enable);
GT_API GTN_GetPressFeedbackType(short core,short pressAxis,short *pFbType,short *pFbIndex);
GT_API GTN_SetPressFeedbackType(short core,short pressAxis,short fbType,short fbIndex);
GT_API GTN_SetPrfPress(short core,short pressProfile,double prfPressValue);
GT_API GTN_SetStopOffset(short core,short axis,TStopOffset *pStopOffset);
GT_API GTN_GetStopOffset(short core,short axis,TStopOffset *pStopOffset);
GT_API GTN_SetPressRange(short core,short pressAxis,short centerSynch,int32_t range);
GT_API GTN_GetPressRange(short core,short pressAxis,int32_t *pCenterPos,int32_t *pRange);
GT_API GTN_SetPressCross(short core,short pressAxis,short dir,int32_t pos);
GT_API GTN_GetPressCross(short core,short pressAxis,short *pDir,int32_t *pPos);
GT_API GTN_SetPressTrigger(short core,short pressAxis,short pressThread,short triggerCondition);
GT_API GTN_GetPressTrigger(short core,short pressAxis,short *pPressThread,short *pTriggerCondition);
GT_API GTN_PressTriggerOn(short core,short pressAxis);
GT_API GTN_PressTriggerOff(short core,short pressAxis);
GT_API GTN_GetPressTriggeredPos(short core,short pressAxis,short *pEnable,double *pCntPos,short *pTriggeredPress);

/*-----------------------------------------------------------*/
/* Command											                              */
/*-----------------------------------------------------------*/
GT_API GTN_GetCommandExecuteTime(short core,short mode,double *pTime);
GT_API GTN_ClearCommandExecuteTime(short core,short mode);
GT_API GTN_GetCmdCount(short core,short crd,short *pResult,short fifo);
GT_API GTN_InitCmdExExtern(short core,short cmd);
GT_API GTN_GetFloat64ExExtern(short core,double *pData);
GT_API GTN_AddFloat64ExExtern(short core,double data);
GT_API GTN_SendCommandExExtern(short core);


typedef struct
{
	short master[2];
	short masterValueSource[2];
	int32_t gearRatioNumerator[2];
	int32_t gearRatioDenominator[2];
} TCombineAxes;

typedef struct
{
	short enable;
	double slavePos;
	double slaveVel;
} TCombineAxesStatus;

GT_API GTN_SetCombineAxes(short core,short index,TCombineAxes *pCombineAxes);
GT_API GTN_GetCombineAxes(short core,short index,TCombineAxes *pCombineAxes);
GT_API GTN_CombineAxesOn(short core,short index);
GT_API GTN_CombineAxesOff(short core,short index);
GT_API GTN_GetCombineAxesStatus(short core,short index,TCombineAxesStatus *pCombineAxesStatus);

typedef struct
{
	short type;
	short index[2];
} TAddition;
GT_API GTN_SetAxisAddition(short core,short axis,short dataType,TAddition *pAddition);
GT_API GTN_GetAxisAddition(short core,short axis,short dataType,TAddition *pAddition);

typedef struct
{
	short profileType[3];
	short profileIndex[3];
	short profileId[3];
	short encoderType[3];
	short encoderIndex[3];
	short encoderId[3];
} TContourConfig;

typedef struct
{
	double kp[3];
	double ki[3];
	short errorWindow[3];
	short limit[3];
} TContourControl;

typedef struct
{
	short cornerStepIn;
	short cornerStepOut;
	short reserved1[2];
	double cornerTheta;
	short reserved2[4];
	double window;
	double para;
} TContourMode;

GT_API GTN_SetContourConfig(short core,short index,TContourConfig *pContourConfig);
GT_API GTN_GetContourConfig(short core,short index,TContourConfig *pContourConfig);
GT_API GTN_SetContourControl(short core,short index,TContourControl *pContourControl);
GT_API GTN_GetContourControl(short core,short index,TContourControl *pContourControl);
GT_API GTN_SetContourMode(short core,short index,TContourMode *pContourMode);
GT_API GTN_GetContourMode(short core,short index,TContourMode *pContourMode);
GT_API GTN_SetContourEnable(short core,short index,short enable);
GT_API GTN_SetControlAddition(short core,short index,TAddition *pAddition);
GT_API GTN_GetControlAddition(short core,short index,TAddition *pAddition);

GT_API GTN_SetDisplayDip(short core,short mode);
GT_API GTN_GetDisplayDip(short core,short *pMode);

#define PROFILE_MAX                     (8)

typedef struct McConfig
{
	TProfileConfig profile[PROFILE_MAX];
	TAxisConfig    axis[AXIS_MAX];
	TControlConfig control[CONTROL_MAX];
	TDacConfig     dac[RES_DAC];
	TStepConfig    step[RES_STEP];
	TCountConfig   encoder[RES_ENCODER];
	TCountConfig   pulse[RES_PULSE];
	TDoConfig      enable[RES_ENABLE];
	TDoConfig      clear[RES_CLEAR];
	TDoConfig      gpo[RES_GPO];
	TDiConfig      limitPositive[RES_LIMIT];
	TDiConfig      limitNegative[RES_LIMIT];
	TDiConfig      alarm[RES_ALARM];
	TDiConfig      home[RES_HOME];
	TDiConfig      gpi[RES_GPI];
	TDiConfig      arrive[RES_ARRIVE];
	TDiConfig      mpg[RES_MPG];
} TMcConfig;

GT_API GTN_SetMcConfig(short core,TMcConfig *pMc);
GT_API GTN_GetMcConfig(short core,TMcConfig *pMc);
GT_API GTN_SetMcConfigToFile(short core,TMcConfig *pMc,char *pFile);
GT_API GTN_GetMcConfigFromFile(short core,TMcConfig *pMc,char *pFile);
GT_API GTN_SetMcOverride(short core,short axis,short mode,double override,double smoothTime);
GT_API GTN_GetMcOverride(short core,short axis,short mode,double *pOverrideTarget,double *pSmoothTime,double *pOverride);

/*-----------------------------------------------------------*/
/* 工具函数，逻辑和物理转换									 */
/*-----------------------------------------------------------*/
typedef struct
{
	short index;
	short id;
	short reverse1[2];
	double reverse2[4];
}TResPhyInfo;
GT_API GTN_GetResPhyInfo(short core,short type,short index,TResPhyInfo *pStationPhyInfo);

// 逻辑信息结构体
typedef struct
{
	short core;                                  // 逻辑核号,从1开始
	short dataType;                              // 需要转换的资源类型
	short index;                                 // 逻辑索引号,从1开始
	short reserve[5];                            // 保留空间
} TLogicInfo;

// 物理信息结构体
typedef struct
{
	short cardIndex;                             // 物理卡号,从1开始
	short stationPhyId;                          // 物理站号
	short modulePhyId;                           // 扩展模块站号
	short dataType;                              // 需要转换的资源类型
	short index;                                 // 物理索引号
	short reserve[3];                            // 保留空间
} TPhysicInfo;

GT_API GTN_ReadPhysicalMap(void);
GT_API ConvertPhysical(short core,short dataType,short terminal,short index);
GT_API ConvertLogical(TLogicInfo *pLogicInfo,TPhysicInfo *pPhysicInfo);

/*---------------------------------------------------------------------------------------------*/
/* 串行通讯指令（通用485/232通信指令）（gts.lib中导出，推荐使用）							   */
/*---------------------------------------------------------------------------------------------*/
GT_API GTN_RN_ComSerialOpen(short cardIndex, short stationPhyId, short comIndex);
GT_API GTN_RN_ComSerialClose(short cardIndex, short stationPhyId, short comIndex);
GT_API GTN_RN_ComSerialRead(short cardIndex, short stationPhyId, short comIndex, uint32_t  readLen, uint32_t *pResLen, unsigned char *pData);
GT_API GTN_RN_ComSerialReadFast(short cardIndex, short stationPhyId, short comIndex, uint32_t readLen, uint32_t *pResLen, unsigned char *pData);
GT_API GTN_RN_ComSerialWrite(short cardIndex, short stationPhyId, short comIndex,uint32_t writeLen, uint32_t *pResLen, unsigned char *pData);
GT_API GTN_RN_ComSerialGetState(short cardIndex, short stationPhyId, short comIndex, unsigned char *pState);
GT_API GTN_RN_ComSerialSetSettings(short cardIndex, short stationPhyId, short comIndex, uint32_t baudrate, unsigned char stopBits, unsigned char parity);
GT_API GTN_RN_ComSerialClearErr(short cardIndex, short stationPhyId, short comIndex, unsigned char flag);
GT_API GTN_RN_ComSerialSetMode(short cardIndex, short stationPhyId, short comIndex, unsigned short comMode);
GT_API GTN_RN_ComSerialGetRecvFifoCnt(short cardIndex, short stationPhyId, short comIndex,short *pCount);

/*---------------------------------------------------------------------------------------------*/
/* Can通讯指令（通用485/232通信指令）                               						   */
/*---------------------------------------------------------------------------------------------*/
typedef struct _CAN_MSG {
	uint32_t uiId;
	unsigned char ubRtr;
	unsigned char ubLen;
	unsigned char ubExtend;
	unsigned char ubData[8];
}StCanMsg;
GT_API GTN_RN_ComCanOpen(short cardIndex, short stationPhyId, short comIndex, int32_t baudrate);
GT_API GTN_RN_ComCanClose(short cardIndex, short stationPhyId, short comIndex);
GT_API GTN_RN_ComCanSend(short cardIndex, short stationPhyId, short comIndex, StCanMsg* pStCanMsg, uint32_t timeOutus);
GT_API GTN_RN_ComCanRecv(short cardIndex, short stationPhyId, short comIndex, StCanMsg* pStCanMsg, short targetMsgCnt, short* pResMsgCnt, uint32_t timeOutus);
GT_API GTN_RN_ComCanGetStatus(short cardIndex, short stationPhyId, short comIndex, short* pSts);
GT_API GTN_RN_ComCanGetRecvBufMsgCnt(short cardIndex, short stationPhyId, short comIndex, short* pMsgCnt);


GT_API GTN_RN_SetExtDiDoReversePhysical(short cardIndex, short stationPhyId, short moduleId, int32_t* pInputReverse, int32_t inputCount,int32_t *pOutputReverse,int32_t outputCount);
GT_API GTN_RN_GetExtDiDoReversePhysical(short cardIndex, short stationPhyId, short moduleId, int32_t* pInputReverse, int32_t inputCount,int32_t *pOutputReverse,int32_t outputCount);
GT_API GTN_RN_GetExtAoPhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aoIndex,double* pValue, int32_t count);
GT_API GTN_RN_GetExtAoValuePhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aoIndex,int32_t* pValue, int32_t count);
GT_API GTN_RN_GetExtAiPhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aiIndex,double* pValue, int32_t count);
GT_API GTN_RN_GetExtAiValuePhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aiIndex,int32_t* pValue, int32_t count);
GT_API GTN_RN_SetExtAoValuePhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aoIndex,int32_t* pValue, int32_t count);
GT_API GTN_RN_SetExtAoPhysical(short cardIndex, short stationPhyId, short moduleId, int32_t aoIndex,double* pValue, int32_t count);
GT_API GTN_RN_GetExtDiBitPhysical(short cardIndex, short stationPhyId, short moduleId, int32_t diIndex, uint32_t* pValue, int32_t count);
GT_API GTN_RN_GetExtDiPhysical(short cardIndex, short stationPhyId, short moduleId, uint32_t* pValue, int32_t count);
GT_API GTN_RN_GetExtDoPhysical(short cardIndex, short stationPhyId, short moduleId,uint32_t* pValue, int32_t count);
GT_API GTN_RN_SetExtDoBitPhysical(short cardIndex,short stationPhyId,short moduleId,int32_t doIndex,uint32_t* pValue,int32_t count);
GT_API GTN_RN_SetExtDoPhysical(short cardIndex, short stationPhyId, short moduleId, uint32_t* pValue, int32_t count);
GT_API GTN_RN_SetExtModuleMode(short cardIndex,short mode);
GT_API GTN_RN_GetExtModuleMode(short cardIndex,short *pMode);

GT_API GTN_RN_UpdateExtModuleFirmware(short cardIndex, short stationPhyId, short moduleId, const char* pFile);
GT_API GTN_RN_SetExtModuleComLedSts(short cardIndex, short stationPhyId, short moduleId, int32_t value);
GT_API GTN_RN_GetExtModuleInfo(short cardIndex, short stationPhyId, short moduleId, short infoType, int32_t *pValue);


























/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        开发版本功能函数		                          */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
// 用户手动配置(XML)初始化网络指令
GT_API GTN_OpenCard(short channel,void *pPrm,char *pFileName);
GT_API GTN_InitRingNet(short mode,char *pFileName);
GT_API GTN_SetJtGshdCount(short core,short gshdCount,short lastStationAxisCount);
GT_API GTN_GetCardType(uint32_t *pCardTypeArray,short cardTypeArraySize,short *pCardTypeCount);

#define NO_NET							(0xf)
#define RING_NET						(6)
#define ETHERCAT_NET					(8)
#define ETHERCAT_RING_MISC_NET			(9)	

#define NET_SRC_NONE					(NO_NET)
#define NET_SRC_FROM_RING_NET			(RING_NET)
#define NET_SRC_FROM_ECAT_NET			(ETHERCAT_NET)
typedef struct  
{
	short relateEnable;
	short netSrc;			// 软件资源对应的外部网络。来源于RingNet还是EtherCAT,或者其他网络。
	short netSrcIndex;		// 外部网络对应的索引。
	short reserve1[9];
	double reserve2[3];
}TSlaveRelateInfo;

GT_API GTN_NetInit(short mode,char *pFileName,short overTime,int32_t *pStatus);
GT_API GTN_SetMcSrcToSlaveRelateInfo(short core,short mcType,TSlaveRelateInfo *pSlaveInfo,short count);
GT_API GTN_GetMcSrcToSlaveRelateInfo(short core,short mcIndex,short type,TSlaveRelateInfo *pSlaveInfo,short count);

GT_API GTN_SetAxisCircularSafetyZone(short core,short safetyZoneIndex,short enable,TAxisCircularSafetyZone *pPrm);
GT_API GTN_GetAxisCircularSafetyZone(short core,short safetyZoneIndex,short *pEnable,TAxisCircularSafetyZone *pPrm);

GT_API GTN_GetAuEncVelFilter(short core,short type,short encoder,short *pFilterNum);
GT_API GTN_SetAuEncVelFilter(short core,short type,short encoder,short filterNum);

GT_API GTN_SetPosEx(short core,short profile,double pos);
GT_API GTN_GetPosEx(short core,short profile,double *pPos);

GT_API GTN_PrfTrapEx(short core,short profile);
GT_API GTN_GetTrapRunTime(short core,short profile,TTrapPrm *pPrm,double currentPos,double targetPos,double targetVel,double runPos,double *pRunTime);
GT_API GTN_GetTrapRunPos(short core,short profile,TTrapPrm *pPrm,double currentPos,double targetPos,double targetVel,double runTime,double *pRunPos);

GT_API GTN_PvtTableUserData(short core,short tableId,short userDataType,double time,void *pData);

typedef struct
{
	double distance;
	double vM;				// 设置的最大速度
	double acc;				// 设置的z最大加速度
	double pa1;				// 加速百分比1
	double pa2;             // 加速百分比2
	double dec;				// 设置的z最大减速度
	double pd1;				// 减速度百分比1
	double pd2;				// 减速度百分比2

	double velM;			// 实际能达到的最大速度,返回读取的
	double accM;			// 实际能达到的最大加速度,返回读取的
	double decM;			// 实际能达到的最大减速度,返回读取的
	double time;			// 实际规划时间，返回读取的
}TPvtTableMoveAbsolutePrm;
GT_API GTN_PvtTableMoveAbsoluteTwoSegment(short core,short profile,TPvtTableMoveAbsolutePrm *pTargetPos1,double *pDelay,TPvtTableMoveAbsolutePrm *pTargetPos2,short count);

/*-----------------------------------------------------------*/
/* Command Array			                                       */
/*-----------------------------------------------------------*/
GT_API GTN_SetTriggerArray(short core,short trigger,TTrigger *pTrigger,short count);

GT_API GTN_GetFunIdPro(short core,char *pFunName,short *pFunId);
GT_API GTN_GetVarIdPro(short core,char *pFunName,char *pVarName,TVarInfo *pVarInfo);

GT_API GTN_SetVarValueEx(short core,short page,TVarInfo *pVarInfo,double *pValue,short count=1);
GT_API GTN_GetVarValueEx(short core,short page,TVarInfo *pVarInfo,double *pValue,short count=1);

#define SLAVE_MIDDLE_POS_DIMENSION_MAX		3
typedef struct
{
	double targetPos;						// 蛙跳目标位置
	double safetyPos;						// 安全工艺位置
	double slaveMiddlePos[SLAVE_MIDDLE_POS_DIMENSION_MAX];				// 从轴中间点位置信息
} TBufLeapFrogInfo;
GT_API GTN_BufLeapFrog(short core,short crd,short axis,TBufLeapFrogInfo *pLeapFrogInfo,short modal,short fifo);
GT_API GTN_SetCrdBufCommandDelay(short core,short crd,short fifo,short enable);

GT_API GTN_SetCrdStopMode(short core,short crd,short stopMode);
GT_API GTN_GetCrdStopMode(short core,short crd,short *pStopMode);

typedef struct
{
	double synDistance;		// 当前段合成距离
	double movedsynDistance;	// 当前段已运动完成的距离
	double reserve1[10];		// 保留
}TCrdMoveInfo;
GT_API GTN_GetCrdMoveInfo(short core,short crd,TCrdMoveInfo *pCrdMoveInfo,short fifo=0);

GT_API GTN_LaserFollowMode(short core,short laserChannel,short *pLaserFollowSource,double *pLaserFollowStartPower,short laserCount);
GT_API GTN_LaserFollowRatio(short core,short laserChannel,double *pLaserFollowRatio,double *pLaserFollowMinPower,double *pLaserFollowMaxPower,short laserCount);

GT_API GTN_BufMultiLaserFollowOn(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserFollowOff(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserOn(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserOff(short core,short crd,int32_t laserChannelMask,short fifo);

GT_API GTN_BufMultiLaserFollowOnEx(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserFollowOffEx(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserOnEx(short core,short crd,int32_t laserChannelMask,short fifo);
GT_API GTN_BufMultiLaserOffEx(short core,short crd,int32_t laserChannelMask,short fifo);

GT_API GTN_SetCrdContourErrorControlPrm(short core,short crd,short crdAxis,short enable,double *pCoef,short coefCount);
GT_API GTN_GetCrdContourErrorControlPrm(short core,short crd,short crdAxis,short *pEnable,double *pCoef,short *pCoefCount);

GT_API GTN_SetAxisMotionSmoothParameter(short core,short mode,void *pPrm,int32_t axisMask,TListInfo *pListInfo);
GT_API GTN_GetAxisMotionSmoothParameter(short core,short axis,short *pMode,void *pPrm,short count);

GT_API GTN_SetStopSmoothTime(short core,short profile,short smoothTime);
GT_API GTN_GetStopSmoothTime(short core,short profile,short *pSmoothTime);

GT_API GTN_SetCrossCompMode(short core,short axis,short cycleMode,short revCompCycle);
GT_API GTN_GetCrossCompMode(short core,short axis,short *pCycleMode,short *pRevCompCycle);

GT_API GTN_WriteDigitalOutputPro(short core,TDigitalOutputPro *pDo,TListInfo *pListInfo);

GT_API GTN_InitFPGAEncoder(short core);
GT_API GTN_SetFPGAEncoder(short core,short enable,int32_t intervalTime,short reverseMask);
GT_API GTN_GetFPGAEncoder(short core,int32_t *pDataChn1,int32_t *pDataChn2,int32_t *pDataChn3,int32_t *pDataChn4,int32_t *pCount,short *pOverFlowFlag);

GT_API GTN_SetExtModuleAccessMode(short core,short mode);
GT_API GTN_GetExtModuleAccessMode(short core,short *pMode);

typedef struct
{
	short followMode;     //位置比较能量跟随模式,0：脉宽跟随
	short hsoOutputMode;  //hso输出模式，0：不输出，1：按照位置比较指令设置参数输出，2：按照该条指令设置值输出。
	short timeScale;      //时间精度，0：1us，1：0.1us
	short pad;
	double maxValue;
	double minValue;
	double k;
	double b;
}TPosCompareFollowCrd;
GT_API GTN_SetCrdPosCompareFollowPulseWidthPrm(short core,short crd,short posCompareIndex,short hsoIndex,TPosCompareFollowCrd *pPosCompareFollowCrd,short count);
GT_API GTN_CrdPosCompareFollowPulseWidthEnable(short core,short crd,short posCompareIndex,short enable);

typedef struct
{
	short index;              //位置比较索引号
	short enable;
	short reserve1[2];
	int32_t  reserve2[2];
}TPosCompareOnOffList;
GT_API GTN_PosCompareOnOffList(short core,short group,TPosCompareOnOffList *pPosCompareOnOff,TListInfo *pListInfo);

typedef struct
{
	short index;
	short reserve[3];
	uint32_t count;
	int32_t syncPos;
	int32_t time;
} TPosComparePsoPrmList;
GT_API GTN_PosComparePsoPrmList(short core,short group,TPosComparePsoPrmList *pPrm,TListInfo *pListInfo);

GT_API GTN_SetScanAlarmAutoStopMode(short core,short mode,uint32_t errorcodeCheckMask,uint32_t errorcodeCheckRef,short crd);
GT_API GTN_GetScanAlarmAutoStopMode(short core,short *pMode,uint32_t *pErrorcodeCheckMask,uint32_t *pErrorcodeCheckRef,short crd);
GT_API GTN_GetScanErrorCode(short core,uint32_t *pErrorCode,short crd);
GT_API GTN_LoadScanCorrectionTable(short core,short scan,char *pFile,short type);

GT_API GTN_AddTaskPro(short core,short taskType,void *pTaskData,short *pTaskIndex);

typedef struct
{
	unsigned short resType;         // watch变量的类型
	unsigned short resIndex;        // watch变量的索引
	unsigned short resSubIndex;     // watch变量的子索引
	unsigned short resCount;        // 一次性需要读取watch变量的数量
	double resValue[256];           // 读取到的watch变量的值
} TMcVarResInfo;
GT_API GTN_GetMcVarArray(short core,short resInfoCount,TMcVarResInfo *pResInfo);

typedef struct
{
	TWatchVar var;
	unsigned short condition;
	double value;
	double continueTime;
} TWatchConditionVar;

typedef struct
{
	short conditionDone;
	short modal;
	short reserve1[2];
	int32_t segNum;
	int32_t userTag;
	int32_t reserve2[8];
}TWaitForConditionInfo;

GT_API GTN_WaitForConditionVar(short core,TWatchConditionVar *pWaitConditionVarList,short conditionCount,short operation,TWaitTimeout *pTimeout,TWatchVar *pWaitResultVar,TListInfo *pListInfo);
GT_API GTN_GetWaitForConditionInfo(short core,short list,TWaitForConditionInfo *pWaitForConditionInfo);

GT_API GTN_BufSetLongVar(short core,short crd,short index,int32_t value,short fifo);
GT_API GTN_BufSetDoubleVar(short core,short crd,short index,double value,short fifo);

GT_API GTN_SetAxisPrediction(short core,short axis,short mode,short enable,double *pCoef,short count1,short count2);
GT_API GTN_GetAxisPrediction(short core,short axis,short *pMode,short *pEnable,double *pCoef,short *pCount1,short *pCount2);

GT_API GTN_InitializeCommandListConfig(short core,TCommandListConfig *pConfig,short count=2);

typedef struct
{
	short mode;				// 保留，只能为0
	short clear;			// 是否清除初始记录值。0，不清除初始值，1：清除初始值，重新记录初始值
	short reserve[2];
}TStartTimerPrm;

typedef struct
{
	short run;
	short reserve[3];
	double value;
}TTimerInfo;

GT_API GTN_StartTimer(short core,short index,TStartTimerPrm *pStartTimerPrm,short count,TListInfo *pListInfo);
GT_API GTN_StopTimer(short core,short index,short count,TListInfo *pListInfo);
GT_API GTN_GetTimerInfo(short core,short index,TTimerInfo *pInfo,short count);

GT_API GTN_CheckTerminalLicenseRingNet(short core,short station,short fuctionType,int32_t *pInfo);

GT_API GTN_RN_Recover(short cardIndex);

GT_API GTN_SetTorqueLimit(short core,short axis,TTorqueLimit *pTorqueLimit);
GT_API GTN_GetTorqueLimit(short core,short axis, TTorqueLimit *pTorqueLimit);

GT_API GTN_SetServoPosLoopPid(short core,short axis,TServoPosLoopPid *pServoPosLoopPid);
GT_API GTN_GetServoPosLoopPid(short core,short axis,TServoPosLoopPid *pServoPosLoopPid);

typedef struct
{
	uint32_t objectIndex;   //一级指令字
	unsigned short subIndex;     //二级指令字
	unsigned short reserve;      //保留位
	short byteSize;              //字节尺寸
	short memType;               //对象存储位置 0-RAM  1-FLASH
}TServoParamReader;
GT_API GTN_ReadServoParamInfo(short core,short axis, TServoParamReader *pTServoParamReader, unsigned char* pData);

GT_API GTN_RN_GetServoAlarmInfo(short core,short axis,uint32_t *pAlarmCode);

GT_API GTN_WriteServo35thHomingFlag(short core, short axis, short flag);

GT_API GTN_SetServoSpdLoopPid(short core,short axis,TServoSpdLoopPid *pServoSpdLoopPid);
GT_API GTN_GetServoSpdLoopPid(short core,short axis,TServoSpdLoopPid *pServoSpdLoopPid);

GT_API GTN_SetServoPid(short core,short axis,short loop,short mode,void *pServoPid);
GT_API GTN_GetServoPid(short core,short axis,short loop,short mode,void *pServoPid);
GT_API GTN_SetServoPidRatio(short core,short axis,double ratio);
GT_API GTN_GetServoPidRatio(short core,short axis,double *pRatio);

typedef struct
{
	double gain1;  // 一段增益
	double gain2;  // 二段增益
	double smoothTime; // 平滑时间，单位ms
	double spdLimitUpper; // 速度上限，单位rpm
	double spdLimitLower; // 速度下限，单位rpm
	double reserved[4]; //保留
}TServoNonlinearGlobalGain;
GT_API GTN_RN_GetServoNonlinearGlobalGain(short core,short axis,short mode,short dataType,TServoNonlinearGlobalGain *pServoPid);
GT_API GTN_RN_SetServoNonlinearGlobalGain(short core,short axis,short mode,short dataType,TServoNonlinearGlobalGain *pServoGlobalGain);

GT_API GTN_RN_SetStationSafeModeControl(short cardIndex,short stationPhyId,short enable,short clearMode);
GT_API GTN_RN_SetStationSafeModeOut(short cardIndex,short stationPhyId,short type,short index,short *pEnable,double *pValue,short count);

GT_API GTN_SetVelLookAheadModeLa(short core,short crd,int VelMode);
GT_API GTN_SetProfileModeLa(short core,short crd,short profileMode);

GT_API GTN_BufLeapFrogEx(short core,short crd,short axis,TBufLeapFrogInfo *pLeapFrogInfo,short modal,short fifo=0);

GT_API GTN_BufSetLongVarEx(short core,short crd,short index,int32_t value,short fifo);
GT_API GTN_BufSetDoubleVarEx(short core,short crd,short index,double value,short fifo);


typedef struct
{
	int32_t segmentNumber;
	int32_t posX;
	int32_t posY;
	short gpo;					// 24V
	short hso;					// 5V
	short reserve[4];			// 确保指令长度为16Word
} TPosCompareCommandPos;

typedef struct
{
	int32_t segmentNumber;
	int32_t time;
	short pad[2];
	short gpo;
	short hso;
}TPosCompareCommandTime;

typedef union
{
	TPosCompareCommandPos pos;
	TPosCompareCommandTime time;
	short data16[12];
}TPosComparePredictiondataUnion;

typedef struct
{
	short type;
	short pad[3];
	TPosComparePredictiondataUnion data;
}TPosComparePredictionData;

GT_API GTN_SetPosComparePredictionDataToDsp(short core,short posCompareIndex,short count,TPosComparePredictionData *pData,short *pWriteCount);

/*-----------------------------------------------------------*/
/* 绝对值辅助编码器相关                                      */
/*-----------------------------------------------------------*/
GT_API GTN_SetAuAbsEncMultiTurnRange(short core,short encoder,double range);
GT_API GTN_ReadAuAbsEncPos(short core,short encoder,double *pPos);

GT_API GTN_SetEncryptConfigRingNet(short core,short station,TEncryptConfigPrm *pConfigPrm, short *pSts);
GT_API GTN_GetEncryptDataRingNet(short core,short station,TEncryptData *pReadPrm);
GT_API GTN_SetEncryptDataRingNet(short core,short station,TEncryptData *pWritePrm);

GT_API GTN_ClearPressStatus(short core,short pressAxis);

/*-----------------------------------------------------------*/
/* 椭圆插补指令                                              */
/*-----------------------------------------------------------*/
#define ELLIPSE_AUX_POINT_COUNT                 (5)

#define ELLIPSE_MODE_AUX_POINT                  (0)
#define ELLIPSE_MODE_STANDARD                   (1)

#define ELLIPSE_MODE_AUX_POINT_2D               (0)
#define ELLIPSE_MODE_STANDARD_2D                (1)

typedef struct
{
	double pos[ELLIPSE_AUX_POINT_COUNT][INTERPOLATION_AXIS_MAX];    // 椭圆上辅助点坐标
}TEllipseAuxPoint;

typedef struct
{
	double centerPoint[INTERPOLATION_AXIS_MAX];// 椭圆圆心坐标
	double theta;                              // 椭圆旋转角度
	double a;                                  // 椭圆长轴
	double b;                                  // 椭圆短轴
}TEllipseStandard;

typedef union
{
	TEllipseAuxPoint auxPoint;                 // 辅助点模式参数
	TEllipseStandard standard;                 // 标准模式参数

	double reserve[60];
} TEllipseParameterUnion;

typedef struct
{
	double endPoint[INTERPOLATION_AXIS_MAX];   // 终点坐标
	short plane;                               // 椭圆平面选择，0：XY；1：YZ；2：ZX
	short dir;                                 // 椭圆方向，0：顺时针；1：逆时针
	short overrideSelect;                      // 速度倍率选择，0：第1组倍率；1：第2组倍率
	short mode;                                // 椭圆模式，目前只支持参数0(辅助点模式)

	TEllipseParameterUnion data;               // 保留参数
} TEllipseParameter;

//-------------------------------------------------------
//功能说明：椭圆插补描述参数,模式：ELLIPSE_MODE_AU_POINT_2D
//plane--------------椭圆平面选择，INTERPOLATION_CIRCLE_PLAT_XY(0)：XY；INTERPOLATION_CIRCLE_PLAT_YZ(1)：YZ；INTERPOLATION_CIRCLE_PLAT_ZX(2)：ZX
//dir----------------椭圆方向，0：顺时针；1：逆时针
//overrideSelect-----速度倍率选择，0：第1组倍率；1：第2组倍率
//pad----------------占位变量，不需要传入
//endPoint1----------终点坐标1,意义根据plane来定，如果palne为XY平面，则endPoint1、pos1为X坐标，endPoint2、pos2为Y坐标
//endPoint2----------终点坐标2
//pos1---------------椭圆上辅助点坐标1
//pos2---------------椭圆上辅助点坐标2
//-------------------------------------------------------
typedef struct
{
	short plane;
	short dir;
	short overrideSelect;
	short pad;

	double endPoint1;
	double endPoint2;

	double pos1[ELLIPSE_AUX_POINT_COUNT];
	double pos2[ELLIPSE_AUX_POINT_COUNT];
} TEllipseAuxPoint2D;

//-------------------------------------------------------
//功能说明：椭圆插补描述参数,模式：ELLIPSE_MODE_STANDARD_2D
//plane--------------椭圆平面选择，INTERPOLATION_CIRCLE_PLAT_XY(0)：XY；INTERPOLATION_CIRCLE_PLAT_YZ(1)：YZ；INTERPOLATION_CIRCLE_PLAT_ZX(2)：ZX
//dir----------------椭圆方向，0：顺时针；1：逆时针
//overrideSelect-----速度倍率选择，0：第1组倍率；1：第2组倍率
//pad----------------占位变量，不需要传入
//endPoint1----------终点坐标1,意义根据plane来定，如果palne为XY平面，则endPoint1为X坐标，endPoint2为Y坐标
//endPoint2----------终点坐标2
//centerPoint1-------椭圆圆心坐标1，意义根据plane来定，如果palne为XY平面，则centerPoint1为X坐标，centerPoint2为Y坐标
//centerPoint2-------椭圆圆心坐标2
//theta--------------椭圆旋转角度，单位：度
//a------------------椭圆长轴
//b------------------椭圆短轴，短轴必须比长轴短
//-------------------------------------------------------
typedef struct
{
	short plane;
	short dir;
	short overrideSelect;
	short pad;

	double endPoint1;
	double endPoint2;

	double centerPoint1;
	double centerPoint2;

	double theta;
	double a;
	double b;
}TEllipseStandard2D;

GT_API GTN_Ellipse(short core,short crd,TEllipseParameter *pEllipse,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);
GT_API GTN_EllipseEx(short core,short crd,TEllipseParameter *pEllipse,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);

GT_API GTN_EllipsePro(short core,short crd,short mode,void *pData,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);
GT_API GTN_EllipseProEx(short core,short crd,short mode,void *pData,double synVel,double synAcc,double velEnd,int32_t segNum,short fifo=0);


/*-----------------------------------------------------------*/
/* 批处理指令功能        	                                   */
/*-----------------------------------------------------------*/
#define BATCH_COMMAND_END_MODE_DEFAULT                     (0)
#define BATCH_COMMAND_END_MODE_GET_MC_VAR_EX               (1)

typedef struct
{
	short commandCount;                // 当前批处理指令总数
	short errorCommandNumber;          // 执行出错的指令序号
	short errorCode;                   // 执行出错的指令错误详细信息（部分指令有详细信息）
	short reserve[5];
}TBatchCommandResult;

GT_API GTN_BatchCommandBegin(short core);
GT_API GTN_BatchCommandEnd(short core,TBatchCommandResult *pResult,short mode,void *pPrm,void *pValue,short count);

/*-----------------------------------------------------------*/
/* 调高器指令           	                                   */
/*-----------------------------------------------------------*/
// 调高器相关函数
#define MC_HEIGHT_FEEDBACK				(450)
// 设置、读取调高器控制参数
typedef struct
{
	short active;				// 使用下列参数处理调高器频率。
	short ctlMode;				// 高度闭环控制还是开环补偿
	short linkTableId;			// 使用那张表差值
	short controlEncoder;		// 差值来源于第几路高度传感器
	short interval;				// 采样间隔时间，中断周期为单位。
	int32_t errorLimit;			// 跟随误差,开环补偿模式才有效
}THeightControlPrm;

GT_API GTN_SetAxisHeightControlPrm(short core,short axis,THeightControlPrm *pPrm);
GT_API GTN_GetAxisHeightControlPrm(short core,short axis,THeightControlPrm *pPrm);

// 设置、读取调高器差值表
GT_API GTN_SetHeightInterpolationTable(short core,short tableId,int32_t n,double *pPosEnc,double *pPosHeight);
GT_API GTN_GetHeightInterpolationTable(short core,short tableId,int32_t n,double *pPosEnc,double *pPosHeight,int32_t *pRealCount);

GT_API GTN_SetAxisHeightControlPid(short core,short axis,TPid *pPid);
GT_API GTN_ZerophaseFilter(short core,int32_t dataLen,double inputdata[],double smoothArray[]);

GT_API GTN_GetAxisErrorStatusLink(short core,short axis,short *pLinkAxis,short *pLinkMode,short *pCcount);
GT_API GTN_SetAxisErrorStatusLink(short core,short axis,short *pLinkAxis,short *pLinkMode,short count);

typedef struct
{
	short type;					// 参考类型是编码器位置还是规划位置还是电容高度
	short index;				// 编码器或者规划的索引
	int32_t  crossPos;				// 启动或者关闭跟随功能的穿越高度
	short dir;					// 启动或者关闭的穿越方向
	short pad1[3];
}THeightCrossPrm;

typedef struct
{
	short keepEnable;				// 0:运动到指定高度,1:运动到指定高度后，保持高度跟随
	short pad1[3];
	THeightCrossPrm enableData;		// 自动启动高度跟随参数
	THeightCrossPrm disableData;	// 自动关闭高度跟随参数
}THeightFollowPrm;

#define HEIGHT_DIR_CROSS_POSITIVE            (1)
#define HEIGHT_DIR_CROSS_NEGATIVE            (2)
#define HEIGHT_DIR_CROSS_EQUAL               (3)

#define HEIGHT_FOLLOW_ENABLE_MODE_MANUAL						(0)
#define HEIGHT_FOLLOW_ENABLE_MODE_AUTO							(1)
// 自动启动/关闭跟随功能还是应用程序启动或者关闭高度跟随功能
GT_API GTN_SetAxisHeightFollowMode(short core,short axis,short mode,THeightFollowPrm *pPrm);


// 调高器位置跟随功能打开、关闭
GT_API GTN_AxisHeightControlEnable(short core,short axis,short enable,int32_t pos=10);
// 读取调高器位置
GT_API GTN_GetHeightInterpolationPos(short core,short heigtFbIndex,double *pHeightFrq,double *pHeightPosRaw,double *pInterpolationPos);
// 读取调高器状态信息

typedef struct
{
	short ctlMode;			// 当前轴闭环模式，电机编码器闭环，还是调高器频率闭环。
	short errorSts;			// 错误状态，
	double cmpValue;		// 补偿值
}THeightInf;
GT_API GTN_GetAxisHeightInf(short core,short axis,THeightInf *pHeightInf);
// 清除调高器状态
GT_API GTN_ClearAxisHeightSts(short core,short axis);

//FIR滤波器
#define HEIGHT_FILTER_TYPE_FREQUENCY			(1)
#define HEIGHT_FILTER_TYPE_CMP_VALUE			(2)
GT_API GTN_GetAxisHeightFrqFilter(short core,short axis,short type,short *pFilterNum);
GT_API GTN_SetAxisHeightFrqFilter(short core,short axis,short type,short filterNum);


// 标定功能

#define CALIBRATION_MODE_DIFFERENT_STEP		(1)
// 标定stage
#define DIFFERENT_STEP_CALIBRATION_STAGE_NONE						(-1)//空闲
#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_SEARCH				(1)//碰板过程
#define DIFFERENT_STEP_CALIBRATION_STAGE_GO_OFFSET					(2)//运动偏置
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_INIT			(3)//开始标定前的初始化
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION				(4)//标定过程
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_DONE			(5)//标定完成
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_STOP			(6)//标定停止

#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_START_ERROR			(100)
#define DIFFERENT_STEP_CALIBRATION_STAGE_HOME_SEARCH_ERROR			(101)
#define DIFFERENT_STEP_CALIBRATION_STAGE_GO_OFFSET_ERROR			(102)
#define DIFFERENT_STEP_CALIBRATION_STAGE_CALIBRATION_ERROR			(103)
#define DIFFERENT_STEP_CALIBRATION_STAGE_NOT_AXIS_ON				(104)
#define DIFFERENT_STEP_CALIBRATION_STAGE_SET_TABLE_ERROR			(105)

// 最多标定点数。
#define DIFFERENT_STEP_CALIBRATION_STEP_MAX							(4000)

#define CALIBRATION_MODE_DIFFERENT_STEP		(1)
#define CALIBRATION_STEP_MAX			(50)

typedef struct
{
	short heightFbIndex;					// 第几路频率输入
	short motorEncoder;						// 第几路编码器
	short pad[2];

	double homeVel;							// 碰板的速度 （脉冲/毫秒）
	double homeDir;							// 碰板的运动方向
	int32_t frqMutationStopValue;				// 碰板时频率突变停止阈值
	int32_t offset;							// 碰板后反向停靠偏置。从停靠位置开始标定。

	double distance;						// 标定距离 (脉冲单位）
	double calibrationVel;					// 标定速度
	double calibrationStepCount;			// 标定段数
	double calibrationCountPerStep[CALIBRATION_STEP_MAX];	// 每段标定点数
	short tableId;						// 标定后数据表下载索引
}TDifferentStepCalibration;

typedef union
{
	TDifferentStepCalibration differentStep;
}THeightCalibration;

typedef struct
{
	short mode;							// 目前只能是变步距线性标定，CALIBRATION_MODE_DIFFERENT_STEP
	THeightCalibration prm;
}THeightCalibrationPrm;

GT_API GTN_ClearAxisHeightCalibrationSts(short core,short axis);
GT_API GTN_StopAxisHeightCalibration(short core,short axis);
GT_API GTN_RunAxisHeightCalibration(short core,short axis,short *pStage);
GT_API GTN_SetAxisHeightCalibration(short core,short axis,THeightCalibrationPrm *pPrm);

typedef struct
{
	int32_t positiveMutationValue;		// 接近挡板时，调高器频率突变量阈值，正整数。
	short positiveMutationStopLinkLimitType;// 接近挡板时,频率突变停止触发后关联的限位类型。
	int32_t negativeMutationValue;		//远离挡板时，调高器频率突变量阈值，正整数。
	short negativeMutationStopLinkLimitType;// 远离挡板时,频率突变停止触发后关联的限位类型。
}TFrqMutationStopPrm;
GT_API GTN_HeightFrqMutationStopEnable(short core,short axis,short enable,TFrqMutationStopPrm mutationStopPrm);
GT_API GTN_GetHeightFrqMutationStopEnable(short core,short axis,short *pEnable,TFrqMutationStopPrm *pMutationStopPrm);

GT_API GTN_GoToAxisFollowHeight(short core,short axis,int32_t followCmd,THeightFollowPrm followPrm,double trapVel,TTrapPrm trapPrm);

GT_API GTN_GetAxisHeightBand(short core,short axis,int32_t *pBand,int32_t *pTime);
GT_API GTN_SetAxisHeightBand(short core,short axis,int32_t band,int32_t time);

#define HEIGHT_FINDEDGE_LATCH_MAX   4

typedef struct
{
	short type;
	short moveAxis;
	short latchNum;
	short reserve1;
	short latchType[HEIGHT_FINDEDGE_LATCH_MAX];
	short latchIndex[HEIGHT_FINDEDGE_LATCH_MAX];
	int32_t  heightDiff;
	int32_t reserve2;
	double findTime;
}THeightFindEdgePrm;

typedef struct
{
	short findEdgeSts;
	short error;
	double latchValue[HEIGHT_FINDEDGE_LATCH_MAX];
}THeightFindEdgeResult;

GT_API GTN_SetHeightFindEdgePrm(short core,short heightFbIndex,THeightFindEdgePrm pFindEdgePrm);
GT_API GTN_GetHeightFindEdgeResult(short core,short heightFbIndex,THeightFindEdgeResult *pFindEdgeResult);
GT_API GTN_GetHeightFindEdgeCoef(short core,short heightFbIndex,double* alpha,double* beta);
GT_API GTN_CloseHeightFindEdge(short core,short heightFbIndex);
GT_API GTN_SetHeightFindEdgeStopMask(short core,short heightFbIndex,int32_t mask,int32_t option);
GT_API GTN_GetHeightFindEdgeStopMask(short core,short heightFbIndex,int32_t* mask,int32_t* option);

typedef struct
{
	double vel;			// 蛙跳速度
	double acc;			// 蛙跳加速度
	double dec;			// 蛙跳减速度
	double smoothTime;  // 平滑时间

	double safetyVel;	// 运动到安全工艺高度时的速度

	short slaveType;					// 从轴的类型。
	short slaveIndex;					// 从轴对应的索引。
	int32_t followCmd;		// 跟随高度
	short followEnable;	// 运动到跟随高度是否保持。
	short enable;		// 蛙跳使能
	short reserve1[2];
	double reserve2[4];
}TSetAxisLeapFrogPrm;
GT_API GTN_SetAxisLeapFrogPrm(short core,short axis,TSetAxisLeapFrogPrm* pAxisLeapFrogPrm);

GT_API GTN_SetLightPowerCtrlPara(short core,unsigned short ctrlMode,unsigned short *pDutyCycle,unsigned short *pPulseWidth);
GT_API GTN_GetLightPowerCtrlPara(short core,unsigned short *pCtrlMode,unsigned short *pDutyCycle,unsigned short *pPulseWidth);

GT_API GTN_UTL_ConvertPhysicToLogic(TPhysicInfo *pPhysicInfo,TLogicInfo *pLogicInfo);

/*-----------------------------------------------------------*/
/* Old Watch                                                */
/*-----------------------------------------------------------*/
#define WATCH_LIST_MAX                                  (8)
#define WATCH_LENGTH_MAX                                (32)
#define WATCH_FIFO_SIZE                                 (0x1E0)       // 480 word

#define WATCH_ADDRESS_TYPE_PRF_POS						(0)
#define WATCH_ADDRESS_TYPE_PRF_VEL						(1)
#define WATCH_ADDRESS_TYPE_ENC_POS						(2)
#define WATCH_ADDRESS_TYPE_ENC_VEL						(3)
#define WATCH_ADDRESS_TYPE_AXIS_PRF_POS					(4)
#define WATCH_ADDRESS_TYPE_AXIS_PRF_VEL					(5)
#define WATCH_ADDRESS_TYPE_AXIS_ENC_POS					(6)
#define WATCH_ADDRESS_TYPE_AXIS_ENC_VEL					(7)
#define WATCH_ADDRESS_TYPE_DAC							(8)
#define WATCH_ADDRESS_TYPE_LMT_POS						(9)
#define WATCH_ADDRESS_TYPE_LMT_NEG						(10)
#define WATCH_ADDRESS_TYPE_ALARM						(11)
#define WATCH_ADDRESS_TYPE_HOME							(12)
#define WATCH_ADDRESS_TYPE_GPI							(13)
#define WATCH_ADDRESS_TYPE_ENABLE						(14)
#define WATCH_ADDRESS_TYPE_CLEAR						(15)
#define WATCH_ADDRESS_TYPE_GPO							(16)
#define WATCH_ADDRESS_TYPE_CLOCK						(17)
#define WATCH_ADDRESS_TYPE_FLASH_BUFFER					(18)
#define WATCH_ADDRESS_TYPE_CRD_VEL						(19)
#define WATCH_ADDRESS_TYPE_ARRIVE						(20)
#define WATCH_ADDRESS_TYPE_POS_LOOP_ERROR				(21)
#define WATCH_ADDRESS_TYPE_POS_LOOP_OUTPUT				(22)
#define WATCH_ADDRESS_TYPE_POS_LOOP_INTERGRAL			(23)
#define WATCH_ADDRESS_TYPE_VEL_LOOP_ERROR				(24)
#define WATCH_ADDRESS_TYPE_VEL_LOOP_INTERGRAL			(25)
#define WATCH_ADDRESS_TYPE_SPRING_OFFSET				(26)
#define WATCH_ADDRESS_TYPE_BEFORE_SPRING				(27)
#define WATCH_ADDRESS_TYPE_AFTER_SPRING					(28)
#define WATCH_ADDRESS_TYPE_OBSERVER_LOWPASS_OUTPUT		(29)
#define WATCH_ADDRESS_TYPE_OBSERVER_OUTPUT_PRE			(30)
#define WATCH_ADDRESS_TYPE_OBSERVER_TORQUE_CMD_PRE		(31)
#define WATCH_ADDRESS_TYPE_OBSERVER_VEL_FEEDBACK_INTERGRAL		(32)
#define WATCH_ADDRESS_TYPE_OBSERVER_VEL_ERR_FEEDBACK_INTERGRAL	(33)

#define WATCH_ADDRESS_TYPE_ADC							(40)

#define WATCH_ADDRESS_TYPE_SCAN_SEGMENT_NUMBER			(200)

#define WATCH_ADDRESS_TYPE_SCAN_PRF_VEL					(210)

#define WATCH_ADDRESS_TYPE_SCAN_CRD_POS_X				(220)
#define WATCH_ADDRESS_TYPE_SCAN_CRD_POS_Y				(221)

#define WATCH_ADDRESS_TYPE_LASER_HSIO					(230)

#define WATCH_ADDRESS_TYPE_LASER_POWER					(240)

typedef struct Watch
{
	short count;
	short interval;
	uint32_t address[WATCH_LIST_MAX];
	short length[WATCH_LIST_MAX];
	uint32_t threshold;
} TWatch;

GT_API GTN_SetWatch(short core,TWatch *pWatch);
GT_API GTN_StartWatch(short core);
GT_API GTN_StartWatchStatic(short core);
GT_API GTN_StopWatch(short core);
GT_API GTN_StopWatchStatic(short core);
GT_API GTN_LoadWatchFifo(short core,short fifo,short *pBuf);
GT_API GTN_GetWatchFifo(short core,short *pFifo);
GT_API GTN_GetWatchAddress(short core,short type,short index,uint32_t *pAddress,short count);
GT_API GTN_GetWatchSts(short core,short *pRun,uint32_t *pCount);
GT_API GTN_GetWatchData(short core,uint32_t offset,short *pData,short count=1);

/*-----------------------------------------------------------*/
/* New Watch                                                  */
/*-----------------------------------------------------------*/
#define MC_WATCH_VAR                    (200)
#define MC_WATCH_EVENT                  (201)


#define WATCH_VAR_CLOCK						(1200)
#define WATCH_VAR_PRF_LOOP					(1201)

#define WATCH_VAR_SKIP_MODULE				(1211)
#define WATCH_VAR_TEST_MODE					(1212)

#define WATCH_VAR_COMMAND_CODE				(1220)
#define WATCH_VAR_COMMAND_DATA				(1221)
#define WATCH_VAR_COMMAND_COUNT				(1222)
#define WATCH_VAR_COMMAND_READ_FLAG 		(1223)

#define WATCH_VAR_LINK_PORT_TX_USE			(1660)
#define WATCH_VAR_LINK_PORT_TX_REQUIRE		(1661)

#define WATCH_VAR_PRF_POS					(6000)
#define WATCH_VAR_PRF_VEL					(6001)
#define WATCH_VAR_PRF_ACC					(6002)
#define WATCH_VAR_PRF_POS_RAW				(6003)
#define WATCH_VAR_PRF_VEL_RAW				(6004)

#define WATCH_VAR_PRF_VEL_MS				(6031)
#define WATCH_VAR_PRF_ACC_MS				(6032)

#define WATCH_VAR_PRF_RUN					(6200)

#define WATCH_VAR_PT_PRF_POS_F64			(6650)
#define WATCH_VAR_PT_SPACE					(6660)
#define WATCH_VAR_PT_RECEIVE				(6662)
#define WATCH_VAR_PT_EXECUTE				(6663)

#define WATCH_VAR_TRAP_TOTAL_TIME           (6670)
#define WATCH_VAR_TRAP_REMAIN_TIME          (6671)
// 7000 MoveAbsolute
#define WATCH_VAR_MOVE_ABSOLUTE_DONE		(7008)

// 7050 MoveVelocity
#define WATCH_VAR_MOVE_VELOCITY_IN_VELOCITY	(7058)

#define WATCH_VAR_PROFILE_MOVE_DONE			(7108)

// 7500 ProfileMove
#define WATCH_VAR_PROFILE_MOVE_VEL_ORG      (7500)    // 平滑模式时的预规划速度
#define WATCH_VAR_PROFILE_MOVE_VEL          (7501)    // 最终规划速度

#define WATCH_VAR_PROFILE_MOVE_VIRTUAL_VEL_ORG (7510) // 虚拟轴预规划速度
#define WATCH_VAR_PROFILE_MOVE_VIRTUAL_VEL     (7511) // 虚拟轴最终规划速度  

#define WATCH_VAR_PROFILE_MOVE_GROUP_VEL_ORG (7520)   // group分支单轴预规划速度    
#define WATCH_VAR_PROFILE_MOVE_GROUP_VEL     (7521)   // group分支单轴最终规划速度    

#define WATCH_VAR_CRD_PRF_POS				(8000)
#define WATCH_VAR_CRD_PRF_VEL				(8001)
#define WATCH_VAR_CRD_PRF_ACC				(8002)

#define WATCH_VAR_CRD_PRF_VEL_MS			(8031)
#define WATCH_VAR_CRD_PRF_ACC_MS			(8032)

#define WATCH_VAR_CRD_RUN					(8200)

#define WATCH_VAR_CRD_SEGMENT_NUMBER		(8202)
#define WATCH_VAR_CRD_SEGMENT_NUMBER_USER	(8203)
#define WATCH_VAR_CRD_COMMAND_RECEIVE		(8204)
#define WATCH_VAR_CRD_COMMAND_EXECUTE		(8205)

#define WATCH_VAR_CRD_FOLLOW_SLAVE_POS		(8600)
#define WATCH_VAR_CRD_FOLLOW_SLAVE_VEL		(8601)

#define WATCH_VAR_CRD_FOLLOW_STAGE			(8610)

#define WATCH_VAR_CRD_MPG_STATUS            (8620)
#define WATCH_VAR_CRD_MPG_DIR            	(8621)
#define WATCH_VAR_CRD_MPG_REF_VEL			(8622)
#define WATCH_VAR_CRD_MPG_RESERVE_HEAD		(8623)
#define WATCH_VAR_CRD_MPG_RESERVE_COUNT		(8624)
#define WATCH_VAR_CRD_MPG_SEG_USE_COUNT		(8625)
#define WATCH_VAR_CRD_MPG_DIR_SUM			(8626)
#define WATCH_VAR_CRD_MPG_ENC_VEL			(8627)

#define WATCH_VAR_CONTOUR_PEDAL_POS			(8800)
#define WATCH_VAR_CONTOUR_PRF_POS			(8801)
#define WATCH_VAR_CONTOUR_PRF_POS_SHAPING	(8802)
#define WATCH_VAR_CONTOUR_ENC_POS			(8803)
#define WATCH_VAR_CONTOUR_DAC               (8804)
#define WATCH_VAR_CONTOUR_SYN_ERROR         (8805)

#define WATCH_VAR_CONTOUR_FIFO_POINT_COUNT	(8808)
#define WATCH_VAR_CONTOUR_FIFO_POINT_MAX	(8809)
#define WATCH_VAR_CONTOUR_ERROR             (8810)
#define WATCH_VAR_CONTOUR_DELETE_COUNT      (8811)

#define WATCH_VAR_CBT_EXECUTE               (8900)
#define WATCH_VAR_CBT_STATE                 (8901)
#define WATCH_VAR_CBT_PIECE_TAG             (8902)
#define WATCH_VAR_CBT_CATCH_UP_START_POS    (8903)
#define WATCH_VAR_CBT_CATCH_UP_PROFILE_POS  (8904)
#define WATCH_VAR_CBT_VIRTUAL_PIECE_POS     (8905)
#define WATCH_VAR_CBT_ACTUAL_PIECE_POS      (8906)
#define WATCH_VAR_CBT_PIECE_POS_PCS         (8907)
#define WATCH_VAR_CBT_STOP_INFO             (8910)

#define WATCH_VAR_GROUP_PRF_VEL_MS			(9031)
#define WATCH_VAR_GROUP_PCS_SYN_VEL_MS  	(9032)
#define WATCH_VAR_GROUP_MCS_SYN_VEL_MS		(9033)
#define WATCH_VAR_GROUP_ACS_SYN_VEL_MS		(9034)

#define WATCH_VAR_GROUP_SMOOTH_WORK_PRF_VEL_MS (9041)

#define WATCH_VAR_GROUP_DYNAMIC_PCS_TRANSLATE  (9050)

#define WATCH_VAR_GROUP_PROJECTION_POS         (9070)
#define WATCH_VAR_GROUP_PROJECTION_POS_BEGIN   (9071)
#define WATCH_VAR_GROUP_PROJECTION_POS_END     (9072)
#define WATCH_VAR_GROUP_SEGMENT_LENGTH_PROFILE (9073)
#define WATCH_VAR_GROUP_SEGMENT_LENGTH_BEGIN   (9074)
#define WATCH_VAR_GROUP_SEGMENT_LENGTH_END     (9075)
#define WATCH_VAR_GROUP_SEGMENT_LENGTH_TOTAL   (9076)
#define WATCH_VAR_GROUP_SEGMENT_LENGTH_TOTAL_PROFILE (9077)

#define WATCH_VAR_PCS_PRF_POS               (9100)
#define WATCH_VAR_PCS_PRF_VEL               (9101)
#define WATCH_VAR_PCS_PRF_ACC               (9102)

#define WATCH_VAR_PCS_PRF_POS_FILTER        (9103)
#define WATCH_VAR_PCS_PRF_VEL_FILTER        (9104)
#define WATCH_VAR_PCS_PRF_ACC_FILTER        (9105)

#define WATCH_VAR_PCS_ENC_POS               (9106)
#define WATCH_VAR_PCS_ENC_VEL               (9107)
#define WATCH_VAR_PCS_ENC_ACC               (9108)

#define WATCH_VAR_MCS_PRF_POS               (9110)
#define WATCH_VAR_MCS_PRF_VEL               (9111)
#define WATCH_VAR_MCS_PRF_ACC               (9112)

#define WATCH_VAR_MCS_PRF_POS_FILTER        (9113)
#define WATCH_VAR_MCS_PRF_VEL_FILTER        (9114)
#define WATCH_VAR_MCS_PRF_ACC_FILTER        (9115)

#define WATCH_VAR_MCS_ENC_POS               (9116)
#define WATCH_VAR_MCS_ENC_VEL               (9117)
#define WATCH_VAR_MCS_ENC_ACC               (9118)

#define WATCH_VAR_ACS_PRF_POS               (9120)
#define WATCH_VAR_ACS_PRF_VEL               (9121)
#define WATCH_VAR_ACS_PRF_ACC               (9122)

#define WATCH_VAR_ACS_PRF_POS_FILTER        (9123)
#define WATCH_VAR_ACS_PRF_VEL_FILTER        (9124)
#define WATCH_VAR_ACS_PRF_ACC_FILTER        (9125)

#define WATCH_VAR_ACS_ENC_POS               (9126)
#define WATCH_VAR_ACS_ENC_VEL               (9127)
#define WATCH_VAR_ACS_ENC_ACC               (9128)

#define WATCH_VAR_GROUP_AXIS_TORQUE         (9130)
#define WATCH_VAR_TCS_PRF_POS               (9140)

#define WATCH_VAR_USER_TCS_PRF_POS          (9143)

#define WATCH_VAR_PCS_PRF_POS_POST          (9150)
#define WATCH_VAR_PCS_PRF_VEL_POST          (9151)
#define WATCH_VAR_PCS_PRF_ACC_POST          (9152)

#define WATCH_VAR_MCS_PRF_POS_POST          (9153)
#define WATCH_VAR_MCS_PRF_VEL_POST          (9154)
#define WATCH_VAR_MCS_PRF_ACC_POST          (9155)

#define WATCH_VAR_ACS_PRF_POS_POST          (9156)
#define WATCH_VAR_ACS_PRF_VEL_POST          (9157)
#define WATCH_VAR_ACS_PRF_ACC_POST          (9158)

#define WATCH_VAR_GROUP_MVCS_SUPERPOSITION_VALUE (9160)

#define WATCH_VAR_GROUP_RUN                 (9200)
#define WATCH_VAR_GROUOP_DYNAMIC_TRANSFORM_ENABLE (9201)
#define WATCH_VAR_GROUP_STOPINFO            (9202)

#define WATCH_VAR_LIST_EXECUTE				(9240)
#define WATCH_VAR_LIST_SEG_NUM              (9250)

#define WATCH_VAR_CATCH_UP_STATE                           (12000)
#define WATCH_VAR_CATCH_UP_COMMAND                         (12001)
#define WATCH_VAR_CATCH_UP_MASTER_POS                      (12002)
#define WATCH_VAR_CATCH_UP_MASTER_VEL                      (12003)
#define WATCH_VAR_CATCH_UP_SLAVE_POS                       (12004)
#define WATCH_VAR_CATCH_UP_SLAVE_VEL                       (12005)
#define WATCH_VAR_CATCH_UP_PIECE_POS                       (12006)
#define WATCH_VAR_CATCH_UP_PIECE_ID                        (12007)
#define WATCH_VAR_CATCH_UP_PIECE_COUNT                     (12008)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_RECEIVE      (12009)
#define WATCH_VAR_CATCH_UP_PIECE_FIFO_COMMAND_SEND         (12010)
#define WATCH_VAR_CATCH_UP_SYNCH_DISTANCE                  (12011)
#define WATCH_VAR_CATCH_UP_SYNCN_TIME                      (12012)
#define WATCH_VAR_CATCH_UP_PIECE_START_POS                 (12013)
#define WATCH_VAR_CATCH_UP_PIECE_SYNCH_POS                 (12014)
#define WATCH_VAR_CATCH_UP_SLAVE_START_POS                 (12015)
#define WATCH_VAR_CATCH_UP_SLAVE_SYNCH_POS                 (12016)

#define WATCH_VAR_POS_COMPARE_COMMAND_RECEIVE              (17422)
#define WATCH_VAR_POS_COMPARE_COMMAND_SEND                 (17423)
#define WATCH_VAR_POS_COMPARE_COMMAND_LEFT                 (17424)
#define WATCH_VAR_POS_COMPARE_COMMAND_TX                   (17425)
#define WATCH_VAR_POS_COMPARE_PULSE_COUNT                  (17426)
#define WATCH_VAR_POS_COMPARE_PSO_ON_OFF                   (17427)
#define WATCH_VAR_POS_COMPARE_PSO_ON_OFF_POS_X             (17428)
#define WATCH_VAR_POS_COMPARE_PSO_ON_OFF_POS_Y             (17429)
#define WATCH_VAR_POS_COMPARE_PSO_REG                      (17430)
#define	WATCH_VAR_POS_COMPARE_PSO_REG_EXT                  (17431)
#define	WATCH_VAR_POS_COMPARE_ENC_PSO_ON_OFF               (17432)
#define	WATCH_VAR_POS_COMPARE_MODULE_OVERFLOW              (17433)
#define	WATCH_VAR_POS_COMPARE_MODULE_SPACE                 (17434)

#define WATCH_VAR_SCAN_PRF_POS				(18000)
#define WATCH_VAR_SCAN_PRF_VEL				(18001)
#define WATCH_VAR_SCAN_PRF_ACC				(18002)

#define WATCH_VAR_SCAN_PRF_POS_X			(18010)
#define WATCH_VAR_SCAN_PRF_POS_Y			(18020)
#define WATCH_VAR_SCAN_PRF_POS_Z			(18030)

#define WATCH_VAR_SCAN_RUN					(18200)

#define WATCH_VAR_SCAN_SEGMENT_NUMBER		(18202)

#define WATCH_VAR_SCAN_COMMAND_RECEIVE		(18422)
#define WATCH_VAR_SCAN_COMMAND_SEND			(18423)
#define WATCH_VAR_SCAN_COMMAND_LEFT			(18424)
#define WATCH_VAR_SCAN_COMMAND_TX			(18425)

#define WATCH_VAR_LASER_HSIO				(18600)
#define WATCH_VAR_LASER_POWER				(18601)
#define WATCH_VAR_LASER_WORK_RATIO			(18602)
#define WATCH_VAR_LASER_POWER_FPGA          (18603)

#define WATCH_VAR_AXIS_PRF_POS				(20000)
#define WATCH_VAR_AXIS_PRF_VEL				(20001)
#define WATCH_VAR_AXIS_PRF_ACC				(20002)

#define WATCH_VAR_AXIS_PRF_POS_OTHER		(20004)
#define WATCH_VAR_AXIS_PRF_VEL_OTHER		(20005)

#define WATCH_VAR_AXIS_PRF_VEL_FILTER		(20011)

#define WATCH_VAR_AXIS_PRF_VEL_MS			(20031)
#define WATCH_VAR_AXIS_PRF_ACC_MS			(20032)

#define WATCH_VAR_AXIS_STATUS		        (20100)
#define WATCH_VAR_AXIS_STATUS_EX			(20101)

#define WATCH_VAR_AXIS_RUN		            (20200)

#define WATCH_VAR_AXIS_ENC_POS_FILTER		(20310)
#define WATCH_VAR_AXIS_ENC_VEL_FILTER		(20311)

#define WATCH_VAR_AXIS_ENC_VEL_64_FILTER	(20321)

#define WATCH_VAR_AXIS_PREDICTION_POS_IN    (20350)
#define WATCH_VAR_AXIS_PREDICTION_POS_IN_2  (20351)
#define WATCH_VAR_AXIS_PREDICTION_POS_OUT   (20352)

#define WATCH_VAR_AXIS_PREDICTION_VEL_IN    (20355)
#define WATCH_VAR_AXIS_PREDICTION_VEL_IN_2  (20356)
#define WATCH_VAR_AXIS_PREDICTION_VEL_OUT   (20357)

#define WATCH_VAR_AXIS_PREDICTION_OUT       (20360)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_POS                    (20361)
#define WATCH_VAR_AXIS_PREDICTION_FILTER_VEL                    (20362)

#define WATCH_VAR_POSCOMPARE_PREDICTION_ENABLE                  (20370)
#define WATCH_VAR_POSCOMPARE_PREDICTION_WORK                    (20371)
#define WATCH_VAR_POSCOMPARE_PREDICTION_OUTPUT_COUNT_TOTAL      (20372)
#define WATCH_VAR_DRIVER_ACT_VEL			(20400)
#define WATCH_VAR_STIMULATE_DATA			(20401)
#define WATCH_VAR_DRIVER_CURRENT_KVFF		(20402)
#define WATCH_VAR_AXIS_TORQUE               (20410)
#define WATCH_VAR_CONTROL_ERROR				(20500)

#define WATCH_VAR_BACKLASH_VALUE			(21000)
#define WATCH_VAR_LEADSCREW_VALUE			(21001)
#define WATCH_VAR_FRICTION_VALUE			(21003)

#define WATCH_VAR_PRESS_COMPENSATE_VALUE    (21004)
#define WATCH_VAR_PRESS_COMPENSATE_VALUE_FLOAT (21005)
#define WATCH_VAR_PRESS_COMPENSATE_VALUE_FLOAT_RAW (21006)


#define WATCH_VAR_ENC_POS					(30000)
#define WATCH_VAR_ENC_VEL					(30001)

#define WATCH_VAR_ENC_VEL_64				(30021)

#define WATCH_VAR_GPI						(31000)
#define WATCH_VAR_LIMIT_POSITIVE			(31010)
#define WATCH_VAR_LIMIT_NEGATIVE			(31020)
#define WATCH_VAR_ALARM   		        	(31030)
#define WATCH_VAR_HOME             			(31040)
#define WATCH_VAR_ARRIVE           			(31050)

#define WATCH_VAR_GPO						(32000)
#define WATCH_VAR_ENABLE					(32010)
#define WATCH_VAR_CLEAR 					(32020)

#define WATCH_VAR_AI     					(33000)
#define WATCH_VAR_AU_AI						(33100)

#define WATCH_VAR_AO     					(34000)

#define WATCH_VAR_TRIGGER_EXECUTE			(38000)
#define WATCH_VAR_TRIGGER_STATUS			(38001)
#define WATCH_VAR_TRIGGER_POSITION			(38002)

#define WATCH_VAR_TRIGGER_COUNT				(38010)

#define WATCH_VAR_TRIGGER_NOTIFY_ENABLE			(38020)
#define WATCH_VAR_TRIGGER_NOTIFY_STATUS_ECHO	(38021)
#define WATCH_VAR_TRIGGER_NOTIFY_CLEAR_WAIT		(38022)

#define WATCH_VAR_TRIGGER_DELTA_CROSS_COUNT		(38110)

#define WATCH_VAR_POS_LOOP_ERROR			(40000)

#define WATCH_VAR_WATCH_TIME				(52001)

#define WATCH_VAR_INT32						(52020)
#define WATCH_VAR_INT64						(52021)
#define WATCH_VAR_FLOAT						(52022)
#define WATCH_VAR_DOUBLE					(52023)
#define WATCH_VAR_BOOL						(52024)

#define WATCH_VAR_TERMINAL_LIMIT_POSITIVE	(53000)
#define WATCH_VAR_TERMINAL_LIMIT_NEGATIVE	(53001)
#define WATCH_VAR_TERMINAL_ALARM			(53002)
#define WATCH_VAR_TERMINAL_HOME				(53003)
#define WATCH_VAR_TERMINAL_GPI				(53004)
#define WATCH_VAR_TERMINAL_ARRIVE			(53005)

#define WATCH_VAR_TERMINAL_MPG				(53009)
#define WATCH_VAR_TERMINAL_ENABLE			(53010)
#define WATCH_VAR_TERMINAL_CLEAR			(53011)
#define WATCH_VAR_TERMINAL_GPO				(53012)

#define WATCH_VAR_TERMINAL_DAC				(53020)

#define WATCH_VAR_TERMINAL_PULSE			(53022)
#define WATCH_VAR_TERMINAL_ENCODER			(53023)
#define WATCH_VAR_TERMINAL_ADC				(53024)

#define WATCH_VAR_TERMINAL_AU_ENCODER		(53026)

#define WATCH_VAR_TERMINAL_PRF_POS			(53030)

#define WATCH_VAR_TERMINAL_TRIGGER_POSITION	(53040)
#define WATCH_VAR_TERMINAL_TRIGGER_STATUS	(53041)

#define WATCH_VAR_TERMINAL_COMMAND			(53100)
#define WATCH_VAR_TERMINAL_STATUS			(53101)

#define WATCH_VAR_DRIVER_COMMAND				(53110)
#define WATCH_VAR_DRIVER_STATUS				    (53111)
#define WATCH_VAR_DRIVER_FOLLOW_ERROR			(53112)

#define WATCH_VAR_HEIGHT_FRQ_RAW				(53200)
#define WATCH_VAR_HEIGHT_FRQ_FILTER_IIR			(53201)
#define WATCH_VAR_HEIGHT_FRQ_FILTER				(53202)
#define WATCH_VAR_HEIGHT_FRQ_INTERPOLATION		(53203)
#define WATCH_VAR_AXIS_HEIGHT_CMP_VALUE			(53204)
#define WATCH_VAR_AXIS_HEIGHT_CMP_VALUE_FILTER	(53205)
#define WATCH_VAR_AXIS_HEIGHT_CROSS_ENABLE_DONE	(53206)
#define WATCH_VAR_AXIS_HEIGHT_TARGETPOS			(53207)
#define WATCH_VAR_AXIS_HEIGHT_REF_POS			(53208)
#define WATCH_VAR_AXIS_HEIGHT_CNT_POS			(53209)
#define WATCH_VAR_AXIS_HEIGHT_CROSS_DISABLE_DONE (53210)
#define WATCH_VAR_AXIS_HEIGHT_FOLLOW_ENABLE 	(53211)
#define WATCH_VAR_AXIS_HEIGHT_STOP_STS 			(53212)
#define WATCH_VAR_EVENT_HIT						(60000)
#define WATCH_VAR_TASK_START					(61000)
#define WATCH_VAR_TASK_WORK						(61001)

#define WATCH_VAR_PRF_TORQUE					(61200)
#define WATCH_VAR_ATL_TORQUE					(61201)
#define WATCH_VAR_ATL_TORQUE_FILTER				(61202)

typedef struct
{
	short enable;						// 采集使能
	short run;							// 采集状态
	uint32_t time;					// 采集次数
	uint32_t head;					// 头指针
	uint32_t threshold;			// 最多容纳采集次数

	short interval;						// 采集间隔
	short mode;							// 采集模式
	unsigned short countBeforeEvent;	// 事件触发之前的采集数量
	unsigned short countAfterEvent;		// 事件触发以后的采集数量
	unsigned short varCount;			// 采集变量数量
	unsigned short eventCount;			// 采集事件数量
} TWatchInfo;

typedef struct
{
	short width;
	short precision;
	char seperator;
	short hex;
} TWatchFormat;

GT_API GTN_GetWatchInfo(short core,TWatchInfo *pInfo);
GT_API GTN_SetWatchFormat(short core,const TWatchFormat *pFormat);
GT_API GTN_GetWatchFormat(short core,TWatchFormat *pFormat);
GT_API GTN_SetWatchFormatEx(short core,short group,const TWatchFormat *pFormat);
GT_API GTN_GetWatchFormatEx(short core,short group,TWatchFormat *pFormat);
GT_API GTN_GetWatchInfoEx(short core,short group,TWatchInfo *pInfo);

#define WATCH_GROUP_TIMER					(0)
#define WATCH_GROUP_BACKGROUND				(1)

#define WATCH_LOAD_MODE_NONE				(0)
#define WATCH_LOAD_MODE_BOOT				(2)
#define WATCH_LOAD_MODE_RUN					(3)

#define WATCH_MODE_STATIC					(0)
#define WATCH_MODE_LOOP						(1)
#define WATCH_MODE_DYNAMIC					(2)

#define WATCH_EVENT_RUN						(1)
#define WATCH_EVENT_START					(10)
#define WATCH_EVENT_STOP					(20)
#define WATCH_EVENT_OFF						(30)

#define WATCH_CONDITION_NONE				(0)

#define WATCH_CONDITION_EQ					(1)
#define WATCH_CONDITION_NE					(2)
#define WATCH_CONDITION_GE					(3)
#define WATCH_CONDITION_LE					(4)
#define WATCH_CONDITION_GT  				(5)
#define WATCH_CONDITION_LT  				(6)

#define WATCH_CONDITION_CHANGE_TO			(11)
#define WATCH_CONDITION_CHANGE				(12)
#define WATCH_CONDITION_UP					(13)
#define WATCH_CONDITION_DOWN				(14)
#define WATCH_CONDITION_CHANGE_BEYOND		(15)
#define WATCH_CONDITION_CHANGE_MAX			(16)

#define WATCH_CONDITION_REMAIN_AT			(21)
#define WATCH_CONDITION_REMAIN				(22)
#define WATCH_CONDITION_CROSS_POSITIVE		(23)
#define WATCH_CONDITION_CROSS_NEGATIVE		(24)

#define WATCH_CONDITION_NEAREST				(31)
#define WATCH_CONDITION_DELTA				(32)

//typedef struct
//{
//	unsigned short type;
//	unsigned short index;
//	unsigned short id;
//} TWatchVar;

typedef struct
{
	unsigned short type;
	unsigned short loop;
	unsigned short watchCount;
	TWatchVar var;
	unsigned short condition;
	double value;
} TWatchEvent;

typedef struct
{
	uint32_t *pAddress;
	unsigned short length;
	short fraction;
	unsigned short format;
	unsigned short hex;
	unsigned short sign;
	unsigned short bit;
} TWatchVarInfo;

GT_API GTN_SetWatchGroup(short core,short group);
GT_API GTN_GetWatchGroup(short core,short *pGroup);
GT_API GTN_LoadWatchConfig(short core,char *pFile);
GT_API GTN_SaveWatchConfig(short core,short group,char *pFile);
GT_API GTN_PrintWatch(short core,const char *pFileName,int32_t start=0,uint32_t printCount=0);
GT_API GTN_ReadWatch(short core,short varIndex,double *pBuffer,uint32_t bufferSize,uint32_t *pReadCount);
GT_API GTN_ProgramWatch(short core,short loadMode);
GT_API GTN_GetWatchLoadMode(short core,short group,short *pLoadMode);

GT_API GTN_ClearWatch(short core,short mode=0);
GT_API GTN_AddWatchVar(short core,const TWatchVar *pVar);
GT_API GTN_AddWatchEvent(short core,const TWatchEvent *pEvent);
GT_API GTN_WatchOn(short core,short interval=0,short mode=WATCH_MODE_STATIC,unsigned short count=0);
GT_API GTN_WatchOff(short core);

GT_API GTN_GetWatchEvent(short core,short index,TWatchEvent *pEvent);
GT_API GTN_GetWatchVar(short core,short index,TWatchVar *pVar,TWatchVarInfo *pVarInfo);
GT_API GTN_SetWatchModeEx(short core,short group,short mode);
GT_API GTN_SetWatchIntervalEx(short core,short group,short interval);

//-------------------------------------------------------------------------------------------------
// 路径补偿功能
//-------------------------------------------------------------------------------------------------
// 路径补偿点结构体
typedef struct
{
	double pos[2];                               // x轴、y轴位置
	double compensateValue;	                     // z轴补偿位置
}TCompensatePathPoint;

// 路径补偿表结构体
typedef struct
{
	short tableIndex;                            // 路径补偿表索引
	short axisType[2];                           // 查询路径补偿表X、Y方向位置类型，MC_PROFILE:规划位置 MC_ENCODER:编码器位置
	short axisIndex[2];                          // 查询路径补偿表X、Y方向所使用的轴号						
	short reserve1[3];                            // 保留值，必须为0
	double offset[2];                            // x轴、y轴偏移
	double compensateVelMax;                     // 补偿值变化的最大速度 ，单位：pulse/ms，用于（1）结束补偿时，消除补偿值的过程（2）超出补偿的误差带时，补偿值变化的过程
	double approachRadius;                       // 补偿起始点和补偿终点的有效区域半径
	double workBand;                             // 补偿运行时的误差带
	double workHeight;                           // z轴工作高度
	double reserve2[2];							 // 保留值，必须为0
}TCompensatePathParameter;

// 路径补偿状态结构体
typedef struct{
	short enable;								 // 路径补偿使能标志
	short state;								 // 路径补偿状态
	short errorStatus;                           // 路径补偿是否超出errorBand  0:未超出误差带 1：超出误差带
	short pointIndex;                            // 路径补偿当前点的补偿起点索引
	double compensateStartPos[2];                // 路径补偿当前点的补偿起点位置
	double compensateValue;                      // 路径补偿当前点的补偿量
	double reserve1[2];                          // 保留值，为0
}TCompensatePathStatus;

// 设置路径补偿表
GT_API GTN_SetCompensatePathTable(short core,short tableIndex, TCompensatePathPoint *pCompensatePathPoint, short pointCount);
// 设置路径补偿参数
GT_API GTN_SetCompensatePathParameter(short core,short axis, TCompensatePathParameter *pCompensatePath);
// 读取路径补偿参数
GT_API GTN_GetCompensatePathParameter(short core,short axis, TCompensatePathParameter *pCompensatePath);
// 启动路径补偿
GT_API GTN_StartCompensatePath(short core,short axis, short startMode = 0, void* pParameter = NULL);
// 关闭路径补偿
GT_API GTN_StopCompensatePath(short core,short axis, short stopMode = 0, void* pParameter = NULL);
// 获取路径补偿状态
GT_API GTN_GetCompensatePathStatus(short core,short axis, TCompensatePathStatus *pCompensatePathStatus);

/*-----------------------------------------------------------*/
/* 辅助绝对值编码器						                              */
/*-----------------------------------------------------------*/
//-----------------------------------------------------------------------------------
// 功能说明：配置辅助绝对值编码器的多圈范围，该指令辅助编码器不包含手轮，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----辅助编码器在当前站的物理序号，取值范围：[1,8]
// input：range----辅助编码器多圈计数的最大值。例如：编码器多圈是16位的，则range=65536，
//-----------------------------------------------------------------------------------
GT_API GTN_RN_SetAuAbsEncMultiTurnRange(short cardIndex,short stationPhyId,short encoder,double range);

//-----------------------------------------------------------------------------------
// 功能说明：读取辅助绝对值编码器的位置，该指令辅助编码器不包含手轮，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----辅助编码器在当前站的物理序号，取值范围：[1,8]
// output：pPos----读取的绝对值辅助编码器的位置值
//-----------------------------------------------------------------------------------
GT_API GTN_RN_ReadAuAbsEncPos(short cardIndex,short stationPhyId,short encoder,double *pPos);

//-----------------------------------------------------------------------------------
// 功能说明：配置绝对值编码器的多圈范围，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// input：range----编码器多圈计数的最大值。例如：编码器多圈是16位的，则range=65536，
//-----------------------------------------------------------------------------------
GT_API GTN_RN_SetAbsEncMultiTurnRange(short cardIndex,short stationPhyId,short encoder,double range);

//-------------------------------------------------------------------------------------------------------
// 功能说明：读取绝对值编码器的位置值，物理索引接口
// input：cardIndex----卡号，取值范围：[1,16]
// input：stationPhyId----物理站号，取值范围：[0,64]
// input：encoder----编码器在当前站的物理序号，取值范围：[1,8]
// output：pPos----读取的绝对值编码器的位置值
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadAbsEncPos(short cardIndex,short stationPhyId,short encoder,double *pPos);

/*-----------------------------------------------------------*/
/* 串行通讯指令（本地485/232通信指令）  (需链接gt_rn.lib，不推荐使用)*/
/*-----------------------------------------------------------*/
short GT_RN_ComOpen(short index);
short GT_RN_ComClose(short index);
short GT_RN_ComRead(short index, uint32_t readLen, uint32_t* pResLen, unsigned char * pData);
short GT_RN_ComWrite(short index, uint32_t writeLen , uint32_t* pResLen, unsigned char * pData);
short GT_RN_ComGetState(short index, unsigned char * pState);
short GT_RN_ComSetSettings(short index, uint32_t baudrate, unsigned char stopBits, unsigned char parity);
short GT_RN_ComClearErr(short index, unsigned char flag);
short GT_RN_ComSetMode(short index,unsigned short comMode);


/*-----------------------------------------------------------*/
/* 串行通讯指令（通用485/232通信指令）(需链接gt_rn.lib，不推荐使用)*/
/*-----------------------------------------------------------*/
short GTN_RN_SerialComOpen(short cardIndex, short stationphyId, short comIndex);
short GTN_RN_SerialComClose(short cardIndex, short stationphyId, short comIndex);
short GTN_RN_SerialComRead(short cardIndex, short stationphyId, short comIndex, uint32_t readLen, uint32_t* pResLen, unsigned char * pData);
short GTN_RN_SerialComWrite(short cardIndex, short stationphyId, short comIndex, uint32_t writeLen , uint32_t* pResLen, unsigned char * pData);
short GTN_RN_SerialComGetState(short cardIndex, short stationphyId, short comIndex, unsigned char *pState);
short GTN_RN_SerialComSetSettings(short cardIndex, short stationphyId, short comIndex, uint32_t baudrate,unsigned char stopBits, unsigned char parity);
short GTN_RN_SerialComClearErr(short cardIndex, short stationphyId, short comIndex, unsigned char flag);
short GTN_RN_SerialComSetMode(short cardIndex, short stationphyId, short comIndex, unsigned short comMode);

//读取网络辅助编码器
GT_API GTN_RN_GetRemotAuEncPos(short cardIndex, short stationPhyId, short auEncIndex,double *pAuEncPos,short anEncCount);

//-------------------------------------------------------------------------------------------------------
//获取轴模块LED显示模式
//GTN_RN_ReadLedDispalyMode(short cardIndex, short stationPhyId, unsigned char *pMode, unsigned char *pRadix)
// mode:灯板数码管显示模式： 
//0：工作模式(上电初始状态显示零，轴使能后显示小数点，具体切换方式待定)；  
//1：站号模式(显示站号，短暂显示拨码变动)；  
//2：报警模式(预留)；
//radix:灯板数码管显示进制： 0：二进制；  1：八进制；  2：十进制；  3：十六进制；
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ReadLedDispalyMode(short cardIndex, short stationPhyId, unsigned char *pMode, unsigned char *pRadix);
//-------------------------------------------------------------------------------------------------------
//设置轴模块LED显示模式
//GTN_RN_WriteLedDisplayMode(short cardIndex, short stationPhyId, unsigned char mode, unsigned char radix)
// mode:灯板数码管显示模式： 
//0：工作模式(上电初始状态显示零，轴使能后显示小数点，具体切换方式待定)；  
//1：站号模式(显示站号，短暂显示拨码变动)；  
//2：报警模式(预留)；
//radix:灯板数码管显示进制： 0：二进制；  1：八进制；  2：十进制；  3：十六进制；
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_WriteLedDisplayMode(short cardIndex, short stationPhyId, unsigned char mode, unsigned char radix);
//-------------------------------------------------------------------------------------------------------
//导出伺服参数
//GTN_RN_ExportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName)
// cardIndex:从1开始
// stationPhyId:从0开始
// axis:轴号
// pFilePath：模板文件的路径
// pFileName：导出的文件路径及名称
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ExportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName);
//-------------------------------------------------------------------------------------------------------
//导入伺服参数
//GTN_RN_ImportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName)
// cardIndex:从1开始
// stationPhyId:从0开始
// axis:轴号
// pFilePath：模板文件的路径
// pFileName：导出的文件路径及名称
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ImportServoParams(short cardIndex, short stationPhyId, short axis, char *pFilePath, char *pFileName);

//获取多轴绝对值编码器
typedef struct 
{
	short stationId;
	short axis;
	uint64_t multiLines;
	double pos;
}StAbsEncPos;
GT_API GTN_RN_GetMultiAbsEncPos(short cardIndex,StAbsEncPos* pStAbsEncPos,short count);

//-------------------------------------------------------------------------------------------------------
// 初始化PC端DLL Buffer大小
// cardIndex:从1开始
// stationPhyId:从0开始
// bufferAddrWidth：设置DLL Buffer的空间大小
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpInitialWrBuffer(short cardIndex, short stationPhyId, unsigned short bufferAddrWidth);
//-------------------------------------------------------------------------------------------------------
// 获取当前DLL Buffer中的有效数据量和还有多少剩余空间可以写入
// cardIndex:从1开始
// stationPhyId:从0开始
// pValidWordNum：当前DLL Buffer还有多少word空间的数据，没有发送到轴模块
// pRemainsWordSpase: 当前DLL Buffer还有多少word空间的数据，允许用户写入
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpGetWrBufferStatus(short cardIndex, short stationPhyId, uint32_t *pValidWordNum, uint32_t *pRemainsWordSpase);
//-------------------------------------------------------------------------------------------------------
// 将数据写入DLL Buffer中
// cardIndex:从1开始
// stationPhyId:从0开始
// pData：要写入的数据指针
// pWordNumVaild: 期望写入的数据的word的个数
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpWrBufferData(short cardIndex, short stationPhyId, unsigned short *pData, uint32_t wordNum, uint32_t *pWordNumVaild);
//-------------------------------------------------------------------------------------------------------
// 根据轴模块的buffer状态，将DLL Buffer中的数据读取，更新到轴模块缓冲区。
// cardIndex:从1开始
// stationPhyId:从0开始
// dataType：需要压入缓冲区的数据类型: 0：SCAN，振镜相关数据1：CMP，位置比较相关数据
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanCmpUpdateBufferDataToFPGA(short cardIndex, short stationPhyId, short dataType);
//-------------------------------------------------------------------------------------------------------
// 使能轴模块缓冲区，使能后轴模块从指令缓冲区读取指令（只应用于振镜）
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanEnableFifo(short cardIndex, short stationPhyId);
//-------------------------------------------------------------------------------------------------------
// 关闭轴模块缓冲区，关闭后轴模块暂停从缓冲区读取指令（只应用于振镜）
// cardIndex:从1开始
// stationPhyId:从0开始
//-------------------------------------------------------------------------------------------------------
GT_API GTN_RN_ScanDisableFifo(short cardIndex, short stationPhyId);

//获取轴停止详细信息功能
GT_API GTN_GetAxisStopInfo(short core,short axis,int32_t *pInfo,double *pPos,short count=1);
GT_API GTN_ClearAxisStopInfo(short core,short axis,short count=1);
GT_API GTN_SetAxisStopInfoMode(short core,short axis,short *pMode,short count);
GT_API GTN_GetAxisStopInfoMode(short core,short axis,short *pMode,short count);

typedef struct TPvtLinePrm
{
	short mode;  //参数生效模式
	short smoothTimer;  //平稳时间
	int32_t moveTime;  //运动时间
	int32_t pad;       //保留位
	double synVel;   //合成目标速度
	double synAcc;  //合成加速度
	double synJeck; //合成加加速度
}TPvtLinePrm;
GT_API GTN_PvtLine(short core,short axisCount,short *moveAxis,int32_t *pos,TPvtLinePrm *pPrm);

GT_API GTN_GetTerminalLinkCount(short core,unsigned short *pTerminalLinkCount);

























/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*                        EtherCAT网络协议功能函数		                    */
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/

//EtherCAT
#define ECAT_STATE_OP    (2)

#define ECAT_AXIS_MAX		12
#define ECAT_MODE_HOMING	6
#define ECAT_MODE_CSP		8
#define ECAT_MODE_CSV		9
#define ECAT_MODE_CST		10
#define ECAT_MODE_PV		3
#define ECAT_MODE_PT		4

#define ECAT_PROBE_TRIGGER_TYPE_FIRST_EVENT	0
#define ECAT_PROBE_TRIGGER_TYPE_CONTINUES	1

#define ECAT_PROBE_TRIGGER_LEVEL_POS		1
#define ECAT_PROBE_TRIGGER_LEVEL_NEG		-1
#define ECAT_PROBE_TRIGGER_LEVEL_DUL		0

#define  ECAT_ACT_ENC_POS	0
#define  ECAT_ACT_ENC_VEL	1
#define  ECAT_ACT_CURRENT	2
#define  ECAT_ACT_TORQUE	3
#define  ECAT_ACT_CT		4

typedef struct
{
	int32_t slave_cnt;
	int32_t slave_type;
	int32_t motion_cnt;
	int32_t io_nmap;
	int32_t io_length;
	uint32_t Vid;
	uint32_t Pid;
	int io_type;             //1 输入  2 输出  3 输入输出
	int dump[10];
} TSlaveInfo;

typedef struct
{
	short dcError; 
	short workingCountErrorCnt; //发现断线计数
	short ecatCommStatus;      //总线通讯状态
	short workingCount;        //当前WorkingCount值
	short offlineFlag;         //断线标志
	short rootOfflineFlag;    //根部断线标志
	short workingCountFirstError;  //发现断线时的计数
	short profileTimeOutFlag;      //规划超时标志
	short dump[20];
} TEcatErrInfo;

//EtherCAT interface
// 设置Home回零参数，如果指针为空，则不需要发sdo指令 [Thunder-2020-9-24]
GT_API GTN_SetEcatHomingPrmEx( short core, short axis,short *psMethod,double *pdSpeed1,double *pdSpeed2,double *pdAcc,int32_t *plOffset,unsigned short *pusProbeFunction );
GT_API GTN_SetEcatHomingPrm(short core,short axis,short method,double speed1,double speed2,double acc,int32_t offset,unsigned short probeFunction);
GT_API GTN_StartEcatHoming(short core,short axis);
GT_API GTN_SetHomingMode(short core,short axis,short mode);
GT_API GTN_GetEcatHomingStatus(short core,short axis,unsigned short *homingStatus);
GT_API GTN_StopEcatHoming(short core,short axis);
GT_API GTN_SetTouchProbeFunction(short core,short axis,short ProbePrm);
GT_API GTN_SetTouchProbeFunctionEx(short core,short axis,short Probe1Enable,short Probe1TriggerType,short Probe1TriggerLevel,short Probe2Enable,short Probe2TriggerType,short Probe2TriggerLevel);
GT_API GTN_GetTouchProbeStatus(short core,short axis,unsigned short *probeStatus,int32_t *probe1PosValue,int32_t *probe1NegValue,int32_t *probe2PosValue,int32_t *probe2NegValue);
GT_API GTN_SetEcatGpioConfig(short core,short effectiveLevel,short direction);
GT_API GTN_SetEcatAxisOnThreshold(short core,short axis,unsigned short threshold);
GT_API GTN_GetEcatAxisOnThreshold(short core,short axis,unsigned short* threshold);
GT_API GTN_InitEcatComm(short core);
GT_API GTN_InitEcatCommEx(short core,char *eniFilePath);
GT_API GTN_StartEcatComm(short core);
GT_API GTN_IsEcatReady(short core,short *pStatus);
GT_API GTN_TerminateEcatComm(short core);
GT_API GTN_GetEcatErrorCode(short core,short axis,unsigned short *pErrorCode);
GT_API GTN_GetDcError(short core,short *pError);
GT_API GTN_GetEcatDcErrorEx(short core,TEcatErrInfo *pEcatErrorInfo);
GT_API GTN_SetEcatAxisMode(short core,short axis,short mode);
GT_API GTN_GetEcatAxisMode(short core,short axis,unsigned short *drvMode);
GT_API GTN_SetEcatAxisPV(short core,short axis,int32_t velocity);
GT_API GTN_SetEcatAxisPT(short core,short axis,short torque);
GT_API GTN_GetEcatEncPos(short core,short axis,int32_t *pEncPos);
GT_API GTN_GetEcatAuxEncPos(short core,short axis,int32_t *pAuxEncPos);
GT_API GTN_GetEcatEncVel(short core,short axis,int32_t *pVelocity);
GT_API GTN_GetEcatAxisAtlCurrent(short core,short axis,short *pCur);
GT_API GTN_GetEcatAxisAtlTorque(short core,short axis,short *pTorque);
GT_API GTN_GetEcatAxisAV(short core,short axis,int32_t *pVel);
GT_API GTN_GetEcatAxisACT(short core,short axis,short *pCur,short *pTorque);
GT_API GTN_GetEcatMCType(short core,short *pType);
GT_API GTN_SetMcEcatAxisNum(short core,short Num);
GT_API GTN_GetMcEcatAxisNum(short core,short *pNum);
GT_API GTN_SetPosScale(short core,short axis,unsigned short posScale);
GT_API GTN_GetPosScale(short core,short axis,unsigned short *posScale);
GT_API GTN_GetEcatAxisAI(short core,short axis,short channel,short *pValue);
GT_API GTN_SetEcatAxisAO(short core,short axis,unsigned short AoValue);
GT_API GTN_GetEcatAxisAO(short core,short axis,unsigned short *pAo);
GT_API GTN_GetEcatAxisDI(short core,short axis,uint32_t *pDi);
GT_API GTN_GetEcatAxisDIEx(short core,short axis,uint32_t *pDi,short count);
GT_API GTN_SetEcatAxisDO(short core,short axis,uint32_t DoValue);
GT_API GTN_GetEcatAxisDO(short core,short axis,uint32_t *pDo);
GT_API GTN_SetEcatAxisDOBit(short core,short axis,short bitOffset,char DoBitValue);
GT_API GTN_GetEcatAxisDOBit(short core,short axis,short bitOffset,char *pDoBitValue);
GT_API GTN_GetEcatSlaves(short core,short *pSlaveMotionCnt,short *pSlaveIOCnt);
GT_API GTN_GetEcatSlaveInfo(short core,short slaveIndex,TSlaveInfo *pSlaveinfo);
GT_API GTN_GetEcatAxisPE(short core,short axis,int32_t *pPosErr);
GT_API GTN_SetEcatRawData(short core,unsigned short offset,unsigned short nByteSize,unsigned char *pValue);
GT_API GTN_GetEcatRawData(short core,unsigned short offset,unsigned short nByteSize,unsigned char *pValue);
GT_API GTN_SetEcatAxisTorqueOffset(short core,short axis,short torqueOffset);
GT_API GTN_GetEcatAxisTorqueOffset(short core,short axis,short *pTorqueOffset);
GT_API GTN_GetEcatPdoLength(short core,short *pPdoLen);
GT_API GTN_EcatSDODownload(short core,unsigned short slave_position,unsigned short index,unsigned char subindex,unsigned char *data,unsigned int data_size,unsigned int *abort_code);
GT_API GTN_EcatSDOUpload(short core,unsigned short slave_position,unsigned short index,unsigned char subindex,unsigned char *target,unsigned int target_size,unsigned int *result_size,unsigned int *abort_code);
GT_API GTN_EcatSDODownloadEx(short core,unsigned short axisNumber,unsigned short index,unsigned char subindex,unsigned char *data,unsigned int data_size,unsigned int *abort_code);
GT_API GTN_EcatSDOUploadEx(short core,unsigned short axisNumber,unsigned short index,unsigned char subindex,unsigned char *target,unsigned int target_size,unsigned int *result_size,unsigned int *abort_code);
GT_API GTN_EcatIOReadInput(short core,unsigned short slaveno,unsigned short offset,unsigned short nSize,unsigned char *pValue);
GT_API GTN_EcatIOReadOutput(short core,unsigned short slaveno,unsigned short offset,unsigned short nSize,unsigned char *pValue);
GT_API GTN_EcatIOWriteOutput(short core,unsigned short slaveno,unsigned short offset,unsigned short nSize,unsigned char *pValue);
GT_API GTN_EcatIOBitWriteOutput(short core,unsigned short slaveno,unsigned short offset,short Index,unsigned char value);
GT_API GTN_EcatIOSynch(short core);
GT_API GTN_EcatIOUpdateUpload(short core);
GT_API GTN_EcatIOUpdateDnload(short core);
GT_API GTN_GetEcatAxisPdoData(short core,short axis,unsigned short object,unsigned char *pValue);
GT_API GTN_SetEcatSlaveObjectData(short core,short slaveIndex,unsigned short object,unsigned short objectSubIndex,unsigned char *pValue,unsigned int data_size,unsigned char *pOpMode);
GT_API GTN_GetEcatSlaveObjectData(short core,short slaveIndex,unsigned short object,unsigned short objectSubIndex,unsigned char *pValue,unsigned int data_size,unsigned char *pOpMode);

// 新增在CSV模式下，目标速度60ff的最大值，单位驱动器60ff单位[Thunder-2019-10-11]
GT_API GTN_SetEcatAxisMaxTargetVel(short core,short axis,uint32_t targetMaxVel);
GT_API GTN_GetEcatAxisMaxTargetVel(short core,short axis,uint32_t *targetMaxVel);

//厂商自定义了对象字0x6040的高8位时，将高8位的值填入指令，内部和低8位位或运算
GT_API GTN_SetEcatAxisCtrlwordEx(short core,short axis,unsigned short ctrlex);
GT_API GTN_GetEcatAxisCtrlwordEx(short core,short axis,unsigned short *ctrlex);

GT_API GTN_RelateEcatIOSlaveToMCIOModule(short core,short slaveindex,short IOModuleindex);
GT_API GTN_SetMCIOModuleValue(short core,short IOModuleindex,short iomapindex,int32_t *val,short cnt);
GT_API GTN_GetMCIOModuleValue(short core,short IOModuleindex,short iomapindex,int32_t *val,short cnt);

GT_API GTN_RelateEcatSlaveToMcAuEncoder(short core,short auenc,short ecatAxisIndex);
GT_API GTN_RelateEcatSlaveToMcMpgEncoder(short core,short mpg,short ecatAxisIndex);
GT_API GTN_RelateEcatSlaveToMcAuEncoderEx(short core,short auenc,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcMpgDi(short core,short mpg,short ecatAxisIndex,short bitoffset);
GT_API GTN_RelateEcatSlaveToMcMpgDiEx(short core,short mpg,short ecatIndex,short ecatType,short bitoffset,short pdoOffset);
GT_API GTN_RelateEcatSlaveToMcMpgEncoderEx(short core,short mpg,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAdc(short core,short adc,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAuAdc(short core,short auadc,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcGpiBit(short core,short gpi,short ecatIndex,short ecatType,short bitoffset,short pdoOffset);
GT_API GTN_RelateEcatSlaveToMcGpoBit(short core,short gpo,short ecatIndex,short ecatType,short bitoffset,short pdoOffset);
GT_API GTN_LoadRelateEcatSlaveConfig(short core,char *pFile);
GT_API GTN_RelateEcatSlaveToMcDac(short core,short dac,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_RelateEcatSlaveToMcAuDac(short core,short auDac,short ecatIndex,short ecatType,short pdoOffset,short pdoByteLength);
GT_API GTN_GetMcRelateEcatSlaveInfo(short core,short mcType,short index,short *pEcatIndex,short *pEcatType,short *pPdoOffset,short *pBitoffset,short *pdPoByteLength);
GT_API GTN_GetAxisModuleInfo(short core,short axis,short *pModule,short *pSubAxis);


