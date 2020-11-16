#ifndef _LDWS_H_
#define _LDWS_H_

#define _LDWS_ENABLED_
#ifdef _LDWS_ENABLED_


/* 车辆类型 */  
enum VehicleType                                                                   
{
	Passenger = 0,
	Commercial
};

/* 关于LDWS用户配置参数 */
typedef struct
{

	double theresholdLine;                             /* 不采用偏离车速触发报警时，定义报警临界线位置
														  采用偏离车速报警时，该值定义在报警范围内，距离最早报警线的距离 */
	double vpLength;                                   /* 发生车道偏离的车辆特定部位相对车中心的纵向位置 */
	double vpWidth;                                    /* 发生车道偏离的车辆特定部位相对车中心的横向位置 */
	
	double vLength;                                    /* 车长 */
	double vWidth;                                     /* 车宽 */
	double wheelbase;                                  /* 轴距 */
	float  factorIN;                                   /* 模拟参数缩放因子 */

	unsigned char vehicleType;                                    /* 车辆类型 */
	unsigned char deparSpeedWarningFlg;                           /* 为1，表示以偏离速度报警，0,则以配置临界线报警 */
	
} LDWS_ConfigInfoStruct;

/* LDWS输出 */
typedef struct
{
	signed long Reserve;

	unsigned char state;                                          /* 输出LDWS系统状态 */
	unsigned char warningSingal;                                  /* 输出LDWS系统结果 */
} LDWS_OutputStruct;

/* Extern define **************************************************************/

#define LDWS_STA_FLG_ACTIVE_OPEN               (0x01)  /* LDWS开/关 */
#define LDWS_STA_FLG_HARDWARE_FAULT            (0x02)  /* 硬件故障状态 */
#define LDWS_STA_FLG_BUS_COM_FAULT             (0x04)  /* 总线通信故障 */

#define LDWS_WARNING_LEFT_IMAGE                (0x01)  /* 左偏离图像报警 */
#define LDWS_WARNING_RIGHT_IMAGE               (0x02)  /* 右偏离图像报警 */
#define LDWS_WARNING_VOICE                     (0x04)  /* 偏离声音报警 */
#define LDWS_WARNING_THERE                     0.21     /* 车道偏离临时值*/

extern int AlarmMain(LDWS_Output *lineAttri);

#endif

#endif
