#include "LDWS_Interface.h"
#include "LDWS_AlarmDecision.h"
#include <stdio.h>
#include <string.h>

#ifdef _LDWS_ENABLED_
#define LDWS_SUP_REQ_FLG_TLEFT          (0x01)           /* 左转向有效 */
#define LDWS_SUP_REQ_FLG_TRIGHT         (0x02)           /* 右转向有效 */
//#define LDWS_SUP_REQ_FLG_AP             (0x08)           /* 加速踏板信号有效 */
//#define LDWS_SUP_REQ_FLG_BRAKE          (0x10)           /* 刹车信号有效 */
//#define LDWS_SUP_REQ_FLG_SOT_TOO_MUCH   (0x40)           /* 方向盘转速过大 */
#define LDWS_SUP_REQ_FLG_WS_VALID       (0x80)           /* 报警车速进入有效状态 */
#define LDWS_SUP_REQ_FLG_TKEEP          (0x100)          /* 进入转向维持状态 */
#define LDWS_SUP_REQ_FLG_CHANGE_L       (0X200)          /* 左变道 */
#define LDWS_SUP_REQ_FLG_CHANGE_R       (0x400)          /* 右变道 */
#define LDWS_SUP_REQ_VOICE_WARNING      (0x20)           /* 声音报警 */
#define LDWS_SUP_LEFT_THETA             (0X04)           /*左边线角度过大*/
#define LDWS_SUP_RIGHT_THETA            (0X800)          /*右边线角度过大*/
#define CAR_ANGLE_VALUE                 (0x03)               /*转向角过大*/

#define LFW_INTO_LV_WARNING_AREA        (-1)             /* 车辆左前轮进入左侧报警区域 */
#define FW_INTO_NO_WARNING_AREA         (0)              /* 车辆前轮进入非报警区域 */
#define RFW_INTO_RV_WARNING_AREA        (1)              /* 车辆右前轮进入右侧报警区域 */

#define LDWS_STATE_LOW_SPEED            (0x01)          /*低速行驶<60*/
#define LDWS_STATE_HIGH_SPEED           (0x02)          /*高速行驶>60*/
#define LDWS_STATE_CAR_LOAD             (0X04)          /*路宽标志*/

#define LS_STEERING_ANGLE_WARNING_MAX        (5.0)           /* 抑制报警的最大前轮转角 */
#define LDWS_KEEP_TIME_CHANGE                (60)             /*变道抑制时间为60帧*/
#define LDWS_LOAD_SIGNAL_LAMP                (10)             /*红绿灯路口抑制*/
#define LDWS_LOAD_LOSE_LINE                   (15)            /*如果有15帧没有线数据则认为到了路口*/
#define NARROW_ROAD_WIDTH                     (2.7)           /*过滤2.7m以下的窄道路*/

float leftAngle,leftOffset,rightAngle,rightOffset,roadWval;
int bmp_cnt=0,lineLevel,totalLevel,routeFlag;
int warningNumL = 0;
int warningVectorL[5] = {0};
int warningNumR = 0;
int warningVectorR[5] = {0};
/* Private typedef ************************************************************/
typedef struct                                           
{
	double earliestLine;                                 /* 最早报警线 */
	double theresholdLine;                               /* 报警临界线 */
	double latestLine;                                   /* 最晚报警线 */

	double speed;                                        /* 当前有效车速 */
	double rateOfDeparture;                              /* 当前偏离速度 */

	unsigned char deparSpeedWarningFlg;                             /* 为1，表示以偏离速度报警，0,则以配置临界线报警 */
} WarningAreaInfoStruct;
/* 报警区域设置参数 */
static WarningAreaInfoStruct	gsv_LDWSWarningAreaParam;  /* 报警区域设置参数 */
static int                      giv_LDWSSREQFlg;           /* 抑制请求标记 */
static char 					roadWidth;                  /* 路宽标志*/ 
static LDWS_OutputStruct		gsv_LDWSOutput;
static double Car_Width;
//static double Car_Length;


static float GetDWOffsetAndRate(WarningAreaInfoStruct *pWAParam, float theta, float offset)
{
	float temp;
#ifdef _CAN_REC_ENABLED_   
	/* 根据车速及偏角，计算出最早报警线位置 */
	if ((speed_60 & LDWS_STATE_HIGH_SPEED)==LDWS_STATE_HIGH_SPEED)
	{	
		pWAParam->rateOfDeparture =
				pWAParam->speed * (float)sin(theta);
		if (pWAParam->rateOfDeparture < 0)
		{
			pWAParam->rateOfDeparture = -pWAParam->rateOfDeparture;
		}
		if (pWAParam->rateOfDeparture > 0.0 && pWAParam->rateOfDeparture <= 0.5)
		{
			pWAParam->theresholdLine = 760.0 + pWAParam->earliestLine*2; //20cm
			
		}
		else if (pWAParam->rateOfDeparture > 0.5 && pWAParam->rateOfDeparture <= 1.0)
		{
			pWAParam->theresholdLine = 1500 * pWAParam->rateOfDeparture + pWAParam->earliestLine*2;//20cm~100cm
			
		}
		else if (pWAParam->rateOfDeparture > 1.0)//1m/s
		{
			pWAParam->theresholdLine = 1500.0 + pWAParam->earliestLine*2;//1m	
		}	
		else if(pWAParam->rateOfDeparture == 0)
		{
	          pWAParam->theresholdLine  = 160; //20cm
	    }
	}
	else if((speed_60 & LDWS_STATE_LOW_SPEED) == LDWS_STATE_LOW_SPEED)
	{
			pWAParam->theresholdLine =20;
	}
#else
	pWAParam->theresholdLine = Car_Width * 0.5;
	 pWAParam->latestLine = -1;
#endif	

	/*if ((float)sin(theta) == 0.0)
	{
		//temp = offset * scale/8;
		temp = offset * scale_tmp - giv_LDWSUserConfig.vpWidth*0.9;
	}
	else
	{
		k = (float)sin(theta) / (float)cos(theta);
		if (k < 0)
		{
			k = -k;
		}	
		temp = offset * scale_tmp - giv_LDWSUserConfig.vpLength * k - giv_LDWSUserConfig.vpWidth;
	}*/
	temp= offset;
	return temp;
}

static char LDWS_flag=0,l_count=0,r_count=0;
static char pltmp=0,r_tmp=0;
static int l_change_bmp=0,r_change_bmp=0;

static int DetWarningArea(WarningAreaInfoStruct *pWAParam)
{
	
	float l_offset,r_offset;
	int sumL = 0;
	int sumR = 0;
	int i = 0;
	float theretmp = 0;
	float camerleftdist = Car_Width / 2;
	float camerrightdist = Car_Width / 2;
	
	double LDWS_WARNING_THERE_L = LDWS_GetLeftDeviation();
	double LDWS_WARNING_THERE_R = LDWS_GetRightDeviation();
	//判断线是否全部消失，如果全部消失，有可能到了红绿灯路口，抑制5s
	if(routeFlag == 0)
	{
		if(pltmp >LDWS_LOAD_LOSE_LINE && r_tmp == 1)
		{
			pltmp =0;
			r_tmp =0;
		}
	}

	if(routeFlag == 0 && pltmp < LDWS_LOAD_LOSE_LINE)
	{
		pltmp++;
	}
	else if(pltmp >= LDWS_LOAD_LOSE_LINE)
	{
		if(routeFlag)
		{			
			pltmp++;
			r_tmp=1;
			if (pltmp < LDWS_LOAD_SIGNAL_LAMP)
			{			
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				l_count=0;
				r_count=0;
				pltmp=0;
				r_tmp=0;
				giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_R;
				giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_L;
				
			}		
		}
	}
	else
	{
		pltmp=0;
	}


    //两根线都是有效线
	if ((routeFlag) && (lineLevel > (totalLevel - 3)))
    {
		 //区分窄车道
		if(roadWval < NARROW_ROAD_WIDTH)
		{
			roadWidth |= LDWS_STATE_CAR_LOAD;
		}
		else
		{
			roadWidth &= ~LDWS_STATE_CAR_LOAD;
		}

		// 计算车道内缘到前轮边缘的距离及偏离速度 
		l_offset = GetDWOffsetAndRate(pWAParam, leftAngle,leftOffset);

		if(l_offset < 0)
		{
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_L;
		}

		//当距离超过临界值时，判断变道结束，解除抑制
		if((l_offset > (camerleftdist + LDWS_WARNING_THERE_L))&& (r_count != 1))
		{
			 LDWS_flag=0;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_R;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_LEFT_THETA;
			 if((giv_LDWSSREQFlg & LDWS_SUP_REQ_VOICE_WARNING) == LDWS_SUP_REQ_VOICE_WARNING)
			 {
				giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_VOICE_WARNING;
			 }
		}
		//重置左偏离临界值，如果已经左压线报警（左轮），则需要左轮远离0.2米有再解除报警
		if(l_offset > camerleftdist)
		{
	   		if((gsv_LDWSOutput.warningSingal & LDWS_WARNING_LEFT_IMAGE)== LDWS_WARNING_LEFT_IMAGE)
	   		{
				theretmp=(float)LDWS_WARNING_THERE_L;
			}
		}
		// 该区域为左侧报警区域 
		//if (l_offset <= (pWAParam->theresholdLine + theretmp) && l_offset >= pWAParam->latestLine)
		if (l_offset <= (camerleftdist + LDWS_WARNING_THERE_L) && l_offset >= pWAParam->latestLine)
		{	
			l_count = 1;
			r_count = 0;
			//判断左线的角度，如果大于2度则抑制
			if(leftAngle > 1.6)
			{
				giv_LDWSSREQFlg |= LDWS_SUP_RIGHT_THETA;
				//printf("leftAngle > 1.6 but turn left !");
			}
			else if(leftAngle < -1.6)
			{
				//giv_LDWSSREQFlg |= LDWS_SUP_LEFT_THETA;
				//printf("leftAngle < -1.6 ????????!");
			}
			else
			{
				giv_LDWSSREQFlg &= ~LDWS_SUP_LEFT_THETA;
			}
			//左变道抑制
			if((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_CHANGE_R) == LDWS_SUP_REQ_FLG_CHANGE_R)
			{
				if(LDWS_flag == 0)
				{
					l_change_bmp = bmp_cnt;
					LDWS_flag=1;
				}
				bmp_cnt++;
				if (bmp_cnt - l_change_bmp > LDWS_KEEP_TIME_CHANGE) //变道5s后没有恢复正常行驶，则重新开启检测
				{
				
					giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_R;	
					LDWS_flag=0;	
					l_change_bmp=0;
					bmp_cnt=0;
				}
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				bmp_cnt=0;
			}
		
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_L;
			warningVectorL[warningNumL++] = 1;
			if(warningNumL >= 5)
				warningNumL = 0;

			giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_R;

			for (i = 0; i < 5; i++)
			{
				sumL += warningVectorL[i];
			}
			if(sumL > 0.6 * 5)
				return LFW_INTO_LV_WARNING_AREA;
			else
				return FW_INTO_NO_WARNING_AREA;				
		}
		else
		{
			warningVectorL[warningNumL++] = 0;
			if(warningNumL >= 5)
				warningNumL = 0;
			theretmp = 0;
		}
	
		
		r_offset = GetDWOffsetAndRate(pWAParam, rightAngle,rightOffset);
		if(r_offset <0)
		{
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_R;
		}
	
		if((r_offset >= (camerrightdist + LDWS_WARNING_THERE_R))&& (l_count != 1))
		{
			 LDWS_flag=0;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_L;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_RIGHT_THETA;
			 if((giv_LDWSSREQFlg & LDWS_SUP_REQ_VOICE_WARNING) == LDWS_SUP_REQ_VOICE_WARNING)
			 {
				giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_VOICE_WARNING;
			 }
		}
		//重置右偏离临界值
		if(r_offset > camerrightdist)
		{
	   		if((gsv_LDWSOutput.warningSingal & LDWS_WARNING_RIGHT_IMAGE)== LDWS_WARNING_RIGHT_IMAGE)
	   		{
				theretmp=(float)LDWS_WARNING_THERE_R;
			}
		}
	
		// 该区域为右侧报警区域 				
		//if (r_offset <= (pWAParam->theresholdLine + theretmp) && r_offset >= pWAParam->latestLine)
		if (r_offset <= (camerrightdist + LDWS_WARNING_THERE_R) && r_offset >= pWAParam->latestLine)
		{ 
			r_count = 1;
			l_count =0;

			//判断右线的角度，如果大于2度则抑制
			if(rightAngle > 1.6)
			{
				//giv_LDWSSREQFlg |= LDWS_SUP_RIGHT_THETA;
				//printf("leftAngle > 1.6 ????????!");
			}
			else if(rightAngle < -1.6)
			{
				giv_LDWSSREQFlg |= LDWS_SUP_LEFT_THETA;
				//printf("leftAngle < -1.6 but turn right!");
			}
			else
			{
				giv_LDWSSREQFlg &= ~LDWS_SUP_RIGHT_THETA;
			}
			//右变道抑制
			if((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_CHANGE_L) == LDWS_SUP_REQ_FLG_CHANGE_L)
			{
				if(LDWS_flag == 0)
				{
					r_change_bmp = bmp_cnt;
					LDWS_flag=1;
				}
				bmp_cnt++;
				if (bmp_cnt - r_change_bmp > LDWS_KEEP_TIME_CHANGE) //变道5s后没有恢复正常行驶，则重新开启检测
				{
					giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_L;
					LDWS_flag=0;	
					r_change_bmp=0;
					bmp_cnt=0;
				}
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				bmp_cnt = 0;
			}
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_R;
			warningVectorR[warningNumR++] = 1;
			if(warningNumR >= 5)
				warningNumR = 0;

			giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_L;

			for (i = 0; i < 5; i++)
			{
				sumR += warningVectorR[i];
			}
			if(sumR > 0.6 * 5)
			   return RFW_INTO_RV_WARNING_AREA;
			else
               return FW_INTO_NO_WARNING_AREA;
		}
		else
		{
			warningVectorR[warningNumR++] = 0;
			if(warningNumR >= 5)
				warningNumR = 0;
			theretmp = 0;
		}
	}	

	r_count=0;
	l_count=0;
	bmp_cnt=0;
    theretmp = 0;
	return FW_INTO_NO_WARNING_AREA;
}

int AlarmMain(LDWS_Output *lineAttri)
{
	 
#define PI 3.14159626
	int lcv_alarmAreaFlg;
	Car_Width = LDWS_GetCarWidth();    //????1.6m
   	routeFlag = lineAttri->Route;
	rightOffset = lineAttri->Param[0] - lineAttri->Param[1];                       //右车道线的距离
	leftOffset = lineAttri->Param[1];  //左车道线的距离
	leftAngle =lineAttri->Param[2] * 180 / PI;// <0 车辆左偏
	rightAngle =lineAttri->Param[2] * 180 / PI;// >0车辆右偏
	roadWval = lineAttri->Param[0];

	if(lineAttri->Route_half == 1)
	   lineLevel =lineAttri->Confidence_detection[1];//可信度>3时处理线数据
	else if(lineAttri->Route == 1)
	   lineLevel =lineAttri->Confidence_detection[0];//可信度>3时处理线数据
	else if(lineAttri->Route == 2)
	   lineLevel =lineAttri->Confidence_detection[2];//可信度>3时处理线数据
	else if(lineAttri->Route == 3)
	   lineLevel =lineAttri->Confidence_detection[3];//可信度>3时处理线数据
	else
	   lineLevel = 0;

	totalLevel = lineAttri->Confidence;
//	 printf("roadw = %f,l_offset =%f, r_offset = %f, psi =%f ,alpha =%f, Ch=%f, cl=%f, level=%d\n",
 //                    roadWval, leftOffset,rightOffset,
//                     leftAngle,
//                     lineAttri->Param[3] * 180 / PI, lineAttri->Param[4] * 180 / PI, lineAttri->Param[5] * 180 / PI,lineLevel);
	lcv_alarmAreaFlg = DetWarningArea(&gsv_LDWSWarningAreaParam);

	if((roadWidth  & LDWS_STATE_CAR_LOAD) == LDWS_STATE_CAR_LOAD)
	{
		lcv_alarmAreaFlg = 0;
	}
	if ((lcv_alarmAreaFlg == LFW_INTO_LV_WARNING_AREA)
	//	&&((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_TLEFT) != LDWS_SUP_REQ_FLG_TLEFT)
		&&((giv_LDWSSREQFlg & LDWS_SUP_LEFT_THETA) != LDWS_SUP_LEFT_THETA))     
	{		
		gsv_LDWSOutput.warningSingal |= LDWS_WARNING_LEFT_IMAGE;
		return 1;
	}
	else if ((lcv_alarmAreaFlg == RFW_INTO_RV_WARNING_AREA)
	//	&&((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_TRIGHT) != LDWS_SUP_REQ_FLG_TRIGHT)
		&&((giv_LDWSSREQFlg & LDWS_SUP_RIGHT_THETA) != LDWS_SUP_RIGHT_THETA))
	{
		
		gsv_LDWSOutput.warningSingal |= LDWS_WARNING_RIGHT_IMAGE;
		return 2;
	}
	else
	{
		gsv_LDWSOutput.warningSingal = 0;
		return 0;	
	}
	
}



#endif

