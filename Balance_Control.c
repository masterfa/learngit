#include "AllHeaders.h"
int16s_t g_nCarAcceVal;           //角速度
int16s_t g_nCarGyroVal;           //角加速度
int16s_t g_nCarAngle;             //车体角度

int16s_t g_nLeftMotorOut;
int16s_t g_nRightMotorOut;
int32s_t Turn_SpeedOut_L;
int32s_t Turn_SpeedOut_R;

int16s_t nSpeed;
int16s_t nLeft;   
int16s_t nRight;
/*------------------------------------------
查表数组，用于对从 -1~+1 的 asin 对应的角度
-------------------------------------------*/
const float  Asin_to_Angle[201]={ -90.000000,-81.890386,-78.521659,-75.930132,-73.739795,-71.805128,-70.051556,-68.434815,
-66.926082,-65.505352,-64.158067,-62.873247,-61.642363,-60.458639,-59.316583,-58.211669,-57.140120,-56.098738,-55.084794,
-54.095931,-53.130102,-52.185511,-51.260575,-50.353889,-49.464198,-48.590378,-47.731416,-46.886394,-46.054480,-45.234915,
-44.427004,-43.630109,-42.843643,-42.067065,-41.299873,-40.541602,-39.791819,-39.050123,-38.316134,-37.589503,-36.869898,
-36.157008,-35.450543,-34.750226,-34.055798,-33.367013,-32.683639,-32.005455,-31.332251,-30.663830,-30.000000,-29.340582,
-28.685402,-28.034297,-27.387108,-26.743684,-26.103881,-25.467560,-24.834587,-24.204835,-23.578178,-22.954499,-22.333683,
-21.715617,-21.100196,-20.487315,-19.876874,-19.268775,-18.662925,-18.059230,-17.457603,-16.857956,-16.260205,-15.664267,
-15.070062,-14.477512,-13.886540,-13.297072,-12.709033,-12.122352,-11.536959,-10.952784,-10.369760,-9.787819,-9.206896,
-8.626927,-8.047846,-7.469592,-6.892103,-6.315316,-5.739170,-5.163607,-4.588566,-4.013987,-3.439813,-2.865984,-2.292443,
-1.719131,-1.145992,-0.572967,0.000000,0.572967,1.145992,1.719131,2.292443,2.865984,3.439813,4.013987,4.588566,5.163607,
5.739170,6.315316,6.892103,7.469592,8.047846,8.626927,9.206896,9.787819,10.369760,10.952784,11.536959,12.122352,12.709033,
13.297072,13.886540,14.477512,15.070062,15.664267,16.260205,16.857956,17.457603,18.059230,18.662925,19.268775,19.876874,
20.487315,21.100196,21.715617,22.333683,22.954499,23.578178,24.204835,24.834587,25.467560,26.103881,26.743684,27.387108,
28.034297,28.685402,29.340582,30.000000,30.663830,31.332251,32.005455,32.683639,33.367013,34.055798,34.750226,35.450543,
36.157008,36.869898,37.589503,38.316134,39.050123,39.791819,40.541602,41.299873,42.067065,42.843643,43.630109,44.427004,
45.234915,46.054480,46.886394,47.731416,48.590378,49.464198,50.353889,51.260575,52.185511,53.130102,54.095931,55.084794,
56.098738,57.140120,58.211669,59.316583,60.458639,61.642363,62.873247,64.158067,65.505352,66.926082,68.434815,70.051556,
71.805128,73.739795,75.930132,78.521659,81.890386,90.000000,
};
/*========================================
函数名：CarVoltageGet()
作用  ：处理采集的AD值，将其转换为角度与角速度
备注  ：无
=========================================*/
float g_CarAngle=0;
INT16S g_CarAngleSet=CARANGLEMIDDLE;
float  g_GyroscopeAngleIntegral=CARANGLEMIDDLE/10,g_GravityAngle=0,g_GyroscopeAngleSpeed;
INT16S g_Gyro_X[GYRO_NUM]={0},g_Acc_X[ACC_NUM]={0},g_Gyro_Cnt=0,g_Acc_Cnt=0,g_Gyro_Y[GYROY_NUM]={0},g_GyroY_Cnt=0;
INT16S g_GyroX,g_AccX;
float fDeltaValue=0;
void MovingAverageFilter()
{
  INT8U i;
  g_GyroX=g_AccX=0;
  g_Gyro_X[g_Gyro_Cnt++]=Gyro_X;
  g_Acc_X[g_Acc_Cnt++]=Acc_X;
  
  if(g_Gyro_Cnt>=GYRO_NUM)g_Gyro_Cnt=0;
   if(g_Acc_Cnt>=ACC_NUM)g_Acc_Cnt=0;
   
  for(i=0;i<GYRO_NUM;i++)
    g_GyroX+=g_Gyro_X[i];
  g_GyroX/=GYRO_NUM;
  
    for(i=0;i<ACC_NUM;i++)
    g_AccX+=g_Acc_X[i];
  g_AccX/=ACC_NUM;
  
  /*  if(g_GyroY_Cnt==GYROY_NUM)
    {
      for(i=0;i<=GYROY_NUM-2;i++)
      g_Gyro_Y[i]=g_Gyro_Y[i+1];
      g_GyroY_Cnt--;
    }
   g_Gyro_Y[g_GyroY_Cnt++]=Gyro_Y;*/
}
void CarVoltageGet(void) 
{
  MovingAverageFilter();
  g_GravityAngle =(INT16S)(Asin_to_Angle[(abs_value(Acc_X)*100)/63]*GRAVITY_ANGLE_RATIO);
  g_GyroscopeAngleSpeed = g_GyroX*GYROSCOPE_ANGLE_RATIO;
          
  g_CarAngle = g_GyroscopeAngleIntegral;
  fDeltaValue = (g_GravityAngle - g_CarAngle)/GRAVITY_ADJUST_TIME_CONSTANT;
  //记住INT不要和float乱加会出bug!!!!!!
  //积分一定要用float型，否则小于1的数都会被忽视，效果太差
  g_GyroscopeAngleIntegral += (g_GyroscopeAngleSpeed +fDeltaValue)/GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
 
  Kalman_Filter(g_CarAngle,g_GyroscopeAngleSpeed);
  
  g_nCarAngle   = ((INT16S)(Angle*10) );                        //调整车体角度  
  g_nCarGyroVal = (INT16S)( Angle_dot ); 
}
/*==========================================
函数名：CarAngleControl()
作用  ：车体角度控制，内环
备注  ：无
===========================================*/
void CarAngleControl(void)
{
  int16s_t nP, nD;
 
  nP = g_nCarAngle-g_CarAngleSet;
  nD = g_nCarGyroVal;
  nP = (int16s_t)(-nP * (UP_KP)); 
  nD = (int16s_t)(-nD * (UP_KD));
 
  nSpeed = nP + nD;
   
  if(nSpeed > MOTOR_SPEED_SET_MAX) 	
  {
    nSpeed = MOTOR_SPEED_SET_MAX;
  }
  else if(nSpeed < MOTOR_SPEED_SET_MIN)	
  {
    nSpeed = MOTOR_SPEED_SET_MIN;
  }
  nLeft  =  (int16s_t)nSpeed;   
  nRight =  (int16s_t)nSpeed;
  g_nLeftMotorOut = -nLeft;
  g_nRightMotorOut = -nRight;
  
  Motor_Control();    
}