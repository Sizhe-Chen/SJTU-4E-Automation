#include <project.h>
#include<stdio.h>
#include<stdlib.h>


// 变量
extern uint8 const CYCODE LCD_Char_1_customFonts[]; //载入传统字体
uint8 port = 0; //0: 左 1: 中 2: 右
int  A[3] = {0, 0, 0}; //左中右传感器数值
char A_char[3][5]; //左中右传感器数值的字符串
int   left_speed, right_speed, correction_speed;
int   error = 0, error_before = 0;


// 参数，需要根据小车，场地，光线调试
int   A_standard = 200;      			  //巡线中，A[1]（中间巡线传感器）可容忍最大阈值
int   black_threshold = 400;			  //黑色阈值
int   speed = 600;           			  //直线速度设定
int   PWM_left = 4550, PWM_right = 4550;  //左右电机速度基准值
float Kp = 3, Kd = 6;  			 	  //PD参数


// 核心控制算法
void Servo_PWM()
{
    error = A[1] - A_standard;
    correction_speed = Kp * error + Kd * (error - error_before);
	error_before = error;
    
    //直行基准速度
	left_speed = right_speed = speed;
	//右传感器为黑，右转
	if (error > 0 && A[2] < black_threshold && A[2] < A[0])
		right_speed -= correction_speed;
	//左传感器为黑，左转
	if (error > 0 && A[0] < black_threshold && A[2] > A[0])
		left_speed  -= correction_speed;
    
    PWMLeft_WriteCompare (PWM_left  - left_speed );
    PWMRight_WriteCompare(PWM_right + right_speed);
	
    A[0] = ADC_SAR_Seq_1_GetResult16(0);
	A[1] = ADC_SAR_Seq_1_GetResult16(1);
	A[2] = ADC_SAR_Seq_1_GetResult16(2);
	sprintf(A_char[0],"%d ", A[0]);
	sprintf(A_char[1],"%d ", A[1]);
	sprintf(A_char[2],"%d ", A[2]);
	LCD_Char_1_Position(1,0);
	LCD_Char_1_PrintString(A_char[0]);
	LCD_Char_1_Position(1,5);
	LCD_Char_1_PrintString(A_char[1]);
	LCD_Char_1_Position(1,10);
	LCD_Char_1_PrintString(A_char[2]);
	
	//PWMLeft_WriteCompare (3850);
    //PWMRight_WriteCompare(5550);
}


// 初始化函数
void Initialization()
{
	CyGlobalIntEnable;
    PWMLeft_Start();
    PWMRight_Start();

    LCD_Char_1_Start();
    LCD_Char_1_LoadCustomFonts(LCD_Char_1_customFonts);
	LCD_Char_1_Position(0,0);
    LCD_Char_1_PrintString("Left Midd Right");
	LCD_Char_1_Position(1,0);
	LCD_Char_1_PrintString("                "); 
	
	ADC_SAR_Seq_1_Start();
    CyDelay(1);
    ADC_SAR_Seq_1_StartConvert();
}


// 主函数
int main()
{
    Initialization();
    while(1) Servo_PWM();
    return 0;
}