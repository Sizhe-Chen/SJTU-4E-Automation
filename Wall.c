#include <project.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

//函数声明
void cruise_revision();      //贴墙误差过大修正
void pid_cruise();           //PID控制贴墙
void turn_right();           //转弯投票结算
void control();              //核心控制算法
void set_state(int state);   //传感器数值和行车状态显示LCD
void PWM_go(float left_ratio, float right_ratio);//控制电机
int get_sound();             //得到超声波传感器数值
void Initialize_Components();//初始化中断，电机，LCD，ADC红外传感器

//变量定义
int PWM_base = 4550;  //左右电机速度基准值
int speed = 800;      //直线基准速度设定
char A_char[4][5];    //LCD打印的字符
int A[4], i;	      //LCD打印的数字
extern const uint8 CYCODE LCD_Char_customFonts[];//载入传统字体
int sensor_sound, sensor_sound_before, sensor_red; //超声传感器数值, 红外传感器数值
int error = 0, error_before = 0, sum_error = 0; 
int count_for_more = 0, count_for_less = 0, total_count = 7; 


//贴墙误差过大修正
void cruise_revision()
{
	//函数局部变量
	float short_ratio = 0.5;
	int short_time = 150;

	if (error >= 20 && sensor_red < 32)
	{
		set_state(2); //右小转
		PWM_go(1, short_ratio);
		CyDelay(short_time);
		PWM_go(1, 1);
		CyDelay(short_time);
		PWM_go(short_ratio, 1);
		CyDelay(short_time);
	}
	
	if (error <= -20 && sensor_red < 32)
	{
		set_state(1); //左小转
		PWM_go(short_ratio, 1);
		CyDelay(short_time);
		PWM_go(1, 1);
		CyDelay(120);
		PWM_go(1, short_ratio);
		CyDelay(120);
	}
}

//PID控制贴墙
void pid_cruise()
{
	//函数局部变量
	float ratio_min = 0.2;
	float ratio = 0;
	float deviation = 0;   
	float Kp = 0.019, Kd = 20, Ki = 0.0002, Ka = 0.31;//PID参数，及偏差量作用于转向的程度

	deviation = Kd * (error - error_before) + Kp * error + Ki * sum_error;

	if (deviation > 0) 
	{
		ratio = 1 - Ka*deviation;
		if (ratio < ratio_min) ratio = ratio_min;
		set_state(4); //向右转
		PWM_go(1, ratio);
	}
	else
	{
		ratio = 1 + Ka*deviation;
		if (ratio < ratio_min) ratio = ratio_min;
		set_state(3);  //向左转
		PWM_go(ratio, 1);
	}
}

//根据投票结果判断是否右转
void turn_right()
{
	//函数局部变量
	float turn_slow_ratio = 0.36;
	int threshold_turn, turn_count = 0;
	int threshold_end_turn = 30; //红外传感器结束转弯阈值 

	if (sensor_sound_before > 50) threshold_turn = 35;
	else threshold_turn = 40;
	
	//投票结束，看看是否要右大转
	if (count_for_more > count_for_less)
	{
		set_state(6); //右大转
		while(0.5/(0.00002*ADC_SAR_Seq_GetResult16(0) - 0.00045)-2 > threshold_end_turn && turn_count < threshold_turn)
		{
			PWM_go(1, turn_slow_ratio);
			CyDelay(100);
			turn_count += 1;
		}
		turn_count = 0;
		PWM_go(turn_slow_ratio, 1);
		CyDelay(50);
	}   

	//投票数置零，若下次还需要投票，则犹豫不决的时间长许多
	total_count = 25;
	count_for_less = 0;
	count_for_more = 0;
}

//核心控制算法
void control()
{
	//函数局部变量
	int threshold_begin_turn = 200;                   //右侧超声传感器开始转弯阈值
	int threshold_cruise = 50, threshold_red = 25;    //右侧超声传感器巡线阈值，红外传感器投票阈值
	float slow_cruise_ratio = 0.35, back_ratio = -1;  //投票时速度，后退速度
	
	//得到传感器数值
	sensor_sound = (10000-get_sound())/6*10;
	sensor_red = 0.5/(0.00002*ADC_SAR_Seq_GetResult16(0) - 0.00045)-2;
	
	//重要变量初始化
	if (Button_Left_Read() * Button_Right_Read() == 0 || sensor_sound > threshold_begin_turn)
	{
		error = 0;
		sum_error = 0;
	}
	if(sensor_sound <= threshold_begin_turn)
	{
		count_for_less = 0;
		count_for_more = 0;
		total_count = 7;
	}

	//情况1：碰撞，则倒退，再左转
	if (Button_Left_Read() * Button_Right_Read() == 0)
	{	
		set_state(-1); //倒车
		PWM_go(0.15*back_ratio, back_ratio);
		CyDelay(500);
		
		set_state(5); //左大转
		PWM_go(-0.8, 0.8);
		CyDelay(950);
	}
	//情况2：当超声传感器检测到示数超过时，对红外连续采样n次，投票判断
	else if(sensor_sound > threshold_begin_turn)
	{
		set_state(-2); //犹豫不决

		//开始投票
		if (count_for_more + count_for_less <= total_count)
		{
			PWM_go(slow_cruise_ratio, slow_cruise_ratio);
			if (sensor_red > threshold_red) count_for_more += 1;
			else count_for_less += 1;
		}
		//计票采取动作
		else turn_right();
	}
	//情况3：超声示数不超过，以超声正常巡线
	else
	{
		error = sensor_sound - threshold_cruise;

		cruise_revision();
		pid_cruise();
		
		error_before = error;
		sensor_sound_before = sensor_sound;
		sum_error += error;
	}
}

//把传感器数值和行车状态显示在LCD上
void set_state(int state)
{	
	A[0] = 0;
	A[1] = sensor_red;
	A[2] = sensor_sound;
	A[3] = state;
	for(i=0; i<4; i++)
	{
		sprintf(A_char[i],"%d ", A[i]);
		LCD_Char_Position(1, 4*i);
		LCD_Char_PrintString(A_char[i]);
	}
}

//控制电机，ratio越小则此侧电机越慢
void PWM_go(float left_ratio, float right_ratio)
{
	PWMLeft_WriteCompare (PWM_base - speed*left_ratio );
	PWMRight_WriteCompare(PWM_base + speed*right_ratio);
}

//得到超声波传感器数值
int get_sound()
{
	Timer_1_WriteCounter(10000);   
    CyDelay(40);
    trigger_Write(1);    
    Timer_Enable();
    Timer_Start();
    
	while(1)
    if(receiver_Read()==1) 
    {
        Timer_1_Enable();
	    Timer_1_Start();
        break;
    }
    
	while(1)
    if(receiver_Read()==0)
    {
        Timer_1_Stop();
	    break;
    }
	
	return Timer_1_ReadCounter();
}

//Timer结束则停止，Timer_1开始计数
CY_ISR(InterruptHandler)
{
    Timer_ReadStatusRegister();
    trigger_Write(0);
    Timer_Stop();
}

//初始化中断，电机，LCD，ADC红外传感器
void Initialize_Components()
{
    CyGlobalIntEnable;   
    isr_Start();
	isr_Disable();
	isr_SetVector(InterruptHandler);
	isr_Enable();
    
    PWMLeft_Start();
    PWMRight_Start();
    
	LCD_Char_Start();
	LCD_Char_LoadCustomFonts(LCD_Char_customFonts);
    LCD_Char_ClearDisplay();
	LCD_Char_Position(0,0);
    LCD_Char_PrintString("For Red Sou Sta");
	
	ADC_SAR_Seq_Start();
    ADC_SAR_Seq_StartConvert();
}

//主函数
int main()
{
    Initialize_Components(); 
    while(1) control();
    return 0;
}