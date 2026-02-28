#include "Task.h"
#include "TLE5012B.h"
#include "FocLib.h"
#include "UART.h"
#include "usbd_cdc_if.h"
#include "TFT_Driver.h"
#include <string.h>
#include "FocCore.h"
#include "FocInterface.h"
#include "PID.h"
#include "Filter.h"
#include "M24C64.h"
#include "SMO.h"
#include "PLL.h"
#include "NLFO.h"

#define ENCODER_DIR 1

/*Handle*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/*结构体变量定义*/
Motor_Def Motor;
UART_DataFrame frame; // usb数据结构体
SMO_STRUCT SMO2;

NLFO_Observer MotorNLFO;

// 声明状态变量
PLL_Struct pll;
/*变量定义*/
float Target = 0.0f;
uint32_t R_IN = 0x0000;
float Speed_MovingFilterBuff[22];

void Sysinit()
{
  /*TFT显示*/
  ST7735_Init();
  ST7735_Clear(BLACK);
  ST7735_ShowString(45, 35, "START INIT", Font_7x10, RED, BLACK);
  HAL_Delay(10);

  SMO2.Rs = 0.17f;
  SMO2.Ld = 0.00006f;
  SMO2.Ts = 0.00005f;
  SMO2.Gain = 3.0f;
  SMO2.EalphaForeLPFFactor = 0.2f;
	SMO2.EbetaForeLPFFactor = 0.2f;

  MotorNLFO.flux_alpha = 0.0f;
  MotorNLFO.flux_beta = 0.0f;
  MotorNLFO.Gain = 1000;
  MotorNLFO.omega_hat = 0.0f;
  MotorNLFO.theta_hat = 0.0f;
  MotorNLFO.y[0] = 0.0f;
  MotorNLFO.y[1] = 0.0f;
  MotorNLFO.y_dot[0] = 0.0f;
  MotorNLFO.y_dot[1] = 0.0f;
  MotorNLFO.pll.Kp = 1;
  MotorNLFO.pll.Ki = 0;

  /*电机参数初始化*/
  Motor.pole = 7;
  Motor.Rs = 0.170f;
  Motor.flux = 0.0023f;
  Motor.Lq = 0.00004f;
  Motor.Ld = 0.00004f;
  Motor.SupplyVolt = 12.0f;
  Motor.TimeCount = 4200;
  Motor.state = MOTORINIT;
  Motor.MaxSpeed = 400;
  Motor.MaxCurrent = 9.0f;
  Motor.MFoc.speed_loop_divider = 10;
  // Motor.MFoc.pos_loop_divider = 20;

  /*PID参数初始化*/
  // Motor1 (1.0f, 0.003f, 0.0f)
  Motor.MFoc.TorquePID.Kp = -1.0f;
  Motor.MFoc.TorquePID.Ki = -0.003f;
  Motor.MFoc.TorquePID.Kd = 0.0f;
  Motor.MFoc.TorquePID.acu_limit = 1000.0f;
  Motor.MFoc.TorquePID.outputLimit = 5.8f;
  PID_Clear(&Motor.MFoc.TorquePID);
	
	Motor.MFoc.IdPID.Kp = -1.0f;
  Motor.MFoc.IdPID.Ki = -0.003f;
  Motor.MFoc.IdPID.Kd = 0.0f;
  Motor.MFoc.IdPID.acu_limit = 1000.0f;
  Motor.MFoc.IdPID.outputLimit = 5.8f;
  PID_Clear(&Motor.MFoc.IdPID);

  Motor.MFoc.SpeedPID.Kp = 0.1f;
  Motor.MFoc.SpeedPID.Ki = 0.001f;
  Motor.MFoc.SpeedPID.Kd = 0.0f;
  Motor.MFoc.SpeedPID.acu_limit = 1000.0f;
  Motor.MFoc.SpeedPID.outputLimit = Motor.MaxCurrent;
  PID_Clear(&Motor.MFoc.SpeedPID);

  Motor.MFoc.PositionPID.Kp = 70.0f;
  Motor.MFoc.PositionPID.Ki = 0.0f;
  Motor.MFoc.PositionPID.Kd = 0.0f;
  Motor.MFoc.PositionPID.acu_limit = 0.0f;
  Motor.MFoc.PositionPID.outputLimit = Motor.MaxSpeed;
  Motor.MFoc.PositionPID.error_limit = 100.0f;
  Motor.MFoc.PositionPID.acu_div = 2.0f;
  PID_Clear(&Motor.MFoc.PositionPID);
  /*滤波器参数初始化*/
  Motor.Speed_LPFilter.alpha = 0.2f;
  Motor.Iabc_LPFilter[0].alpha = 0.8f;
  Motor.Iabc_LPFilter[1].alpha = 0.8f;
  Motor.Iabc_LPFilter[2].alpha = 0.8f;
  MovingAverageFilter_Init(&Motor.Speed_MovingFilter, Speed_MovingFilterBuff, 20, 20);

  Motor.MFoc.Iq_LPFilter.alpha = 0.8f;
  Motor.MFoc.Id_LPFilter.alpha = 0.8f;

  /*进行SMO初始化*/
  SMO_Init(&Motor.SMO, 0.17f, 0.00004f, 0.00004f, 7, 0.0023f, 20.0f  ,0.00005f);
  /*PLL参数初始化*/
  PLL_Params params = {
        .Ts = 0.00005f,    // 采样周期1ms（1000Hz）
        .Kp = 100.0f,      // 比例增益
        .Ki = 1.0f,     // 积分增益（1/s）
        .Kd = 0.0f,     // 微分增益（s）
        .K = 1.0f        // 积分模块增益
  };
  PLL_Init(&pll, &params);
  /*开启PWM输出*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_Base_Start(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4000);

  /*ADC校准*/
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // ADC校准
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // ADC校准
  HAL_Delay(100);
  /*开启注入中断采样*/
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_Delay(1000);

  HAL_TIM_Base_Start_IT(&htim4);
  // 初始化代码示例
  // SMO_Init(&observer, 0.170f, 0.00006, 0.0023f, 7, 12.0f, 20000, -1, 1.0f, 1.0f);

  /*TFT初始化*/
  ST7735_Clear(BLACK);
  ST7735_ShowString(0, 2, "RUN MODE:", Font_7x10, RED, BLACK);
  ST7735_ShowString(0, 12, "Angle:       rad", Font_7x10, RED, BLACK);
  ST7735_ShowString(0, 22, "Uqout:       v", Font_7x10, RED, BLACK);
  ST7735_ShowString(0, 32, "Target:", Font_7x10, RED, BLACK);
  ST7735_ShowString(0, 42, "Speed:", Font_7x10, RED, BLACK);
  ST7735_ShowString(0, 52, "IqCur:", Font_7x10, RED, BLACK);
  HAL_Delay(10);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc2)
  {
    // Adc2Value[0] = ADC2->DR;
    // Adc2Value[1] = ADC2->DR;
    // Adc2Value[2] = ADC2->DR;
    // Adc2Value[3] = ADC2->DR;
    // Adc2Value[4] = ADC2->DR;
    // Adc2Value[5] = ADC2->DR;
    // Adc2Value[6] = ADC2->DR;
    // Adc2Value[7] = ADC2->DR;

    HAL_ADC_Start_IT(&hadc2);
  }
  UNUSED(hadc);
}
void SysRun()
{
  /*TFT屏显示*/
  ST7735_ClearRect(63, 2, 160 - 1, 12, BLACK);
  if (Motor.state == MOTORINIT)
    ST7735_ShowString(63, 2, "MOTOR INIT", Font_7x10, BLUE, BLACK);
  else if (Motor.MFoc.state == OPENCTR)
    ST7735_ShowString(63, 2, "OPEN MODE", Font_7x10, BLUE, BLACK);
  else if (Motor.MFoc.state == POSITIONCTR)
    ST7735_ShowString(63, 2, "POSITION MODE", Font_7x10, BLUE, BLACK);
  else if (Motor.MFoc.state == SPEEDCTR)
    ST7735_ShowString(63, 2, "SPEED MODE", Font_7x10, BLUE, BLACK);
  else if (Motor.MFoc.state == TORQUECTR)
    ST7735_ShowString(63, 2, "TORQUE MODE", Font_7x10, BLUE, BLACK);
  else
    ST7735_ShowString(63, 2, "NULL", Font_7x10, BLUE, BLACK);
  /*角度显示(deg)*/
  ST7735_ClearRect(42, 12, 66, 32, BLACK);
  ST7735_ShowFloat(42, 12, Motor.AngleMech, 2, Font_7x10, BLUE, BLACK);
  /*输出Uq显示(v)*/
  ST7735_ClearRect(42, 22, 75, 32, BLACK);
  ST7735_ShowFloat(42, 22, Motor.MFoc.TorquePID.PID_output, 2, Font_7x10, BLUE, BLACK);
  /*目标显示*/
  ST7735_ClearRect(42, 32, 84, 52, BLACK);
  ST7735_ShowFloat(42, 32, Target, 2, Font_7x10, BLUE, BLACK);
  /*速度显示*/
  ST7735_ClearRect(42, 42, 84, 52, BLACK);
  ST7735_ShowFloat(42, 42, Motor.Speed, 2, Font_7x10, BLUE, BLACK);
  /*Q轴电流显示*/
  ST7735_ClearRect(42, 52, 84, 52, BLACK);
  ST7735_ShowFloat(42, 52, Motor.MFoc.Iq, 2, Font_7x10, BLUE, BLACK);

}
float h[2];
float theta = 0.0f;
float speed = 0.0f;
uint8_t switch_t  = 0;
float offset_theta = 0.0f;
volatile uint8_t observe_mode = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    R_IN = ADC1->JDR4;
    if (Motor.MFoc.state == POSITIONCTR)
      Target = (R_IN * 6.28f * 2.0f / 4096.0f);
    else if (Motor.MFoc.state == SPEEDCTR)
      Target = (R_IN * Motor.MaxSpeed / 4096.0f);
    else if (Motor.MFoc.state == TORQUECTR)
      Target = (R_IN * Motor.MaxCurrent / 4096.0f);
    else if (Motor.MFoc.state == OPENCTR)
      Target = (R_IN * Motor.SupplyVolt / 4096.0f);
    Motor_Run(&Motor, 0, Target);
    NLFO_Observer_Updata(&MotorNLFO, &Motor);
    NLFO_PLL_Updata(&MotorNLFO.pll, &MotorNLFO);
    // SMO_Updata(&Motor.SMO, Motor.MFoc.Ualpha, Motor.MFoc.Ubeta, Motor.we,
    //            Motor.MFoc.Ialpha, Motor.MFoc.Ibeta, Motor.MFoc.Iq, Motor.MFoc.Id);

    // PLL_Update(&pll, -Motor.SMO.e_alpha_hat, -Motor.SMO.e_beta_hat, &Motor.SMO.theta_hat, &speed);

    // USB串口数据
    frame.data[0] = MotorNLFO.flux_alpha;
    frame.data[1] = MotorNLFO.flux_beta;
    frame.data[2] = Motor.MFoc.Ialpha;
    frame.data[3] = Motor.MFoc.Ibeta;
    frame.data[4] = Motor.MFoc.Ualpha;
    frame.data[5] = Motor.MFoc.Ubeta;
    frame.data[6] = MotorNLFO.x[0];
    frame.data[7] = MotorNLFO.theta_hat;
    frame.data[8] = Motor.MFoc.Ealpha;
    frame.data[9] = Motor.MFoc.Ebeta;
    frame.data[10] = Motor.SMO.theta_hat;
    frame.data[11] = Motor.MFoc.AngleEle;


    frame.footer[0] = 0x00;
    frame.footer[1] = 0x00;
    frame.footer[2] = 0x80;
    frame.footer[3] = 0x7f;

    CDC_Transmit_FS((uint8_t *)(&frame), sizeof(UART_DataFrame));

    HAL_ADCEx_InjectedStart_IT(hadc);

    // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
  }
  UNUSED(hadc);
}

uint8_t KeyState[4] = {0};
uint8_t KeyOldV[4] = {0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim4) // 1ms涓柇
  {
    static uint16_t Time4Tick = 0;
    Time4Tick++;
    if (Time4Tick % 10 == 0)
    {
      uint8_t KeyBuff[4] = {0};
      KeyBuff[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);  // BUTTON1
      KeyBuff[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);  // BUTTON2
      KeyBuff[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);  // BUTTON3
      KeyBuff[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // BUTTON4
      KeyState[0] = KeyBuff[0] & (KeyOldV[0] ^ KeyBuff[0]);
      KeyState[1] = KeyBuff[1] & (KeyOldV[1] ^ KeyBuff[1]);
      KeyState[2] = KeyBuff[2] & (KeyOldV[2] ^ KeyBuff[2]);
      KeyState[3] = KeyBuff[3] & (KeyOldV[3] ^ KeyBuff[3]);
      KeyOldV[0] = KeyBuff[0];
      KeyOldV[1] = KeyBuff[1];
      KeyOldV[2] = KeyBuff[2];
      KeyOldV[3] = KeyBuff[3];

      if (KeyState[0] == 1)
      {
        if(Motor.MFoc.state == OPENCTR){Motor.AngleMech_acu = 0.0f;Motor.MFoc.PositionPID.error_acu = 0.0f; Motor.MFoc.state = POSITIONCTR;}
        else if (Motor.MFoc.state == POSITIONCTR)
          Motor.MFoc.state = SPEEDCTR;
        else if (Motor.MFoc.state == SPEEDCTR)
          Motor.MFoc.state = TORQUECTR;
        else if (Motor.MFoc.state == TORQUECTR)
          Motor.MFoc.state = OPENCTR;
      };
      
      if(KeyState[3] == 1)
      {
        // if(Motor.state == MOTORERROR)Motor.state = MOTORREADY, Motor.MFoc.state = OPENCTR;
        // else if(Motor.state != MOTORERROR)Motor.state = MOTORERROR;
      }
      if (KeyState[0] == 1)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
      if (KeyState[1] == 1)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
      if (KeyState[2] == 1)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
      if (KeyState[3] == 1)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    }
  }
  UNUSED(htim);
}