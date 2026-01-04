/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <uxr/client/transport.h>
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>
#include "motor.h"
#include "encoder.h"
#include "app_config.h"
#include "current_sense.h"
#include "telemetry.h"
#include "ramp.h"
#include "pid.h"
#include "control_state.h"
#include "sensing.h"
#include "fault_monitor.h"
#include "control_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for control_task */
osThreadId_t control_taskHandle;
uint32_t control_taskBuffer[ 1500 ];
osStaticThreadDef_t control_taskControlBlock;
const osThreadAttr_t control_task_attributes = {
  .name = "control_task",
  .cb_mem = &control_taskControlBlock,
  .cb_size = sizeof(control_taskControlBlock),
  .stack_mem = &control_taskBuffer[0],
  .stack_size = sizeof(control_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ros_exec_task */
/* Definitions for ros_pub_task */
osThreadId_t ros_pub_taskHandle;
uint32_t ros_pub_taskBuffer[ 2000 ];
osStaticThreadDef_t ros_pub_taskControlBlock;
const osThreadAttr_t ros_pub_task_attributes = {
  .name = "ros_pub_task",
  .cb_mem = &ros_pub_taskControlBlock,
  .cb_size = sizeof(ros_pub_taskControlBlock),
  .stack_mem = &ros_pub_taskBuffer[0],
  .stack_size = sizeof(ros_pub_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
#if SERIAL_TELEMETRY_ENABLE
/* Definitions for telemetry_task */
osThreadId_t telemetry_taskHandle;
uint32_t telemetry_taskBuffer[ 1200 ];
osStaticThreadDef_t telemetry_taskControlBlock;
const osThreadAttr_t telemetry_task_attributes = {
  .name = "telemetry_task",
  .cb_mem = &telemetry_taskControlBlock,
  .cb_size = sizeof(telemetry_taskControlBlock),
  .stack_mem = &telemetry_taskBuffer[0],
  .stack_size = sizeof(telemetry_taskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
#endif
/* USER CODE BEGIN PV */
static uint32_t last_header_tick = 0;
static volatile uint8_t control_tick_flag = 0;
static volatile uint32_t control_tick_count = 0;
static volatile float control_tick_hz = 0.0f;
static volatile uint32_t control_tick_missed = 0;
static volatile uint32_t control_loop_count = 0;
static volatile uint32_t control_tick_lag = 0;
static volatile uint32_t control_tick_lag_max = 0;
static volatile uint32_t control_loop_last_ms = 0;
static volatile uint32_t control_loop_max_ms = 0;
static volatile uint32_t cmd_age_ms = 0;
static volatile float dbg_cmd_v_mps = 0.0f;
static volatile float dbg_cmd_w_rps = 0.0f;
static volatile float dbg_v_cmd_mps = 0.0f;
static volatile float dbg_w_cmd_rps = 0.0f;
static volatile float dbg_v_l_mps = 0.0f;
static volatile float dbg_v_r_mps = 0.0f;
static volatile float dbg_rpm_target_l = 0.0f;
static volatile float dbg_rpm_target_r = 0.0f;
static volatile float dbg_rpm_meas_l = 0.0f;
static volatile float dbg_rpm_meas_r = 0.0f;
static volatile float dbg_pid_p_l = 0.0f;
static volatile float dbg_pid_i_l = 0.0f;
static volatile float dbg_pid_d_l = 0.0f;
static volatile float dbg_pid_err_l = 0.0f;
static volatile float dbg_pid_out_l = 0.0f;
static volatile float dbg_pid_p_r = 0.0f;
static volatile float dbg_pid_i_r = 0.0f;
static volatile float dbg_pid_d_r = 0.0f;
static volatile float dbg_pid_err_r = 0.0f;
static volatile float dbg_pid_out_r = 0.0f;
static volatile float dbg_duty_cmd_l = 0.0f;
static volatile float dbg_duty_cmd_r = 0.0f;
static volatile uint8_t dbg_dir_l = 0U;
static volatile uint8_t dbg_dir_r = 0U;
//static uint32_t telemetry_decim_counter = 0;
static EncoderChannel enc_left;
static EncoderChannel enc_right;
static CurrentSense current_sense;
static ControlStateMgr ctrl_state;
static Sensing sensing;
static SensingData sense;
static FaultMonitor fault_mon;
static ControlLoop control_loop;
static MotorChannel m_left;
static MotorChannel m_right;
static rcl_allocator_t ros_allocator;
static rclc_support_t ros_support;
static rcl_node_t ros_node;
static rclc_executor_t ros_executor;
static rcl_publisher_t pub_rpm_left;
static rcl_publisher_t pub_rpm_right;
static rcl_publisher_t pub_fault_mask;
static rcl_publisher_t pub_duty_left;
static rcl_publisher_t pub_duty_right;
static rcl_publisher_t pub_wheel_state;
static rcl_publisher_t pub_safety_state;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_enable;
static rcl_subscription_t sub_estop;
static rcl_subscription_t sub_clear_fault;
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Float32 msg_rpm_left;
static std_msgs__msg__Float32 msg_rpm_right;
static std_msgs__msg__Int32 msg_fault_mask;
static std_msgs__msg__Float32 msg_duty_left;
static std_msgs__msg__Float32 msg_duty_right;
static sensor_msgs__msg__JointState msg_wheel_state;
static std_msgs__msg__UInt32 msg_safety_state;
static std_msgs__msg__Bool msg_enable;
static std_msgs__msg__Bool msg_estop;
static std_msgs__msg__Empty msg_clear_fault;
static volatile uint32_t cmd_rx_count = 0;
static volatile bool ros_ready = false;
static volatile int ros_init_fail_stage = 0;
static volatile float cmd_v_mps = 0.0f;
static volatile float cmd_w_rps = 0.0f;
static volatile uint32_t last_cmd_ms = 0U;
static volatile float duty_cmd_l_pub = 0.0f;
static volatile float duty_cmd_r_pub = 0.0f;
static volatile bool enable_cmd = true;
static volatile bool soft_estop_cmd = false;
static volatile bool clear_fault_cmd = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartControlTask(void *argument);
void StartRosPubTask(void *argument);
#if SERIAL_TELEMETRY_ENABLE
void StartTelemetryTask(void *argument);
#endif
void CmdVelCallback(const void * msgin);
void EnableCallback(const void * msgin);
void EstopCallback(const void * msgin);
void ClearFaultCallback(const void * msgin);

/* USER CODE BEGIN PFP */

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Current sensing moved to current_sense.c */

// Store latest cmd_vel values (linear x, angular z) with timestamp
void CmdVelCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_v_mps = CMD_V_POLARITY * msg->linear.x;
  cmd_w_rps = CMD_W_POLARITY * msg->angular.z;
  last_cmd_ms = HAL_GetTick();
  cmd_rx_count++;
}

void EnableCallback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  enable_cmd = msg->data;
}

void EstopCallback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  soft_estop_cmd = msg->data;
}

void ClearFaultCallback(const void * msgin)
{
  (void)msgin;
  clear_fault_cmd = true;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Initialize and run both motors at 10% duty (Cytron MDD20A)
  // Left motor (M1): PA8 PWM (TIM1_CH1), PB4 DIR
  Motor_Init(&m_left, &htim1, TIM_CHANNEL_1, DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, __HAL_TIM_GET_AUTORELOAD(&htim1));
  Motor_SetDirection(&m_left, LEFT_DIR_POLARITY);   // forward (adjust if wiring requires inversion)
  Motor_SetDuty(&m_left, 0.0f);     // keep off initially

  // Right motor (M2): PA9 PWM (TIM1_CH2), PB5 DIR
  Motor_Init(&m_right, &htim1, TIM_CHANNEL_2, DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, __HAL_TIM_GET_AUTORELOAD(&htim1));
  Motor_SetDirection(&m_right, RIGHT_DIR_POLARITY);  // forward (adjust if wiring requires inversion)
  Motor_SetDuty(&m_right, 0.0f);    // keep off initially

  // Start PWM at 0% to bias driver, then calibrate zero
  Motor_Start(&m_left);
  Motor_Start(&m_right);
  HAL_Delay(50);  // allow rails/driver to settle at 0% duty
  CurrentSense_Init(&current_sense, &hadc1);
  CurrentSense_Calibrate(&current_sense);
  ControlState_Init(&ctrl_state);
  Sensing_Init(&sensing, &enc_left, &enc_right, &current_sense, RPM_LPF_ALPHA);
  FaultMonitor_Init(&fault_mon);
  ControlLoop_Init(&control_loop);

  // Initialize and start encoders (Left: TIM3 16-bit, Right: TIM2 32-bit)
  Encoder_Init(&enc_left,  &htim3, ENCODER_COUNTS_PER_REV, 0xFFFFU, LEFT_ENCODER_POLARITY);
  Encoder_Init(&enc_right, &htim2, ENCODER_COUNTS_PER_REV, 0xFFFFFFFFU, RIGHT_ENCODER_POLARITY);
  Encoder_Start(&enc_left);
  Encoder_Start(&enc_right);
  HAL_TIM_Base_Start_IT(&htim4);
  last_header_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of control_task */
  control_taskHandle = osThreadNew(StartControlTask, NULL, &control_task_attributes);

#if SERIAL_TELEMETRY_ENABLE
  /* creation of telemetry_task */
  telemetry_taskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetry_task_attributes);
#else
  /* creation of ros_pub_task */
  ros_pub_taskHandle = osThreadNew(StartRosPubTask, NULL, &ros_pub_task_attributes);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    osDelay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI2;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI2;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_LEFT_Pin|DIR_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_LEFT_Pin DIR_RIGHT_Pin */
  GPIO_InitStruct.Pin = DIR_LEFT_Pin|DIR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure GPIO pin : ESTOP_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    if (control_tick_flag) {
      control_tick_missed++;
    }
    control_tick_flag = 1;
    control_tick_count++;
  } else if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the control_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    // Wait for 100 Hz control tick (TIM4 interrupt sets flag)
    if (!control_tick_flag) {
      osDelay(1);
      continue;
    }
    control_tick_flag = 0;
    uint32_t loop_start_ms = HAL_GetTick();
    control_loop_count++;
    uint32_t lag = control_tick_count - control_loop_count;
    control_tick_lag = lag;
    if (lag > control_tick_lag_max) {
      control_tick_lag_max = lag;
    }
    uint32_t now = HAL_GetTick();
    static uint32_t last_tick_ms = 0U;
    static uint32_t last_tick_cnt = 0U;
    if ((now - last_tick_ms) >= 1000U) {
      uint32_t dc = control_tick_count - last_tick_cnt;
      uint32_t dt = now - last_tick_ms;
      if (dt > 0U) {
        control_tick_hz = (1000.0f * (float)dc) / (float)dt;
      }
      last_tick_ms = now;
      last_tick_cnt = control_tick_count;
    }
    float dt_s = CONTROL_LOOP_DT_S;

    // Debounce hardware e-stop input.
    bool estop_active = false;
    {
      static bool estop_init = false;
      static bool estop_raw_prev = false;
      static uint32_t estop_change_ms = 0U;
      static bool estop_debounced = false;

      GPIO_PinState estop_pin = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin);
      bool estop_raw = ESTOP_ACTIVE_LOW ? (estop_pin == GPIO_PIN_RESET) : (estop_pin == GPIO_PIN_SET);

      if (!estop_init) {
        estop_init = true;
        estop_raw_prev = estop_raw;
        estop_change_ms = now;
        estop_debounced = estop_raw;
      } else if (estop_raw != estop_raw_prev) {
        estop_raw_prev = estop_raw;
        estop_change_ms = now;
      } else if ((uint32_t)(now - estop_change_ms) >= ESTOP_DEBOUNCE_MS) {
        estop_debounced = estop_raw;
      }

      estop_active = estop_debounced;
    }
    estop_active = estop_active || soft_estop_cmd;

    // Sense encoders/currents
    sense.rpm_l = sense.rpm_r = 0.0f;
    Sensing_Step(&sensing, dt_s, &sense);


    // Control loop (PI + ramp) gated by state
    bool enabled = ControlState_IsEnabled(&ctrl_state) && !estop_active && enable_cmd;
    float rpm_target_l = 0.0f;
    float rpm_target_r = 0.0f;
    float duty_cmd_l = 0.0f, duty_cmd_r = 0.0f;
    dbg_cmd_v_mps = cmd_v_mps;
    dbg_cmd_w_rps = cmd_w_rps;
#if MOTOR_OPEN_LOOP_TEST
    if (enabled) {
      duty_cmd_l = MOTOR_OPEN_LOOP_DUTY_LEFT;
      duty_cmd_r = MOTOR_OPEN_LOOP_DUTY_RIGHT;
    }
    dbg_v_cmd_mps = 0.0f;
    dbg_w_cmd_rps = 0.0f;
    dbg_v_l_mps = 0.0f;
    dbg_v_r_mps = 0.0f;
    dbg_rpm_target_l = 0.0f;
    dbg_rpm_target_r = 0.0f;
#else
    float v_cmd = 0.0f;
    float w_cmd = 0.0f;
    bool cmd_stopped = false;
#if PID_TUNING_ENABLE
    static uint32_t last_toggle_ms = 0U;
    static uint8_t test_high = 0U;
    const float rpm_low = PID_TUNING_RPM_MAX * PID_TUNING_LOW_FRAC;
    const float rpm_high = PID_TUNING_RPM_MAX * PID_TUNING_HIGH_FRAC;
    if (!enabled) {
      test_high = 0U;
      last_toggle_ms = now;
    } else if (last_toggle_ms == 0U) {
      last_toggle_ms = now;
    } else if ((now - last_toggle_ms) >= PID_TUNING_TOGGLE_MS) {
      test_high = !test_high;
      last_toggle_ms = now;
    }
    float rpm_cmd = test_high ? rpm_high : rpm_low;
    const float two_pi = 6.28318530718f;
    v_cmd = (rpm_cmd * two_pi * WHEEL_RADIUS_M) / 60.0f;
    w_cmd = 0.0f;
    cmd_age_ms = 0U;
    cmd_stopped = !enabled;
#else
    v_cmd = cmd_v_mps;
    w_cmd = cmd_w_rps;
    uint32_t cmd_age = (last_cmd_ms > 0U) ? (now - last_cmd_ms) : (CMD_TIMEOUT_MS + 1U);
    cmd_age_ms = cmd_age;
    if (cmd_age > CMD_TIMEOUT_MS) {
      v_cmd = 0.0f;
      w_cmd = 0.0f;
    }
    cmd_stopped = (cmd_age > CMD_TIMEOUT_MS) ||
                  (fabsf(v_cmd) < CMD_STOP_EPS_MPS && fabsf(w_cmd) < CMD_STOP_EPS_RPS) ||
                  !enabled;
#endif
    static bool cmd_stopped_prev = false;
    if (cmd_stopped && !cmd_stopped_prev) {
      PID_Reset(&control_loop.pid_l, sense.rpm_l);
      PID_Reset(&control_loop.pid_r, sense.rpm_r);
    }
    cmd_stopped_prev = cmd_stopped;
    bool control_enabled = enabled && !cmd_stopped;
    ControlLoop_Update(&control_loop, sense.rpm_l, sense.rpm_r, control_enabled, v_cmd, w_cmd, dt_s, now, &duty_cmd_l, &duty_cmd_r, &rpm_target_l, &rpm_target_r);
    dbg_v_cmd_mps = v_cmd;
    dbg_w_cmd_rps = w_cmd;
    dbg_v_l_mps = control_loop.v_l_mps;
    dbg_v_r_mps = control_loop.v_r_mps;
    dbg_rpm_target_l = rpm_target_l;
    dbg_rpm_target_r = rpm_target_r;
#endif
    dbg_rpm_meas_l = sense.rpm_l;
    dbg_rpm_meas_r = sense.rpm_r;
    dbg_pid_p_l = control_loop.pid_l.last_p;
    dbg_pid_i_l = control_loop.pid_l.integrator;
    dbg_pid_d_l = control_loop.pid_l.last_d;
    dbg_pid_err_l = control_loop.pid_l.last_error;
    dbg_pid_out_l = control_loop.pid_l.last_out;
    dbg_pid_p_r = control_loop.pid_r.last_p;
    dbg_pid_i_r = control_loop.pid_r.integrator;
    dbg_pid_d_r = control_loop.pid_r.last_d;
    dbg_pid_err_r = control_loop.pid_r.last_error;
    dbg_pid_out_r = control_loop.pid_r.last_out;
    duty_cmd_l_pub = duty_cmd_l;
    duty_cmd_r_pub = duty_cmd_r;
    // Set direction pins based on commanded sign, then apply magnitude as duty
    uint8_t dir_l = (duty_cmd_l >= 0.0f) ? LEFT_DIR_POLARITY : (LEFT_DIR_POLARITY ? 0U : 1U);
    uint8_t dir_r = (duty_cmd_r >= 0.0f) ? RIGHT_DIR_POLARITY : (RIGHT_DIR_POLARITY ? 0U : 1U);
    dbg_dir_l = dir_l;
    dbg_dir_r = dir_r;
    Motor_SetDirection(&m_left, dir_l);
    Motor_SetDirection(&m_right, dir_r);
    Motor_SetDuty(&m_left, duty_cmd_l);
    Motor_SetDuty(&m_right, duty_cmd_r);
    dbg_duty_cmd_l = duty_cmd_l;
    dbg_duty_cmd_r = duty_cmd_r;

    // Compute commanded duty (percent) from timer compare
    float duty_l = 0.0f;
    float duty_r = 0.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    if (arr > 0U) {
      duty_l = (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) * 100.0f) / (float)(arr + 1U);
      duty_r = (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2) * 100.0f) / (float)(arr + 1U);
    }

    // Fault detection (overcurrent, stall, encoder timeout, ADC stuck)
    uint32_t fault_bits = FaultMonitor_Update(&fault_mon, &sense, rpm_target_l, rpm_target_r, duty_l, duty_r, CONTROL_LOOP_DT_MS);

    // Update control state (faults from detection above, debounced e-stop)
    bool clear_cmd = clear_fault_cmd;
    clear_fault_cmd = false;
    ControlInputs cin = {0};
    cin.enable_cmd = enable_cmd;
    cin.disable_cmd = !enable_cmd;
    cin.clear_cmd = clear_cmd;
    cin.estop = estop_active;
    cin.fault_bits = fault_bits;
    ControlState_Update(&ctrl_state, &cin, now);
    // enabled flag will be used on next tick

    // Legacy UART telemetry disabled to avoid contention with micro-ROS on USART2
    uint32_t loop_elapsed_ms = HAL_GetTick() - loop_start_ms;
    control_loop_last_ms = loop_elapsed_ms;
    if (loop_elapsed_ms > control_loop_max_ms) {
      control_loop_max_ms = loop_elapsed_ms;
    }
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartRosPubTask */
/**
* @brief Function implementing the ros_pub_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRosPubTask */
void StartRosPubTask(void *argument)
{
  /* USER CODE BEGIN StartRosPubTask */
  // micro-ROS init and publishers (single-threaded to avoid rcl concurrency)
  rmw_uros_set_custom_transport(
      true,
      (void *)&huart2,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  ros_allocator = rcl_get_default_allocator();
  ros_allocator.allocate = microros_allocate;
  ros_allocator.deallocate = microros_deallocate;
  ros_allocator.reallocate = microros_reallocate;
  ros_allocator.zero_allocate = microros_zero_allocate;
  (void)rcutils_set_default_allocator(&ros_allocator);

  if (rclc_support_init(&ros_support, 0, NULL, &ros_allocator) != RCL_RET_OK) {
    ros_init_fail_stage = 1;
    goto ros_init_fail;
  }

  if (rclc_node_init_default(&ros_node, "amr_firmware", "", &ros_support) != RCL_RET_OK) {
    ros_init_fail_stage = 2;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_rpm_left,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/amr/wheel_rpm_left") != RCL_RET_OK) {
    ros_init_fail_stage = 3;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_rpm_right,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/amr/wheel_rpm_right") != RCL_RET_OK) {
    ros_init_fail_stage = 4;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_fault_mask,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "/amr/fault_mask") != RCL_RET_OK) {
    ros_init_fail_stage = 5;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_duty_left,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/amr/duty_cmd_left") != RCL_RET_OK) {
    ros_init_fail_stage = 6;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_duty_right,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/amr/duty_cmd_right") != RCL_RET_OK) {
    ros_init_fail_stage = 7;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_wheel_state,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "/amr/wheel_state") != RCL_RET_OK) {
    ros_init_fail_stage = 8;
    goto ros_init_fail;
  }

  if (rclc_publisher_init_best_effort(
          &pub_safety_state,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
          "/amr/safety_state") != RCL_RET_OK) {
    ros_init_fail_stage = 9;
    goto ros_init_fail;
  }

  if (!sensor_msgs__msg__JointState__init(&msg_wheel_state)) {
    ros_init_fail_stage = 10;
    goto ros_init_fail;
  }

  if (!rosidl_runtime_c__String__Sequence__init(&msg_wheel_state.name, 2)) {
    ros_init_fail_stage = 11;
    goto ros_init_fail;
  }

  if (!rosidl_runtime_c__double__Sequence__init(&msg_wheel_state.position, 2)) {
    ros_init_fail_stage = 12;
    goto ros_init_fail;
  }

  if (!rosidl_runtime_c__double__Sequence__init(&msg_wheel_state.velocity, 2)) {
    ros_init_fail_stage = 13;
    goto ros_init_fail;
  }

  if (!rosidl_runtime_c__String__assign(&msg_wheel_state.name.data[0], "wheel_left") ||
      !rosidl_runtime_c__String__assign(&msg_wheel_state.name.data[1], "wheel_right")) {
    ros_init_fail_stage = 14;
    goto ros_init_fail;
  }

  // teleop_twist_keyboard publishes /cmd_vel as RELIABLE, so keep subscriber RELIABLE to match
  if (rclc_subscription_init_default(
          &sub_cmd_vel,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    ros_init_fail_stage = 15;
    goto ros_init_fail;
  }

  if (rclc_subscription_init_default(
          &sub_enable,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "/amr/enable") != RCL_RET_OK) {
    ros_init_fail_stage = 16;
    goto ros_init_fail;
  }

  if (rclc_subscription_init_default(
          &sub_estop,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "/amr/estop") != RCL_RET_OK) {
    ros_init_fail_stage = 17;
    goto ros_init_fail;
  }

  if (rclc_subscription_init_default(
          &sub_clear_fault,
          &ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
          "/amr/clear_fault") != RCL_RET_OK) {
    ros_init_fail_stage = 18;
    goto ros_init_fail;
  }

  if (rclc_executor_init(&ros_executor, &ros_support.context, 4, &ros_allocator) != RCL_RET_OK) {
    ros_init_fail_stage = 19;
    goto ros_init_fail;
  }

  if (rclc_executor_add_subscription(&ros_executor, &sub_cmd_vel, &msg_cmd_vel, CmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    ros_init_fail_stage = 20;
    goto ros_init_fail;
  }

  if (rclc_executor_add_subscription(&ros_executor, &sub_enable, &msg_enable, EnableCallback, ON_NEW_DATA) != RCL_RET_OK) {
    ros_init_fail_stage = 21;
    goto ros_init_fail;
  }

  if (rclc_executor_add_subscription(&ros_executor, &sub_estop, &msg_estop, EstopCallback, ON_NEW_DATA) != RCL_RET_OK) {
    ros_init_fail_stage = 22;
    goto ros_init_fail;
  }

  if (rclc_executor_add_subscription(&ros_executor, &sub_clear_fault, &msg_clear_fault, ClearFaultCallback, ON_NEW_DATA) != RCL_RET_OK) {
    ros_init_fail_stage = 23;
    goto ros_init_fail;
  }

  ros_ready = true;

  /* Infinite loop */
  bool led_state = false;
  uint32_t last_pub_ms = 0U;
  for(;;)
  {
    // Pump executor to receive cmd_vel
    rclc_executor_spin_some(&ros_executor, 1000000ULL); // 1 ms

    uint32_t now = HAL_GetTick();
    if ((last_pub_ms == 0U) || ((now - last_pub_ms) >= ROS_PUB_PERIOD_MS)) {
      last_pub_ms = now;
      // Publish wheel RPM, wheel state, and safety status
      uint32_t fault_mask = ControlState_GetFaultMask(&ctrl_state);
      // safety_state: upper 16 bits = ControlState, lower 16 bits = fault mask.
      uint32_t safety_state = ((uint32_t)ControlState_GetState(&ctrl_state) << 16) | (fault_mask & 0xFFFFu);
      const double two_pi = 6.28318530717958647692;
      const double rpm_to_rad_s = two_pi / 60.0;
      const double counts_to_rad = two_pi / (double)ENCODER_COUNTS_PER_REV;
      msg_rpm_left.data = sense.rpm_l;
      msg_rpm_right.data = sense.rpm_r;
      msg_fault_mask.data = (int32_t)fault_mask;
      msg_safety_state.data = safety_state;
      msg_duty_left.data = duty_cmd_l_pub * 100.0f;
      msg_duty_right.data = duty_cmd_r_pub * 100.0f;
      msg_wheel_state.header.stamp.sec = (int32_t)(now / 1000U);
      msg_wheel_state.header.stamp.nanosec = (uint32_t)((now % 1000U) * 1000000U);
      msg_wheel_state.position.data[0] = (double)sense.cnt_l * counts_to_rad;
      msg_wheel_state.position.data[1] = (double)sense.cnt_r * counts_to_rad;
      msg_wheel_state.velocity.data[0] = (double)sense.rpm_l * rpm_to_rad_s;
      msg_wheel_state.velocity.data[1] = (double)sense.rpm_r * rpm_to_rad_s;

      rcl_ret_t rc1 = rcl_publish(&pub_rpm_left, &msg_rpm_left, NULL);
      rcl_ret_t rc2 = rcl_publish(&pub_rpm_right, &msg_rpm_right, NULL);
      rcl_ret_t rc3 = rcl_publish(&pub_fault_mask, &msg_fault_mask, NULL);
      rcl_ret_t rc4 = rcl_publish(&pub_duty_left, &msg_duty_left, NULL);
      rcl_ret_t rc5 = rcl_publish(&pub_duty_right, &msg_duty_right, NULL);
      rcl_ret_t rc6 = rcl_publish(&pub_wheel_state, &msg_wheel_state, NULL);
      rcl_ret_t rc7 = rcl_publish(&pub_safety_state, &msg_safety_state, NULL);

      // Blink LED when publish succeeds; hold solid ON if any publish fails
      bool pub_ok = (rc1 == RCL_RET_OK) && (rc2 == RCL_RET_OK) && (rc3 == RCL_RET_OK) &&
                    (rc4 == RCL_RET_OK) && (rc5 == RCL_RET_OK) && (rc6 == RCL_RET_OK) &&
                    (rc7 == RCL_RET_OK);
      if (pub_ok) {
        led_state = !led_state;
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      }
    }

    osDelay(ROS_EXEC_DELAY_MS);
  }

ros_init_fail:
  // Blink stage count to indicate where init failed (1-23)
  while (1) {
    for (int i = 0; i < ros_init_fail_stage; i++) {
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      osDelay(150);
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
      osDelay(150);
    }
    osDelay(500);
  }
  /* USER CODE END StartRosPubTask */
}

#if SERIAL_TELEMETRY_ENABLE
/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetry_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  Telemetry_SendHeader(&huart2);
  uint32_t last_header_ms = HAL_GetTick();
  for(;;)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_header_ms) >= SERIAL_TELEMETRY_HEADER_PERIOD_MS) {
      Telemetry_SendHeader(&huart2);
      last_header_ms = now;
    }

    TelemetryFrame f = {0};
    f.t_ms = now;
    f.cnt_l = sense.cnt_l;
    f.cnt_r = sense.cnt_r;
    f.rpm_l_x10 = (int32_t)(sense.rpm_l * 10.0f);
    f.rpm_r_x10 = (int32_t)(sense.rpm_r * 10.0f);
    f.rpm_l_tgt_x10 = (int32_t)(dbg_rpm_target_l * 10.0f);
    f.rpm_r_tgt_x10 = (int32_t)(dbg_rpm_target_r * 10.0f);
    f.duty_l_pct = (int32_t)(duty_cmd_l_pub * 100.0f);
    f.duty_r_pct = (int32_t)(duty_cmd_r_pub * 100.0f);
    f.pid_p_l_pct = (int32_t)(dbg_pid_p_l * 100.0f);
    f.pid_p_r_pct = (int32_t)(dbg_pid_p_r * 100.0f);
    f.pid_i_l_pct = (int32_t)(dbg_pid_i_l * 100.0f);
    f.pid_i_r_pct = (int32_t)(dbg_pid_i_r * 100.0f);
    f.pid_d_l_pct = (int32_t)(dbg_pid_d_l * 100.0f);
    f.pid_d_r_pct = (int32_t)(dbg_pid_d_r * 100.0f);
    f.pid_err_l_x10 = (int32_t)(dbg_pid_err_l * 10.0f);
    f.pid_err_r_x10 = (int32_t)(dbg_pid_err_r * 10.0f);
    f.adc_l_counts = sense.adc_l_counts;
    f.adc_r_counts = sense.adc_r_counts;
    f.zero_l_counts = sense.zero_l_counts;
    f.zero_r_counts = sense.zero_r_counts;
    f.curr_l_mA = sense.curr_l_mA;
    f.curr_r_mA = sense.curr_r_mA;
    f.state = (uint32_t)ControlState_GetState(&ctrl_state);
    f.fault_mask = ControlState_GetFaultMask(&ctrl_state);
    Telemetry_SendFrame(&huart2, &f);

    osDelay(SERIAL_TELEMETRY_PERIOD_MS);
  }
  /* USER CODE END StartTelemetryTask */
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
