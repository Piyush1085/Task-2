
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* === tunable constants ===================================================*/
#define CONTROL_PERIOD_MS   10          // 100 Hz control loop
#define PWM_MAX             1000        // TIM1->ARR. 1000 = 100 % duty
#define KP                  0.8f        // proportional gain
#define KI                  0.02f       // integral gain
#define I_MAX               500.0f      // integral clamp
#define DEADBAND_TICKS      5           // |error| below ⇒ position considered reached
#define HOME_SPEED          350         // duty while homing toward limit switch

/* === globals =============================================================*/
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;   // PWM
TIM_HandleTypeDef htim2;   // Encoder

volatile int32_t encoder_raw = 0;       // updated in TIM2 IRQ
volatile uint32_t ms = 0;               // SysTick time‑base

/* runtime params ----------------------------------------------------------*/
typedef struct {
    int32_t start;      // encoder counts
    int32_t end;
    uint8_t stops;
} traj_t;

volatile traj_t traj = {0, 8000, 0};    // default trajectory
static int32_t setpoints[12];           // max 10 stops + endpoints
static uint8_t current_sp_idx = 0;      // index in setpoints[]
static float  i_term = 0;               // integral accumulator

/* === helper macros =======================================================*/
#define DIR_FWD()   do {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  \
                       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);} while(0)
#define DIR_REV()   do {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);\
                       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);} while(0)
#define PWM_WRITE(val)   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (val))

/* === prototypes ==========================================================*/
static void buildTrajectory(void);
static void controlLoop(void);
static void setMotor(int16_t effort);      // effort ‑1000…+1000
static void homeToStart(void);

/* === interrupt / callback ===============================================*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        /* 16‑bit timer → promote to signed 32 bit, maintaining wrap‑around */
        static int16_t last = 0;
        int16_t now = __HAL_TIM_GET_COUNTER(htim);
        int16_t diff = now - last;
        encoder_raw += diff;   // diff already signed
        last = now;
    }
}
void SysTick_Handler(void)
{
    HAL_IncTick();
    ms++;
}

/* UART Rx interrupt: accumulate ASCII line ------------------------------*/
#define RX_BUF_SZ   64
static char rxbuf[RX_BUF_SZ];
static uint8_t rx_idx = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;
    ch = rxbuf[rx_idx];   // last DMA byte
    if(ch == '\r' || ch == '\n')
    {
        rxbuf[rx_idx] = '\0';
        /* parse command --------------------------------------------------*/
        /* format: s=<int> e=<int> n=<int> */
        traj_t tmp = traj;       // copy current
        char *tok = strtok(rxbuf, " ");
        while(tok)
        {
            if(sscanf(tok, "s=%ld", &tmp.start)==1) ;
            else if(sscanf(tok, "e=%ld", &tmp.end)==1) ;
            else if(sscanf(tok, "n=%hhu", &tmp.stops)==1) ;
            tok = strtok(NULL, " ");
        }
        traj = tmp;
        buildTrajectory();
        current_sp_idx = 0;
        i_term = 0;
    }
    else
    {
        if(++rx_idx >= RX_BUF_SZ) rx_idx = 0;  // safety wrap
    }
    /* restart DMA reception for 1 byte */
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxbuf[rx_idx], 1);
}

/* === main ===============================================================*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();   // Cube‑generated

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    /* start peripherals ---------------------------------------------------*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxbuf[0], 1);

    buildTrajectory();

    /* home to physical start limit (optional) ----------------------------*/
    homeToStart();

    uint32_t t_next = ms + CONTROL_PERIOD_MS;
    while (1)
    {
        if((int32_t)(ms - t_next) >= 0)
        {
            t_next += CONTROL_PERIOD_MS;
            controlLoop();
        }
    }
}

/* === functions ==========================================================*/
static void buildTrajectory(void)
{
    /* compute setpoint array: start, stops…, end */
    uint8_t n = 0;
    setpoints[n++] = traj.start;
    for(uint8_t i=1;i<=traj.stops;i++)
    {
        int32_t p = traj.start + (int32_t)((float)(traj.end - traj.start)
                          * (float)i / (float)(traj.stops + 1));
        setpoints[n++] = p;
    }
    setpoints[n++] = traj.end;
    /* fill rest with last value for safety */
    for(; n<12; n++) setpoints[n] = traj.end;
}

static void homeToStart(void)
{
#ifdef USE_LIMIT_SWITCH
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
    {
        setMotor(-HOME_SPEED);
    }
    setMotor(0);
    encoder_raw = traj.start;   // zero reference
#endif
}

static void controlLoop(void)
{
    int32_t pos = encoder_raw;
    int32_t target = setpoints[current_sp_idx];

    int32_t err = target - pos;

    /* PI controller ------------------------------------------------------*/
    float p = KP * (float)err;
    i_term += KI * (float)err;
    if(i_term > I_MAX) i_term = I_MAX;
    if(i_term < -I_MAX) i_term = -I_MAX;

    float u = p + i_term;

    if(u > PWM_MAX) u = PWM_MAX;
    if(u < -PWM_MAX) u = -PWM_MAX;

    setMotor((int16_t)u);

    /* transition to next setpoint when within deadband -------------------*/
    if(abs(err) < DEADBAND_TICKS)
    {
        if(current_sp_idx < traj.stops + 1)   // index 0…N+1
        {
            current_sp_idx++;
        }
        else
        {
            setMotor(0);   // trajectory finished
        }
    }
}

static void setMotor(int16_t effort)
{
    if(effort > 0)
    {
        DIR_FWD();
        PWM_WRITE((uint16_t)effort);
    }
    else if(effort < 0)
    {
        DIR_REV();
        PWM_WRITE((uint16_t)(-effort));
    }
    else
    {
        PWM_WRITE(0);     // brake
    }
}
