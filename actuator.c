

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*---------------------------------------------------------------------
 * *** Hardware handles ***
 *--------------------------------------------------------------------*/
/*  STM32CubeMX still creates the peripheral handles; we just declare
 *  them extern so the linker can find them in stm32f1xx_hal_msp.c  */
extern TIM_HandleTypeDef htim2;   /* 1 kHz base timer                */
extern TIM_HandleTypeDef htim3;   /* PWM output → BTS7960 / L298N    */
extern TIM_HandleTypeDef htim4;   /* Quadrature‑encoder interface    */
extern UART_HandleTypeDef huart1; /* USB‑UART for CLI                */

/*---------------------------------------------------------------------
 * *** Compile‑time constants ***
 *--------------------------------------------------------------------*/
#define ENC_TICKS_PER_MM   100      /* Adjust to your encoder / gear  */
#define CTRL_LOOP_HZ       1000     /* 1 kHz PID scheduler            */
#define PWM_MAX            999      /* TIM3 ARR value (1 kHz PWM)     */

/*---------------------------------------------------------------------
 * Forward‑declarations for clarity
 *--------------------------------------------------------------------*/
static void ctrl_init(void);
static void ctrl_periodic(void);
static void ctrl_setpoint_mm(float mm);
static float ctrl_error_mm(void);
static void motion_init(void);
static void motion_set_path(float s, float e, uint8_t stops);
static void motion_task(void);
static bool  motion_reached(float mm);
static void ui_init(void);
static void ui_task(void);

/*---------------------------------------------------------------------
 * *********************  GLOBALS & PID STATE  ***********************
 *--------------------------------------------------------------------*/
static float g_kp = 2.0f, g_ki = 0.01f, g_kd = 0.1f;
static float g_integral = 0.0f, g_prev_err = 0.0f;
static int32_t g_setpoint_ticks = 0;

/* Motion‑planner state */
static float g_start_mm = 0, g_end_mm = 200;
static uint8_t g_nStops = 0, g_currentStop = 0;
static enum { M_IDLE, M_MOVING } g_state = M_IDLE;
static uint32_t g_settleTimer = 0;

/* UART ring buffer (tiny) */
#define RX_BUF_SZ  64
static uint8_t rx_byte;
static char    rx_line[RX_BUF_SZ];
static uint8_t rx_idx = 0;

/*---------------------------------------------------------------------
 * *********************  HELPER INLINE FUNCs  ************************
 *--------------------------------------------------------------------*/
static inline int32_t encoder_ticks(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/*---------------------------------------------------------------------
 * *********************  LOW‑LEVEL MOTOR DRIVE  **********************
 *  DIR pin is any GPIO output you configured in CubeMX (e.g. PB12)
 *--------------------------------------------------------------------*/
#define DIR_GPIO_Port   GPIOB
#define DIR_Pin         GPIO_PIN_12

static void motor_drive(float pwm)
{
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm >= 0) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);   /* forward */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)pwm);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET); /* reverse */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-pwm));
    }
}

/*---------------------------------------------------------------------
 * ************************  CONTROL LAYER  ***************************
 *--------------------------------------------------------------------*/
static void ctrl_init(void)
{
    g_integral = 0.0f;
    g_prev_err = 0.0f;
    ctrl_setpoint_mm(0);     /* default home */
}

/* Convert millimetres → encoder counts */
static inline int32_t mm2ticks(float mm)
{
    return (int32_t)(mm * ENC_TICKS_PER_MM);
}

static inline float ticks2mm(int32_t ticks)
{
    return (float)ticks / ENC_TICKS_PER_MM;
}

static void ctrl_setpoint_mm(float mm)
{
    g_setpoint_ticks = mm2ticks(mm);
}

static float ctrl_error_mm(void)
{
    return ticks2mm(g_setpoint_ticks - encoder_ticks());
}

/* Called from 1 kHz IRQ (TIM2) */
static void ctrl_periodic(void)
{
    int32_t pos  = encoder_ticks();
    int32_t err  = g_setpoint_ticks - pos;

    /* PI‑D */
    g_integral += err * (1.0f / CTRL_LOOP_HZ);
    float deriv = (err - g_prev_err) * CTRL_LOOP_HZ;
    g_prev_err  = err;

    float out = g_kp*err + g_ki*g_integral + g_kd*deriv;
    motor_drive(out);
}

/*---------------------------------------------------------------------
 * ************************  MOTION LAYER  ****************************
 *--------------------------------------------------------------------*/
static void motion_init(void)
{
    g_state = M_IDLE;
}

static void motion_set_path(float s, float e, uint8_t stops)
{
    g_start_mm   = s;
    g_end_mm     = e;
    g_nStops     = stops;
    g_currentStop= 0;
    g_state      = M_MOVING;
    g_settleTimer= HAL_GetTick();  /* start immediately */
}

/* ±1 mm tolerance */
static bool motion_reached(float mm)
{
    return fabsf(ctrl_error_mm()) < 1.0f;
}

static void motion_task(void)
{
    if (g_state == M_IDLE) return;

    /* Small dwell at each landing */
    if (HAL_GetTick() < g_settleTimer) return;

    float segment = (g_end_mm - g_start_mm) / (g_nStops + 1);
    float target  = g_start_mm + segment * g_currentStop;

    if (motion_reached(target)) {
        ++g_currentStop;
        if (g_currentStop > g_nStops + 1) {
            g_state = M_IDLE;
            printf("DONE\r\n");
            return;
        }
        g_settleTimer = HAL_GetTick() + 500; /* 0.5 s dwell */
    }
    ctrl_setpoint_mm(target);
}

/*---------------------------------------------------------------------
 * **************************  UI LAYER  ******************************
 *  Very small CLI over UART1 (115200‑8‑N‑1)
 *
 *  Commands:
 *    help                   – list
 *    set  <s> <e> <stops>   – define path mm mm N
 *    go                     – start motion
 *    stop                   – emergency halt
 *    kp|ki|kd <val>         – tune gains
 *--------------------------------------------------------------------*/
static void cli_help(void)
{
    printf("Commands:\r\n"
           "  set  <start> <end> <stops>\r\n"
           "  go\r\n"
           "  stop\r\n"
           "  kp <val>   | ki <val> | kd <val>\r\n"
           "  help\r\n");
}

static void ui_init(void)
{
    /* Start UART Rx interrupt for single byte */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    cli_help();
}

static void cli_parse_line(char *line)
{
    char *tok = strtok(line, " \r\n");
    if (!tok) return;

    /* ----------- set ----------- */
    if (!strcasecmp(tok, "set")) {
        float s = atof(strtok(NULL, " "));
        float e = atof(strtok(NULL, " "));
        int  n  = atoi (strtok(NULL, " "));
        motion_set_path(s, e, (uint8_t)n);
        printf("> Path set: %.1f → %.1f mm with %d stops\r\n", s, e, n);
    }
    /* ----------- go ------------ */
    else if (!strcasecmp(tok, "go")) {
        g_state = M_MOVING;
        g_currentStop = 0;
        g_settleTimer = HAL_GetTick();
        printf("> GO\r\n");
    }
    /* ----------- stop ---------- */
    else if (!strcasecmp(tok, "stop")) {
        g_state = M_IDLE;
        motor_drive(0);
        printf("> STOPPED\r\n");
    }
    /* --------- PID tune -------- */
    else if (!strcasecmp(tok, "kp")) { g_kp = atof(strtok(NULL," ")); }
    else if (!strcasecmp(tok, "ki")) { g_ki = atof(strtok(NULL," ")); }
    else if (!strcasecmp(tok, "kd")) { g_kd = atof(strtok(NULL," ")); }
    /* --------- help ------------ */
    else if (!strcasecmp(tok, "help")) { cli_help(); }
    /* -------- unknown ---------- */
    else {
        printf("?\r\n");
    }
}

static void ui_task(void)
{
    /* Nothing—processing is IRQ‑driven */
}

/* UART Rx Complete ISR */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (rx_byte == '\r' || rx_byte == '\n') {  /* end of line? */
        rx_line[rx_idx] = '\0';
        cli_parse_line(rx_line);
        rx_idx = 0;
    } else if (rx_idx < RX_BUF_SZ-1) {
        rx_line[rx_idx++] = rx_byte;
    }

    /* Re‑arm reception */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/*---------------------------------------------------------------------
 * ***************************  MAIN  *********************************
 *--------------------------------------------------------------------*/
static void SystemClock_Config(void);  /* CubeMX will generate these */
static void Error_Handler(void);

int main(void)
{
    /* HAL & System init (CubeMX) */
    HAL_Init();
    SystemClock_Config();

    /* MX_* functions here (GPIO, TIM2, TIM3, TIM4, USART1) */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* Start peripherals */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_ENCODERMODE_TI12);
    HAL_TIM_Base_Start_IT(&htim2);            /* 1 kHz timer */
    ui_init();
    ctrl_init();
    motion_init();

    printf("\r\n=== Linear‑Actuator Controller (single‑file) ===\r\n");

    /* Super‑loop – only high‑level non‑realtime jobs */
    while (1)
    {
        motion_task();
        /* Low‑power wait until next interrupt */
        __WFI();
    }
}

/*---------------------------------------------------------------------
 * 1 kHz scheduler interrupt (TIM2) – PID controller
 *--------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        ctrl_periodic();
    }
}

