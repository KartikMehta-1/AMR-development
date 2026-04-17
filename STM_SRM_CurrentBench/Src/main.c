#include <stdint.h>
#include <stdio.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define BIT(n) (1U << (n))

/* Clock assumptions */
#define SYSCLK_HZ           16000000U
#define UART2_BAUD          460800U

/* Bench behavior */
#define SAMPLE_PERIOD_MS    10U
#define PRINT_PERIOD_MS     20U
#define ADC_AVG_SAMPLES     8U
#define CAL_SAMPLES         256U
#define TEST_DUTY_PCT       15U
#define PHASE_IDLE_MS       3000U
#define PHASE_LEFT_MS       5000U
#define PHASE_BETWEEN_MS    3000U
#define PHASE_RIGHT_MS      5000U
#define CURR_LPF_ALPHA      0.15f

/* Current conversion (ACS758 + divider from AMR hardware) */
#define ADC_VREF_VOLTS      3.3f
#define ADC_MAX_COUNTS      4095.0f
#define CURR_DIV_RATIO      0.667f
#define CURR_SENS_V_PER_A   0.040f
#define LEFT_CURR_POLARITY  1
#define RIGHT_CURR_POLARITY 1

/* RCC */
#define RCC_BASE                0x40023800U
#define RCC_AHB1ENR             REG32(RCC_BASE + 0x30U)
#define RCC_APB1ENR             REG32(RCC_BASE + 0x40U)
#define RCC_APB2ENR             REG32(RCC_BASE + 0x44U)
#define RCC_AHB1ENR_GPIOAEN     BIT(0)
#define RCC_AHB1ENR_GPIOBEN     BIT(1)
#define RCC_AHB1ENR_GPIOCEN     BIT(2)
#define RCC_APB1ENR_USART2EN    BIT(17)
#define RCC_APB2ENR_TIM1EN      BIT(0)
#define RCC_APB2ENR_ADC1EN      BIT(8)

/* GPIO */
#define GPIOA_BASE              0x40020000U
#define GPIOB_BASE              0x40020400U
#define GPIOC_BASE              0x40020800U
#define GPIOA_MODER             REG32(GPIOA_BASE + 0x00U)
#define GPIOA_OTYPER            REG32(GPIOA_BASE + 0x04U)
#define GPIOA_OSPEEDR           REG32(GPIOA_BASE + 0x08U)
#define GPIOA_PUPDR             REG32(GPIOA_BASE + 0x0CU)
#define GPIOA_AFRL              REG32(GPIOA_BASE + 0x20U)
#define GPIOA_AFRH              REG32(GPIOA_BASE + 0x24U)
#define GPIOB_MODER             REG32(GPIOB_BASE + 0x00U)
#define GPIOB_OTYPER            REG32(GPIOB_BASE + 0x04U)
#define GPIOB_OSPEEDR           REG32(GPIOB_BASE + 0x08U)
#define GPIOB_PUPDR             REG32(GPIOB_BASE + 0x0CU)
#define GPIOB_BSRR              REG32(GPIOB_BASE + 0x18U)
#define GPIOC_MODER             REG32(GPIOC_BASE + 0x00U)
#define GPIOC_PUPDR             REG32(GPIOC_BASE + 0x0CU)

/* TIM1 */
#define TIM1_BASE               0x40010000U
#define TIM1_CR1                REG32(TIM1_BASE + 0x00U)
#define TIM1_EGR                REG32(TIM1_BASE + 0x14U)
#define TIM1_CCMR1              REG32(TIM1_BASE + 0x18U)
#define TIM1_CCER               REG32(TIM1_BASE + 0x20U)
#define TIM1_PSC                REG32(TIM1_BASE + 0x28U)
#define TIM1_ARR                REG32(TIM1_BASE + 0x2CU)
#define TIM1_CCR1               REG32(TIM1_BASE + 0x34U)
#define TIM1_CCR2               REG32(TIM1_BASE + 0x38U)
#define TIM1_BDTR               REG32(TIM1_BASE + 0x44U)

/* ADC1 */
#define ADC1_BASE               0x40012000U
#define ADC1_SR                 REG32(ADC1_BASE + 0x00U)
#define ADC1_CR1                REG32(ADC1_BASE + 0x04U)
#define ADC1_CR2                REG32(ADC1_BASE + 0x08U)
#define ADC1_SMPR1              REG32(ADC1_BASE + 0x0CU)
#define ADC1_SMPR2              REG32(ADC1_BASE + 0x10U)
#define ADC1_SQR1               REG32(ADC1_BASE + 0x2CU)
#define ADC1_SQR3               REG32(ADC1_BASE + 0x34U)
#define ADC1_DR                 REG32(ADC1_BASE + 0x4CU)

/* USART2 */
#define USART2_BASE             0x40004400U
#define USART2_SR               REG32(USART2_BASE + 0x00U)
#define USART2_DR               REG32(USART2_BASE + 0x04U)
#define USART2_BRR              REG32(USART2_BASE + 0x08U)
#define USART2_CR1              REG32(USART2_BASE + 0x0CU)

/* SysTick */
#define SYST_CSR                REG32(0xE000E010U)
#define SYST_RVR                REG32(0xE000E014U)
#define SYST_CVR                REG32(0xE000E018U)

/* AMR wiring pins */
#define PWM_LEFT_PIN            8U   /* PA8 TIM1_CH1 */
#define PWM_RIGHT_PIN           9U   /* PA9 TIM1_CH2 */
#define DIR_LEFT_PIN            4U   /* PB4 */
#define DIR_RIGHT_PIN           5U   /* PB5 */
#define UART2_TX_PIN            2U   /* PA2 AF7 */
#define UART2_RX_PIN            3U   /* PA3 AF7 */
#define CURR_LEFT_ADC_CH        8U   /* PB0 ADC1_IN8 */
#define CURR_RIGHT_ADC_CH       11U  /* PC1 ADC1_IN11 */

/* 16 MHz / 20 kHz = 800 ticks */
#define PWM_PERIOD_TICKS        800U

typedef enum {
    PHASE_IDLE = 0,
    PHASE_LEFT_DRIVE,
    PHASE_COAST,
    PHASE_RIGHT_DRIVE
} BenchPhase;

static volatile uint32_t g_ms_ticks = 0U;

static void systick_init(void)
{
    SYST_RVR = (SYSCLK_HZ / 1000U) - 1U;
    SYST_CVR = 0U;
    SYST_CSR = BIT(2) | BIT(1) | BIT(0);
}

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
    }
}

static void clock_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;
}

static void gpio_init(void)
{
    /* PA8/PA9 -> TIM1 AF1 */
    GPIOA_MODER = (GPIOA_MODER & ~(3U << (PWM_LEFT_PIN * 2U) | 3U << (PWM_RIGHT_PIN * 2U))) |
                  (2U << (PWM_LEFT_PIN * 2U) | 2U << (PWM_RIGHT_PIN * 2U));
    GPIOA_AFRH = (GPIOA_AFRH & ~((0xFU << ((PWM_LEFT_PIN - 8U) * 4U)) | (0xFU << ((PWM_RIGHT_PIN - 8U) * 4U)))) |
                 (1U << ((PWM_LEFT_PIN - 8U) * 4U)) | (1U << ((PWM_RIGHT_PIN - 8U) * 4U));

    /* PA2/PA3 -> USART2 AF7 */
    GPIOA_MODER = (GPIOA_MODER & ~(3U << (UART2_TX_PIN * 2U) | 3U << (UART2_RX_PIN * 2U))) |
                  (2U << (UART2_TX_PIN * 2U) | 2U << (UART2_RX_PIN * 2U));
    GPIOA_AFRL = (GPIOA_AFRL & ~(0xFU << (UART2_TX_PIN * 4U) | 0xFU << (UART2_RX_PIN * 4U))) |
                 (7U << (UART2_TX_PIN * 4U)) | (7U << (UART2_RX_PIN * 4U));

    GPIOA_OTYPER &= ~(BIT(PWM_LEFT_PIN) | BIT(PWM_RIGHT_PIN) | BIT(UART2_TX_PIN) | BIT(UART2_RX_PIN));
    GPIOA_PUPDR &= ~(3U << (PWM_LEFT_PIN * 2U) | 3U << (PWM_RIGHT_PIN * 2U) |
                     3U << (UART2_TX_PIN * 2U) | 3U << (UART2_RX_PIN * 2U));
    GPIOA_PUPDR |= (1U << (UART2_TX_PIN * 2U)) | (1U << (UART2_RX_PIN * 2U));
    GPIOA_OSPEEDR |= (2U << (PWM_LEFT_PIN * 2U)) | (2U << (PWM_RIGHT_PIN * 2U)) |
                     (2U << (UART2_TX_PIN * 2U)) | (2U << (UART2_RX_PIN * 2U));

    /* PB4/PB5 -> direction outputs, PB0 -> analog current input */
    GPIOB_MODER = (GPIOB_MODER & ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U) | 3U << (0U * 2U))) |
                  (1U << (DIR_LEFT_PIN * 2U)) | (1U << (DIR_RIGHT_PIN * 2U)) | (3U << (0U * 2U));
    GPIOB_OTYPER &= ~(BIT(DIR_LEFT_PIN) | BIT(DIR_RIGHT_PIN));
    GPIOB_PUPDR &= ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U) | 3U << (0U * 2U));
    GPIOB_OSPEEDR = (GPIOB_OSPEEDR & ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U))) |
                    (2U << (DIR_LEFT_PIN * 2U)) | (2U << (DIR_RIGHT_PIN * 2U));

    /* PC1 -> analog current input */
    GPIOC_MODER = (GPIOC_MODER & ~(3U << (1U * 2U))) | (3U << (1U * 2U));
    GPIOC_PUPDR &= ~(3U << (1U * 2U));
}

static void uart2_init(void)
{
    /* 16 MHz / 460800 = 34.72 -> BRR = 0x22C (mantissa 34, fraction 12) */
    USART2_CR1 = 0U;
    USART2_BRR = 0x022CU;
    USART2_CR1 = BIT(13) | BIT(3) | BIT(2);
}

static void uart2_write_char(char c)
{
    while ((USART2_SR & BIT(7)) == 0U) {
    }
    USART2_DR = (uint32_t)(uint8_t)c;
}

static void uart2_write_str(const char *s)
{
    while (*s != '\0') {
        uart2_write_char(*s++);
    }
}

static void tim1_pwm_init(void)
{
    TIM1_CR1 = BIT(7);  /* ARPE */
    TIM1_PSC = 0U;
    TIM1_ARR = PWM_PERIOD_TICKS - 1U;
    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;

    /* PWM mode 1 on CH1/CH2 with preload */
    TIM1_CCMR1 = (6U << 4) | BIT(3) | (6U << 12) | BIT(11);
    TIM1_CCER = BIT(0) | BIT(4);
    TIM1_BDTR = BIT(15);   /* MOE */
    TIM1_EGR = BIT(0);     /* UG */
    TIM1_CR1 |= BIT(0);    /* CEN */
}

static void motor_set_direction(uint8_t left_forward, uint8_t right_forward)
{
    uint32_t bsrr = 0U;
    bsrr |= left_forward ? BIT(DIR_LEFT_PIN) : BIT(DIR_LEFT_PIN + 16U);
    bsrr |= right_forward ? BIT(DIR_RIGHT_PIN) : BIT(DIR_RIGHT_PIN + 16U);
    GPIOB_BSRR = bsrr;
}

static void motor_set_duty_pct(uint32_t left_pct, uint32_t right_pct)
{
    if (left_pct > 100U) left_pct = 100U;
    if (right_pct > 100U) right_pct = 100U;

    uint32_t ccr_l = ((TIM1_ARR + 1U) * left_pct) / 100U;
    uint32_t ccr_r = ((TIM1_ARR + 1U) * right_pct) / 100U;
    if (ccr_l > TIM1_ARR) ccr_l = TIM1_ARR;
    if (ccr_r > TIM1_ARR) ccr_r = TIM1_ARR;
    TIM1_CCR1 = ccr_l;
    TIM1_CCR2 = ccr_r;
}

static void adc1_init(void)
{
    const uint32_t sample_144_cycles = 6U;

    ADC1_CR1 = BIT(8);   /* SCAN */
    ADC1_CR2 = BIT(10);  /* EOCS: EOC after each conversion */

    /* Sampling times: ch8 in SMPR2[26:24], ch11 in SMPR1[5:3] */
    ADC1_SMPR2 = (ADC1_SMPR2 & ~(7U << 24U)) | (sample_144_cycles << 24U);
    ADC1_SMPR1 = (ADC1_SMPR1 & ~(7U << 3U)) | (sample_144_cycles << 3U);

    ADC1_SQR1 = (1U << 20U); /* L=1 -> 2 conversions */
    ADC1_SQR3 = (CURR_LEFT_ADC_CH << 0U) | (CURR_RIGHT_ADC_CH << 5U);

    ADC1_CR2 |= BIT(0);  /* ADON */
}

static uint8_t adc1_read_pair(uint16_t *left, uint16_t *right)
{
    uint32_t timeout = 200000U;

    ADC1_CR2 |= BIT(30); /* SWSTART */

    while ((ADC1_SR & BIT(1)) == 0U) {
        if (--timeout == 0U) return 0U;
    }
    *left = (uint16_t)ADC1_DR;

    timeout = 200000U;
    while ((ADC1_SR & BIT(1)) == 0U) {
        if (--timeout == 0U) return 0U;
    }
    *right = (uint16_t)ADC1_DR;
    return 1U;
}

static uint8_t adc1_read_avg_pair(uint16_t *left, uint16_t *right, uint32_t samples)
{
    uint32_t acc_l = 0U, acc_r = 0U, good = 0U;
    for (uint32_t i = 0U; i < samples; i++) {
        uint16_t l = 0U, r = 0U;
        if (adc1_read_pair(&l, &r)) {
            acc_l += l;
            acc_r += r;
            good++;
        }
    }
    if (good == 0U) return 0U;
    *left = (uint16_t)(acc_l / good);
    *right = (uint16_t)(acc_r / good);
    return 1U;
}

static int32_t counts_to_mA(uint16_t adc_counts, uint16_t zero_counts, int polarity)
{
    float vadc = ((float)adc_counts * ADC_VREF_VOLTS) / ADC_MAX_COUNTS;
    float vzero = ((float)zero_counts * ADC_VREF_VOLTS) / ADC_MAX_COUNTS;
    float vsense = (vadc - vzero) / CURR_DIV_RATIO;
    float amps = vsense / CURR_SENS_V_PER_A;
    int32_t mA = (int32_t)(amps * 1000.0f);
    return mA * polarity;
}

static const char *phase_name(BenchPhase phase)
{
    switch (phase) {
    case PHASE_IDLE: return "idle";
    case PHASE_LEFT_DRIVE: return "left";
    case PHASE_COAST: return "coast";
    case PHASE_RIGHT_DRIVE: return "right";
    default: return "unknown";
    }
}

void SystemInit(void)
{
    /* Keep default HSI clocking for bare-metal bench test. */
}

int main(void)
{
    uint16_t zero_left = 2048U;
    uint16_t zero_right = 2048U;
    float filt_left_mA = 0.0f;
    float filt_right_mA = 0.0f;
    uint8_t filt_init = 0U;
    BenchPhase phase = PHASE_IDLE;
    uint32_t phase_start_ms = 0U;
    uint32_t last_sample_ms = 0U;
    uint32_t last_print_ms = 0U;
    uint32_t duty_l = 0U;
    uint32_t duty_r = 0U;
    char line[256];

    clock_init();
    gpio_init();
    systick_init();
    uart2_init();
    tim1_pwm_init();
    adc1_init();

    motor_set_direction(1U, 1U);
    motor_set_duty_pct(0U, 0U);
    delay_ms(200U);

    {
        uint32_t acc_l = 0U, acc_r = 0U, good = 0U;
        for (uint32_t i = 0U; i < CAL_SAMPLES; i++) {
            uint16_t l = 0U, r = 0U;
            if (adc1_read_pair(&l, &r)) {
                acc_l += l;
                acc_r += r;
                good++;
            }
            delay_ms(2U);
        }
        if (good > 0U) {
            zero_left = (uint16_t)(acc_l / good);
            zero_right = (uint16_t)(acc_r / good);
        }
    }

    uart2_write_str("# SRM current bench start\r\n");
    snprintf(line, sizeof(line), "#INFO: zero_left=%u,zero_right=%u\r\n", zero_left, zero_right);
    uart2_write_str(line);
    uart2_write_str("#HEADER: t_ms,phase,duty_l_pct,duty_r_pct,adc_l,adc_r,zero_l,zero_r,raw_l_mA,raw_r_mA,filt_l_mA,filt_r_mA\r\n");

    phase_start_ms = millis();
    last_sample_ms = phase_start_ms;
    last_print_ms = phase_start_ms;

    for (;;) {
        uint32_t now = millis();
        uint32_t phase_age = now - phase_start_ms;

        if (phase == PHASE_IDLE && phase_age >= PHASE_IDLE_MS) {
            phase = PHASE_LEFT_DRIVE;
            phase_start_ms = now;
        } else if (phase == PHASE_LEFT_DRIVE && phase_age >= PHASE_LEFT_MS) {
            phase = PHASE_COAST;
            phase_start_ms = now;
        } else if (phase == PHASE_COAST && phase_age >= PHASE_BETWEEN_MS) {
            phase = PHASE_RIGHT_DRIVE;
            phase_start_ms = now;
        } else if (phase == PHASE_RIGHT_DRIVE && phase_age >= PHASE_RIGHT_MS) {
            phase = PHASE_IDLE;
            phase_start_ms = now;
        }

        duty_l = 0U;
        duty_r = 0U;
        if (phase == PHASE_LEFT_DRIVE) {
            duty_l = TEST_DUTY_PCT;
        } else if (phase == PHASE_RIGHT_DRIVE) {
            duty_r = TEST_DUTY_PCT;
        }
        motor_set_duty_pct(duty_l, duty_r);

        if ((now - last_sample_ms) < SAMPLE_PERIOD_MS) {
            continue;
        }
        last_sample_ms += SAMPLE_PERIOD_MS;

        uint16_t adc_l = 0U, adc_r = 0U;
        if (!adc1_read_avg_pair(&adc_l, &adc_r, ADC_AVG_SAMPLES)) {
            continue;
        }

        int32_t raw_l_mA = counts_to_mA(adc_l, zero_left, LEFT_CURR_POLARITY);
        int32_t raw_r_mA = counts_to_mA(adc_r, zero_right, RIGHT_CURR_POLARITY);

        if (!filt_init) {
            filt_left_mA = (float)raw_l_mA;
            filt_right_mA = (float)raw_r_mA;
            filt_init = 1U;
        } else {
            filt_left_mA += CURR_LPF_ALPHA * ((float)raw_l_mA - filt_left_mA);
            filt_right_mA += CURR_LPF_ALPHA * ((float)raw_r_mA - filt_right_mA);
        }

        if ((now - last_print_ms) >= PRINT_PERIOD_MS) {
            last_print_ms += PRINT_PERIOD_MS;
            int len = snprintf(line, sizeof(line),
                               "%lu,%s,%lu,%lu,%u,%u,%u,%u,%ld,%ld,%ld,%ld\r\n",
                               (unsigned long)now,
                               phase_name(phase),
                               (unsigned long)duty_l,
                               (unsigned long)duty_r,
                               (unsigned)adc_l,
                               (unsigned)adc_r,
                               (unsigned)zero_left,
                               (unsigned)zero_right,
                               (long)raw_l_mA,
                               (long)raw_r_mA,
                               (long)filt_left_mA,
                               (long)filt_right_mA);
            if (len > 0) {
                uart2_write_str(line);
            }
        }
    }
}
