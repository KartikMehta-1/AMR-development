#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define BIT(n) (1U << (n))

/* RCC */
#define RCC_BASE        0x40023800U
#define RCC_AHB1ENR     REG32(RCC_BASE + 0x30U)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x44U)
#define RCC_AHB1ENR_GPIOAEN BIT(0)
#define RCC_AHB1ENR_GPIOBEN BIT(1)
#define RCC_APB2ENR_TIM1EN  BIT(0)

/* GPIO */
#define GPIOA_BASE      0x40020000U
#define GPIOB_BASE      0x40020400U
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00U)
#define GPIOA_OTYPER    REG32(GPIOA_BASE + 0x04U)
#define GPIOA_OSPEEDR   REG32(GPIOA_BASE + 0x08U)
#define GPIOA_PUPDR     REG32(GPIOA_BASE + 0x0CU)
#define GPIOA_AFRH      REG32(GPIOA_BASE + 0x24U)
#define GPIOB_MODER     REG32(GPIOB_BASE + 0x00U)
#define GPIOB_OTYPER    REG32(GPIOB_BASE + 0x04U)
#define GPIOB_OSPEEDR   REG32(GPIOB_BASE + 0x08U)
#define GPIOB_PUPDR     REG32(GPIOB_BASE + 0x0CU)
#define GPIOB_BSRR      REG32(GPIOB_BASE + 0x18U)

/* TIM1 */
#define TIM1_BASE       0x40010000U
#define TIM1_CR1        REG32(TIM1_BASE + 0x00U)
#define TIM1_EGR        REG32(TIM1_BASE + 0x14U)
#define TIM1_CCMR1      REG32(TIM1_BASE + 0x18U)
#define TIM1_CCER       REG32(TIM1_BASE + 0x20U)
#define TIM1_PSC        REG32(TIM1_BASE + 0x28U)
#define TIM1_ARR        REG32(TIM1_BASE + 0x2CU)
#define TIM1_CCR1       REG32(TIM1_BASE + 0x34U)
#define TIM1_CCR2       REG32(TIM1_BASE + 0x38U)
#define TIM1_BDTR       REG32(TIM1_BASE + 0x44U)

/* Motor pins from AMR firmware: PA8/PA9 PWM (TIM1_CH1/CH2), PB4/PB5 DIR */
#define PWM_LEFT_PIN    8U
#define PWM_RIGHT_PIN   9U
#define DIR_LEFT_PIN    4U
#define DIR_RIGHT_PIN   5U

/* 16 MHz HSI -> 20 kHz PWM with ARR=799 (PSC=0). */
#define PWM_PERIOD_TICKS 800U
#define PWM_DUTY_TICKS   (PWM_PERIOD_TICKS / 10U)

void SystemInit(void)
{
    /* Keep default clocking (HSI) for simple bring-up. */
}

static void clock_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
}

static void gpio_init(void)
{
    /* PA8/PA9 -> AF1 (TIM1_CH1/CH2). */
    GPIOA_MODER = (GPIOA_MODER & ~(3U << (PWM_LEFT_PIN * 2U) | 3U << (PWM_RIGHT_PIN * 2U))) |
                  (2U << (PWM_LEFT_PIN * 2U) | 2U << (PWM_RIGHT_PIN * 2U));
    GPIOA_AFRH = (GPIOA_AFRH & ~((0xFU << ((PWM_LEFT_PIN - 8U) * 4U)) | (0xFU << ((PWM_RIGHT_PIN - 8U) * 4U)))) |
                 (1U << ((PWM_LEFT_PIN - 8U) * 4U)) | (1U << ((PWM_RIGHT_PIN - 8U) * 4U));
    GPIOA_OTYPER &= ~(BIT(PWM_LEFT_PIN) | BIT(PWM_RIGHT_PIN));
    GPIOA_PUPDR &= ~(3U << (PWM_LEFT_PIN * 2U) | 3U << (PWM_RIGHT_PIN * 2U));
    GPIOA_OSPEEDR = (GPIOA_OSPEEDR & ~(3U << (PWM_LEFT_PIN * 2U) | 3U << (PWM_RIGHT_PIN * 2U))) |
                    (2U << (PWM_LEFT_PIN * 2U) | 2U << (PWM_RIGHT_PIN * 2U));

    /* PB4/PB5 -> DIR outputs (set high for forward). */
    GPIOB_MODER = (GPIOB_MODER & ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U))) |
                  (1U << (DIR_LEFT_PIN * 2U) | 1U << (DIR_RIGHT_PIN * 2U));
    GPIOB_OTYPER &= ~(BIT(DIR_LEFT_PIN) | BIT(DIR_RIGHT_PIN));
    GPIOB_PUPDR &= ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U));
    GPIOB_OSPEEDR = (GPIOB_OSPEEDR & ~(3U << (DIR_LEFT_PIN * 2U) | 3U << (DIR_RIGHT_PIN * 2U))) |
                    (2U << (DIR_LEFT_PIN * 2U) | 2U << (DIR_RIGHT_PIN * 2U));
    GPIOB_BSRR = BIT(DIR_LEFT_PIN) | BIT(DIR_RIGHT_PIN);
}

static void tim1_pwm_init(void)
{
    TIM1_CR1 = BIT(7); /* ARPE */
    TIM1_PSC = 0U;
    TIM1_ARR = PWM_PERIOD_TICKS - 1U;
    TIM1_CCR1 = PWM_DUTY_TICKS;
    TIM1_CCR2 = PWM_DUTY_TICKS;

    /* PWM mode 1 on CH1/CH2 with preload enabled. */
    TIM1_CCMR1 = (6U << 4) | BIT(3) | (6U << 12) | BIT(11);
    TIM1_CCER = BIT(0) | BIT(4);
    TIM1_BDTR |= BIT(15); /* MOE */
    TIM1_EGR = BIT(0);    /* UG */
    TIM1_CR1 |= BIT(0);   /* CEN */
}

int main(void)
{
    clock_init();
    gpio_init();
    tim1_pwm_init();

    for (;;)
    {
    }
}
