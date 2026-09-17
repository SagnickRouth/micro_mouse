#include "main.h"
#include "config.h"
#include "oled.h"
#include "i2c.h"
#include "timer.h"

#include <stdbool.h>
#include <stdint.h>

#define START_COUNTDOWN_SECONDS 5U
#define KEY_DEBOUNCE_MS         40U
#define COUNTDOWN_STEP_MS       1000U

typedef enum
{
    ROBOT_READY = 0,
    ROBOT_COUNTDOWN,
    ROBOT_RUNNING
} RobotState;

static void SystemClock_Config(void);
static void ui_gpio_init(void);
static const char *read_algorithm_name(void);
static uint8_t read_dip_switches(void);
static void show_ready_screen(void);
static void show_countdown_screen(uint8_t seconds);
static void show_running_screen(void);
static bool key_pressed_event(void);

/* 84 MHz system clock: provides a stable 84 MHz timer clock for TIM1/3/4. */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    /* APB1 = 42 MHz; STM32F4 timers receive 2x APB1 clock = 84 MHz. */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void ui_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB3/PB4 are released as GPIOs; SWD remains on PA13/PA14. */
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* PA0 button: connect the button between PA0 and GND. */
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* PC13 onboard LED is active-low. */
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

static uint8_t read_dip_switches(void)
{
    uint8_t dip = 0U;

    /* PB4 = switch 0, active-low. */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
        dip |= 1U;

    /* PB3 = switch 1, active-low. */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
        dip |= 2U;

    return dip;
}

static const char *read_algorithm_name(void)
{
    bool sw0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
    bool sw1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);

    if (!sw0 && !sw1) return "FLOOD FILL";
    if ( sw0 && !sw1) return "LEFT WALL";
    if (!sw0 &&  sw1) return "RIGHT WALL";
    return "A STAR";
}

static void show_ready_screen(void)
{
    oled_show_message("MICRO MOUSE", "READY");
    oled_show_algorithm(read_algorithm_name(), false);
}

static void show_countdown_screen(uint8_t seconds)
{
    char countdown[2];

    countdown[0] = (char)('0' + seconds);
    countdown[1] = '\0';

    oled_show_message("GET READY", countdown);
}

static void show_running_screen(void)
{
    oled_show_message("ROBOT RUNNING", "SIMULATION");
    oled_show_algorithm(read_algorithm_name(), true);
}

/* Returns true once per debounced HIGH-to-LOW button press. */
static bool key_pressed_event(void)
{
    static GPIO_PinState previous_state = GPIO_PIN_SET;
    GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if ((previous_state == GPIO_PIN_SET) &&
        (current_state == GPIO_PIN_RESET))
    {
        HAL_Delay(KEY_DEBOUNCE_MS);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            previous_state = GPIO_PIN_RESET;
            return true;
        }
    }

    if (current_state == GPIO_PIN_SET)
        previous_state = GPIO_PIN_SET;

    return false;
}

int main(void)
{
    RobotState robot_state = ROBOT_READY;
    uint8_t previous_dip = 0xFFU;
    uint8_t current_dip;
    uint32_t countdown_start_time = 0U;
    uint8_t countdown_value = START_COUNTDOWN_SECONDS;

    HAL_Init();
    SystemClock_Config();
    timer_gpio_and_peripheral_init();
    ui_gpio_init();

    /* Give the OLED supply time to settle on cold power-up. */
    HAL_Delay(1000);
    MX_I2C1_Init();
    oled_init();
    show_ready_screen();

    while (1)
    {
        current_dip = read_dip_switches();

        /* Lock the algorithm selection once countdown starts. */
        if ((current_dip != previous_dip) &&
            (robot_state == ROBOT_READY))
        {
            previous_dip = current_dip;
            show_ready_screen();
        }

        switch (robot_state)
        {
            case ROBOT_READY:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

                if (key_pressed_event())
                {
                    countdown_value = START_COUNTDOWN_SECONDS;
                    countdown_start_time = HAL_GetTick();
                    show_countdown_screen(countdown_value);
                    robot_state = ROBOT_COUNTDOWN;
                }
                break;

            case ROBOT_COUNTDOWN:
            {
                uint32_t elapsed = HAL_GetTick() - countdown_start_time;

                if (elapsed >= COUNTDOWN_STEP_MS)
                {
                    countdown_start_time = HAL_GetTick();

                    if (countdown_value > 1U)
                    {
                        countdown_value--;
                        show_countdown_screen(countdown_value);
                    }
                    else
                    {
                        /* Motors are not connected yet: simulate RUNNING. */
                        robot_state = ROBOT_RUNNING;
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                        show_running_screen();
                    }
                }
                break;
            }

            case ROBOT_RUNNING:
                /* PC13 ON represents simulated motor activity. */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

                if (key_pressed_event())
                {
                    robot_state = ROBOT_READY;
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                    previous_dip = 0xFFU;
                    show_ready_screen();
                }
                break;

            default:
                robot_state = ROBOT_READY;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                show_ready_screen();
                break;
        }

        HAL_Delay(10);
    }
}

void Error_Handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(250);
    }
}
