/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "tusb.h"
#include "usb_descriptors.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
hid_gamepad_report_t report;

uint32_t start, end;

volatile int8_t lx, ly, rx, ry;
volatile uint16_t adcData[4]; // Ensure this is the buffer name used in DMA config
volatile uint16_t isADC1Converted = 0;
volatile  uint32_t counter = 0;
float time_us = 0;

// Center calibration
volatile int16_t center[4] = {2048, 2048, 2048, 2048};

// Filter state (use int for speed)
volatile int16_t filt[4] = {0};

// Tunable parameters
#define ALPHA_DIV 5     // EMA strength (5 → smooth, 3 → faster)
#define DEADZONE 20     // adjust based on noise

float angle = 0.0f;
#define RADIUS 100.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void led_blinking_task(void);
void hid_task(void);
void BareMetal_PortB_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BareMetal_PortB_Init(void)
{
    // 1. Enable the clock for GPIOB on the AHB1 bus
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 2. Configure MODER (Mode Register) for Inputs
    // Input mode is '00'. Clearing the entire register sets PB0-PB15 as inputs.
    GPIOB->MODER &= ~(0xFFFFFFFF);

    // 3. Configure OSPEEDR (Output Speed Register)
    GPIOB->OSPEEDR &= ~(0xFFFFFFFF); // Clear first
    GPIOB->OSPEEDR |= 0xAAAAAAAA;    // Set to High Speed

    // 4. Configure PUPDR (Pull-Up/Pull-Down Register) for Pull-Ups
    GPIOB->PUPDR &= ~(0xFFFFFFFF); // Clear first
    GPIOB->PUPDR |= 0x55555555;    // Set all 16 pins to Pull-Up
}

void ADC_BareMetal_Init(void) {
    // 1. Enable Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Configure Pins as Analog Mode (11)
    // PC2 (CH12), PC1 (CH11), PC4 (CH14), PC5 (CH15)
    // Clear bits first then set to be safe
    GPIOC->MODER &= ~((3U << (2 * 2)) | (3U << (1 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOC->MODER |=  ((3U << (2 * 2)) | (3U << (1 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));

    // 3. DMA2 Stream 0 Configuration (Remains same)
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);

    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adcData;
    DMA2_Stream0->NDTR = 4;

    DMA2_Stream0->CR = (0U << 25) | DMA_SxCR_CIRC | DMA_SxCR_MINC |
                       (1U << 13) | (1U << 11) | DMA_SxCR_PL;

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // 4. ADC Common Config
    ADC->CCR |= (1U << 16);

    // 5. ADC1 Setup
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;

    // 6. Sequence Length (4 conversions)
    ADC1->SQR1 = (3U << 20);

    // 7. Channel Order (CHANGE HERE)
    // Rank 1 changed from 10 (PC0) to 12 (PC2)
    ADC1->SQR3 = (12U << 0)  |           // Rank 1: PC2 (Channel 12)
                 (11U << 5)  |           // Rank 2: PC1 (Channel 11)
                 (14U << 10) |           // Rank 3: PC4 (Channel 14)
                 (15U << 15);            // Rank 4: PC5 (Channel 15)

    // 8. Sampling Time (Update SMPR1 for CH12)
    // CH10 was bits [0:2], CH12 is bits [6:8]
    ADC1->SMPR1 |= (7U << 6)  | (7U << 3) | (7U << 12) | (7U << 15);

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

static inline uint32_t board_button_read(void) {
  return 0;
}

static inline void board_led_write(bool state) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void process_joystick(volatile int8_t *lx,
                      volatile int8_t *ly,
                      volatile int8_t *rx,
                      volatile int8_t *ry){
    int16_t raw[4];

    // 1. Center offset
    raw[0] = adcData[0] - center[0];
    raw[1] = adcData[1] - center[1];
    raw[2] = adcData[2] - center[2];
    raw[3] = adcData[3] - center[3];

    // 2. EMA filter
    for (int i = 0; i < 4; i++)
    {
        filt[i] = filt[i] + (raw[i] - filt[i]) / ALPHA_DIV;
    }

    // 3. Scale to -127 to +127
    int16_t scaled[4];
    for (int i = 0; i < 4; i++)
    {
        scaled[i] = filt[i] / 16;
    }

    // 4. Deadzone
    for (int i = 0; i < 4; i++)
    {
    	if (abs(scaled[i]) < DEADZONE)
    	    scaled[i] = 0;
    }

    // 5. Output
    *lx = (int8_t)scaled[0];
    *ly = (int8_t)scaled[1];
    *rx = (int8_t)scaled[2];
    *ry = (int8_t)scaled[3];
}


void calibrate(void)
{
    HAL_Delay(100);

    center[0] = adcData[0];
    center[1] = adcData[1];
    center[2] = adcData[2];
    center[3] = adcData[3];
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
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  BareMetal_PortB_Init();

  tusb_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    tud_task();
    led_blinking_task();
    hid_task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  HAL_PCD_MspInit(&hpcd_USB_OTG_FS);
  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin   = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn) {
  // skip if hid is not ready yet
  if (!tud_hid_ready()) {
    return;
  }

  switch (report_id) {
    case REPORT_ID_KEYBOARD: {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if (btn != 0u) {
        uint8_t keycode[6] = {0};
        keycode[0]         = HID_KEY_A;

        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        has_keyboard_key = true;
      } else {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) {
          tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        }
        has_keyboard_key = false;
      }
      break;
    }

    case REPORT_ID_MOUSE: {
      int8_t const delta = 5;

      // no button, right + down, no scroll, no pan
      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
      break;
    }


/*
    case REPORT_ID_GAMEPAD: {
      static bool has_gamepad_key = false;
      static uint32_t start_ms = 0;
      static bool test_done = false;

      // Start the timer the first time this runs (when HID is ready)
      if (start_ms == 0) {
        start_ms = HAL_GetTick();
      }

      hid_gamepad_report_t report = {.x = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0, .hat = 0, .buttons = 0};

      if (!test_done && (HAL_GetTick() - start_ms < 1000)) {
        report.hat     = GAMEPAD_HAT_CENTERED;
        // Turn on 8 buttons (bits 0 to 7)
        report.buttons = GAMEPAD_BUTTON_0 | GAMEPAD_BUTTON_1 | GAMEPAD_BUTTON_2 | GAMEPAD_BUTTON_3 |
                         GAMEPAD_BUTTON_4 | GAMEPAD_BUTTON_5 | GAMEPAD_BUTTON_6 | GAMEPAD_BUTTON_7;
        tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
        has_gamepad_key = true;
      } else {
        report.hat     = GAMEPAD_HAT_CENTERED;
        report.buttons = 0;
        if (has_gamepad_key) {
          tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
        }
        has_gamepad_key = false;
        test_done = true;
      }
      break;
    }
*/


    default: break; // unknown report id
  }
}


void hid_task(void)
{
  const uint32_t interval_ms = 5;
  static uint32_t start_ms = 0;

  if (HAL_GetTick() - start_ms < interval_ms) return;
  start_ms += interval_ms;

  if (!tud_hid_ready()) return;

  // ============================================================
  // 1. ANALOG PROCESSING (ADC -> Filter -> scaled int8_t)
  // ============================================================
  // Call the processing function you wrote earlier.
  // It uses adcData[4] and updates the global lx, ly, rx, ry.
//  process_joystick(&lx, &ly, &rx, &ry);
  // -------- TEST: Circular joystick --------
  lx = (int8_t)(RADIUS * cosf(angle));
  ly = (int8_t)(RADIUS * sinf(angle));

  rx = (int8_t)(RADIUS * cosf(angle + M_PI)); // opposite phase
  ry = (int8_t)(RADIUS * sinf(angle + M_PI));

  angle += 0.1f;
  if (angle > 2 * M_PI)
      angle -= 2 * M_PI;
  // ============================================================
  // 2. DIGITAL BUTTON READ (GPIOB)
  // ============================================================
  // PB12-15 are used for D-Pad, others are buttons.
  // Using ~ because BareMetal_PortB_Init configured Pull-ups (Active Low).
  uint16_t buttons = (uint16_t)(~(GPIOB->IDR));

  // ============================================================
  // 3. D-PAD / HAT MAPPING
  // ============================================================
  bool dpad_up    = (buttons & (1U << 12)) != 0;
  bool dpad_down  = (buttons & (1U << 13) ) != 0;
  bool dpad_left  = (buttons & (1U << 14)) != 0;
  bool dpad_right = (buttons & (1U << 15)) != 0;

  uint8_t hat = GAMEPAD_HAT_CENTERED;

  // Clear the D-pad bits from the button bitmask so they don't show up as buttons
  buttons &= ~((1U << 12) | (1U << 13) | (1U << 14) | (1U << 15));

  uint8_t dpad_state = (dpad_up ? 1 : 0) |
                       (dpad_down ? 2 : 0) |
                       (dpad_left ? 4 : 0) |
                       (dpad_right ? 8 : 0);

  switch (dpad_state) {
    case 1: hat = GAMEPAD_HAT_UP; break;
    case 2: hat = GAMEPAD_HAT_DOWN; break;
    case 4: hat = GAMEPAD_HAT_LEFT; break;
    case 8: hat = GAMEPAD_HAT_RIGHT; break;
    case 5: hat = GAMEPAD_HAT_UP_LEFT; break;
    case 9: hat = GAMEPAD_HAT_UP_RIGHT; break;
    case 6: hat = GAMEPAD_HAT_DOWN_LEFT; break;
    case 10: hat = GAMEPAD_HAT_DOWN_RIGHT; break;
    default: hat = GAMEPAD_HAT_CENTERED; break;
  }

  // ============================================================
  // 4. SEND USB REPORT
  // ============================================================
  memset(&report, 0, sizeof(report));

  report.x  = lx;   // Left Stick X -> Axis 0
  report.y  = ly;   // Left Stick Y -> Axis 1
  report.z  = rx; // Right Stick X -> Axis 2 (Standard)
  report.rz =  ry;  // Right Stick Y -> Axis 3 (Standard)
  report.rx =  rx;    // Axis 4 (Trigger / Extra)
  report.ry = ry;    // Axis 5 (Trigger / Extra)
  report.hat = hat;
  report.buttons = buttons;

  tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
}
// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {
  (void)instance;
  (void)len;

  uint8_t next_report_id = report[0] + 1u;

  if (next_report_id < REPORT_ID_COUNT) {
    send_hid_report(next_report_id, board_button_read());
  }
}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
  // TODO not Implemented
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
  (void)instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT) {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD) {
      // bufsize should be (at least) 1
      if (bufsize < 1) {
        return;
      }

      uint8_t const kbd_leds = buffer[0];

      if ((kbd_leds & KEYBOARD_LED_CAPSLOCK) != 0u) {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      } else {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms  = 0;
  static bool     led_state = false;

  // blink is disabled
  if (0u == blink_interval_ms) {
    return;
  }

  // Blink every interval ms
  if (HAL_GetTick() - start_ms < blink_interval_ms) {
    return; // not enough time
  }
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

// Get board unique ID for USB serial number.
size_t board_get_unique_id(uint8_t id[], size_t max_len)
{
  (void) max_len;
  // STM32F407 Unique ID address is 0x1FFF7A10 (96 bits)
  uint8_t const* stm32_id = (uint8_t const*) 0x1FFF7A10;
  memcpy(id, stm32_id, 12);
  return 12;
}
/* USER CODE END 4 */

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
