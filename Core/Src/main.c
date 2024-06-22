/* USER CODE BEGIN Header */
/*
 * timer1 for delay us
 * timer2 for sampling
 * */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include <stdbool.h>
#include "FLASH_SECTOR_F4.h"
#include "stdio.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_CM4

#define SAMPLE_BUFFER_LENGTH        256
#define SAMPLE_BUFFER_LENGTH_HALF   (SAMPLE_BUFFER_LENGTH/2)
#define CURRENT_1 ADC_CHANNEL_0
#define CURRENT_2 ADC_CHANNEL_1
#define CURRENT_3 ADC_CHANNEL_2

#define VOLTAGE_1 ADC_CHANNEL_3
#define VOLTAGE_2 ADC_CHANNEL_4
#define VOLTAGE_3 ADC_CHANNEL_5

#define TEMP ADC_CHANNEL_6
#define ERTEASH ADC_CHANNEL_7

#define approximation 1e-5
// fft
#define fftElementSendCount 50
#define  instantaneousElementSendCount 32
#define data_unit_size 7 // xxx.xx,


#define FlashAddress 0x080FAEC0
#define ratedVoltageAddress  FlashAddress+(0*0x32)
#define ratedCurrentAddress FlashAddress+(1*0x32)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
int elementNumber = 0;

struct adcValue {
    float instantaneousValues[SAMPLE_BUFFER_LENGTH];
    char instantaneousValuesCharArray[SAMPLE_BUFFER_LENGTH];
    float fft[SAMPLE_BUFFER_LENGTH_HALF];
    char fft_char_array[(fftElementSendCount * data_unit_size) + 2]; // [fft_array]
    float rms;
    float thd;
};

struct errorsStruct {
    bool voltageRmsError;
    int voltageRmsTimeout;


    bool currentRmsError;
    int currentRmsTimeout;

    bool voltageHarmonicOverLimit;
    char errorPhase[2];
} errors;
struct errorsStruct errors = {false, 0,
                              false, 0,
                              false};

struct adcValue c1, c2, c3, v1, v2, v3;

float settings[100];

uint8_t Rx_data[30] = {0};
uint8_t MESSAGE_LEN = sizeof(Rx_data);
bool new_request = false;

float fft_output[SAMPLE_BUFFER_LENGTH];
uint8_t ifftFlag = 0;

uint32_t time1, time2, result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_ADC1_Init(void);

static void MX_TIM1_Init(void);

static void MX_TIM2_Init(void);

static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//utils
void readParamsFromMemory() {
    for (int i = 0; i < 100; ++i) {
        settings[i] = Flash_Read_NUM(FlashAddress + (i * 0x32));
    }
}

void writeParamsToMemory() {
    Flash_Write_NUM(ratedVoltageAddress, settings[0]);
    Flash_Write_NUM(ratedCurrentAddress, settings[1]);
}

// serial
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(&huart4, Rx_data, MESSAGE_LEN);
    new_request = true;
}

bool str_cmp_rx_data(char str2[]) {
    int size = sizeof(str2);
    for (int i = 0; i < size; i++) {
        if (Rx_data[i] != str2[i]) { return false; };
    }
    return true;
}


void floatArrToList(char *dest, float *numbers, int array_size, int step) {
    char tempStr[9] = {"\0"};
    int usedLength = 0;
    int tempLength;
    for (int i = 0; i < array_size; i += step) {
        tempLength = sprintf(tempStr, "%.2f,", numbers[i]);
        sprintf(dest + usedLength, "%s", tempStr);
        usedLength = usedLength + tempLength;
    }
    dest[usedLength - 1] = '\0';
}

void floatArrToListInstantaneousValues() {
    floatArrToList(c1.instantaneousValuesCharArray, c1.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
    floatArrToList(c2.instantaneousValuesCharArray, c2.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
    floatArrToList(c3.instantaneousValuesCharArray, c3.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
    floatArrToList(v1.instantaneousValuesCharArray, v1.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
    floatArrToList(v2.instantaneousValuesCharArray, v2.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
    floatArrToList(v3.instantaneousValuesCharArray, v3.instantaneousValues, SAMPLE_BUFFER_LENGTH,
                   SAMPLE_BUFFER_LENGTH / instantaneousElementSendCount);
}

void floatArrToListFft() {
    floatArrToList(c1.fft_char_array, c1.fft, fftElementSendCount, 1);
    floatArrToList(c2.fft_char_array, c2.fft, fftElementSendCount, 1);
    floatArrToList(c3.fft_char_array, c3.fft, fftElementSendCount, 1);
    floatArrToList(v1.fft_char_array, v1.fft, fftElementSendCount, 1);
    floatArrToList(v2.fft_char_array, v2.fft, fftElementSendCount, 1);
    floatArrToList(v3.fft_char_array, v3.fft, fftElementSendCount, 1);
}

void stringToArrayReceivedSettings() {

    char *token = strtok(Rx_data, ","); //remove saveSettings
    token = strtok(NULL, ","); // number1 ...
    int i = 0;
    while (token[i] != NULL) {
        settings[i] = atof(token);
        token = strtok(NULL, ",");
        i++;
    }
}

void sendRealTimeData() {
    char buffer[5000] = {'\0'};
    char *fromat = "{"
                   "\"type\":\"RealTimeData\","
                   "\"current\":"
                   "{\"a\":[%s],"
                   "\"b\":[%s],"
                   "\"c\":[%s]},"
                   "\"voltage\":"
                   "{\"a\":[%s],"
                   "\"b\":[%s],"
                   "\"c\":[%s]}"
                   "}\r\n";
    snprintf(buffer, sizeof(buffer), fromat,
             c1.instantaneousValuesCharArray,
             c2.instantaneousValuesCharArray,
             c3.instantaneousValuesCharArray,
             v1.instantaneousValuesCharArray,
             v2.instantaneousValuesCharArray,
             v3.instantaneousValuesCharArray
    );
    HAL_UART_Transmit(&huart4, buffer, sizeof(buffer), 1000);

}

static void send_serial() {
    static uint8_t buffer[5000] = {'\0'};
    memset(buffer, ' ', sizeof(buffer));
    if (str_cmp_rx_data("send_data")) {
        static char *fromat = "{"
                              "\"type\":\"calculatedData\","
                              "\"current\":"
                              "{\"a\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]},"
                              "\"b\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]},"
                              "\"c\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]}},"
                              "\"voltage\":"
                              "{\"a\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]},"
                              "\"b\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]},"
                              "\"c\":{\"rms\":%.2f,\"thd\":%.2f,\"fft\":[%s]}}"
                              "}\r\n";
        snprintf(buffer, sizeof(buffer), fromat,
                /*rms-thd-fft*/
                 c1.rms, c1.thd, c1.fft_char_array,
                 c2.rms, c2.thd, c2.fft_char_array,
                 c3.rms, c3.thd, c3.fft_char_array,
                 v1.rms, v1.thd, v1.fft_char_array,
                 v2.rms, v2.thd, v2.fft_char_array,
                 v3.rms, v3.thd, v3.fft_char_array);
    } else if (str_cmp_rx_data("saveSettings")) {
        stringToArrayReceivedSettings();
        writeParamsToMemory();
        strcpy(buffer, "OK\r\n");
    } else if (str_cmp_rx_data("ping")) {
        strcpy(buffer, "pong\r\n");
    } else {
        strcpy(buffer, "404\r\n");
    }

    HAL_UART_Transmit(&huart4, buffer, sizeof(buffer), 1000);
    new_request = false;
}

// adc
void selectChannel(uint32_t ch) {
    ADC_ChannelConfTypeDef conf = {
            .Channel = ch,
            .Rank = 1,
            .SamplingTime = ADC_SAMPLETIME_15CYCLES,
    };
    if (HAL_ADC_ConfigChannel(&hadc1, &conf) != HAL_OK) {
        Error_Handler();
    }
}

void get_adc_value(float *element) {
    while (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_BUSY) {}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    *element = (HAL_ADC_GetValue(&hadc1) * 3.3 / 4095);
    HAL_ADC_Stop(&hadc1);
}

void getAdcValue(uint32_t ch, float *element) {
    selectChannel(ch);
    get_adc_value(element);
}

void getAdcValues() {
    time1 = HAL_GetTick();
    getAdcValue(CURRENT_1, &c1.instantaneousValues[elementNumber]);
    getAdcValue(CURRENT_2, &c2.instantaneousValues[elementNumber]);
    getAdcValue(CURRENT_3, &c3.instantaneousValues[elementNumber]);
    getAdcValue(VOLTAGE_1, &v1.instantaneousValues[elementNumber]);
    getAdcValue(VOLTAGE_2, &v2.instantaneousValues[elementNumber]);
    getAdcValue(VOLTAGE_3, &v3.instantaneousValues[elementNumber]);
    elementNumber++;
    if (elementNumber >= SAMPLE_BUFFER_LENGTH) {
        HAL_TIM_Base_Stop_IT(&htim2);
        elementNumber = 0;
    }
    time2 = HAL_GetTick();
    result = time2 - time1;
}

//timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    getAdcValues();
}

// calculation
void doFft(float input[], float output[]) {
    arm_status status;
    arm_rfft_fast_instance_f32 fft;
    status = arm_rfft_fast_init_f32(&fft, SAMPLE_BUFFER_LENGTH);
    arm_rfft_fast_f32(&fft, input, fft_output, ifftFlag);
    arm_cmplx_mag_f32(fft_output, output, SAMPLE_BUFFER_LENGTH_HALF);
    for (int i = 0; i < SAMPLE_BUFFER_LENGTH_HALF; i++) {
        output[i] = output[i] / 128;
        if (output[i] < approximation)output[i] = 0;
    }
}

void doThd(float input[], float output) {
    float temp = 0;
    for (int i = 2; i < SAMPLE_BUFFER_LENGTH_HALF; i++) {
        temp = temp + (pow(input[i], 2));
    }
    output = sqrt(temp) * 100 / input[1];
}

void calculation() {
    // rms
    arm_rms_f32(c1.instantaneousValues, SAMPLE_BUFFER_LENGTH, &c1.rms);
    arm_rms_f32(c2.instantaneousValues, SAMPLE_BUFFER_LENGTH, &c2.rms);
    arm_rms_f32(c3.instantaneousValues, SAMPLE_BUFFER_LENGTH, &c3.rms);

    arm_rms_f32(v1.instantaneousValues, SAMPLE_BUFFER_LENGTH, &v1.rms);
    arm_rms_f32(v2.instantaneousValues, SAMPLE_BUFFER_LENGTH, &v2.rms);
    arm_rms_f32(v3.instantaneousValues, SAMPLE_BUFFER_LENGTH, &v3.rms);
    // fft
    doFft(c1.instantaneousValues, c1.fft);
    doFft(c2.instantaneousValues, c2.fft);
    doFft(c3.instantaneousValues, c3.fft);

    doFft(v1.instantaneousValues, v1.fft);
    doFft(v2.instantaneousValues, v2.fft);
    doFft(v3.instantaneousValues, v3.fft);
    // thd
    doThd(c1.fft, c1.thd);
    doThd(c2.fft, c2.thd);
    doThd(c3.fft, c3.thd);

    doThd(v1.fft, v1.thd);
    doThd(v2.fft, v2.thd);
    doThd(v3.fft, v3.thd);
}

void currentRmsControl() {}

void voltageRmsControl() {}

void voltageHarmonicControl() {}

void control() {

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

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
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_UART4_Init();
    /* USER CODE BEGIN 2 */
    //start timer,uart and interupts
    HAL_UART_Receive_IT(&huart4, Rx_data, MESSAGE_LEN);
    readParamsFromMemory();
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_UART_Receive_IT(&huart4, Rx_data, MESSAGE_LEN);
        floatArrToListInstantaneousValues();
        sendRealTimeData();
//        time1=HAL_GetTick();
        calculation();
        control();
        floatArrToListFft();
//        time2=HAL_GetTick();
//        result=time2-time1;

        if (new_request) {
            send_serial();
        }
//        str_append_float(&c1);
        HAL_UART_AbortReceive_IT(&huart4);
        HAL_TIM_Base_Start_IT(&htim2);
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {

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
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 167;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 13124;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void) {

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 230400;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
