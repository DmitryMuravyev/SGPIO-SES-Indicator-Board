/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define SUPPORTED_PAGES_COUNT		4						// Number of supported SES diagnostic pages
#define SUPPORTED_DRIVES_COUNT		4						// This firmware is designed for STM32A070F6 MCU with 20 pins. Devices based on more powerful MCUs could support a larger number of disks.
#define CONTROL_ELEMENTS_COUNT		4						// Enclosure, Disk array, Temperature sensor and Voltage sensor
#define DESCRIPTOR_TEXT_LENGTH		8						// Text descriptor length for each element


typedef struct __attribute__((__packed__)) {				// LEDs structure for each disk (based on the SGPIO protocol)
	uint8_t ACTIVE;
	uint8_t IDENTIFY;
	uint8_t ERROR;
} Drive;


typedef struct __attribute__((__packed__)) {				// Common SEP Header
	uint8_t CMD_TYPE;
	uint8_t CHECKSUM;
	uint8_t ADDRESS;
	uint8_t SEQ;
	uint8_t CMD;
} SEP_Header;


typedef struct {											// Common SEP Answer
	SEP_Header HEADER;
	uint8_t STATUS;
	uint8_t CHECKSUM;
} SEP_Common_Answer;


typedef struct __attribute__((__packed__)) {				// Descriptive strings that are included in both SEP Identifier and SES Configuration diagnostic page
	char ENCLOSURE_LOGICAL_IDENTIFIER[8];
	char ENCLOSURE_VENDOR_IDENTIFICATION[8];
	char PRODUCT_IDENTIFICATION[16];
	char PRODUCT_REVISION_LEVEL[4];
} SES_Product_Strings;


typedef struct __attribute__((__packed__)) {				// SEP Identifier structure
	uint8_t ENCLOSURE_DESCRIPTOR_LENGTH;
	uint8_t SUB_ENCLOSURE_IDENTIFIER;
	SES_Product_Strings PRODUCT_STRINGS;
	uint8_t CHANNEL_IDENTIFIER;
	char FIRMWARE_REVISION_LEVEL[4];
	char INTERFACE_IDENTIFICATION_STRING[6];
	char INTERFACE_SPECIFICATION_REVISION_LEVEL[4];
	char VENDOR_SPECIFIC_ENCLOSURE_INFORMATION[11];
} SEP_Identifier;


typedef struct __attribute__((__packed__)) {				// SEP request: Receive SES Diagnostic
	SEP_Header HEADER;
	uint8_t CHECKSUM;
} SEP_Receive_Diagnostic_Request;


typedef struct __attribute__((__packed__)) {				// SEP answer: Identifier
	SEP_Header HEADER;
	uint8_t STATUS;
	SEP_Identifier INENTIFIER;
	uint8_t CHECKSUM;
} SEP_Idetifier_Answer;


typedef struct __attribute__((__packed__)) {				// SES Supported Pages diagnostic page structure
	uint8_t PAGE_CODE;
	uint8_t reserved;
	uint16_t PAGE_LENGTH;
	uint8_t SUPPORTED_PAGES[SUPPORTED_PAGES_COUNT];
} SES_Supported_Pages;


typedef struct __attribute__((__packed__)) {				// SEP answer: Supported SES diagnostic pages
	SEP_Header HEADER;
	uint8_t STATUS;
	SES_Supported_Pages SUPPORTED_PAGES;
	uint8_t CHECKSUM;
} SEP_Supported_Pages_Answer;


typedef struct __attribute__((__packed__)) {				// SES Control Element structure
	uint8_t ELEMENT_TYPE;
	uint8_t NUMBER_OF_POSSIBLE_ELEMENTS;
	uint8_t SUB_ENCLOSURE_IDENTIFIER;
	uint8_t TYPE_DESCRIPTOR_TEXT_LENGTH;
} SES_Control_Element;


typedef struct __attribute__((__packed__)) {				// SES Configuration structure
	uint8_t PAGE_CODE;
	uint8_t NUMBER_OF_SUB_ENCLOSURES;
	uint16_t PAGE_LENGTH;
	uint32_t GENERATION_CODE;
	uint8_t PROCESS;
	uint8_t SUB_ENCLOSURE_IDENTIFIER;
	uint8_t NUMBER_OF_ELEMENT_TYPES_SUPPORTED;
	uint8_t ENCLOSURE_DESCRIPTOR_LENGTH;
} SES_Configuration;


typedef struct __attribute__((__packed__)) {				// SEP answer: SES Configuration page
	SEP_Header HEADER;
	uint8_t STATUS;
	SES_Configuration CONFIGURATION;
	SES_Product_Strings PRODUCT_STRINGS;
	SES_Control_Element CONTROL_ELEMENTS[CONTROL_ELEMENTS_COUNT];
	uint8_t CHECKSUM;
} SEP_Configuration_Answer;


typedef struct __attribute__((__packed__)) {				// SES Element Status/Control structure
	uint8_t COMMON;
	uint8_t CUSTOM_1;
	uint8_t CUSTOM_2;
	uint8_t CUSTOM_3;
} SES_Element_Status_Control;


typedef struct __attribute__((__packed__)) {				// SES Status page structure
	uint32_t GENERATION_CODE;
	SES_Element_Status_Control DRIVES_OVERALL;
	SES_Element_Status_Control DRIVES[SUPPORTED_DRIVES_COUNT];
	SES_Element_Status_Control ENCLOSURE_OVERALL;
	SES_Element_Status_Control ENCLOSURE;
	SES_Element_Status_Control TSENSOR_OVERALL;
	SES_Element_Status_Control TSENSOR;
	SES_Element_Status_Control VSENSOR_OVERALL;
	SES_Element_Status_Control VSENSOR;
} SES_Status_Page;


typedef struct __attribute__((__packed__)) {				// SEP request: get/set SES Status/Control diagnostic page
	SEP_Header HEADER;
	uint8_t PAGE_CODE;
	uint8_t CONDITIONS;
	uint16_t PAGE_LENGTH;
	SES_Status_Page STATUS_PAGE;
	uint8_t CHECKSUM;
} SEP_Status_Control;


typedef struct __attribute__((__packed__)) {				// SEP answer: SES Status diagnostic page
	SEP_Header HEADER;
	uint8_t STATUS;
	uint8_t PAGE_CODE;
	uint8_t CONDITIONS;
	uint16_t PAGE_LENGTH;
	SES_Status_Page STATUS_PAGE;
	uint8_t CHECKSUM;
} SEP_Status_Answer;


typedef struct __attribute__((__packed__)) {				// SES Element Descriptor structure
	uint8_t reserved1;
	uint8_t reserved2;
	uint16_t DESCRIPTOR_LENGTH;
} SES_Element_Descriptor;


typedef struct __attribute__((__packed__)) {				// SES Element Descriptor with Text structure
	SES_Element_Descriptor DESCRIPTOR;
	char DESCRIPTOR_TEXT[DESCRIPTOR_TEXT_LENGTH];
} SES_Element_Descriptor_Text;


typedef struct __attribute__((__packed__)) {				// SES all supported Elements Descriptor structure
	uint32_t GENERATION_CODE;
	SES_Element_Descriptor DRIVES_OVERALL;
	SES_Element_Descriptor_Text DRIVES[SUPPORTED_DRIVES_COUNT];
	SES_Element_Descriptor ENCLOSURE_OVERALL;
	SES_Element_Descriptor_Text ENCLOSURE;
	SES_Element_Descriptor TSENSOR_OVERALL;
	SES_Element_Descriptor_Text TSENSOR;
	SES_Element_Descriptor VSENSOR_OVERALL;
	SES_Element_Descriptor_Text VSENSOR;
} SES_Elements_Descriptor_Page;


typedef struct __attribute__((__packed__)) {				// SEP answer: SES Elements Descriptor
	SEP_Header HEADER;
	uint8_t STATUS;
	uint8_t PAGE_CODE;
	uint8_t reserved;
	uint16_t PAGE_LENGTH;
	SES_Elements_Descriptor_Page DESCRIPTORS;
	uint8_t CHECKSUM;
} SEP_Elements_Descriptor_Answer;


typedef enum {												// Selected and Active interface types (e.g. Selected interface could be InterfaceAuto and Active could be InterfaceI2C or InterfaceSGPIO)
	InterfaceUnknown	= 0x00,
	InterfaceAuto		= 0x01,
	InterfaceI2C		= 0x02,
	InterfaceSGPIO		= 0x03
} InterfaceSelection;


typedef enum {												// I2C line state (Hold if Address = 0 or Reset = 1)
	StateHold			= 0x00,
	StateActive			= 0x01
} I2C_Line_State;


typedef enum {												// I2C interchange state
	StateListening		= 0x00,
	StateDataReceived	= 0x01,
	StateError			= 0x02
} I2C_Interchange_State;


typedef enum {												// Notification Severities
	OkSeverity			= 0b00000000,						// No notification
	LowSeverity			= 0b00000001,						// Warning (slow blinking)
	HighSeverity		= 0b00000010,						// Error (fast blinking)
	CriticalSeverity	= 0b00000100,						// Critical error (continuously lit)
} NotificationSeverity;


typedef enum {												// Demo mode steps
	NoAction			= 0x00,
	RunningLight		= 0x01,
	AllBlinking			= 0x02
} DemoSteps;


typedef struct {											// SES Status bits for Array Elements (drives) and for Enclosure Element
	SES_Element_Status_Control SES_STATUS;
	uint8_t alarm_severity;
	uint8_t identify_severity;
} Element_Status;


typedef struct __attribute__((__packed__)) {				// ADC data structure for raw and filtered ADC readings
	uint16_t address_select_voltage;
	uint16_t mcu_temperature;
	uint16_t vref_voltage;
} adc_data;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Base firmware configuration
///////////////////////////////////////////////////////////////////////////////////////////////////////////


// If the board installed upside down, then you can invert drives sequence to 3-2-1-0 by uncommenting next define statement.
//#define INVERTED_DRIVES_DIRECTION


// Comment next define statement if you want to disable LEDs blinking demo at system startup.
#define ENABLE_STARTUP_DEMO


// Uncomment next define statement if for the infinite Demo mode (the demo will be repeated until the first status data arrives).
//#define INFINITE_DEMO


// Disable next statement if you don't need Reset signal detection in I2C mode
#define I2C_ENABLE_RESET_DETECTION


// Disable next statement if you don't need Address signal detection in I2C mode
#define I2C_ENABLE_ADDRESS_DETECTION


// Timeout in milliseconds during which, if there is no communication via I2C, the indication will be reset and the I2C module reinitialized
#define NO_DATA_TIMEOUT								30000


// Uncomment next define statement if you have large capacitance (and low pin speed) in LED transistor nets.
#define SMALL_LED_CONTROL_NET_SPEED


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// End of base firmware configuration
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TRUE										1
#define FALSE										0

#define BUFFER_SIZE									128													// This buffer size should be enough for the largest sent/received amount of data (Element Descriptor diagnostic page is about 107 bytes)
#define LEDS_PER_DRIVE								sizeof(Drive)
#define LEDS_COUNT									(SUPPORTED_DRIVES_COUNT * LEDS_PER_DRIVE)
#define LED_ON_STATE_PERIOD							50													// Time in 1/100s of second (0.5s is SES recommendation)
#define LED_FAST_TO_SLOW_BLINKING_FACTOR			4													// The ratio of the duration of long and short blinks
#define MINIMAL_DELAY								1
#define DEBOUNCING_READINGS_COUNT					10													// The number of pin readings to determine its exact value (must be even)

#define SEP_TO_HOST_COMMAND_TYPE					0x02
#define HOST_TO_SEP_COMMAND_TYPE					0x82
#define SEP_IDENTIFY_PAGE							0xEC
#define SEP_OK_STATUS								0x50
#define SEP_ERROR_STATUS							0x51

#define SEP_CHANNEL_IDENTIFIER						0x00
#define SES_NUMBER_OF_SUBENCLOSURES					0x00
#define SES_SUB_ENCLOSURE_IDENTIFIER				0x00
#define SES_GENERATION								0x00000000
#define SES_PROCESS									0x11												// Relative Enclosure Service Process Identifier = 1, Number of Enclosure Service Processes = 1
#define SES_ENCLOSURE_STATUS_PAGE_CONDITIONS		0x00												// Invalid operation requested = 0, information conditions = 0, noncritical conditions = 0, critical condition = 0, unrecoverable conditions = 0

#define SES_SUPPORTED_PAGES_PAGE					0x00
#define SES_CONFIGURATION_PAGE						0x01
#define SES_STATUS_PAGE								0x02
#define SES_ELEMENT_DESCRIPTOR_PAGE					0x07

#define SES_ARRAY_ELEMENT							0x17
#define SES_ENCLOSURE_ELEMENT						0x0E
#define SES_TEMP_SENSOR_ELEMENT						0x04
#define SES_VOLT_SENSOR_ELEMENT						0x12

#define SES_ELEMENT_OK_STATUS						0x01

#define SEP_HEADER_SIZE								sizeof(SEP_Header)									//	5
#define SEP_HEADER_CMD_TYPE_SIZE					1
#define SEP_CHECKSUM_SIZE							1
#define SEP_HEADER_CMD_TYPE_W_CHECKSUM_SIZE			(SEP_HEADER_CMD_TYPE_SIZE + SEP_CHECKSUM_SIZE)		//  2
#define SEP_COMMON_REQUEST_SIZE						sizeof(SEP_Receive_Diagnostic_Request)				//  6
#define SES_PAGE_LENGTH_SIZE						3
#define SEP_COMMON_ANSWER_SIZE						sizeof(SEP_Common_Answer)							//	7
#define SEP_RECEIVE_DIAGNOSTIC_REQUEST_SIZE			sizeof(SEP_Receive_Diagnostic_Request)				//	6
#define SES_PRODUCT_STRINGS_SIZE					sizeof(SES_Product_Strings)							//	36
#define SEP_IDENTIFIER_SIZE							sizeof(SEP_Identifier)								//	64
#define SEP_IDENTIFIER_ANSWER_SIZE					sizeof(SEP_Idetifier_Answer)						//	71
#define SES_SUPPORTED_PAGES_SIZE					sizeof(SES_Supported_Pages)							//	SUPPORTED_PAGES_COUNT + 4
#define SES_SUPPORTED_PAGES_ANSWER_SIZE				sizeof(SEP_Supported_Pages_Answer)					//	SEP_HEADER_SIZE + SES_SUPPORTED_PAGES_SIZE + 2
#define SES_CONTROL_ELEMENT_SIZE					sizeof(SES_Control_Element)							//	4
#define SES_CONTROL_ELEMENTS_SIZE					SES_CONTROL_ELEMENT_SIZE * CONTROL_ELEMENTS_COUNT
#define SES_CONFIGURATION_SIZE						sizeof(SES_Configuration)
#define SES_CONFIGURATION_PAGE_SIZE					SES_CONFIGURATION_SIZE + SES_PRODUCT_STRINGS_SIZE + SES_CONTROL_ELEMENTS_SIZE - 4
#define SEP_CONFIGURATION_ANSWER_SIZE				sizeof(SEP_Configuration_Answer)					//	SEP_HEADER_SIZE + SES_CONFIGURATION_SIZE + SES_PRODUCT_STRINGS_SIZE + SES_CONTROL_ELEMENTS_SIZE + 2
#define SES_ELEMENT_STATUS_CONTROL_SIZE				sizeof(SES_Element_Status_Control)
#define SES_STATUS_SIZE								sizeof(SES_Status_Page)
#define SEP_STATUS_CONTROL_SIZE						sizeof(SEP_Status_Control)
#define SEP_STATUS_ANSWER_SIZE						sizeof(SEP_Status_Answer)
#define SES_ELEMENT_DESCRIPTOR_SIZE					sizeof(SES_Element_Descriptor)
#define SES_ELEMENT_DESCRIPTOR_TEXT_SIZE			sizeof(SES_Element_Descriptor_Text)
#define SES_ELEMENTS_DESCRIPTOR_SIZE				sizeof(SES_Elements_Descriptor_Page)
#define SEP_ELEMENTS_DESCRIPTOR_ANSWER_SIZE			sizeof(SEP_Elements_Descriptor_Answer)

#define SES_COMMON_SELECT_BIT						0b10000000
#define SES_COMMON_PREDICTED_FAILURE_BIT			0b01000000

#define SES_ARRAY_ELEMENT_IN_CRITICAL_ARRAY_BIT		0b00001000
#define SES_ARRAY_ELEMENT_IN_FAILED_ARRAY_BIT		0b00000100
#define SES_ARRAY_ELEMENT_REBUILD_REMAP_BIT			0b00000010
#define SES_ARRAY_ELEMENT_R_R_ABORT_BIT				0b00000001
#define SES_ARRAY_ELEMENT_ACTIVE_BIT				0b10000000
#define SES_ARRAY_ELEMENT_IDENTIFY_BIT				0b00000010
#define SES_ARRAY_ELEMENT_REMOVE_INSERT_BITS		0b00001100
#define SES_ARRAY_ELEMENT_FAULT_BIT					0b00100000

#define SES_ENCLOSURE_ERROR_BIT						0b00000010
#define SES_ENCLOSURE_WARNING_BIT					0b00000001
#define SES_ENCLOSURE_IDENTIFY_BIT					0b10000000

#define ADC_CHANNELS_COUNT							3
#define ADC_ADDRESS_JUMPER_THRESHOLD_1				684													// Threshold between 0 and 1.1 volts
#define ADC_ADDRESS_JUMPER_THRESHOLD_2				2048												// Threshold between 1.1 and 2.2 volts
#define ADC_ADDRESS_JUMPER_THRESHOLD_3				3414												// Threshold between 2.2 and 3.3 volts
#define EMA_SMOOTHING_FACTOR						3													// Exponential Moving Average

#define I2C_SLAVE_ADDRESS_1							0xC0												// Default addresses for I2C backplanes
#define I2C_SLAVE_ADDRESS_2							0xC2
#define I2C_SLAVE_ADDRESS_3							0xD2
#define I2C_SLAVE_ADDRESS_4							0xE6												// You can also use one more address: 0xE8

#define TEMP30_CAL_ADDR								((uint16_t*) ((uint32_t)0x1FFFF7B8))				// 30 degrees calibration constant
#define TEMP110_CAL_ADDR							((uint16_t*) ((uint32_t)0x1FFFF7C2))				// 110 degrees calibration constant
#define VREFINT_CAL_ADDR							((uint16_t*) ((uint32_t)0x1FFFF7BA))				// Vref calibration constant

#define DEMO_RUNNING_LIGHT_LED_ON_PERIOD			4
#define DEMO_RUNNING_LIGHT_REPEATS					4
#define INFINITE_DEMO_RUNNING_LIGHT_REPEATS			8
#define DEMO_ALL_BLINKING_PERIOD					40
#define DEMO_ALL_BLINKING_REPEATS					2
#define INFINITE_DEMO_ALL_BLINKING_REPEATS			4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

// Constants
const SEP_Identifier deviceDescription = {SEP_IDENTIFIER_SIZE, SES_SUB_ENCLOSURE_IDENTIFIER, {"09112023", "MURAVYEV", "INDICATOR BOARD ", "1.00"}, SEP_CHANNEL_IDENTIFIER, "0001", "S-E-S ", "2.00", "STM32F070  "};
const SES_Supported_Pages supportedPages = {SES_SUPPORTED_PAGES_PAGE, 0x00, __builtin_bswap16(SUPPORTED_PAGES_COUNT), {0x00, 0x01, 0x02, 0x07}};
const SES_Configuration configuration = {SES_CONFIGURATION_PAGE, SES_NUMBER_OF_SUBENCLOSURES, __builtin_bswap16(SES_CONFIGURATION_PAGE_SIZE), SES_GENERATION, SES_PROCESS, SES_SUB_ENCLOSURE_IDENTIFIER, CONTROL_ELEMENTS_COUNT, SES_PRODUCT_STRINGS_SIZE};
const SES_Control_Element controlElements[CONTROL_ELEMENTS_COUNT] = {{SES_ARRAY_ELEMENT, 0x04, 0x00, 0x00}, {SES_ENCLOSURE_ELEMENT, 0x01, 0x00, 0x00}, {SES_TEMP_SENSOR_ELEMENT, 0x01, 0x00, 0x00}, {SES_VOLT_SENSOR_ELEMENT, 0x01, 0x00, 0x00}};
const char driveDescriptor[DESCRIPTOR_TEXT_LENGTH] = "DRIVE 00";
const char enclosureDescriptor[DESCRIPTOR_TEXT_LENGTH] = "BOARD 01";
const char tsensorDescriptor[DESCRIPTOR_TEXT_LENGTH] = "MCU TEMP";
const char vsensorDescriptor[DESCRIPTOR_TEXT_LENGTH] = "MCU VDD ";

#ifdef INVERTED_DRIVES_DIRECTION
	const uint16_t drivePins[4] = {DRIVE3_Pin, DRIVE2_Pin, DRIVE1_Pin, DRIVE0_Pin};
	#ifdef ENABLE_STARTUP_DEMO
		const uint8_t demoLedsSequence[LEDS_COUNT] = {1, 0, 2, 4, 3, 5, 7, 6, 8, 10, 9, 11};
	#endif
#else
	const uint16_t drivePins[4] = {DRIVE0_Pin, DRIVE1_Pin, DRIVE2_Pin, DRIVE3_Pin};
	#ifdef ENABLE_STARTUP_DEMO
		const uint8_t demoLedsSequence[LEDS_COUNT] = {2, 0, 1, 5, 3, 4, 8, 6, 7, 11, 9, 10};
	#endif
#endif


// Global variables
volatile Drive drives[SUPPORTED_DRIVES_COUNT] = {0};		// LEDS array for drives
volatile uint8_t buffer[BUFFER_SIZE];						// Common buffer

volatile Element_Status enclosureStatus = {0};
volatile Element_Status drivesStatus[SUPPORTED_DRIVES_COUNT] = {0};

volatile uint8_t selectedInterface = InterfaceAuto;
volatile uint8_t activeInterface = InterfaceUnknown;

volatile uint8_t SGPIOposition = 0;
volatile uint8_t SGPIOframeStarted = FALSE;

volatile uint8_t i2cSlaveAddress = I2C_SLAVE_ADDRESS_1;
volatile uint8_t i2cLineState = StateHold;					// Additional I2C lines state: Hold = Reset | !Address
volatile uint8_t i2cTransmitionState = StateListening;
volatile uint8_t i2cBytesReceived = 0;

volatile uint32_t lastDataReceived;							// The time when the data was last received
volatile uint8_t fastBlinkSource = FALSE;					// Fast blinking flag for the High Severity
volatile uint8_t slowBlinkSource = FALSE;					// Slow blinking flag for the Low Severity
volatile uint8_t fastToSlowBlinkCounter = 0;
volatile uint8_t lastBlinkingState = FALSE;					// Variable is used to reset the TIM16 counter so that when an error occurs, the LED flashes a full cycle

volatile uint8_t demoStep = NoAction;
uint8_t demoStepCounter = 0;
uint8_t demoActionCounter = 0;
uint8_t demoCurrentLed = 1;
int8_t demoLedIncrement = -1;

volatile union {
	adc_data data;
	uint16_t array[ADC_CHANNELS_COUNT];
} adcRaw = {0};

volatile union {
	adc_data data;
	uint16_t array[ADC_CHANNELS_COUNT];
} adcFiltered = {0};

uint32_t lastLedSwitchTick;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */


void SGPIO_Init(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Calculate CRC over the buffer - 0x00 is correct
uint8_t calculateCRC(uint8_t firstValue, volatile uint8_t * data, uint8_t count) {
	while (count) {
		firstValue += data[--count];
	}
	return (-firstValue);
}


// Reliable pin reading to avoid bouncing
GPIO_PinState debouncedReadPin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin) {

	uint8_t numberOfSetValues = 0;
	for (uint8_t i = 0; i < DEBOUNCING_READINGS_COUNT; i++) {
		if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != GPIO_PIN_RESET) numberOfSetValues++;
	}

	if (numberOfSetValues >= (DEBOUNCING_READINGS_COUNT / 2)) return (GPIO_PIN_SET); else return (GPIO_PIN_RESET);

}


// Get highest severity and update corresponding variable
uint8_t updateSeverity(volatile uint8_t * variable, uint8_t severity) {

	uint8_t blinkingState = FALSE;

	if (severity & CriticalSeverity) {
		*variable = TRUE;
	} else
	if (severity & HighSeverity) {
		*variable = fastBlinkSource;
		blinkingState = TRUE;
	} else
	if (severity & LowSeverity) {
		*variable = slowBlinkSource;
		blinkingState = TRUE;
	} else {
		*variable = FALSE;
	}

	return (blinkingState);

}


// Updating the status of Error/Identify LEDs according to their severities
void updateSeverityLeds(uint8_t resetTimer) {

	// Reset blinking timer if there was no errors before
	if (resetTimer && !lastBlinkingState) {
		TIM16->CNT = 0;
		fastBlinkSource = TRUE;
		slowBlinkSource = TRUE;
	}

	// Calculate Error/Identify Severities and update LEDs
	uint8_t newBlinkingState = FALSE;

	for (uint8_t i = 0; i < SUPPORTED_DRIVES_COUNT; i++) {
		uint8_t currentDriveAlarmSeverity = enclosureStatus.alarm_severity | drivesStatus[i].alarm_severity;
		uint8_t currentDriveIdentifySeverity = enclosureStatus.identify_severity | drivesStatus[i].identify_severity;

		// Update Error Severity
		newBlinkingState |= updateSeverity(&drives[i].ERROR, currentDriveAlarmSeverity);

		// Update Identify Severity
		newBlinkingState |= updateSeverity(&drives[i].IDENTIFY, currentDriveIdentifySeverity);

	}

	// Check if we need blinking timer interrupts
	if (newBlinkingState) {
		if (!lastBlinkingState) HAL_TIM_Base_Start_IT(&htim16);					// TIM16 is responsible for the synchronous blinking of the Error LEDs
	} else {
		if (lastBlinkingState) HAL_TIM_Base_Stop_IT(&htim16);
	}

	lastBlinkingState = newBlinkingState;

}


// Clean LEDs, Drives and Enclosure data
void resetDevicesData() {

	memset((uint8_t*)&enclosureStatus, 0, sizeof(Element_Status));
	memset((uint8_t*)drivesStatus, 0, SUPPORTED_DRIVES_COUNT * sizeof(Element_Status));
	memset((uint8_t*)drives, 0, LEDS_COUNT);

}


// Restart I2C listening in slave mode
void i2cRestartListening() {

	i2cTransmitionState = StateListening;
	i2cBytesReceived = 0;
	HAL_I2C_Slave_Receive_DMA(&hi2c1,  (uint8_t *)buffer, BUFFER_SIZE);			// Start listening with DMA

}


// Read I2C additional lines state (Reset/Address)
void i2cUpdateLineState() {

#ifdef I2C_ENABLE_RESET_DETECTION
	GPIO_PinState reset_state = debouncedReadPin(SDATAOUT_RESET_GPIO_Port, SDATAOUT_RESET_Pin);
#else
	GPIO_PinState reset_state = GPIO_PIN_SET;
#endif


#ifdef I2C_ENABLE_ADDRESS_DETECTION
	GPIO_PinState address_state = debouncedReadPin(SDATAIN_ADDRESS_GPIO_Port, SDATAIN_ADDRESS_Pin);
#else
	GPIO_PinState address_state = GPIO_PIN_SET;
#endif

	// Check if we can use I2C communication at the moment
	if ((reset_state != GPIO_PIN_RESET) && (address_state != GPIO_PIN_RESET)) {

		i2cLineState = StateActive;

	} else {

		i2cLineState = StateHold;

	}

}


// Deinitialize I2C module
void i2cDeInit() {

	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_RESET) HAL_I2C_DeInit(&hi2c1);						// In DMA mode only full reinitialization works well (not sure about other modes)
//	__HAL_I2C_DISABLE(&hi2c1);
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C1_FORCE_RESET();

}


// Reinitialize I2C module
void i2cReInit() {

	__HAL_RCC_I2C1_RELEASE_RESET();
	__HAL_RCC_I2C1_CLK_ENABLE();
//	__HAL_I2C_ENABLE(&hi2c1);
	if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET) MX_I2C1_Init();								// In DMA mode only full reinitialization works well (not sure about other modes)
	i2cRestartListening();

}


// Check interface, selected by the controller (in case if Protocol Auto Detection option selected by the board's jumper)
void checkSelectedInterface(InterfaceSelection selectedInterface) {

	if (selectedInterface == InterfaceAuto) {
		// Get CONTROLLER TYPE signal state
		if (debouncedReadPin(CONTROLLER_TYPE_GPIO_Port, CONTROLLER_TYPE_Pin) != GPIO_PIN_RESET) {
			selectedInterface = InterfaceI2C;
		} else {
			selectedInterface = InterfaceSGPIO;
		}
	}

	if (selectedInterface != activeInterface) {

		resetDevicesData();									// Clean state of the devices and LEDs before switching to another protocol

		if (selectedInterface == InterfaceI2C) {

			lastBlinkingState = FALSE;

			// Read I2C Address Jumper state
			uint16_t addressJumperVoltage = adcFiltered.data.address_select_voltage >> 4;
			if (addressJumperVoltage < ADC_ADDRESS_JUMPER_THRESHOLD_1) {
				// Option 3
				i2cSlaveAddress = I2C_SLAVE_ADDRESS_3;
			} else if ((addressJumperVoltage >= ADC_ADDRESS_JUMPER_THRESHOLD_1) && (addressJumperVoltage < ADC_ADDRESS_JUMPER_THRESHOLD_2)) {
				// Option 2
				i2cSlaveAddress = I2C_SLAVE_ADDRESS_2;
			} else if ((addressJumperVoltage >= ADC_ADDRESS_JUMPER_THRESHOLD_2) && (addressJumperVoltage < ADC_ADDRESS_JUMPER_THRESHOLD_3)) {
				// Option 1
				i2cSlaveAddress = I2C_SLAVE_ADDRESS_1;
			} else {
				// Option 4
				i2cSlaveAddress = I2C_SLAVE_ADDRESS_4;
			}

			MX_I2C1_Init();
			i2cUpdateLineState();

			if (i2cLineState == StateActive) {

				i2cReInit();

			} else {

				i2cDeInit();

			}
			HAL_GPIO_WritePin(BACKPLANE_TYPE_GPIO_Port, BACKPLANE_TYPE_Pin, GPIO_PIN_SET);

		} else {

			SGPIOposition = 0;
			SGPIOframeStarted = 0;
			SGPIO_Init();
			HAL_GPIO_WritePin(BACKPLANE_TYPE_GPIO_Port, BACKPLANE_TYPE_Pin, GPIO_PIN_RESET);

		}

		activeInterface = selectedInterface;
		lastDataReceived = HAL_GetTick();

	}

}


// Process I2C input buffer
void processBuffer() {

	SEP_Receive_Diagnostic_Request * request = (SEP_Receive_Diagnostic_Request *)buffer;
	if ((i2cBytesReceived >= SEP_COMMON_REQUEST_SIZE) && (calculateCRC(i2cSlaveAddress, (uint8_t *)request, SEP_HEADER_CMD_TYPE_W_CHECKSUM_SIZE) == 0) && (calculateCRC(0, &(request->HEADER.ADDRESS), i2cBytesReceived - SEP_HEADER_CMD_TYPE_W_CHECKSUM_SIZE) == 0)) {

		lastDataReceived = HAL_GetTick();

		uint8_t protocolStatus = SEP_OK_STATUS;
		uint8_t answerSize = SEP_COMMON_ANSWER_SIZE;

		uint8_t cmdType = request->HEADER.CMD_TYPE;
		uint8_t cmd = request->HEADER.CMD;
		uint8_t address = request->HEADER.ADDRESS;
		uint8_t seq = request->HEADER.SEQ;

		switch (cmdType) {

			case SEP_TO_HOST_COMMAND_TYPE: {				// Status diagnostic page request

				switch (cmd) {

					case SEP_IDENTIFY_PAGE: {

						SEP_Idetifier_Answer * answer = (SEP_Idetifier_Answer *)buffer;
						memcpy(&(answer->INENTIFIER), &deviceDescription, SEP_IDENTIFIER_SIZE);

						answerSize = SEP_IDENTIFIER_ANSWER_SIZE;

					} break;


					case SES_SUPPORTED_PAGES_PAGE: {

						SEP_Supported_Pages_Answer * answer = (SEP_Supported_Pages_Answer *)buffer;
						memcpy(&(answer->SUPPORTED_PAGES), &supportedPages, SES_SUPPORTED_PAGES_SIZE);

						answerSize = SES_SUPPORTED_PAGES_ANSWER_SIZE;

					} break;


					case SES_CONFIGURATION_PAGE: {

						SEP_Configuration_Answer * answer = (SEP_Configuration_Answer *)buffer;
						memcpy(&(answer->CONFIGURATION), &configuration, SES_CONFIGURATION_SIZE);
						memcpy(&(answer->PRODUCT_STRINGS), &(deviceDescription.PRODUCT_STRINGS), SES_PRODUCT_STRINGS_SIZE);
						memcpy(&(answer->CONTROL_ELEMENTS), &controlElements, SES_CONTROL_ELEMENTS_SIZE);

						answerSize = SEP_CONFIGURATION_ANSWER_SIZE;

					} break;


					case SES_STATUS_PAGE: {

						SEP_Status_Answer * answer = (SEP_Status_Answer *)buffer;
						memset(answer, 0, SEP_STATUS_ANSWER_SIZE);

						answer->PAGE_CODE = SES_STATUS_PAGE;
						answer->CONDITIONS = SES_ENCLOSURE_STATUS_PAGE_CONDITIONS;
						answer->PAGE_LENGTH = __builtin_bswap16(SES_STATUS_SIZE);
						answer->STATUS_PAGE.GENERATION_CODE = configuration.GENERATION_CODE;

						for (uint8_t i = 0; i < SUPPORTED_DRIVES_COUNT; i++) {
							answer->STATUS_PAGE.DRIVES[i] = drivesStatus[i].SES_STATUS;
						}

						answer->STATUS_PAGE.ENCLOSURE = enclosureStatus.SES_STATUS;

						// MCU temperature
						//
						// This is based on a standard example of temperature measurement, adjusted by MCU supply voltage and taking into account all applicable coefficients.
						//
						int32_t MCUTemp;
						MCUTemp = (adcFiltered.data.mcu_temperature * (int32_t)*VREFINT_CAL_ADDR / adcFiltered.data.vref_voltage - (int32_t)*TEMP30_CAL_ADDR) << 4;
						MCUTemp = MCUTemp * (int32_t)(110 - 30);
						MCUTemp = MCUTemp / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
						MCUTemp += (50 << 4); // + 30 + 20 (20 degree is for SES protocol)
						if ((MCUTemp & 0x0F) >= 8) MCUTemp += (1 << 4);
						MCUTemp >>= 4;
						if (MCUTemp < 0) MCUTemp = 0; else if (MCUTemp > 0xFF) MCUTemp = 0xFF;

						answer->STATUS_PAGE.TSENSOR.COMMON = SES_ELEMENT_OK_STATUS;
						answer->STATUS_PAGE.TSENSOR.CUSTOM_2 = MCUTemp;


						// MCU power voltage
						//
						// MCU Vcc = (Vcal/Vref) * 3.3
						//
						// All adcFiltered values are shifted left by 4 bits (equivalent to multiplying by 16).
						// So, we have to multiply result by 16 to compensate adcFiltered.data.vref_voltage coefficient
						// Also we have to multiply result by 100, because of SES voltage format.
						// The sign can be ignored, because we have to deal with positive values only.
						//
						// 3.3 * 100 = 330
						// Vcc = (Vcal * 16 * 330) / adcFiltered.data.vref_voltage = (Vcal * 5280) / adcFiltered.data.vref_voltage
						//
						int32_t voltage = (int32_t)*VREFINT_CAL_ADDR * 5280 / adcFiltered.data.vref_voltage;
						answer->STATUS_PAGE.VSENSOR.COMMON = SES_ELEMENT_OK_STATUS;
						answer->STATUS_PAGE.VSENSOR.CUSTOM_2 = (voltage >> 8) & 0xFF;
						answer->STATUS_PAGE.VSENSOR.CUSTOM_3 = voltage & 0xFF;

						answerSize = SEP_STATUS_ANSWER_SIZE;

					} break;


					case SES_ELEMENT_DESCRIPTOR_PAGE: {

						SEP_Elements_Descriptor_Answer * answer = (SEP_Elements_Descriptor_Answer *)buffer;
						memset(answer, 0, SEP_ELEMENTS_DESCRIPTOR_ANSWER_SIZE);

						answer->PAGE_CODE = SES_ELEMENT_DESCRIPTOR_PAGE;
						answer->PAGE_LENGTH = __builtin_bswap16(SES_ELEMENTS_DESCRIPTOR_SIZE);
						answer->DESCRIPTORS.GENERATION_CODE = configuration.GENERATION_CODE;

						for (uint8_t i = 0; i < SUPPORTED_DRIVES_COUNT; i++) {
							answer->DESCRIPTORS.DRIVES[i].DESCRIPTOR.DESCRIPTOR_LENGTH = __builtin_bswap16(DESCRIPTOR_TEXT_LENGTH);
							memcpy(&(answer->DESCRIPTORS.DRIVES[i].DESCRIPTOR_TEXT), driveDescriptor, DESCRIPTOR_TEXT_LENGTH);
							uint8_t tens = i / 10;
							uint8_t ones = i - tens * 10;
							answer->DESCRIPTORS.DRIVES[i].DESCRIPTOR_TEXT[DESCRIPTOR_TEXT_LENGTH - 1] = '0' + ones;
							answer->DESCRIPTORS.DRIVES[i].DESCRIPTOR_TEXT[DESCRIPTOR_TEXT_LENGTH - 2] = '0' + tens;
						}

						answer->DESCRIPTORS.ENCLOSURE.DESCRIPTOR.DESCRIPTOR_LENGTH = __builtin_bswap16(DESCRIPTOR_TEXT_LENGTH);
						memcpy(&(answer->DESCRIPTORS.ENCLOSURE.DESCRIPTOR_TEXT), enclosureDescriptor, DESCRIPTOR_TEXT_LENGTH);

						answer->DESCRIPTORS.TSENSOR.DESCRIPTOR.DESCRIPTOR_LENGTH = __builtin_bswap16(DESCRIPTOR_TEXT_LENGTH);
						memcpy(&(answer->DESCRIPTORS.TSENSOR.DESCRIPTOR_TEXT), tsensorDescriptor, DESCRIPTOR_TEXT_LENGTH);

						answer->DESCRIPTORS.VSENSOR.DESCRIPTOR.DESCRIPTOR_LENGTH = __builtin_bswap16(DESCRIPTOR_TEXT_LENGTH);
						memcpy(&(answer->DESCRIPTORS.VSENSOR.DESCRIPTOR_TEXT), vsensorDescriptor, DESCRIPTOR_TEXT_LENGTH);

						answerSize = SEP_ELEMENTS_DESCRIPTOR_ANSWER_SIZE;

					} break;


					default: protocolStatus = SEP_ERROR_STATUS;

				}

			} break;


			case HOST_TO_SEP_COMMAND_TYPE: {				// Update status (Control) request

				switch (cmd) {

					case SES_STATUS_PAGE: {

						if (demoStep) {
							demoStep = NoAction;
							memset((uint8_t*)drives, 0, LEDS_COUNT);
						}

						SEP_Status_Control * control = (SEP_Status_Control *)buffer;

						uint8_t overallActivity = FALSE;

						if (control->STATUS_PAGE.ENCLOSURE.COMMON & SES_COMMON_SELECT_BIT) {
							// Save Enclosure Status
							enclosureStatus.SES_STATUS = control->STATUS_PAGE.ENCLOSURE;
							enclosureStatus.SES_STATUS.COMMON &= ~SES_COMMON_SELECT_BIT;

							// Reset Enclosure Alarm Severity
							enclosureStatus.alarm_severity = OkSeverity;
							// If Enclosure Error then set Alarm Severity to Critical
							if (enclosureStatus.SES_STATUS.CUSTOM_3 & SES_ENCLOSURE_ERROR_BIT) enclosureStatus.alarm_severity |= CriticalSeverity;
							// If Enclosure Failure predicted or Warning detected then set Alarm Severity to Low Level
							if ((enclosureStatus.SES_STATUS.CUSTOM_3 & SES_ENCLOSURE_WARNING_BIT) | (enclosureStatus.SES_STATUS.COMMON & SES_COMMON_PREDICTED_FAILURE_BIT)) enclosureStatus.alarm_severity |= LowSeverity;

							// Set Enclosure Identification
							if (enclosureStatus.SES_STATUS.CUSTOM_1 & SES_ENCLOSURE_IDENTIFY_BIT) enclosureStatus.identify_severity = CriticalSeverity; else enclosureStatus.identify_severity = OkSeverity;

						}

						for (uint8_t i = 0; i < SUPPORTED_DRIVES_COUNT; i++) {

							if (control->STATUS_PAGE.DRIVES[i].COMMON & SES_COMMON_SELECT_BIT) {
								// Save Drive Status
								drivesStatus[i].SES_STATUS = control->STATUS_PAGE.DRIVES[i];
								drivesStatus[i].SES_STATUS.COMMON &= ~SES_COMMON_SELECT_BIT;

								// Reset Drive Alarm and Identify Severities
								drivesStatus[i].alarm_severity = OkSeverity;
								drivesStatus[i].identify_severity = OkSeverity;
								// If Drive in failed Array then set Alarm Severity to Critical
								if ((drivesStatus[i].SES_STATUS.CUSTOM_1 & SES_ARRAY_ELEMENT_IN_FAILED_ARRAY_BIT) | (drivesStatus[i].SES_STATUS.CUSTOM_3 & SES_ARRAY_ELEMENT_FAULT_BIT)) drivesStatus[i].alarm_severity |= CriticalSeverity;
								// If Drive in critical Array or Rebuild/Remap Aborted then set Alarm Severity to High Level
								if (drivesStatus[i].SES_STATUS.CUSTOM_1 & (SES_ARRAY_ELEMENT_IN_CRITICAL_ARRAY_BIT | SES_ARRAY_ELEMENT_R_R_ABORT_BIT)) drivesStatus[i].alarm_severity |= HighSeverity;
								// If Drive Failure predicted or Rebuild/Remap processing then set Alarm Severity to Low Level
								if (drivesStatus[i].SES_STATUS.COMMON & SES_COMMON_PREDICTED_FAILURE_BIT) drivesStatus[i].alarm_severity |= LowSeverity;

								// If the Drive needs to be removed or inserted then set Identify Severity to Low Level
								if (drivesStatus[i].SES_STATUS.CUSTOM_2 & SES_ARRAY_ELEMENT_REMOVE_INSERT_BITS) drivesStatus[i].identify_severity |= LowSeverity;
								// If drive in Rebuild state then Identify it
								if (drivesStatus[i].SES_STATUS.CUSTOM_1 & SES_ARRAY_ELEMENT_REBUILD_REMAP_BIT) drivesStatus[i].identify_severity |= HighSeverity;
								// Check if Identify requested
								if (drivesStatus[i].SES_STATUS.CUSTOM_2 & SES_ARRAY_ELEMENT_IDENTIFY_BIT) drivesStatus[i].identify_severity |= CriticalSeverity;

								// Set Drive Active state
								uint8_t localActivity = ((drivesStatus[i].SES_STATUS.CUSTOM_2 & SES_ARRAY_ELEMENT_ACTIVE_BIT) != 0);
								drives[i].ACTIVE = localActivity;
								overallActivity |= localActivity;
							}

						}

						updateSeverityLeds(TRUE);			// Update Error and Identify LEDs
						if (overallActivity && (htim14.State != HAL_TIM_STATE_BUSY)) HAL_TIM_Base_Start_IT(&htim14);				// It is necessary to control the lighting duration of the Activity LEDs

						answerSize = SEP_COMMON_ANSWER_SIZE;

					} break;


					default: protocolStatus = SEP_ERROR_STATUS;

				}


			} break;


			default: protocolStatus = SEP_ERROR_STATUS;

		}

		// Send answer
		SEP_Common_Answer * answer = (SEP_Common_Answer *)buffer;
		answer->HEADER.CMD_TYPE = cmdType;
		answer->HEADER.CHECKSUM = calculateCRC(address, &(answer->HEADER.CMD_TYPE), SEP_HEADER_CMD_TYPE_SIZE);
		answer->HEADER.ADDRESS = i2cSlaveAddress;
		answer->HEADER.SEQ = seq;
		answer->HEADER.CMD = cmd;
		answer->STATUS = protocolStatus;
		buffer[answerSize - SEP_CHECKSUM_SIZE] = calculateCRC(0x00, &(buffer[SEP_HEADER_CMD_TYPE_W_CHECKSUM_SIZE]), answerSize - SEP_HEADER_CMD_TYPE_W_CHECKSUM_SIZE - SEP_CHECKSUM_SIZE);
		HAL_I2C_Master_Transmit_DMA(&hi2c1, address,  (uint8_t *)buffer, answerSize);

	} else {

		// Restart listening in case of incorrect checksum (no answer)
		i2cRestartListening();

	}

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
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


  	// Init EXT Interrupts for automatic Interface selection, SGPIO mode input sync and I2C mode Reset/Address detection
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Start Timers, DMA and prepare ADC data
  	HAL_ADCEx_Calibration_Start(&hadc);
  	HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adcRaw, ADC_CHANNELS_COUNT);
  	HAL_TIM_Base_Start(&htim1);

  	// Set Initial values for faster EMA results
  	while (adcRaw.data.vref_voltage == 0);
  	for (uint8_t i = 0; i < ADC_CHANNELS_COUNT; i++) {
  		adcFiltered.array[i] = adcRaw.array[i] << 4;
  	}

  	// Detect selected Interface Type
  	GPIO_InitTypeDef GPIO_InitStruct = {0};
  	/*Configure GPIO pin : BPTYPE_SELECT_Pin */
  	GPIO_InitStruct.Pin = BPTYPE_SELECT_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(BPTYPE_SELECT_GPIO_Port, &GPIO_InitStruct);
  	HAL_Delay(MINIMAL_DELAY);
  	uint8_t pullupValue = HAL_GPIO_ReadPin(BPTYPE_SELECT_GPIO_Port, BPTYPE_SELECT_Pin);
  	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  	HAL_GPIO_Init(BPTYPE_SELECT_GPIO_Port, &GPIO_InitStruct);
  	HAL_Delay(MINIMAL_DELAY);
  	uint8_t pulldownValue = HAL_GPIO_ReadPin(BPTYPE_SELECT_GPIO_Port, BPTYPE_SELECT_Pin);
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	HAL_GPIO_Init(BPTYPE_SELECT_GPIO_Port, &GPIO_InitStruct);

  	if (pullupValue == GPIO_PIN_RESET) {
  		selectedInterface = InterfaceI2C;
  	} else if (pulldownValue != GPIO_PIN_RESET) {
  		selectedInterface = InterfaceSGPIO;
  	}

  	checkSelectedInterface(selectedInterface);

	/*Configure GPIO pin : CONTROLLER_TYPE_Pin */
	GPIO_InitStruct.Pin = CONTROLLER_TYPE_Pin;

  	if (selectedInterface == InterfaceAuto) {
  		// Add EXTI to current configuration of CONTROLLER_TYPE_Pin to detect changes automatically
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
  	} else {
  		// Configure CONTROLLER_TYPE_Pin to ignore signal
  		HAL_GPIO_DeInit(CONTROLLER_TYPE_GPIO_Port, CONTROLLER_TYPE_Pin);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
  	}

	HAL_GPIO_Init(CONTROLLER_TYPE_GPIO_Port, &GPIO_InitStruct);


  	// Start Demo if configured
  #ifdef ENABLE_STARTUP_DEMO
  	((uint8_t *)drives)[demoLedsSequence[0]] = TRUE;
  	demoStep = RunningLight;
  	HAL_TIM_Base_Start_IT(&htim14);							// TIM14 is responsible for the lighting duration of the Activity LEDs and for the Demo mode
  #endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	uint8_t currentDrive = 0;
#ifdef SMALL_LED_CONTROL_NET_SPEED
	uint8_t lastDrive = 0;
#endif

	lastLedSwitchTick = HAL_GetTick();

	while (TRUE) {

		// Disable interrupts
		__disable_irq();

		// Turn off all LEDs rows (for all drives)
		HAL_GPIO_WritePin(GPIOA, DRIVE0_Pin | DRIVE1_Pin | DRIVE2_Pin | DRIVE3_Pin, GPIO_PIN_RESET);

#ifdef SMALL_LED_CONTROL_NET_SPEED
		// Wait until the actual status of the logical control signal reaches zero (in worst cases it also possible to use debouncedReadPin)
//		__enable_irq();										// We don't need to enable interrupts, as the wait is very short, but just in case, I'll leave these lines commented.
		while (HAL_GPIO_ReadPin(GPIOA, drivePins[lastDrive]) != GPIO_PIN_RESET) {};
//		__disable_irq();
		lastDrive = currentDrive;
#endif

		// Set control signals for the current drive
		HAL_GPIO_WritePin(GPIOA, ACTIVITY_Pin, drives[currentDrive].ACTIVE != 0);
		HAL_GPIO_WritePin(GPIOA, IDENTIFY_Pin, drives[currentDrive].IDENTIFY != 0);
		HAL_GPIO_WritePin(GPIOA, ALARM_Pin, drives[currentDrive].ERROR != 0);
		HAL_GPIO_WritePin(GPIOA, drivePins[currentDrive], GPIO_PIN_SET);

		// Enable interrupts
		__enable_irq();

		// Increment drive
		currentDrive++;
		if (currentDrive >= SUPPORTED_DRIVES_COUNT) currentDrive = 0;


		// While the LEDs are lit for a given time, we can do data processing
		if (activeInterface == InterfaceI2C) {

			// Check if we have to process buffer or restart listening
			switch (i2cTransmitionState) {

				case StateDataReceived: {
					// Have some data
					processBuffer();

				} break;


				case StateError: {
					// Need to restart listening
					i2cRestartListening();

				} break;


				default: {}

			}

		}


		// Check if the data has not been received for a long time
		if ((HAL_GetTick() - lastDataReceived > NO_DATA_TIMEOUT)) {

			resetDevicesData();							// At the moment we do not have up-to-date data to display

			// Turn off all LEDs rows (for all drives)
			HAL_GPIO_WritePin(GPIOA, DRIVE0_Pin | DRIVE1_Pin | DRIVE2_Pin | DRIVE3_Pin, GPIO_PIN_RESET);

			// If current interface is I2C - restart it
			if (activeInterface == InterfaceI2C) {
				// Disable EXT interrupts for the case when Reset/Address/ControllerType signals changed while resetting I2C module
				HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
				HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

				// If line active then reset I2C module
				if (i2cLineState == StateActive) {
					i2cDeInit();
					HAL_Delay(MINIMAL_DELAY);
					i2cReInit();
				}

				HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
				HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
			}

			lastDataReceived = HAL_GetTick();

		}


		// This delay is taking into account the time spent on data processing and interrupts
		uint32_t currentTick;
		do {
			currentTick = HAL_GetTick();
		} while ((currentTick - lastLedSwitchTick) < MINIMAL_DELAY);
		lastLedSwitchTick = currentTick;


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

	// Deinitialize pins used for SGPIO
	HAL_GPIO_DeInit(GPIOF, SLOAD_SDA_Pin | SCLOCK_SCL_Pin);
	HAL_GPIO_DeInit(GPIOA, SDATAIN_ADDRESS_Pin | SDATAOUT_RESET_Pin);

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = i2cSlaveAddress;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */


	// Initialize EXTI pins for Reset/Address lines
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	//__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : SDATAIN_ADDRESS_Pin */
	GPIO_InitStruct.Pin = SDATAIN_ADDRESS_Pin;

#ifdef I2C_ENABLE_ADDRESS_DETECTION
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif

	HAL_GPIO_Init(SDATAIN_ADDRESS_GPIO_Port, &GPIO_InitStruct);


	/*Configure GPIO pin : SDATAOUT_RESET_Pin */
	GPIO_InitStruct.Pin = SDATAOUT_RESET_Pin;

#ifdef I2C_ENABLE_RESET_DETECTION
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif

	HAL_GPIO_Init(SDATAOUT_RESET_GPIO_Port, &GPIO_InitStruct);


  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4800-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 4800-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRIVE3_Pin|DRIVE2_Pin|DRIVE1_Pin|DRIVE0_Pin
                          |ALARM_Pin|ACTIVITY_Pin|IDENTIFY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BACKPLANE_TYPE_GPIO_Port, BACKPLANE_TYPE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SDATAIN_ADDRESS_Pin SDATAOUT_RESET_Pin */
  GPIO_InitStruct.Pin = SDATAIN_ADDRESS_Pin|SDATAOUT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRIVE3_Pin DRIVE2_Pin DRIVE1_Pin DRIVE0_Pin
                           ALARM_Pin ACTIVITY_Pin IDENTIFY_Pin */
  GPIO_InitStruct.Pin = DRIVE3_Pin|DRIVE2_Pin|DRIVE1_Pin|DRIVE0_Pin
                          |ALARM_Pin|ACTIVITY_Pin|IDENTIFY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CONTROLLER_TYPE_Pin */
  GPIO_InitStruct.Pin = CONTROLLER_TYPE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONTROLLER_TYPE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BPTYPE_SELECT_Pin */
  GPIO_InitStruct.Pin = BPTYPE_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BPTYPE_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BACKPLANE_TYPE_Pin */
  GPIO_InitStruct.Pin = BACKPLANE_TYPE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BACKPLANE_TYPE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


// Initialize pins for SGPIO mode
void SGPIO_Init(void) {

	// Deinitialize I2C peripherals and timers
	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_RESET) HAL_I2C_DeInit(&hi2c1);
	HAL_TIM_Base_Stop_IT(&htim16);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	//__HAL_RCC_GPIOA_CLK_ENABLE();
	//__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin : SDATAIN_ADDRESS_Pin */
	HAL_GPIO_DeInit(SDATAIN_ADDRESS_GPIO_Port, SDATAIN_ADDRESS_Pin);
	HAL_GPIO_WritePin(SDATAIN_ADDRESS_GPIO_Port, SDATAIN_ADDRESS_Pin, GPIO_PIN_SET);					// Leave Panel's output pin pulled up to VCC by Controller. We're not going to tell the controller anything.
	GPIO_InitStruct.Pin = SDATAIN_ADDRESS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SDATAIN_ADDRESS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SDATAOUT_RESET_Pin */
	HAL_GPIO_DeInit(SDATAOUT_RESET_GPIO_Port, SDATAOUT_RESET_Pin);
	GPIO_InitStruct.Pin = SDATAOUT_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;				// Additionally, we can pull data pins to the GND to avoid a bogus indication, e.g. in case we touched an unplugged Sideband connector with hand.
														// This is not necessary because further there are additional checks of the packets correctness, but it will be a little bit better this way.
	HAL_GPIO_Init(SDATAOUT_RESET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SLOAD_SDA_Pin */
	GPIO_InitStruct.Pin = SLOAD_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SLOAD_SDA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SCLOCK_SCL_Pin */
	GPIO_InitStruct.Pin = SCLOCK_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SCLOCK_SCL_GPIO_Port, &GPIO_InitStruct);

}


// EXTI rising/falling edges handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// SGPIO protocol parser
	if ((GPIO_Pin == SCLOCK_SCL_Pin) && (activeInterface == InterfaceSGPIO)) {

		// Here we don't need any debouncing while reading because it is provided by the protocol
		GPIO_PinState sload_state = HAL_GPIO_ReadPin(SLOAD_SDA_GPIO_Port, SLOAD_SDA_Pin);
		GPIO_PinState sdataout_state = HAL_GPIO_ReadPin(SDATAOUT_RESET_GPIO_Port, SDATAOUT_RESET_Pin);

		if (SGPIOframeStarted && (SGPIOposition < LEDS_COUNT)) {

			if (sdataout_state != GPIO_PIN_RESET) {
				lastDataReceived = HAL_GetTick();
				if (demoStep) {								// If the demo mode is active and one of the indicators is turned on, then the demo ends prematurely.
					memset((uint8_t*)drives, 0, LEDS_COUNT);
					demoStep = NoAction;
				}
			}

			buffer[SGPIOposition] = sdataout_state;			// Save the packet data We can ignore the current state of the indicators while they are off and while the demo mode is active

		}

		SGPIOposition++;

		if (sload_state != GPIO_PIN_RESET) {

			//
			// Perform some checks:
			// - whether the packet was received from its beginning,
			// - is the length of the packet a multiple of the number of LEDs by the Drive,
			// - are we in demo mode?
			//
			// SGPIO is not very reliable protocol, but by performing first 2 checks we can minimize bogus indications due to noisy SCLOCK falling edge.
			// Also we can ignore the current state of the indicators while they are off and while the demo mode is active.
			//
			if (SGPIOframeStarted && (SGPIOposition % LEDS_PER_DRIVE == 0) && !demoStep) {
				if (SGPIOposition > LEDS_COUNT) SGPIOposition = LEDS_COUNT;
				memcpy((uint8_t*)drives, (uint8_t*)buffer, SGPIOposition);
			}
			SGPIOframeStarted = TRUE;
			SGPIOposition = 0;
		}

	} else
	// I2C mode Reset/Address lines control
	if (((GPIO_Pin == SDATAOUT_RESET_Pin) || (GPIO_Pin == SDATAIN_ADDRESS_Pin)) && (activeInterface == InterfaceI2C)) {

		I2C_Line_State oldLineState = i2cLineState;
		i2cUpdateLineState();

		if ((oldLineState == StateActive) && (i2cLineState == StateHold)) {

			i2cDeInit();

		} else
		if ((oldLineState == StateHold) && (i2cLineState == StateActive)) {

			i2cReInit();

		}

	} else
	// Detecting the protocol selected by the RAID controller
	if (GPIO_Pin == CONTROLLER_TYPE_Pin) {

    	checkSelectedInterface(selectedInterface);

	}

}


// We won't be able to receive the entire buffer size (128 bytes) because it's too much for our messages, but just in case I'll leave the handler here.
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef * hi2c) {

	i2cBytesReceived = BUFFER_SIZE;
	i2cTransmitionState = StateDataReceived;

}


// Our answer was successfully transmitted
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c) {

	i2cRestartListening();

}


// Checking for cases when we received an amount of data less than the buffer size, or another error occurred (for example, NACK while sending a response)
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c) {

	// Get received data size
	i2cBytesReceived = BUFFER_SIZE - hdma_i2c1_rx.Instance->CNDTR;					// We can also use I2C_GET_DMA_REMAIN_DATA(hi2c->hdmarx) from stm32f0xx_hal_dma.h

	if ((HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) && (i2cBytesReceived > 0) && (i2cBytesReceived != BUFFER_SIZE)) {
		// Process the buffer in main() function
		i2cTransmitionState = StateDataReceived;

	} else {
		// Restart listening
		i2cTransmitionState = StateError;

	}

}


// Calculate Exponential Moving Average for ADC channels
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	for (uint8_t i = 0; i < ADC_CHANNELS_COUNT; i++) {
		adcFiltered.array[i] += (((adcRaw.array[i] << 4) - adcFiltered.array[i]) >> EMA_SMOOTHING_FACTOR);
	}

}


// Timers handler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM14) {							// TIM14 is responsible for the lighting duration of the Activity LEDs and for the Demo mode

		if (demoStep) {

			switch (demoStep) {								// Process Demo steps

				case RunningLight: {						// Running lights
					demoActionCounter++;
					if (demoActionCounter >= DEMO_RUNNING_LIGHT_LED_ON_PERIOD) {

						demoActionCounter = 0;
						((uint8_t *)drives)[demoLedsSequence[demoCurrentLed]] = FALSE;
						demoCurrentLed += demoLedIncrement;
						if ((demoCurrentLed == 0) || (demoCurrentLed == (LEDS_COUNT - 1))) {
							demoLedIncrement = -demoLedIncrement;
							demoStepCounter++;
#ifdef INFINITE_DEMO
							if (demoStepCounter > INFINITE_DEMO_RUNNING_LIGHT_REPEATS) {
#else
							if (demoStepCounter > DEMO_RUNNING_LIGHT_REPEATS) {
#endif
								demoStepCounter = 0;
								demoStep = AllBlinking;
								memset((uint8_t*)drives, 1, LEDS_COUNT);
								break;
							}
						}

						((uint8_t *)drives)[demoLedsSequence[demoCurrentLed]] = TRUE;
					}
				} break;


				case AllBlinking: {							// Blinking
					demoActionCounter++;
					if (demoActionCounter >= DEMO_ALL_BLINKING_PERIOD) {
						memset((uint8_t*)drives, !drives[0].ACTIVE, LEDS_COUNT);
						demoActionCounter = 0;
						demoStepCounter++;
#ifdef INFINITE_DEMO
						if (demoStepCounter > INFINITE_DEMO_ALL_BLINKING_REPEATS) {
							demoStep = RunningLight;
#else
						if (demoStepCounter > DEMO_ALL_BLINKING_REPEATS) {
							demoStep = NoAction;
#endif
							demoStepCounter = 0;
							memset((uint8_t*)drives, 0, LEDS_COUNT);
						}
					}
				} break;


				default: {}

			}

		} else {

			// Using the Activity LED byte as an incremental counter to determine the period that has elapsed since the last activity update
			uint8_t overallActivity = FALSE;
			for (uint8_t i = 0; i < SUPPORTED_DRIVES_COUNT; i++) {
				if (drives[i].ACTIVE) {
					overallActivity = TRUE;
					if (++drives[i].ACTIVE > LED_ON_STATE_PERIOD) {
						drives[i].ACTIVE = FALSE;														// If there has been no activity for a long time, then turn off the LED
						drivesStatus[i].SES_STATUS.CUSTOM_2 &= ~SES_ARRAY_ELEMENT_ACTIVE_BIT;
					}
				}
			}

		    if (!overallActivity) HAL_TIM_Base_Stop_IT(&htim14);										// If none of the disks is active then stop TIM14 interrupts

		}


    } else if (htim->Instance == TIM16) {					// TIM16 is responsible for the synchronous blinking of the Error LEDs

    	fastBlinkSource = !fastBlinkSource;																// TIM16 by itself is used for fast blinking
    	fastToSlowBlinkCounter++;
    	if (fastToSlowBlinkCounter >= LED_FAST_TO_SLOW_BLINKING_FACTOR) {								// Slow blinking is calculated using a divider
    		fastToSlowBlinkCounter = 0;
    		slowBlinkSource = !slowBlinkSource;
    	}

    	updateSeverityLeds(FALSE);							// Update Error (and in some cases Identify) LEDs

    }

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
