/* HLK-FPM383C Fingerprint Controller
 * STM32F446RE
 *
 * USART1: PA9 TX, PA10 RX, 57600 baud
 * USART2: PA2 TX, PA3 RX, 115200 baud
 * TOUCHOUT: PC0 input
 *
 * Commands:
 * HELP
 * ENROLL 1
 * ENROLL 2
 * ADD
 * MATCH
 * MATCH 1
 * MATCH 2
 * DELETE 1
 * COUNT
 * STATUS
 */

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);

static void FP_App_Init(void);
static void FP_App_Task(void);
static void FP_Enroll(uint16_t id);
static void FP_Match(int32_t requested_id);
static void FP_Delete(uint16_t id);
static void FP_Add(void);
static void FP_PrintCount(void);
static void FP_PrintHelp(void);
static void FP_ProcessCommand(char *line);

#define FP_MAX_TEMPLATES          60U
#define FP_MIN_ID                 0U
#define FP_MAX_ID                 59U
#define FP_ENROLL_CAPTURE_COUNT   6U
#define FP_ENROLL_TIMEOUT_MS      120000UL
#define FP_RX_APP_MAX             128U
#define CONSOLE_LINE_MAX          64U
#define FP_PASSWORD               0x00000000UL

#define FP_CAT_FINGERPRINT        0x01U
#define FP_CAT_SYSTEM             0x02U
#define FP_CAT_MAINTENANCE        0x03U

#define FP_CMD_AUTO_ENROLL        0x18U
#define FP_CMD_MATCH_SYNC         0x23U
#define FP_CMD_DELETE             0x31U
#define FP_CMD_CHECK_ID_EXIST     0x33U
#define FP_CMD_GET_TEMPLATE_COUNT 0x03U
#define FP_CMD_HEARTBEAT          0x03U

#define FP_ERR_SUCCESS            0x00000000UL
#define FP_ERR_TIMEOUT            0x00000008UL
#define FP_ERR_EXTRACTION         0x00000009UL
#define FP_ERR_TEMPLATE_EMPTY     0x0000000AUL
#define FP_ERR_STORAGE_FULL       0x0000000BUL
#define FP_ERR_POOR_IMAGE         0x0000000EUL
#define FP_ERR_DUPLICATE          0x0000000FUL

static const uint8_t fp_header[8] =
{
    0xF1U, 0x1FU, 0xE2U, 0x2EU,
    0xB6U, 0x6BU, 0xA8U, 0x8AU
};

typedef enum
{
    FP_RX_ERROR = -1,
    FP_RX_TIMEOUT = 0,
    FP_RX_OK = 1
} FP_RxStatus;

static char console_line[CONSOLE_LINE_MAX];
static uint16_t console_line_len = 0U;
static GPIO_PinState previous_touch_state = GPIO_PIN_RESET;

/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    /* Allow the fingerprint module time to initialize. */
    HAL_Delay(250U);

    FP_App_Init();

    while (1)
    {
        FP_App_Task();
    }
}

/* =========================================================
   GPIO INITIALIZATION
   ========================================================= */

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* USART1: PA9 TX, PA10 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2: PA2 TX, PA3 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Fingerprint sensor TOUCHOUT status */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* =========================================================
   UART INITIALIZATION
   ========================================================= */

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 57600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
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
}

/* =========================================================
   CONSOLE OUTPUT
   ========================================================= */

static void Console_Printf(const char *format, ...)
{
    char buffer[320];
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }

    if ((size_t)length >= sizeof(buffer))
    {
        length = (int)(sizeof(buffer) - 1U);
    }

    (void)HAL_UART_Transmit(
        &huart2,
        (uint8_t *)buffer,
        (uint16_t)length,
        1000U
    );
}

/* =========================================================
   CHECKSUM FUNCTIONS
   ========================================================= */

static uint8_t FP_TwoComplementChecksum(
    const uint8_t *data,
    uint16_t length)
{
    uint32_t sum = 0U;
    uint16_t i;

    for (i = 0U; i < length; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(0U - sum);
}

static uint8_t FP_FrameChecksum(uint16_t frame_length)
{
    uint32_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        sum += fp_header[i];
    }

    sum += (uint8_t)(frame_length >> 8);
    sum += (uint8_t)(frame_length & 0xFFU);

    return (uint8_t)(0U - sum);
}

/* =========================================================
   UART RECEIVE HELPERS
   ========================================================= */

static HAL_StatusTypeDef FP_ReadByte(
    uint8_t *byte,
    uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    HAL_StatusTypeDef status;

    do
    {
        status = HAL_UART_Receive(&huart1, byte, 1U, 1U);

        if (status == HAL_OK)
        {
            return HAL_OK;
        }

        if ((status != HAL_TIMEOUT) && (status != HAL_BUSY))
        {
            return status;
        }

    } while ((HAL_GetTick() - start) < timeout_ms);

    return HAL_TIMEOUT;
}

static int FP_ReadExact(
    uint8_t *buffer,
    uint16_t length,
    uint32_t per_byte_timeout_ms)
{
    uint16_t i;

    for (i = 0U; i < length; i++)
    {
        if (FP_ReadByte(&buffer[i], per_byte_timeout_ms) != HAL_OK)
        {
            return 0;
        }
    }

    return 1;
}

/* =========================================================
   SEND COMMAND
   ========================================================= */

static int FP_SendCommand(
    uint8_t category,
    uint8_t command,
    const uint8_t *data,
    uint16_t data_length)
{
    uint8_t frame[160];
    uint16_t app_length;
    uint16_t total_length;
    uint16_t i;
    uint8_t *app;
    uint32_t password = FP_PASSWORD;

    if (data_length > 120U)
    {
        return 0;
    }

    /*
     * Application:
     * Password(4) + CMD1(1) + CMD2(1) + Data(N) + Checksum(1)
     */
    app_length = (uint16_t)(7U + data_length);
    total_length = (uint16_t)(11U + app_length);

    memcpy(frame, fp_header, sizeof(fp_header));

    frame[8] = (uint8_t)(app_length >> 8);
    frame[9] = (uint8_t)(app_length & 0xFFU);
    frame[10] = FP_FrameChecksum(app_length);

    app = &frame[11];

    app[0] = (uint8_t)(password >> 24);
    app[1] = (uint8_t)(password >> 16);
    app[2] = (uint8_t)(password >> 8);
    app[3] = (uint8_t)password;

    app[4] = category;
    app[5] = command;

    for (i = 0U; i < data_length; i++)
    {
        app[6U + i] = data[i];
    }

    app[app_length - 1U] =
        FP_TwoComplementChecksum(app, (uint16_t)(app_length - 1U));

    return (HAL_UART_Transmit(
        &huart1, frame, total_length, 1000U) == HAL_OK) ? 1 : 0;
}

/* =========================================================
   RECEIVE RESPONSE
   ========================================================= */

static FP_RxStatus FP_ReceiveResponse(
    uint8_t expected_category,
    uint8_t expected_command,
    uint32_t header_timeout_ms,
    uint8_t *response_data,
    uint16_t response_capacity,
    uint16_t *response_length,
    uint32_t *error_code)
{
    uint8_t byte;
    uint8_t header_index = 0U;
    uint8_t len_bytes[3];
    uint8_t app[FP_RX_APP_MAX];

    uint16_t app_length;
    uint16_t body_length;
    uint16_t i;
    uint32_t start = HAL_GetTick();

    if (response_length != NULL)
    {
        *response_length = 0U;
    }

    if (error_code != NULL)
    {
        *error_code = 0xFFFFFFFFUL;
    }

    /* Find the 8-byte frame header. */
    while ((HAL_GetTick() - start) < header_timeout_ms)
    {
        HAL_StatusTypeDef status = FP_ReadByte(&byte, 1U);

        if (status == HAL_OK)
        {
            if (byte == fp_header[header_index])
            {
                header_index++;

                if (header_index == 8U)
                {
                    break;
                }
            }
            else
            {
                header_index =
                    (byte == fp_header[0]) ? 1U : 0U;
            }
        }
        else if ((status != HAL_TIMEOUT) && (status != HAL_BUSY))
        {
            return FP_RX_ERROR;
        }
    }

    if (header_index != 8U)
    {
        return FP_RX_TIMEOUT;
    }

    /* Read application length and frame checksum. */
    if (!FP_ReadExact(len_bytes, 3U, 500U))
    {
        return FP_RX_ERROR;
    }

    app_length = (uint16_t)(
        ((uint16_t)len_bytes[0] << 8) | len_bytes[1]
    );

    if ((app_length < 11U) || (app_length > FP_RX_APP_MAX))
    {
        return FP_RX_ERROR;
    }

    if (len_bytes[2] != FP_FrameChecksum(app_length))
    {
        return FP_RX_ERROR;
    }

    if (!FP_ReadExact(app, app_length, 500U))
    {
        return FP_RX_ERROR;
    }

    /* Verify application checksum. */
    if (app[app_length - 1U] !=
        FP_TwoComplementChecksum(app, (uint16_t)(app_length - 1U)))
    {
        return FP_RX_ERROR;
    }

    /* The response must correspond to the command sent. */
    if ((app[4] != expected_category) ||
        (app[5] != expected_command))
    {
        return FP_RX_ERROR;
    }

    if (error_code != NULL)
    {
        *error_code =
            ((uint32_t)app[6] << 24) |
            ((uint32_t)app[7] << 16) |
            ((uint32_t)app[8] << 8) |
            (uint32_t)app[9];
    }

    body_length = (uint16_t)(app_length - 11U);

    if ((response_data != NULL) &&
        (response_capacity > 0U) &&
        (body_length > 0U))
    {
        uint16_t copy_length =
            (body_length < response_capacity)
            ? body_length : response_capacity;

        for (i = 0U; i < copy_length; i++)
        {
            response_data[i] = app[10U + i];
        }
    }

    if (response_length != NULL)
    {
        *response_length = body_length;
    }

    return FP_RX_OK;
}

/* =========================================================
   SEND COMMAND AND RECEIVE RESPONSE
   ========================================================= */

static FP_RxStatus FP_Transaction(
    uint8_t category,
    uint8_t command,
    const uint8_t *data,
    uint16_t data_length,
    uint32_t response_timeout_ms,
    uint8_t *response_data,
    uint16_t response_capacity,
    uint16_t *response_length,
    uint32_t *error_code)
{
    if (!FP_SendCommand(category, command, data, data_length))
    {
        return FP_RX_ERROR;
    }

    return FP_ReceiveResponse(
        category,
        command,
        response_timeout_ms,
        response_data,
        response_capacity,
        response_length,
        error_code
    );
}

/* =========================================================
   SENSOR ERROR MESSAGES
   ========================================================= */

static const char *FP_ErrorText(uint32_t code)
{
    switch (code)
    {
        case 0x00000000UL:
            return "Success";

        case 0x00000001UL:
            return "Unknown command";

        case 0x00000002UL:
            return "Invalid data length";

        case 0x00000003UL:
            return "Invalid command data";

        case 0x00000004UL:
            return "Sensor busy";

        case 0x00000005UL:
            return "No request sent";

        case 0x00000006UL:
            return "Sensor software error";

        case 0x00000007UL:
            return "Sensor hardware error";

        case 0x00000008UL:
            return "No finger detected before timeout";

        case 0x00000009UL:
            return "Fingerprint feature extraction failed";

        case 0x0000000AUL:
            return "No fingerprint templates stored";

        case 0x0000000BUL:
            return "Fingerprint storage is full";

        case 0x0000000CUL:
            return "Template write failed";

        case 0x0000000DUL:
            return "Template read failed";

        case 0x0000000EUL:
            return "Poor fingerprint image; try again";

        case 0x0000000FUL:
            return "Fingerprint already enrolled";

        case 0x00000010UL:
            return "Finger contact area too small";

        case 0x00000011UL:
            return "Finger moved too far; keep it steady";

        case 0x00000012UL:
            return "Finger movement too small; reposition slightly";

        case 0x00000013UL:
            return "Fingerprint ID is already occupied";

        case 0x00000014UL:
            return "Sensor capture failed";

        case 0x00000015UL:
            return "Command interrupted";

        case 0x00000016UL:
            return "Fingerprint feature update not required";

        case 0x00000017UL:
            return "Invalid fingerprint ID";

        case 0x00000018UL:
            return "Gain adjustment failed";

        case 0x00000019UL:
            return "Sensor data buffer overflow";

        case 0x0000001AUL:
            return "Sensor is asleep";

        case 0x0000001CUL:
            return "Checksum error";

        case 0x00000022UL:
            return "Flash write failed during enrollment";

        default:
            return "Unknown sensor error";
    }
}

/* =========================================================
   READ TEMPLATE COUNT
   ========================================================= */

static int FP_GetTemplateCount(uint16_t *count)
{
    uint8_t data[4];
    uint16_t length = 0U;
    uint32_t error = 0xFFFFFFFFUL;

    FP_RxStatus status = FP_Transaction(
        FP_CAT_SYSTEM,
        FP_CMD_GET_TEMPLATE_COUNT,
        NULL,
        0U,
        2500U,
        data,
        sizeof(data),
        &length,
        &error
    );

    if ((status != FP_RX_OK) ||
        (error != FP_ERR_SUCCESS) ||
        (length < 2U))
    {
        return 0;
    }

    *count = (uint16_t)(
        ((uint16_t)data[0] << 8) | data[1]
    );

    return 1;
}

/* =========================================================
   CHECK WHETHER AN ID IS OCCUPIED
   ========================================================= */

static int FP_CheckIdExists(uint16_t id, uint8_t *exists)
{
    uint8_t payload[2];
    uint8_t response[4];
    uint16_t length = 0U;
    uint32_t error = 0xFFFFFFFFUL;

    FP_RxStatus status;

    payload[0] = (uint8_t)(id >> 8);
    payload[1] = (uint8_t)(id & 0xFFU);

    status = FP_Transaction(
        FP_CAT_FINGERPRINT,
        FP_CMD_CHECK_ID_EXIST,
        payload,
        sizeof(payload),
        2500U,
        response,
        sizeof(response),
        &length,
        &error
    );

    if ((status != FP_RX_OK) ||
        (error != FP_ERR_SUCCESS) ||
        (length < 1U))
    {
        return 0;
    }

    *exists = (response[0] == 1U) ? 1U : 0U;

    return 1;
}

/* =========================================================
   PRINT TEMPLATE COUNT
   ========================================================= */

static void FP_PrintCount(void)
{
    uint16_t count;

    if (FP_GetTemplateCount(&count))
    {
        Console_Printf(
            "Stored fingerprints: %u/%u\r\n",
            (unsigned int)count,
            (unsigned int)FP_MAX_TEMPLATES
        );

        if (count >= FP_MAX_TEMPLATES)
        {
            Console_Printf(
                "FINGERPRINT LIMIT REACHED - storage is full.\r\n"
            );
        }
    }
    else
    {
        Console_Printf(
            "Could not read template count. Check sensor UART/awake state.\r\n"
        );
    }
}

/* =========================================================
   FIND FREE ID
   ========================================================= */

static int FP_FindFreeId(uint16_t *free_id)
{
    uint16_t id;
    uint8_t exists;

    /* Prefer IDs 1..59; try ID 0 last. */
    for (id = 1U; id <= FP_MAX_ID; id++)
    {
        if (!FP_CheckIdExists(id, &exists))
        {
            return 0;
        }

        if (exists == 0U)
        {
            *free_id = id;
            return 1;
        }
    }

    if (!FP_CheckIdExists(0U, &exists))
    {
        return 0;
    }

    if (exists == 0U)
    {
        *free_id = 0U;
        return 1;
    }

    return 2;
}

/* =========================================================
   ENROLL FINGERPRINT
   ========================================================= */

static void FP_Enroll(uint16_t id)
{
    uint16_t count;
    uint8_t exists;
    uint8_t payload[4];
    uint8_t response[16];
    uint16_t response_length;
    uint32_t error;
    uint32_t start;
    uint8_t last_progress = 0xFFU;

    FP_RxStatus status;

    if (id > FP_MAX_ID)
    {
        Console_Printf("Invalid ID. Use an ID from 0 to 59.\r\n");
        return;
    }

    if (!FP_GetTemplateCount(&count))
    {
        Console_Printf(
            "Could not query sensor storage; enrollment cancelled.\r\n"
        );
        return;
    }

    if (count >= FP_MAX_TEMPLATES)
    {
        Console_Printf(
            "FINGERPRINT LIMIT REACHED (%u/%u). Delete an ID first.\r\n",
            (unsigned int)count,
            (unsigned int)FP_MAX_TEMPLATES
        );
        return;
    }

    if (!FP_CheckIdExists(id, &exists))
    {
        Console_Printf(
            "Could not check ID %u; enrollment cancelled.\r\n",
            (unsigned int)id
        );
        return;
    }

    if (exists != 0U)
    {
        Console_Printf(
            "ID %u is already enrolled. Use DELETE %u first or choose another ID.\r\n",
            (unsigned int)id,
            (unsigned int)id
        );
        return;
    }

    /*
     * Auto-enrollment command: CMD1=0x01, CMD2=0x18.
     * payload[0]: 1 means wait for finger lift between captures.
     * payload[1]: capture count, range 1..6.
     * payload[2]: fingerprint ID high byte.
     * payload[3]: fingerprint ID low byte.
     */

    payload[0] = 1U;
    payload[1] = FP_ENROLL_CAPTURE_COUNT;
    payload[2] = (uint8_t)(id >> 8);
    payload[3] = (uint8_t)(id & 0xFFU);

    Console_Printf(
        "Starting enrollment for ID %u.\r\n",
        (unsigned int)id
    );

    Console_Printf(
        "Use the SAME finger. Lift and place it again when prompted.\r\n"
    );

    Console_Printf("Enrollment can take up to 120 seconds.\r\n");

    if (!FP_SendCommand(
            FP_CAT_FINGERPRINT,
            FP_CMD_AUTO_ENROLL,
            payload,
            sizeof(payload)))
    {
        Console_Printf(
            "UART transmit failed while starting enrollment.\r\n"
        );
        return;
    }

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < FP_ENROLL_TIMEOUT_MS)
    {
        response_length = 0U;
        error = 0xFFFFFFFFUL;

        status = FP_ReceiveResponse(
            FP_CAT_FINGERPRINT,
            FP_CMD_AUTO_ENROLL,
            800U,
            response,
            sizeof(response),
            &response_length,
            &error
        );

        if (status == FP_RX_TIMEOUT)
        {
            continue;
        }

        if (status != FP_RX_OK)
        {
            Console_Printf(
                "Invalid or incomplete enrollment response. Check UART and retry.\r\n"
            );
            return;
        }

        if (error == FP_ERR_STORAGE_FULL)
        {
            Console_Printf(
                "FINGERPRINT LIMIT REACHED - sensor reported storage full.\r\n"
            );
            return;
        }

        if (error == FP_ERR_DUPLICATE)
        {
            Console_Printf(
                "DUPLICATE FINGERPRINT: sensor rejected this enrollment.\r\n"
            );
            return;
        }

        
if (error != FP_ERR_SUCCESS)
{
    printf("ENROLLMENT SCAN FAILED: 0x%08lX\r\n",
           (unsigned long)error);

    printf("Please lift your finger, wait briefly, "
           "and place the SAME finger again.\r\n");

    printf("To restart enrollment, enter: ENROLL %u\r\n",
           (unsigned int)id);

    if (response_length > 0U)
    {
        printf("Enrollment response data (%u bytes): ",
               (unsigned int)response_length);

        for (uint16_t i = 0; i < response_length; i++)
        {
            printf("%02X ", response[i]);
        }

        printf("\r\n");
    }

    return;
}


        if (response_length >= 4U)
        {
            uint8_t captures = response[0];
            uint8_t progress = response[3];

            if ((captures == 0xFFU) && (progress >= 100U))
            {
                Console_Printf(
                    "ENROLLMENT SUCCESS: fingerprint saved as ID %u.\r\n",
                    (unsigned int)id
                );

                FP_PrintCount();
                return;
            }

            if (progress != last_progress)
            {
                Console_Printf(
                    "Enrollment status: progress %u%%, capture stage %u.\r\n",
                    (unsigned int)progress,
                    (unsigned int)captures
                );

                last_progress = progress;
            }
        }
        else
        {
            Console_Printf(
                "Enrollment status received; follow the sensor prompts.\r\n"
            );
        }
    }

    Console_Printf(
        "Enrollment timed out. Retry ENROLL %u.\r\n",
        (unsigned int)id
    );
}



/* Wait until a finger is detected on TOUCHOUT PC0.
   Based on the observed signal: LOW = finger detected. */

static int FP_WaitForFingerTouch(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            /* Allow the finger contact to stabilize. */
            HAL_Delay(200U);

            /* Verify that the finger is still touching. */
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
            {
                return 1;
            }
        }

        HAL_Delay(10U);
    }

    return 0;
}




/* =========================================================
   MATCH FINGERPRINT
   ========================================================= */


static void FP_Match(int32_t requested_id)
{
    uint16_t count;
    uint8_t exists;
    uint8_t response[16];
    uint16_t response_length = 0U;
    uint32_t error = 0xFFFFFFFFUL;

    FP_RxStatus status;

    uint16_t match_result;
    uint16_t found_id;
    uint16_t score;

    /* Check the requested ID, if one was specified. */
    if (requested_id >= 0)
    {
        if ((requested_id < (int32_t)FP_MIN_ID) ||
            (requested_id > (int32_t)FP_MAX_ID))
        {
            Console_Printf(
                "Invalid ID. Use an ID from 0 to 59.\r\n"
            );
            return;
        }

        if (!FP_CheckIdExists((uint16_t)requested_id, &exists))
        {
            Console_Printf(
                "Could not check ID %ld; verify sensor connection.\r\n",
                (long)requested_id
            );
            return;
        }

        if (exists == 0U)
        {
            Console_Printf(
    "Touch the sensor and hold your finger still...\r\n"
);

/* Wait for a real touch before requesting a match. */
if (!FP_WaitForFingerTouch(10000U))
{
    Console_Printf(
        "No finger detected within 10 seconds. "
        "Check TOUCHOUT PC0 wiring and signal polarity.\r\n"
    );

    return;
}

Console_Printf(
    "Finger detected. Scanning...\r\n"
);

status = FP_Transaction(
    FP_CAT_FINGERPRINT,
    FP_CMD_MATCH_SYNC,
    NULL,
    0U,
    15000U,
    response,
    sizeof(response),
    &response_length,
    &error
);
            return;
        }
    }
    else
    {
        if (!FP_GetTemplateCount(&count))
        {
            Console_Printf(
                "Could not query sensor storage; match cancelled.\r\n"
            );
            return;
        }

        if (count == 0U)
        {
            Console_Printf(
                "No fingerprints are enrolled. Use ENROLL ID first.\r\n"
            );
            return;
        }
    }

    Console_Printf(
        "Place a finger on the sensor for matching...\r\n"
    );

    status = FP_Transaction(
        FP_CAT_FINGERPRINT,
        FP_CMD_MATCH_SYNC,
        NULL,
        0U,
        15000U,
        response,
        sizeof(response),
        &response_length,
        &error
    );

    if (status == FP_RX_TIMEOUT)
    {
        Console_Printf(
            "No match response within 15 seconds. "
            "Place your finger firmly and retry.\r\n"
        );
        return;
    }

    if (status != FP_RX_OK)
    {
        Console_Printf(
            "Invalid sensor response during matching. "
            "Check UART settings and retry.\r\n"
        );
        return;
    }

    if (error != FP_ERR_SUCCESS)
    {
        Console_Printf(
            "Match failed: 0x%08lX (%s).\r\n",
            (unsigned long)error,
            FP_ErrorText(error)
        );
        return;
    }

    if (response_length < 6U)
    {
        Console_Printf(
            "Match response too short: %u bytes.\r\n",
            (unsigned int)response_length
        );
        return;
    }

    /*
     * Correct response layout:
     *
     * response[0..1] = match result (16-bit)
     * response[2..3] = match score  (16-bit)
     * response[4..5] = matched ID   (16-bit)
     *
     * Example:
     * 00 01 27 0F 00 03
     * Result = 1, score = 9999, fingerprint ID = 3
     */

    match_result = (uint16_t)(
        ((uint16_t)response[0] << 8) |
        response[1]
    );

    score = (uint16_t)(
        ((uint16_t)response[2] << 8) |
        response[3]
    );

    found_id = (uint16_t)(
        ((uint16_t)response[4] << 8) |
        response[5]
    );

Console_Printf(
    "Match result=%u, ID=%u, score=%u\r\n",
    (unsigned int)match_result,
    (unsigned int)found_id,
    (unsigned int)score
);

if ((match_result != 1U) || (score == 0U))
{
    Console_Printf("NOT MATCHED.\r\n");
    Console_Printf(
        "Please lift your finger, place it again, "
        "and retry the MATCH command.\r\n"
    );
}
else if (requested_id < 0)
{
    Console_Printf(
        "MATCHED: fingerprint ID %u (score %u).\r\n",
        (unsigned int)found_id,
        (unsigned int)score
    );
}
else if (found_id == (uint16_t)requested_id)
{
    Console_Printf(
        "MATCHED: fingerprint matches ID %ld (score %u).\r\n",
        (long)requested_id,
        (unsigned int)score
    );
}
else
{
    Console_Printf(
        "NOT MATCHED to ID %ld. "
        "Sensor recognized ID %u instead.\r\n",
        (long)requested_id,
        (unsigned int)found_id
    );
}

}


/* =========================================================
   DELETE FINGERPRINT
   ========================================================= */

static void FP_Delete(uint16_t id)
{
    uint8_t exists;
    uint8_t payload[3];
    uint16_t response_length = 0U;
    uint32_t error = 0xFFFFFFFFUL;

    FP_RxStatus status;

    if (id > FP_MAX_ID)
    {
        Console_Printf("Invalid ID. Use an ID from 0 to 59.\r\n");
        return;
    }

    if (!FP_CheckIdExists(id, &exists))
    {
        Console_Printf(
            "Could not check ID %u; delete cancelled.\r\n",
            (unsigned int)id
        );
        return;
    }

    if (exists == 0U)
    {
        Console_Printf(
            "No fingerprint is stored at ID %u.\r\n",
            (unsigned int)id
        );
        return;
    }

    /* Single-ID deletion payload. */
    payload[0] = 0U;
    payload[1] = (uint8_t)(id >> 8);
    payload[2] = (uint8_t)(id & 0xFFU);

    status = FP_Transaction(
        FP_CAT_FINGERPRINT,
        FP_CMD_DELETE,
        payload,
        sizeof(payload),
        3000U,
        NULL,
        0U,
        &response_length,
        &error
    );

    if ((status != FP_RX_OK) ||
        (error != FP_ERR_SUCCESS))
    {
        Console_Printf(
            "Delete failed for ID %u. status=%d, error=0x%08lX (%s).\r\n",
            (unsigned int)id,
            (int)status,
            (unsigned long)error,
            FP_ErrorText(error)
        );
        return;
    }

    Console_Printf(
        "DELETED fingerprint ID %u.\r\n",
        (unsigned int)id
    );

    FP_PrintCount();
}

/* =========================================================
   ADD FINGERPRINT USING FIRST FREE ID
   ========================================================= */

static void FP_Add(void)
{
    uint16_t count;
    uint16_t id;
    int result;

    if (!FP_GetTemplateCount(&count))
    {
        Console_Printf(
            "Could not query sensor storage; ADD cancelled.\r\n"
        );
        return;
    }

    if (count >= FP_MAX_TEMPLATES)
    {
        Console_Printf(
            "FINGERPRINT LIMIT REACHED (%u/%u). Delete an ID first.\r\n",
            (unsigned int)count,
            (unsigned int)FP_MAX_TEMPLATES
        );
        return;
    }

    result = FP_FindFreeId(&id);

    if (result == 0)
    {
        Console_Printf(
            "Could not check available IDs; ADD cancelled.\r\n"
        );
        return;
    }

    if (result == 2)
    {
        Console_Printf(
            "FINGERPRINT LIMIT REACHED - no free slot from ID 0 to 59.\r\n"
        );
        return;
    }

    Console_Printf(
        "ADD selected free fingerprint ID %u.\r\n",
        (unsigned int)id
    );

    FP_Enroll(id);
}

/* =========================================================
   HELP
   ========================================================= */

static void FP_PrintHelp(void)
{
    Console_Printf(
        "\r\nFPM383C COMMANDS (IDs 0-59; capacity 60)\r\n"
    );

    Console_Printf("  ENROLL 1  - enroll a finger at ID 1\r\n");
    Console_Printf("  ENROLL 2  - enroll a finger at ID 2\r\n");
    Console_Printf("  ADD       - choose a free ID and enroll\r\n");
    Console_Printf("  MATCH     - match against all enrolled fingerprints\r\n");
    Console_Printf("  MATCH 1   - check whether detected ID is 1\r\n");
    Console_Printf("  MATCH 2   - check whether detected ID is 2\r\n");
    Console_Printf("  DELETE 1  - delete fingerprint ID 1\r\n");
    Console_Printf("  DELETE 2  - delete fingerprint ID 2\r\n");
    Console_Printf("  COUNT     - display stored fingerprint count\r\n");
    Console_Printf("  STATUS    - display stored fingerprint count\r\n");
    Console_Printf("  HELP      - show commands\r\n\r\n");
}

/* =========================================================
   SERIAL COMMAND PROCESSOR
   ========================================================= */

static void FP_ProcessCommand(char *line)
{
    unsigned int id;
    char *p;

    while ((*line == ' ') || (*line == '\t'))
    {
        line++;
    }

    if (*line == '\0')
    {
        return;
    }

    for (p = line; *p != '\0'; p++)
    {
        *p = (char)toupper((unsigned char)*p);
    }

    if ((strcmp(line, "HELP") == 0) ||
        (strcmp(line, "?") == 0))
    {
        FP_PrintHelp();
    }
    else if ((strcmp(line, "COUNT") == 0) ||
             (strcmp(line, "STATUS") == 0))
    {
        FP_PrintCount();
    }
    else if (strcmp(line, "ADD") == 0)
    {
        FP_Add();
    }
    else if (strcmp(line, "MATCH") == 0)
    {
        FP_Match(-1);
    }
    else if (sscanf(line, "ENROLL %u", &id) == 1)
    {
        if (id > FP_MAX_ID)
        {
            Console_Printf(
                "Invalid ID. Use an ID from 0 to 59.\r\n"
            );
        }
        else
        {
            FP_Enroll((uint16_t)id);
        }
    }
    else if (sscanf(line, "MATCH %u", &id) == 1)
    {
        if (id > FP_MAX_ID)
        {
            Console_Printf(
                "Invalid ID. Use an ID from 0 to 59.\r\n"
            );
        }
        else
        {
            FP_Match((int32_t)id);
        }
    }
    else if (sscanf(line, "DELETE %u", &id) == 1)
    {
        if (id > FP_MAX_ID)
        {
            Console_Printf(
                "Invalid ID. Use an ID from 0 to 59.\r\n"
            );
        }
        else
        {
            FP_Delete((uint16_t)id);
        }
    }
    else
    {
        Console_Printf(
            "Unknown command: %s. Type HELP.\r\n",
            line
        );
    }
}

/* =========================================================
   APPLICATION INITIALIZATION
   ========================================================= */

static void FP_App_Init(void)
{
    uint16_t count;
    uint8_t response[8];
    uint16_t response_length = 0U;
    uint32_t error = 0xFFFFFFFFUL;

    FP_RxStatus status;

    console_line_len = 0U;
    previous_touch_state =
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    Console_Printf(
        "\r\nHLK-FPM383C Fingerprint Controller\r\n"
    );

    Console_Printf(
        "Sensor UART: USART1 / PA9-TX PA10-RX / 57600 baud\r\n"
    );

    Console_Printf(
        "PC console: USART2 / PA2-TX PA3-RX / 115200 baud\r\n"
    );

    Console_Printf(
        "TOUCHOUT: PC0 status only\r\n"
    );

    /* Heartbeat command: CMD1 0x03, CMD2 0x03. */
    status = FP_Transaction(
        FP_CAT_MAINTENANCE,
        FP_CMD_HEARTBEAT,
        NULL,
        0U,
        1500U,
        response,
        sizeof(response),
        &response_length,
        &error
    );

    if ((status == FP_RX_OK) &&
        (error == FP_ERR_SUCCESS))
    {
        Console_Printf("Sensor heartbeat: OK\r\n");
    }
    else
    {
        Console_Printf(
            "Sensor did not answer heartbeat (status=%d, error=0x%08lX).\r\n",
            (int)status,
            (unsigned long)error
        );

        Console_Printf(
            "Check sensor power, TX/RX wiring, and baud rate.\r\n"
        );
    }

    if (FP_GetTemplateCount(&count))
    {
        Console_Printf(
            "Stored fingerprints at startup: %u/%u\r\n",
            (unsigned int)count,
            (unsigned int)FP_MAX_TEMPLATES
        );
    }
    else
    {
        Console_Printf(
            "Initial template count unavailable. Retry COUNT after startup.\r\n"
        );
    }

    FP_PrintHelp();
}

/* =========================================================
   APPLICATION TASK
   ========================================================= */

static void FP_App_Task(void)
{
    uint8_t ch;
    GPIO_PinState touch_state;

    /* Read serial-console commands, terminated by CR or LF. */
    if (HAL_UART_Receive(&huart2, &ch, 1U, 1U) == HAL_OK)
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            if (console_line_len > 0U)
            {
                console_line[console_line_len] = '\0';

                FP_ProcessCommand(console_line);

                console_line_len = 0U;
            }
        }
        else if ((ch == '\b') || (ch == 0x7FU))
        {
            if (console_line_len > 0U)
            {
                console_line_len--;
            }
        }
        else if (console_line_len < (CONSOLE_LINE_MAX - 1U))
        {
            console_line[console_line_len++] = (char)ch;
        }
        else
        {
            console_line_len = 0U;
            Console_Printf(
                "Command too long; line cleared.\r\n"
            );
        }
    }

    /* TOUCHOUT is a status signal, not a fingerprint-match result. */
    touch_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if (touch_state != previous_touch_state)
    {
        Console_Printf(
            "TOUCHOUT PC0 changed to %s.\r\n",
            (touch_state == GPIO_PIN_SET) ? "HIGH" : "LOW"
        );

        previous_touch_state = touch_state;
    }
}

/* =========================================================
   SYSTEM CLOCK
   HSI 16 MHz -> SYSCLK 84 MHz
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

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

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   ERROR HANDLER
   ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* Stop here if initialization fails. */
    }
}