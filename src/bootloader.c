#include "mik32_hal_pcc.h"
#include "mik32_hal_spifi_w25.h"

#include "power_manager.h"
#include "uart.h"
#include "pad_config.h"
#include "mik32_memory_map.h"

#include "riscv_csr_encoding.h"
#include "csr.h"

#include "string.h"

#define JALR_TO_SPIFI()                           \
    asm volatile(	"la ra, 0x80000000\n\t"     \
                    "jalr ra"                   \
                );

#define CHIP_MODE 1   /* Режим работы МФП (SPIFI Memory Mode): 0 = QSPI или 1 = QPI */

#define ACK  0x0F     /* Подтверждение */
#define NACK 0xF0     /* Нет подтверждения */
#define MAX_PACKAGE_SIZE 256 /* максимальный размер пакета */
#define TIMEOUT_VALUE 1000000 /* Время ожидания загрузчика до прыжка по умолчанию в RAM 1000000 */

/* Виды команд */
#define FULL_ERASE_CMD_LEN   4
typedef enum
{
    PACKAGE_SIZE = 0x30,        /* Команда размера пакета */
    SEND_PACKAGE = 0x60,        /* Команда отправить пакет */
    FULL_ERASE   = 0xBADC0FEE   /* Команда стирания spifi */
} BotloaderComand;

typedef enum
{
    SPIFI_ADDRESS = 0x80000000
} AddressMemory;

/* Виды ошибок */
typedef enum
{
    ERROR_NONE = 0,
    ERROR_TIMEOUT = 1, // Время ожидания истекло
} Bootloader_error;

typedef struct
{
    uint8_t* address; // Адрес для записи присылаемых байт
    uint16_t size_package; // Размер пакета
    uint8_t error; 
    uint8_t command; // Текущая принятая загрузчиком команда
} Bootloader_attributes;

Bootloader_attributes hBootloader = {(uint8_t*) SPIFI_ADDRESS, 0, ERROR_NONE, 0};
uint32_t timeout = 0;
uint32_t validCommandsTimeout = 0;

void go_to_spifi();


/* Инициализация UART */
void Bootloader_UART_Init()
{
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_UART_0_M; // Включение тактирования UART0

    // Настройка выводов PORT0.5 и PORT0.6 на последовательный интерфейс
    PAD_CONFIG->PORT_0_CFG |= (0b01 << (5 << 1)) | (0b01 << (6 << 1)); 
    // Включение притяжки к питанию на линии rx
    PAD_CONFIG->PORT_0_PUPD |= (0b01 << (5 << 1));

    /*
     * Настройки USART:
     * Асинхронный режим. Включен RX и TX;
     * Кадр: 8 бит данных, бит четности выключен, 1 стоп бит;
     * Байт LSB - первый бит нулевой.
     */
    UART_0->CONTROL1 = 0;
    UART_0->CONTROL2 = 0;
    UART_0->CONTROL3 = 0;
    UART_0->DIVIDER = 138; /* Baudrate = 230400 */
    UART_0->FLAGS = 0xFFFFFFFF;
    UART_0->CONTROL1 = UART_CONTROL1_RE_M | UART_CONTROL1_TE_M | UART_CONTROL1_UE_M;

    /* Ожидание флагов готовности RX и TX */
    while (!(UART_0->FLAGS & (UART_FLAGS_REACK_M | UART_FLAGS_TEACK_M)))
        ;
}

void Bootloader_UART_Deinit()
{
    UART_0->CONTROL1 = 0;
    UART_0->CONTROL2 = 0;
    UART_0->CONTROL3 = 0;
    UART_0->DIVIDER = 0x0000; // сброс бодрейта
    UART_0->FLAGS = 0xFFFFFFFF; // сброс всех флагов
    UART_0->TXDATA = 0x00;

    // Настройка выводов PORT0.5 и PORT0.6 на порт общего назначения
    PAD_CONFIG->PORT_0_CFG &= ~((0b11 << (5 << 1)) | (0b11 << (6 << 1))); 
    // Отключение притяжки на линии rx
    PAD_CONFIG->PORT_0_PUPD &= ~(0b01 << (5 << 1));

    PM->CLK_APB_P_SET &= !PM_CLOCK_APB_P_UART_0_M; // Выключение тактирования UART0   
}

/* Отправить байт */
void Bootloader_UART_WriteByte(uint16_t Write_Byte)
{
    UART_0->TXDATA =  Write_Byte;
    /* Ожидаем успешную передачу */
    while (!(UART_0->FLAGS & UART_FLAGS_TC_M))
        ;
}

/* Ожидание и считывание байта */
uint16_t Bootloader_UART_ReadByte()
{
    timeout = 0;
    while ((!(UART_0->FLAGS & UART_FLAGS_RXNE_M)) && (timeout != TIMEOUT_VALUE))
    {
        timeout++;
        validCommandsTimeout++; // Увеличить таймаут валидных команд
    }

    if (timeout == TIMEOUT_VALUE)
    {
        hBootloader.error = ERROR_TIMEOUT;
    }
    
    return (uint16_t)UART_0->RXDATA;
}

/* Обработчик ошибок */
void Bootloader_ErrorHandler()
{
    switch (hBootloader.error)
    {
    case ERROR_TIMEOUT:
        if (UART_0->FLAGS & UART_FLAGS_ORE_M)
            UART_0->FLAGS |= UART_FLAGS_ORE_M;
        
        go_to_spifi(); // переход в основную программу, если в течение TIMEOUT_VALUE нет принятых данных
        break;
    }

    hBootloader.error = ERROR_NONE;
}

SPIFI_HandleTypeDef spifi = {.Instance = SPIFI_CONFIG};

uint8_t erase_chip(SPIFI_HandleTypeDef *spifi)
{
    const uint32_t cmd_chip_erase =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(0) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE) |
        SPIFI_CONFIG_CMD_OPCODE(0xC7); //CHIP_ERASE = 0x60 или 0xC7 
    HAL_SPIFI_W25_WriteEnable(spifi);
    uint8_t stat = HAL_SPIFI_SendCommand_LL(spifi, cmd_chip_erase, 0, 0, 0, 0, 0, HAL_SPIFI_TIMEOUT);
    stat = HAL_SPIFI_W25_WaitBusy(spifi, 10000000); // SPIFI_W25_PROGRAM_BUSY = 100000
    return stat;
}

/* Загрузить данные пакета в RAM */
#define SIZE_4K 4096

// разметка строки в хекс-файле
#define BYTE_COUNT_POS  0   // индекс счетчика байт данных
#define ADDRES_POS      1   // индекс адреса
#define ADDRES_QTY      2   // количество байт адреса
#define RECORD_TYPE_POS 3   // индекс типа записи
#define DATA_POS        4   // индекс начала данных в команде

// типы записей в хекс-файле
#define REC_TYPE_DATA           0x00
#define REC_TYPE_EOF            0x01
#define REC_TYPE_EXT_LIN_ADDR   0x04

uint32_t abs_addr = 0; // адрес из хекса
uint32_t rel_addr = 0; // адрес от начала области spifi, по нему определяем, надо ли стирать сектор и какой именно
#define TAIL_SIZE 15             // если попадутся строки хекса, в которых не 16 байт, то мы рискуем записать данные уарта мимо буфера package_data. а если больше 16 байт, то это проблема завтрашнего дня
uint8_t page_data[MAX_PACKAGE_SIZE + TAIL_SIZE] = {0}; // сюда собираем распарсенные данные из хекса
uint16_t page_fill_size = 0;  // счетчик, сколько заполнно в page_data. когда page_data заполнена до конца - будем записывать в spifi

void mem_write()
{
    // если адрес дошел до начала нового сектора, стираем новый сектор
    rel_addr = (uint32_t)(hBootloader.address) - 0x80000000;
    if ((rel_addr % SIZE_4K) == 0)
        HAL_SPIFI_W25_SectorErase4K(&spifi, rel_addr);

    // записываем страницу в 256 байт в spifi
    HAL_SPIFI_W25_PageProgram(&spifi, (uint32_t)hBootloader.address, MAX_PACKAGE_SIZE, page_data);
    // увеличиваем адреса, по которым писать и стирать
    hBootloader.address += MAX_PACKAGE_SIZE;
    // обнуляем часть буфера, которая была записана в память и уменьшаем счетчик заполнения буфера на столько, сколько было записано
    if (page_fill_size <= MAX_PACKAGE_SIZE)
        page_fill_size = 0;
    else
        page_fill_size -= MAX_PACKAGE_SIZE;
    memset(&page_data[0], 0, MAX_PACKAGE_SIZE);
    // хвост копируем в начало буфера, чтобы записать его в следующий раз
    memcpy(&page_data[0], &page_data[MAX_PACKAGE_SIZE], TAIL_SIZE);
    // а сам хвост обнуляем 
    memset(&page_data[MAX_PACKAGE_SIZE], 0, TAIL_SIZE);
}
void Bootloader_parseHexAndLoadInMemory(uint8_t rx_data[])
{
    // из принятых данных вытаскиваем тип записи
    uint8_t rec_type = rx_data[RECORD_TYPE_POS];
    switch (rec_type)
    {
        case REC_TYPE_EXT_LIN_ADDR:
            // если так получилось, что нам слали данные, буфер на 256 не заполнился, а тут прилетела команда смены адреса, то пишем сколько есть
            if (page_fill_size != 0)
                mem_write();
            // собираем адрес, с которого начинаем писать из данных команды смены адреса. нам присылают только 2 старших байта адреса
            abs_addr = (rx_data[DATA_POS] << 24) + (rx_data[DATA_POS+1] << 16);
            hBootloader.address = (uint8_t*)abs_addr;
            break;

        case REC_TYPE_DATA:
            // перекладываем из приемного буфера данные команды в буфер для записи
            memcpy(&page_data[page_fill_size], &rx_data[DATA_POS], rx_data[BYTE_COUNT_POS]);
            // указываем, на сколько заполнился буфер
            page_fill_size += rx_data[BYTE_COUNT_POS];
            // если пора записывать целую страницу - пишемм
            if (page_fill_size >= 256)
                mem_write();
            break;
        
        case REC_TYPE_EOF: // конец прошивки
            Bootloader_UART_WriteByte(ACK);
            // если есть недозаполненная страница, записываем ее как есть
            if (page_fill_size != 0)
                mem_write();
            // и идем в записанную программу
            go_to_spifi();
            break;
        
        default:
            break;
    }
}
uint8_t uart_data[MAX_PACKAGE_SIZE] = {0}; // Массив данных из полученного пакета
void Bootloader_UART_ReadPackage()
{
    for (uint32_t counter = 0; counter < hBootloader.size_package; counter++)
    {
        timeout = 0;
        while ((!(UART_0->FLAGS & UART_FLAGS_RXNE_M)) && (timeout != TIMEOUT_VALUE)) // Ожидание байта пакета
        {
            timeout++;
        }

        if (timeout == TIMEOUT_VALUE)
        {
            hBootloader.error = ERROR_TIMEOUT;
            break;
        }
        
        uart_data[counter] = UART_0->RXDATA;
    }

    if (!hBootloader.error)
    {
        Bootloader_parseHexAndLoadInMemory(uart_data);
    }
}

uint8_t eraseChipBufferIndex = 0;  // Индекс для накопления команды erase chip
void Bootloader_Commands()
{
    while (1)
    {
        hBootloader.command = Bootloader_UART_ReadByte(); // Ожидание и считывание команды

        // если долго не приходили валидные команды, переход в основную программу
        if (validCommandsTimeout >= TIMEOUT_VALUE)
            go_to_spifi();

        if (hBootloader.error)
            Bootloader_ErrorHandler(); // Обработчик ошибок
        else 
        {
            switch (hBootloader.command)
            {
            case PACKAGE_SIZE:
                validCommandsTimeout = 0;           // Сброс таймаута валидных команд
                eraseChipBufferIndex = 0;
                Bootloader_UART_WriteByte(ACK);     // Подтвердить команду
                hBootloader.size_package = Bootloader_UART_ReadByte() + 1; // Прочитать размер пакета
                Bootloader_UART_WriteByte(ACK);     // Подтвердить
                break;
            case SEND_PACKAGE:
                validCommandsTimeout = 0;           // Сброс таймаута валидных команд
                eraseChipBufferIndex = 0;
                Bootloader_UART_WriteByte(ACK);     // Подтвердить команду
                Bootloader_UART_ReadPackage();      // Получить пакет и скопировать его в RAM
                if (hBootloader.error)
                    Bootloader_ErrorHandler();      // Обработчик ошибок
                else
                    Bootloader_UART_WriteByte(ACK); // Подтвердить считывание и копирования пакета в RAM
                break;
            default:
                // Вычислить ожидаемый байт команды FULL_ERASE на основе текущего индекса
                uint8_t expectedByte = (FULL_ERASE >> ((FULL_ERASE_CMD_LEN - eraseChipBufferIndex - 1) * 8)) & 0xFF;
                // Если полученный байт совпадает с ожидаемым байтом команды FULL_ERASE
                if (hBootloader.command == expectedByte)
                {
                    // Переход к следующему байту
                    eraseChipBufferIndex++;
                    // Если команда FULL_ERASE полностью получена
                    if (eraseChipBufferIndex == FULL_ERASE_CMD_LEN)
                    {
                        Bootloader_UART_WriteByte(ACK);     // Подтвердить получение команды
                        // Очистка чипа
                        if (erase_chip(&spifi) != HAL_OK)
                            Bootloader_UART_WriteByte(NACK);
                        else    
                            Bootloader_UART_WriteByte(ACK); // Подтвердить успешную очистку
                        eraseChipBufferIndex = 0;           // Сброс индекса для следующего приема
                    }
                    validCommandsTimeout = 0;               // Сброс таймаута валидных команд
                } 
                else // Если байт не совпал ни с какими валидными командами
                    eraseChipBufferIndex = 0;               // Сброс индекса команды FULL_ERASE
                break;
            }
        }
    }
}


void SystemClock_Config();

int main() 
{
    SystemClock_Config();
    HAL_SPIFI_MspInit(&spifi);
    HAL_SPIFI_Reset(&spifi);

     /* Переключение флеш-памяти в нормальный режим с командами, передав ей "0" в промежуточном байте */
    const uint32_t cmd_chip_read_xip_init =
        SPIFI_DIRECTION_INPUT |
#if CHIP_MODE == 1
        SPIFI_CONFIG_CMD_INTLEN(1) |
#else
        SPIFI_CONFIG_CMD_INTLEN(3) |
#endif
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_PARALLEL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_3ADDR) |
        SPIFI_CONFIG_CMD_OPCODE(0xEB);
    uint8_t tmp_byte_xip_init[1] = {0};
    HAL_SPIFI_SendCommand_LL(&spifi, cmd_chip_read_xip_init, 0, 1, tmp_byte_xip_init, 0, 0, HAL_SPIFI_TIMEOUT);   

#if CHIP_MODE == 1
    /* Переключение флеш-памяти из режима QPI в обычный режим SPI */
    const uint32_t cmd_qpi_disable =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(0) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_PARALLEL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE) |
        SPIFI_CONFIG_CMD_OPCODE(0xFF);
    HAL_SPIFI_SendCommand_LL(&spifi, cmd_qpi_disable, 0, 0, 0, 0, 0, HAL_SPIFI_TIMEOUT);
#endif

    Bootloader_UART_Init(); // Инициализация UART. Настройка выводов и тактирования
      
    Bootloader_Commands(); // Обработка и ожидание команд

    while (1)
    {
        /* code */
    }
}

void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 128;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}
void SPIFI_disableDataCache(SPIFI_MemoryModeConfig_HandleTypeDef *spifi)
{
    spifi->Instance->CTRL |= SPIFI_CONFIG_CTRL_D_CACHE_DIS_M;
}
void SPIFI_Init()
{
    HAL_SPIFI_MspInit(&spifi);
    HAL_SPIFI_Reset(&spifi);

    /* В Winbond для выставления QE используется команда 0x01 в 1-м бите 2го статус регистра. */
    uint8_t sreg2 = HAL_SPIFI_W25_ReadSREG(&spifi, W25_SREG2);
    if (!(sreg2 & (1 << 1)))
    {
        uint8_t sreg1 = HAL_SPIFI_W25_ReadSREG(&spifi, W25_SREG1);
        HAL_SPIFI_W25_WriteSREG(&spifi, sreg1, sreg2 | (1 << 1)); // ? HAL_SPIFI_W25_QuadEnable(&spifi); 
    }

#if CHIP_MODE == 1
    /* Переключение флеш-памяти в режим QPI, когда весь обмен четырёхпроводной */
    const uint32_t cmd_qpi_enable =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(0) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE) |
        SPIFI_CONFIG_CMD_OPCODE(0x38);
    HAL_SPIFI_SendCommand_LL(&spifi, cmd_qpi_enable, 0, 0, 0, 0, 0, HAL_SPIFI_TIMEOUT);

    /* Переключение флеш-памяти в режим без последующих команд чтения, передав ей "0x20" в промежуточном байте */
    const uint32_t cmd_chip_read_qpi_xip_init =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(1) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_PARALLEL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE_3ADDR) |
        SPIFI_CONFIG_CMD_OPCODE(0xEB);
    uint8_t tmp_byte_xip_init[1] = {0};
    HAL_SPIFI_SendCommand_LL(&spifi, cmd_chip_read_qpi_xip_init, 0, 1, tmp_byte_xip_init, 0, 0x20, HAL_SPIFI_TIMEOUT);
#else
    /* Переключение флеш-памяти в режим без последующих команд чтения, передав ей "0x20" в первом промежуточном байте */
    const uint32_t cmd_chip_read_xip_init =
        SPIFI_DIRECTION_INPUT |
        SPIFI_CONFIG_CMD_INTLEN(3) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_OPCODE_SERIAL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE_3ADDR) |
        SPIFI_CONFIG_CMD_OPCODE(0xEB);
    uint8_t tmp_byte_xip_init[1] = {0};
    HAL_SPIFI_SendCommand_LL(&spifi, cmd_chip_read_xip_init, 0, 1, tmp_byte_xip_init, 0, 0x20, HAL_SPIFI_TIMEOUT);
#endif
    /* Режим SPIFI без передачи команд, но с "0x20" в первых промежуточных байтах. */
    SPIFI_MemoryCommandTypeDef cmd_mem = {
        .OpCode = 0xEB,
        .FieldForm = SPIFI_CONFIG_CMD_FIELDFORM_ALL_PARALLEL,
        .FrameForm = SPIFI_CONFIG_CMD_FRAMEFORM_NOOPCODE_3ADDR,
        .InterimData = 0x20,
#if CHIP_MODE == 1
        .InterimLength = 1 /* Количество промежуточных данных в команде 0xEB режима QPI равно 1 байт. */
#else
        .InterimLength = 3 /* Количество промежуточных данных в команде 0xEB режима QSPI равно 3 байта. */
#endif
    };

    SPIFI_MemoryModeConfig_HandleTypeDef spifi_mem = {
        .Instance = spifi.Instance,
        .CacheEnable = SPIFI_CACHE_ENABLE,
        .CacheLimit = 0x00010000,
        .Command = cmd_mem
    };

    SPIFI_CONFIG->CTRL &= 0xFFF0FFFF;
    HAL_SPIFI_MemoryMode_Init(&spifi_mem);
    // SPIFI_disableDataCache(&spifi_mem);
}

void go_to_spifi()
{
    Bootloader_UART_Deinit();
    SPIFI_Init();
    write_csr(mtvec, 0x80000000);
    JALR_TO_SPIFI();
}
