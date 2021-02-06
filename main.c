/*
 * Author:      Cooper Brotherton
 * Date:        February 5, 2021
 * Libraries:   GPIO, I2C, UART, and Timer32 from DriverLib
 */
/******************************************************************************
 * MSP432 Project 6 ECE230 Winter 2020-2021
 *
 * Description: GY-521 MEMS sensor outputs its accelerometer to the terminal
 *              using UART.
 *
 *                                /|\  /|\
 *                  GY-521        10k  10k      MSP432P401
 *                Peripheral       |    |       Controller
 *             -----------------   |    |   -----------------
 *            |     P1.6/UCB0SDA|<-|----+->|P1.6/UCB0SDA     |
 *            |                 |  |       |                 |
 *            |                 |  |       |    P1.3/UCA0TXD |----> PC
 *            |                 |  |       |                 |
 *            |     P1.7/UCB0SCL|<-+------>|P1.7/UCB0SCL     |
 *            |                 |          |                 |
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Addresses and constants
#define SLAVE_ADDRESS       0x68
#define NUM_OF_REC_BYTES    6
#define ACCEL_BASE          0x3B
#define ACCEL_CONFIG        0x1C
#define PWR_MGMT            0x6B

// Variables
const uint8_t TXData[] = { 0x04 };
static uint8_t RXData[NUM_OF_REC_BYTES];
static volatile uint32_t xferIndex;
static volatile bool stopSent;
static volatile int16_t accel_x, accel_y, accel_z;

const eUSCI_I2C_MasterConfig i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                  // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
        };

// UART 57600 8O1
const eUSCI_UART_ConfigV1 uartConfig = {
EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        13,                                     // BRDIV = 13
        0,                                      // UCxBRF = 0
        73,                                     // UCxBRS = 73
        EUSCI_A_UART_ODD_PARITY,                // No Parity
        EUSCI_A_UART_LSB_FIRST,                 // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,              // One stop bit
        EUSCI_A_UART_MODE,                      // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
        };

void setup(void)
{
    WDT_A_holdTimer();

    // P1.6 for UCB0SIMO/UCB0SDA, P1.7 for UCB0SOMI/UCB0SCL for I2C
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN6 + GPIO_PIN7,
                                               GPIO_PRIMARY_MODULE_FUNCTION);
    stopSent = false;
    memset(RXData, 0x00, NUM_OF_REC_BYTES);

    // Configure I2C
    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS);
    I2C_enableModule(EUSCI_B0_BASE);
    Interrupt_enableInterrupt(INT_EUSCIB0);
    // Synchronize GY-521
    while (I2C_masterIsStopSent(EUSCI_B0_BASE))
        ;
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, PWR_MGMT);
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0);
    // Set in ±4g mode
    while (I2C_masterIsStopSent(EUSCI_B0_BASE))
        ;
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_CONFIG);
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0x10);

    // P1.2 and P1.3 in UART mode
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN2 | GPIO_PIN3,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    // Configure UART
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);

    // 1s Timer32 in One-shot mode
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
    TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_0_BASE, CS_getMCLK());
    Timer32_startTimer(TIMER32_0_BASE, true);

    Interrupt_enableMaster();
}

/*!
 * \brief This function transmits a string from the UART module
 *
 * \param moduleInstance is the instance of the eUSCI A (UART) module.
 *          Valid parameters vary from part to part, but can include:
 *          - EUSCI_A0_BASE
 *          - EUSCI_A1_BASE
 *          - EUSCI_A2_BASE
 *          - EUSCI_A3_BASE
 *
 *  It is important to note that for eUSCI modules, only "A" modules such as
 *  EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the UART
 *  mode
 *
 *  \param transmitData data to be transmitted from the UART module
 *
 *  \param length is the length of the String being sent
 *
 *  \return None
 */
void UART_transmitString(uint32_t moduleInstance, char *transmitData,
                         int length)
{
    int i;
    for (i = 0; i < length; i++)
    {
        UART_transmitData(moduleInstance, transmitData[i]);
    }
}

/*!
 * \brief This function samples and displays the accelerometer data at 1Hz
 *
 * This function takes the data from the accelerometer buffers and transmits the
 * data of the X, Y, and Z-axis over UART once every second.
 *
 * \return None
 */
void loop(void)
{
    while (TIMER32_1->VALUE != 0)
    {
    }
    // Restart timer (1 second)
    Timer32_setCount(TIMER32_0_BASE, CS_getMCLK());
    while (I2C_masterIsStopSent(EUSCI_B0_BASE))
        ;
    // Load data into variables
    accel_x = (uint16_t) (RXData[0] << 8) + RXData[1];
    accel_y = (uint16_t) (RXData[2] << 8) + RXData[3];
    accel_z = (uint16_t) (RXData[4] << 8) + RXData[5];

    char num[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

    // print values
    UART_transmitString(EUSCI_A0_BASE, "Accel_X: ", 9);

    if (accel_x < 0)
        UART_transmitData(EUSCI_A0_BASE, '-');
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_x / 4096)]);
    UART_transmitData(EUSCI_A0_BASE, '.');
    int a = abs(accel_x * 10 / 4096);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_x * 10 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_x * 100 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_x * 1000 / 4096 % 10)]);

    UART_transmitData(EUSCI_A0_BASE, ' ');
    UART_transmitData(EUSCI_A0_BASE, 'g');
    UART_transmitData(EUSCI_A0_BASE, '\t');

    UART_transmitString(EUSCI_A0_BASE, "Accel_Y: ", 9);
    if (accel_y < 0)
        UART_transmitData(EUSCI_A0_BASE, '-');
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_y / 4096)]);
    UART_transmitData(EUSCI_A0_BASE, '.');
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_y * 10 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_y * 100 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_y * 1000 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, ' ');
    UART_transmitData(EUSCI_A0_BASE, 'g');
    UART_transmitData(EUSCI_A0_BASE, '\t');

    UART_transmitString(EUSCI_A0_BASE, "Accel_Z: ", 9);
    if (accel_z < 0)
        UART_transmitData(EUSCI_A0_BASE, '-');
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_z / 4096)]);
    UART_transmitData(EUSCI_A0_BASE, '.');
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_z * 10 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_z * 100 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, num[abs(accel_z * 1000 / 4096 % 10)]);
    UART_transmitData(EUSCI_A0_BASE, ' ');
    UART_transmitData(EUSCI_A0_BASE, 'g');

    UART_transmitData(EUSCI_A0_BASE, '\n');
    UART_transmitData(EUSCI_A0_BASE, '\r');

    // Start data transfer
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_BASE);
    xferIndex = 0;
    I2C_masterReceiveStart(EUSCI_B0_BASE);
    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}

int main(void)
{
    setup();

    while (1)
    {
        loop();
    }
}

/*!
 * \brief This function receives the accelerometer data from the GY-521
 *
 * This function receives bytes from the GY-521 into the buffer. Once all the
 * data has been received, sends a STOP condition.
 *
 * \return None
 */
void EUSCIB0_IRQHandler(void)
{
    uint_fast16_t status = I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);

    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        if (xferIndex == NUM_OF_REC_BYTES - 2)
        {
            // Receive data, send stop signal b/c buffer will be filled
            I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);
            I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
            RXData[xferIndex++] = I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
        }
        else if (xferIndex < NUM_OF_REC_BYTES)
        {
            // Receive data, put in buffer
            RXData[xferIndex++] = I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE);
        }
    }
    else if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
        Interrupt_disableSleepOnIsrExit();
        I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);
    }
}
