#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;
static uint8_t spi_failed;

void BMI088_BeginTransfer(void)
{
    spi_failed = 0;
}

uint8_t BMI088_TransferFailed(void)
{
    return spi_failed;
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    if (spi_failed)
        return 0;
    // 预留 tick 边界和任务切换时间；1 ms 可能使已完成的字节传输仍被 HAL 判为超时。
    if (HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 10) != HAL_OK)
    {
        spi_failed = 1;
        return 0;
    }
    return rx_data;
}
