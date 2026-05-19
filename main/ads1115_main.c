#include <stdio.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           9                           /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8                           /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                           /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000                       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// ADS1115
#define ADS1115_ADDR                 0x48        /*!< Address of the ADS1x15 sensor */
#define CONVERSION_REG               0X00        /* Conversion Register*/
#define CONFIG_REG                   0x01         /* Configuration Register*/

#define BATTERY_MUX                  0x42         /*AN0 High Byte Battery connection with proper gain*/
#define REGULATOR_MUX                0x52         /*AN1 High Byte Regulator connection with proper gain*/
#define LOWCONF_BYTE                 0xA3          /* 250 SPS and Comparator Disabled*/
#define READ_BATTERY                 0             /* Used to select read_adc_voltage battery or regulator*/
#define READ_REGULATOR               1              /* Used to select read_adc_voltage battery or regulator*/


float calc_voltage(uint8_t,  uint8_t);

/**
 * @brief Read a sequence of bytes.
 */
static esp_err_t ads1x15_register_read(i2c_master_dev_handle_t dev_handle, uint8_t *read_buffer, size_t len)
{   
    return i2c_master_receive(dev_handle, read_buffer, len, I2C_MASTER_TIMEOUT_MS);
}

/*
 * @brief Write a byte to address pointer
 */
static esp_err_t ads1x15_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t data[], int16_t numbytes)
{
    return (i2c_master_transmit(dev_handle, data, numbytes, I2C_MASTER_TIMEOUT_MS));
}

/* 
Read either battery voltage using BATTERY or REGULATOR. whichVoltage = 0 for battery else regulator  
*/
static int  read_adc_voltage(int whichVoltage, i2c_master_dev_handle_t dev_handle, float *batVolts)
{
    uint8_t data[6];
    uint8_t buffer[6] = {0};

    buffer[0] = 1; // Select control register and write control bytes
    if (whichVoltage == READ_BATTERY)
    {
        buffer[1] = BATTERY_MUX;
    }
    else
    {
        buffer[1] = REGULATOR_MUX;
    }
    buffer[2] = LOWCONF_BYTE;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buffer, 3, data, 2, I2C_MASTER_TIMEOUT_MS));
    buffer[0] = 0;
    ESP_ERROR_CHECK(ads1x15_register_write_byte(dev_handle, buffer, 1));
    ads1x15_register_read(dev_handle, buffer, 2);

    *batVolts = calc_voltage(buffer[0], buffer[1]);
    return 0;
}

/*
 * @brief i2c master initialization1
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/* 
Calculate actual voltage with 4.096 gain
*/
float calc_voltage(uint8_t lowByte, uint8_t highByte)
{
    return ((lowByte*256 + highByte) * 0.000125);
}

void app_main(void)
{
    float batVolts;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    read_adc_voltage(READ_BATTERY, dev_handle, &batVolts);
    printf("Battery Voltage %.3f\n", batVolts);

    read_adc_voltage(READ_REGULATOR, dev_handle, &batVolts);
    printf("Regulator Voltage %.3f\n", batVolts);

    usleep(1000);
    while (1)
    {
        read_adc_voltage(READ_BATTERY, dev_handle, &batVolts);
        printf("Battery Voltage %.3f\n", batVolts);

        sleep(0.5);

        read_adc_voltage(READ_REGULATOR, dev_handle, &batVolts);
        printf("Regulator Voltage %.3f\n", batVolts);
    }

    usleep(1000);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
