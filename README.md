# ESP32 ISM330DHCX Driver

An ESP-IDF component driver for the STMicroelectronics ISM330DHCX 6-axis inertial measurement unit (IMU) sensor.

## Overview

The ISM330DHCX is a high-performance 3-axis accelerometer and 3-axis gyroscope featuring:
- **Accelerometer**: ±2/±4/±8/±16 g full scale
- **Gyroscope**: ±125/±250/±500/±1000/±2000/±4000 dps full scale
- Output data rates from 12.5 Hz to 6.67 kHz
- I²C and SPI interfaces
- Advanced embedded features:
  - Free fall detection
  - Wake-up detection
  - Single/double tap detection
  - 6D/4D orientation detection
  - Step counter and pedometer
  - Significant motion detection
  - Tilt detection
  - FIFO buffer (up to 4 kB)

This driver provides an easy-to-use interface for ESP32-based projects using ESP-IDF.

## Features

- ✅ I²C interface support
- ✅ Accelerometer and gyroscope data reading
- ✅ Configurable output data rates (ODR)
- ✅ Configurable full-scale ranges
- ✅ Event detection (tap, double tap, free fall, wake-up, 6D orientation)
- ✅ Raw and converted axis data
- ✅ Compatible with ESP-IDF component manager
- ✅ Example application included

## Installation

### Using ESP-IDF Component Manager

Add the component to your project's `idf_component.yml`:

```yaml
dependencies:
  esp32-ism330dhcx:
    git: https://github.com/ozarrai/esp32-ism330dhcx.git
    version: "0.1.1"
```

### Manual Installation

Clone this repository into your project's `components` directory:

```bash
cd your_project/components
git clone https://github.com/ozarrai/esp32-ism330dhcx.git
```

## Hardware Setup

### I²C Connection

| ISM330DHCX Pin | ESP32 Pin | Description |
|----------------|-----------|-------------|
| VDD            | 3.3V      | Power supply |
| GND            | GND       | Ground |
| SCL            | GPIO 9    | I²C Clock (configurable) |
| SDA            | GPIO 8    | I²C Data (configurable) |
| SA0/SDO        | GND or VDD | I²C address select |
| INT1           | GPIO (optional) | Interrupt pin 1 |
| INT2           | GPIO (optional) | Interrupt pin 2 |

**I²C Address:**
- `0x6A` (when SA0/SDO = GND) - Default in example
- `0x6B` (when SA0/SDO = VDD)

## Quick Start

### Basic Usage

```c
#include "ism330dhcx.h"
#include "driver/i2c.h"

// 1. Initialize I2C
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 8,
    .scl_io_num = 9,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
};
i2c_param_config(I2C_NUM_0, &conf);
i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

// 2. Configure ISM330DHCX object
ISM330DHCX_Object_t imu_obj;
ISM330DHCX_IO_t io_ctx;

io_ctx.BusType = ISM330DHCX_I2C_BUS;
io_ctx.Address = (ISM330DHCX_I2C_ADD_L >> 1);  // 0x6A
io_ctx.Init = platform_init;
io_ctx.DeInit = platform_deinit;
io_ctx.ReadReg = platform_read;
io_ctx.WriteReg = platform_write;
io_ctx.Delay = platform_delay;

// 3. Register and initialize
ISM330DHCX_RegisterBusIO(&imu_obj, &io_ctx);
ISM330DHCX_Init(&imu_obj);

// 4. Enable accelerometer
ISM330DHCX_ACC_Enable(&imu_obj);
ISM330DHCX_ACC_SetOutputDataRate(&imu_obj, 416.0f);  // 416 Hz
ISM330DHCX_ACC_SetFullScale(&imu_obj, 2);            // ±2g

// 5. Enable gyroscope
ISM330DHCX_GYRO_Enable(&imu_obj);
ISM330DHCX_GYRO_SetOutputDataRate(&imu_obj, 416.0f); // 416 Hz
ISM330DHCX_GYRO_SetFullScale(&imu_obj, 2000);        // ±2000 dps

// 6. Read sensor data
ISM330DHCX_Axes_t acc_data, gyro_data;
ISM330DHCX_ACC_GetAxes(&imu_obj, &acc_data);
ISM330DHCX_GYRO_GetAxes(&imu_obj, &gyro_data);

printf("Accel [mg]: X=%ld, Y=%ld, Z=%ld\n", acc_data.x, acc_data.y, acc_data.z);
printf("Gyro [mdps]: X=%ld, Y=%ld, Z=%ld\n", gyro_data.x, gyro_data.y, gyro_data.z);
```

### Event Detection

```c
// Enable double tap detection
ISM330DHCX_ACC_Enable_Double_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN);

// Enable free fall detection
ISM330DHCX_ACC_Enable_Free_Fall_Detection(&imu_obj, ISM330DHCX_INT1_PIN);

// Enable wake-up detection
ISM330DHCX_ACC_Enable_Wake_Up_Detection(&imu_obj, ISM330DHCX_INT2_PIN);

// Check event status
ISM330DHCX_Event_Status_t event_status;
ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status);

if (event_status.DoubleTapStatus) {
    printf("Double tap detected!\n");
}
if (event_status.FreeFallStatus) {
    printf("Free fall detected!\n");
}
```

## API Reference

### Initialization Functions

| Function | Description |
|----------|-------------|
| `ISM330DHCX_RegisterBusIO()` | Register I2C/SPI bus interface |
| `ISM330DHCX_Init()` | Initialize sensor with default settings |
| `ISM330DHCX_DeInit()` | Deinitialize sensor |
| `ISM330DHCX_ReadID()` | Read WHO_AM_I register (should return 0x6B) |
| `ISM330DHCX_GetCapabilities()` | Get sensor capabilities |

### Accelerometer Functions

| Function | Description |
|----------|-------------|
| `ISM330DHCX_ACC_Enable()` | Enable accelerometer |
| `ISM330DHCX_ACC_Disable()` | Disable accelerometer |
| `ISM330DHCX_ACC_GetAxes()` | Read acceleration data in mg |
| `ISM330DHCX_ACC_GetAxesRaw()` | Read raw acceleration data |
| `ISM330DHCX_ACC_SetOutputDataRate()` | Set ODR (12.5 to 6667 Hz) |
| `ISM330DHCX_ACC_GetOutputDataRate()` | Get current ODR |
| `ISM330DHCX_ACC_SetFullScale()` | Set full scale (2, 4, 8, 16 g) |
| `ISM330DHCX_ACC_GetFullScale()` | Get current full scale |
| `ISM330DHCX_ACC_GetSensitivity()` | Get sensitivity in mg/LSB |

### Gyroscope Functions

| Function | Description |
|----------|-------------|
| `ISM330DHCX_GYRO_Enable()` | Enable gyroscope |
| `ISM330DHCX_GYRO_Disable()` | Disable gyroscope |
| `ISM330DHCX_GYRO_GetAxes()` | Read gyroscope data in mdps |
| `ISM330DHCX_GYRO_GetAxesRaw()` | Read raw gyroscope data |
| `ISM330DHCX_GYRO_SetOutputDataRate()` | Set ODR (12.5 to 6667 Hz) |
| `ISM330DHCX_GYRO_GetOutputDataRate()` | Get current ODR |
| `ISM330DHCX_GYRO_SetFullScale()` | Set full scale (125, 250, 500, 1000, 2000, 4000 dps) |
| `ISM330DHCX_GYRO_GetFullScale()` | Get current full scale |
| `ISM330DHCX_GYRO_GetSensitivity()` | Get sensitivity in mdps/LSB |

### Event Detection Functions

| Function | Description |
|----------|-------------|
| `ISM330DHCX_ACC_Enable_Free_Fall_Detection()` | Enable free fall detection |
| `ISM330DHCX_ACC_Enable_Wake_Up_Detection()` | Enable wake-up detection |
| `ISM330DHCX_ACC_Enable_Single_Tap_Detection()` | Enable single tap detection |
| `ISM330DHCX_ACC_Enable_Double_Tap_Detection()` | Enable double tap detection |
| `ISM330DHCX_ACC_Enable_6D_Orientation()` | Enable 6D orientation detection |
| `ISM330DHCX_ACC_Get_Event_Status()` | Read event status register |
| `ISM330DHCX_ACC_Set_Tap_Threshold()` | Set tap detection threshold |
| `ISM330DHCX_ACC_Set_Tap_Shock_Time()` | Set tap shock time window |
| `ISM330DHCX_ACC_Set_Tap_Quiet_Time()` | Set tap quiet time window |
| `ISM330DHCX_ACC_Set_Tap_Duration_Time()` | Set tap duration time window |

### Advanced Functions

| Function | Description |
|----------|-------------|
| `ISM330DHCX_ACC_Enable_HP_Filter()` | Enable high-pass filter |
| `ISM330DHCX_FIFO_Get_Num_Samples()` | Get number of samples in FIFO |
| `ISM330DHCX_FIFO_Set_Mode()` | Set FIFO mode |
| `ISM330DHCX_ACC_Enable_DRDY_On_INT1()` | Route data ready to INT1 |
| `ISM330DHCX_GYRO_Enable_DRDY_On_INT2()` | Route data ready to INT2 |

## Examples

A complete example is provided in `examples/esp32-ism330dhcx-i2c/`:

```bash
cd examples/esp32-ism330dhcx-i2c
idf.py set-target esp32s3
idf.py build flash monitor
```

The example demonstrates:
- I²C initialization and communication
- Accelerometer and gyroscope data reading
- Double tap event detection
- FreeRTOS task-based implementation

## Configuration

### Supported Output Data Rates (ODR)

Both accelerometer and gyroscope support these ODR values:
- 12.5, 26, 52, 104, 208, 416, 833, 1667, 3333, 6667 Hz

### Supported Full Scale Ranges

**Accelerometer:**
- ±2 g
- ±4 g
- ±8 g
- ±16 g

**Gyroscope:**
- ±125 dps
- ±250 dps
- ±500 dps
- ±1000 dps
- ±2000 dps
- ±4000 dps

## Troubleshooting

### Device Not Detected

1. Check I²C wiring and pull-up resistors
2. Verify power supply is 3.3V
3. Check I²C address matches SA0 pin configuration
4. Test I²C bus with `i2cdetect` command

### Incorrect Readings

1. Ensure sensor is stable during initialization
2. Check full-scale range is appropriate for your application
3. Verify ODR is sufficient for your sampling rate
4. Consider enabling high-pass filter for drift compensation

### Event Detection Not Working

1. Adjust threshold values for tap/wake-up detection
2. Ensure interrupt pin is properly connected if using hardware interrupts
3. Check event status register at appropriate polling rate
4. Review datasheet for timing parameters

## License

This component is based on STMicroelectronics' ISM330DHCX driver. See `LICENSE` and `Licences/LICENSE.STMicroelectronics.txt` for details.

## Contributing

Contributions are welcome! Please submit issues and pull requests on [GitHub](https://github.com/ozarrai/esp32-ism330dhcx).

## References

- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [STMicroelectronics MEMS Drivers](https://github.com/STMicroelectronics/STMems_Standard_C_drivers)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)

## Author

Developed by ozarrai

## Version

Current version: 0.1.1
