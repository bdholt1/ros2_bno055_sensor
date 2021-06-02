// Copyright 2021 Brian Holt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BNO055_I2C_H
#define BNO055_I2C_H

#include "bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

/*  \Brief: Open the I2C device file descriptor
 *  \Return : Result of the open operation
 *  \param i2c_bus : The address of the I2C bus e.g. /dev/i2c-1
 *  \param dev_addr : The device address of the sensor (0x28 or 0x29)
 */ 
int init_i2cbus(const char *i2c_bus, const char *dev_addr);


/*  \Brief: Close the I2C file descriptor
 *  \Return : Status of the close operation
 */ 
void close_i2cbus();

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */ 
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The API is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *  will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek);

#ifdef __cplusplus
}
#endif 

#endif // BNO055_I2C_H
