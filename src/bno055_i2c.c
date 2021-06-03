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

#include <stdio.h>
#include <stdlib.h>

#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "bno055_sensor/bno055_i2c.h"

// global variable for the i2c file descriptor
int i2c_fd = -1;

#define BNO055_ID 0xA0

int init_i2cbus(const char *i2c_bus, const char *dev_addr)
{

   if ((i2c_fd = open(i2c_bus, O_RDWR)) < 0)
   {
      printf("Error failed to open I2C bus [%s].\n", i2c_bus);
      return -1;
   }
   
   int addr = (int)strtol(dev_addr, NULL, 16);
   printf("Debug: Sensor address: [0x%02X]\n", addr);

   if(ioctl(i2c_fd, I2C_SLAVE, addr) != 0) {
      printf("Error can't find sensor at address [0x%02X].\n", addr);
      return -1;
   }

   return 0;
}

void close_i2cbus()
{
  if (close(i2c_fd) < 0)
  {
    printf("Error failed to close I2C file descriptor [%d].\n", i2c_fd);
  }
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  int result = 0;
  if (cnt <= 0)
  {
    printf("BNO055_I2C_bus_read cnt = [%d] is <= 0", cnt);
    return 1;
  }
  else if (cnt == 1)
  {
    result = i2c_smbus_read_byte_data(i2c_fd, reg_addr);
    if (result < 0)
    {
      printf("BNO055_I2C_bus_read failed to read byte from I2C at [0x%02X]\n", reg_addr);
      return 1;
    }
    else
    {
      reg_data[0] = result;
    }
  }
  else
  {
    result = i2c_smbus_read_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
    if (result != cnt)
    {
      printf("BNO055_I2C_bus_read failed to read %d bytes from I2C at [0x%02X]\n", cnt, reg_addr);
      return 1;
    }
  }

  return 0;
}
  
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  int result = 0;
  if (cnt <= 0)
  {
    printf("BNO055_I2C_bus_write cnt = [%d] is <= 0", cnt);
    return 1;
  }
  else if (cnt == 1)
  {
    result = i2c_smbus_write_byte_data(i2c_fd, reg_addr, reg_data[0]);
    if (result < 0)
    {
      printf("BNO055_I2C_bus_read failed to write byte from I2C at [0x%02X]\n", reg_addr);
      return 1;
    }
  }
  else
  {
    result = i2c_smbus_write_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
    if (result != cnt)
    {
      printf("BNO055_I2C_bus_read failed to write %d to I2C at [0x%02X]\n", cnt, reg_addr);
      return 1;
    }
  }
  return 0;
}

void BNO055_delay_msek(u32 msek)
{
  usleep(msek*1000);
}
