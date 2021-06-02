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

/*
int bno055_get_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        return 0;
    }
    *val = inbuf;

    return 1;
}
*/
  
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

  int result = i2c_smbus_read_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
  if (result != cnt)
  {
    printf("Failed to read block from I2C at [0x%02X]\n", reg_addr);
    return 1;
  }

  return 0;
}
  
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  int result = i2c_smbus_write_i2c_block_data(i2c_fd, reg_addr, cnt, reg_data);
  if (result != cnt)
  {
    printf("Failed to write block to I2C at [0x%02X]\n", reg_addr);
    return 1;
  }

  return 0;
}

void BNO055_delay_msek(u32 msek)
{
  usleep(msek);
}
