/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator firmware for MCU
 * Author: Andrés Calderón [andres.calderon@admobilize.com]
 *
 * MATRIX Creator firmware for MCU is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"
#include "board.h"
#include "wdt.h"


#include <math.h>
#include <string.h>
#include <mcuconf.h>

#include "./i2c.h"
#include "./sensors_data.h"
#include "./mpl3115a2.h"
#include "./lsm9ds1.h"
#include "./hts221.h"
#include "./veml6070.h"

extern "C" {
#include "atmel_psram.h"
}

const uint32_t kFirmwareCreatorID = 0x10;
const uint32_t kFirmwareVersion = 0x171017; /* 0xYYMMDD */

/* Global objects */
creator::I2C i2c;  // TODO(andres.calderon@admobilize.com): avoid global objects

void psram_copy(uint16_t mem_offset, char *data, uint8_t len) {
  register char *psram = (char *)PSRAM_BASE_ADDRESS;

  for (int i = 0; i < len; i++) {
    psram[mem_offset + i] = data[i];
  }
}

static WORKING_AREA(waEnvThread, 256);
static msg_t EnvThread(void *arg) {
  (void)arg;

  creator::MPL3115A2 mpl3115a2(&i2c);
  creator::HTS221 hts221(&i2c);
  creator::VEML6070 veml6070(&i2c);

  mpl3115a2.Begin();
  hts221.Begin();
  veml6070.Begin();

  PressureData press;
  HumidityData hum;
  UVData uv;
  MCUData mcu_info;

  mcu_info.ID = kFirmwareCreatorID;
  mcu_info.version = kFirmwareVersion;

  while (true) {
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(1);
    palClearPad(IOPORT3, 17);

    hts221.GetData(hum.humidity, hum.temperature);

    press.altitude = mpl3115a2.GetAltitude();
    press.pressure = mpl3115a2.GetPressure();
    press.temperature = mpl3115a2.GetTemperature();

    uv.UV = veml6070.GetUV();

    psram_copy(mem_offset_mcu, (char *)&mcu_info, sizeof(mcu_info));
    psram_copy(mem_offset_press, (char *)&press, sizeof(press));
    psram_copy(mem_offset_humidity, (char *)&hum, sizeof(hum));
    psram_copy(mem_offset_uv, (char *)&uv, sizeof(uv));
  }
  return (0);
}

//PWM 

void set_period (PWMData *data,char d1, char d2, char d3){ 
  data->period_1=d1;  
  data->period_2=d2;
  data->period_3=d3;   
  psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
}

void set_duty (PWMData *data,char motor,char d1, char d2, char d3){
  switch(motor){
  case 1:
    data->duty1_1   = d1;
    data->duty1_2   = d2;
    data->duty1_3   = d3;
    psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
  break;
  case 2:
    data->duty2_1   = d1;
    data->duty2_2   = d2;
    data->duty2_3   = d3;
    psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
  break;
  case 3:
    data->duty3_1   = d1;
    data->duty3_2   = d2;
    data->duty3_3   = d3;
    psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
  break;
  case 4:
    data->duty4_1   = d1;
    data->duty4_2   = d2;
    data->duty4_3   = d3;
    psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
  break;
  default:
    data->duty1_1   = 0;
    data->duty1_2   = 0;
    data->duty1_3   = 0;
    psram_copy(mem_offset_pwm,(char  *)data, sizeof(*data));
  }
}


void initMotors(PWMData *data)
{
    set_period(data,0x3D,0X09,0X00);
    set_duty (data,1,0x03,0X0D,0X40);
    set_duty (data,2,0x03,0X0D,0X40);
    set_duty (data,3,0x03,0X0D,0X40);
    set_duty (data,4,0x03,0X0D,0X40);
    //chThdSleepMilliseconds(3000);     
}

static WORKING_AREA(waIMUThread, 512);
static msg_t IMUThread(void *arg) {
  (void)arg;

  LSM9DS1 imu(&i2c, IMU_MODE_I2C, 0x6A, 0x1C);
  //int time = 0;
  int deltaTime = 20;

  imu.begin();

  IMUData data;
  PWMData pwmdata;

  initMotors(&pwmdata);

  while (true) {

    //time = chTimeNow();
    imu.readGyro();
    data.gyro_x = (imu.calcGyro(imu.gx) - kGyroOffsetX) * M_PI/180;
    data.gyro_y = (imu.calcGyro(imu.gy) - kGyroOffsetY) * M_PI/180;
    data.gyro_z = (imu.calcGyro(imu.gz) - kGyroOffsetZ) * M_PI/180;

    imu.readMag();
    data.mag_x = imu.calcMag(imu.mx);
    data.mag_y = imu.calcMag(imu.my);
    data.mag_z = imu.calcMag(imu.mz);

    imu.readAccel();
    data.accel_x = imu.calcAccel(imu.ax);
    data.accel_y = imu.calcAccel(imu.ay);
    data.accel_z = imu.calcAccel(imu.az);

    float pitch_ang = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y +
                                           data.accel_z * data.accel_z)) *
                 180.0 / M_PI;
    float roll_ang = atan2(data.accel_y, data.accel_x * data.accel_x + 
                                           data.accel_z * data.accel_z) * 
                180.0 / M_PI;

    
    //deltaTime = time - chTimeNow();

    data.yaw = 0;
    data.roll =  0.94 *(data.roll + data.gyro_x * (deltaTime/1000) ) + 0.06 * roll_ang;
    data.pitch = 0.94 *(data.pitch + data.gyro_y * (deltaTime/1000) ) + 0.06 * pitch_ang;

    psram_copy(mem_offset_imu, (char *)&data, sizeof(data));

    chThdSleepMilliseconds(deltaTime);

    WDT_Restart( WDT ) ;
  }
  return (0);
}

/*
 * Application entry point.
 */
int main(void) {


  halInit();

  chSysInit();

  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  i2c.Init();

  /* Creates the imu thread. */
  chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread,
                    NULL);

  /* Creates the hum thread. */
  chThdCreateStatic(waEnvThread, sizeof(waEnvThread), NORMALPRIO, EnvThread,
                    NULL);

  return (0);
}
