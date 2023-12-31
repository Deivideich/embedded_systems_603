/*
 * mpu9250.c
 *
 *  Created on: Nov 9, 2023
 *      Author: demia
 */
#include "mpu9250.h"
#include "spi.h"
#include "stm32h7xx_hal.h"
#include "myprintf.h"

void initMPU9250(struct mpu9250 * mpu9250, uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{
  for (int i = 0 ; i < 3 ; i++){
    for(int j = 0 ; j < filt_size ; j++){
      mpu9250->accBuff[i][j] = 0;
      mpu9250->gyroBuff[i][j] = 0;
    }
    mpu9250->acc[i] = 0;
    mpu9250->gyro[i] = 0;
    mpu9250->pose[i] = 0;
  }

  mpu9250->buffPointer = 0;

	uint8_t c;
 // wake up device
  mpu9250_write_reg(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  HAL_Delay(100); // Wait for all registers to reset

 // get stable time source
  mpu9250_write_reg(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  HAL_Delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum HAL_HAL_Delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  mpu9250_write_reg(CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  mpu9250_write_reg(SMPLRT_DIV, sampleRate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                       // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  mpu9250_read_reg(GYRO_CONFIG, &c,sizeof(c)); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  mpu9250_write_reg(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  mpu9250_read_reg(ACCEL_CONFIG, &c, sizeof(c)); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  mpu9250_write_reg(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  mpu9250_read_reg(ACCEL_CONFIG2, &c, sizeof(c)); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  mpu9250_write_reg(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
//   mpu9250_write_reg(INT_PIN_CFG, 0x22);
   mpu9250_write_reg(INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
   mpu9250_write_reg(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   HAL_Delay(100);
}

void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  mpu9250_write_reg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  mpu9250_write_reg(PWR_MGMT_1, 0x01);
  mpu9250_write_reg(PWR_MGMT_2, 0x00);
  HAL_Delay(200);

// Configure device for bias calculation
  mpu9250_write_reg(INT_ENABLE, 0x00);   // Disable all interrupts
  mpu9250_write_reg(FIFO_EN, 0x00);      // Disable FIFO
  mpu9250_write_reg(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  mpu9250_write_reg(I2C_MST_CTRL, 0x00); // Disable I2C master
  mpu9250_write_reg(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  mpu9250_write_reg(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  mpu9250_write_reg(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  mpu9250_write_reg(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  mpu9250_write_reg(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  mpu9250_write_reg(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  mpu9250_write_reg(USER_CTRL, 0x40);   // Enable FIFO
  mpu9250_write_reg(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  mpu9250_write_reg(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  mpu9250_read_reg(FIFO_COUNTH,  &data[0], 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    mpu9250_read_reg(FIFO_R_W,  &data[0], 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  mpu9250_write_reg(XG_OFFSET_H, data[0]);
  mpu9250_write_reg(XG_OFFSET_L, data[1]);
  mpu9250_write_reg(YG_OFFSET_H, data[2]);
  mpu9250_write_reg(YG_OFFSET_L, data[3]);
  mpu9250_write_reg(ZG_OFFSET_H, data[4]);
  mpu9250_write_reg(ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  mpu9250_read_reg(XA_OFFSET_H, &data[0] ,2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  mpu9250_read_reg(YA_OFFSET_H, &data[0], 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  mpu9250_read_reg(ZA_OFFSET_H,  &data[0], 2);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
//  mpu9250_write_reg(XA_OFFSET_H, data[0]);
//  mpu9250_write_reg(XA_OFFSET_L, data[1]);
//  mpu9250_write_reg(YA_OFFSET_H, data[2]);
//  mpu9250_write_reg(YA_OFFSET_L, data[3]);
//  mpu9250_write_reg(ZA_OFFSET_H, data[4]);
//  mpu9250_write_reg(ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  mpu9250_read_reg(ACCEL_XOUT_H ,rawData, 14);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

float getGres(struct mpu9250 * mpu9250) {
  uint8_t Gscale = mpu9250->Gscale;
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    case GFS_250DPS:
    		mpu9250->_gRes = 250.0/32768.0;
          return mpu9250->_gRes;
          break;
    case GFS_500DPS:
    		mpu9250->_gRes = 500.0/32768.0;
          return mpu9250->_gRes;
          break;
    case GFS_1000DPS:
    		mpu9250->_gRes = 1000.0/32768.0;
         return mpu9250->_gRes;
         break;
    case GFS_2000DPS:
    		mpu9250->_gRes = 2000.0/32768.0;
         return mpu9250->_gRes;
         break;
  }
  return 0.0;
}

float getAres(struct mpu9250 * mpu9250) {
  uint8_t Ascale = mpu9250->Ascale;
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
		 mpu9250->_aRes = 2.0f/32768.0f;
         return mpu9250->_aRes;
         break;
    case AFS_4G:
		mpu9250->_aRes = 4.0f/32768.0f;
         return mpu9250->_aRes;
         break;
    case AFS_8G:
		mpu9250->_aRes = 8.0f/32768.0f;
         return mpu9250->_aRes;
         break;
    case AFS_16G:
		mpu9250->_aRes = 16.0f/32768.0f;
         return mpu9250->_aRes;
         break;
  }
  return 0.0;
}

void updateData(struct mpu9250 * mpu9250, float dt, float vel){
	readMPU9250Data(mpu9250->rawData);

  // Update buffers' values
	for (int i = 0; i<3; i++){
		mpu9250->accBuff[i][mpu9250->buffPointer] = (float)(mpu9250->rawData[i] * getAres(mpu9250));
	}
	for (int i = 4; i<7; i++){
		mpu9250->gyroBuff[i-4][mpu9250->buffPointer] = (float)(mpu9250->rawData[i] * getGres(mpu9250));
	}

	mpu9250->buffPointer = (mpu9250->buffPointer+1) % filt_size;
	mpu9250->lastAngVel = mpu9250->gyro[2];

  // Calculate moving average's new iteration
	for(int i = 0 ; i < 3 ; i++){
		mpu9250->acc[i] = 0;
		mpu9250->gyro[i] = 0;
		for(int j = 0 ; j < filt_size ; j++){
			mpu9250->acc[i]+= mpu9250->accBuff[i][j];
			mpu9250->gyro[i]+= mpu9250->gyroBuff[i][j];
		}
		mpu9250->acc[i]/= filt_size;
		mpu9250->gyro[i]/= filt_size;
	}

  // Update orientation
	// 1.1 = gyroscope's error
	mpu9250->pose[2] += 1.1* dt * (mpu9250->gyro[2] + mpu9250->lastAngVel) / 2;
	if(mpu9250->pose[2] < -180)
		mpu9250->pose[2] += 360;
	else if(mpu9250->pose[2] > 180)
		mpu9250->pose[2] -= 360;

  //Update position
  mpu9250->pose[0] += cos(M_PI/180 * mpu9250->pose[2]) * dt * vel;
  mpu9250->pose[1] += sin(M_PI/180 * mpu9250->pose[2]) * dt * vel;
}

void setPose(struct mpu9250 * mpu9250, float *pos){
	for(int i = 0 ; i < 3 ; i++){
		mpu9250->pose[i] = pos[i];
	}
}

