#include"mpu_sensor.h"

int DR_mpu6050_atualizar(dr_mpu_data_t *sensor)
{
    uint8_t leitura[14] = { 0 };
    uint8_t ready = 0;
    int8_t status_erro;
    erro_watch[1] = 100;
    /* interrupï¿½ï¿½o dado Pronto */
    while (((ready && 1) != 1) && erro_watch[1] > 0)
    {
        status_erro = DR_mpu_read(sensor->I2C, sensor->address, 0x3A, &ready);
        if (status_erro != 1)
        {
            DR_mpu_ligar(100);
            DR_mpu6050_init(sensor);
            diagnostic_erro[0]++;	// Comunication Error
            return status_erro; // erro nos cabos de comunicaï¿½ï¿½o

        }
        erro_watch[1]--;
    }
    if (erro_watch[1] == 0)
    {
        DR_mpu_ligar(100);
        DR_mpu6050_init(sensor);
        diagnostic_erro[1]++; // Supply Error
        return -1; // erro nos cabos de alimentaï¿½ï¿½o
    }

    DR_mpu_readraw(sensor->I2C, sensor->address, 0x3B, 14, leitura); // Ta dando erro aqui dentro
    sensor->ax = (leitura[0] << 8) | (leitura[1]);
    sensor->ay = (leitura[2] << 8) | (leitura[3]);
    sensor->az = (leitura[4] << 8) | (leitura[5]);
    sensor->temp = (leitura[6] << 8) | (leitura[7]);
    sensor->gx = (leitura[8] << 8) | (leitura[9]);
    sensor->gy = (leitura[10] << 8) | (leitura[11]);
    sensor->gz = (leitura[12] << 8) | (leitura[13]);
    return 1;
}
int DR_mpu6050_init(dr_mpu_data_t *sensor)
{
    uint8_t data = 0;
    uint16_t status_erro;
    status_erro = DR_mpu_read(sensor->I2C, sensor->address, 0x75, &data) - 10;
    if (data != 0x68)
    {
        return status_erro;
    }
    //Acelerometro mede a resoluï¿½ï¿½o +-2g
    DR_i2c_write(sensor->I2C, sensor->address, 0x1C, 0b00000000);
    //pequeno filtro digital
    DR_i2c_write(sensor->I2C, sensor->address, 0x1A, 0b00000001);
    //liga interrupï¿½ï¿½o dado pronto
    DR_i2c_write(sensor->I2C, sensor->address, 0x38, 0b00000001);
    //DESLIGA GYRO	//data = 0b00000111;
    DR_i2c_write(sensor->I2C, sensor->address, 0x6C, 0b00000000);
    //configura Gyro em  ï¿½ 250 ï¿½/s
    DR_i2c_write(sensor->I2C, sensor->address, 0x1B, 0b00000000);
    //Divisor de clock - amostragem //OR = 1Khz / (1+data)
    DR_i2c_write(sensor->I2C, sensor->address, 0x19, 24);
    //ACORDA
    DR_i2c_write(sensor->I2C, sensor->address, 0x6B, 0b00000000);
    return 1;
}
void DR_mpu_ligar(uint16_t tempo_ms)
{
    /* Rotina para ligar o sensor, ou sensores */
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
    DR_delay_ms(tempo_ms);  // mï¿½nmo 50
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
    DR_delay_ms(tempo_ms);
}
int DR_mpu_read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                uint8_t *data)
{

    /* Selecionar I2C */
    EUSCI_B_Type *moduleI2C;
    if (n_I2C == 0)
    {
        moduleI2C = EUSCI_B0;
    }
    else if (n_I2C == 1)
    {
        moduleI2C = EUSCI_B1;
    }
    else if (n_I2C == 2)
    {
        moduleI2C = EUSCI_B2;
    }
    else
    {
        moduleI2C = EUSCI_B3;
    }

    // dr_I2C_read() begin
    /* Read a single byte at memAddr
     * read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(saddr+r)-ACK-data-NACK-P
     */
    erro_watch[2] = 500;
    erro_watch[3] = 500;
    erro_watch[4] = 500;

    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    /* wait until slave address is sent *//*Essa flag nï¿½o estar esperando*/
    while ((moduleI2C->CTLW0 & 2) && erro_watch[2] > 0)
    {
        erro_watch[2]--;
        diagnostic_erro[2]++; /*[2]Erro de ??*/
    }
    if (erro_watch[2] == 0)
        return -2;

    moduleI2C->TXBUF = memAddr; /* send memory address to slave */
    /* wait till it's ready to transmit */
    while (!(moduleI2C->IFG & 2) && erro_watch[3] > 0)
    {
        erro_watch[3]--;
        diagnostic_erro[3]++; /*[3] Erro de ?*/
    }

    if (erro_watch[3] == 0)
        return -3;

    moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
    moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
    while (moduleI2C->CTLW0 & 2)
        ; /* wait till restart is finished */
    moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the byte is received */

    /* wait till data is received */ // flag de erro aqui simm kk
    while (!(moduleI2C->IFG & 1) && erro_watch[4] > 0)
    {
        erro_watch[4]--;
        diagnostic_erro[4]++;
    }
    if (erro_watch[4] == 0)
        return -4;

    *data = moduleI2C->RXBUF; /* read the received data */
    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */
    // dr_I2C_read() end
    return 1;
}
int DR_mpu_readraw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                   uint8_t byteCount, uint8_t *data)
{
    /* Selecionar I2C */ //0.001ms
    EUSCI_B_Type *moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0;
        break;
    case 1:
        moduleI2C = EUSCI_B1;
        break;
    case 2:
        moduleI2C = EUSCI_B2;
        break;
    case 3:
        moduleI2C = EUSCI_B3;
        break;
    default:
        moduleI2C = EUSCI_B0;
        break;
    }
// Variaveis de Robustez à erro de conexão
    erro_watch[2] = 500;
    erro_watch[3] = 500;
    erro_watch[4] = 500;

    /* Read Raw begin */
    if (byteCount <= 0)
        return -1; /* no read was performed */

    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    /* wait until slave address is sent */
    while ((moduleI2C->CTLW0 & 2) && erro_watch[2] > 0)
    {
        erro_watch[2]--;
        diagnostic_erro[2]++; /*[2]Erro de ??*/
    }
    if (erro_watch[2] == 0)
        return -2;
    /* send memory address to slave */
    moduleI2C->TXBUF = memAddr;
    /* wait till last transmit is done */
    while (!(moduleI2C->IFG & 2) && erro_watch[3] > 0)
    {
        erro_watch[3]--;
        diagnostic_erro[3]++; /*[3] Erro de ?*/
    }
    if (erro_watch[3] == 0)
        return -3;
    /* enable receiver */
    moduleI2C->CTLW0 &= ~0x0010;
    moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
    while (moduleI2C->CTLW0 & 2)
        ; /* wait till RESTART is finished */

    /* receive data one byte at a time */
    do
    {
        if (byteCount == 1) /* when only one byte of data is left */
            moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

        /* wait till data is received */
        while (!(moduleI2C->IFG & 1) && erro_watch[4] > 0)
        {
            erro_watch[4]--;
            diagnostic_erro[4]++;
        }
        if (erro_watch[4] == 0)
            return -4;

        *data++ = moduleI2C->RXBUF; /* read the received data */
        byteCount--;
    }
    while (byteCount);

    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */

    return 0; /* no error */
}
void DR_gyroscope_calibrate(dr_mpu_data_t *sensor, uint16_t calibration_size)
{ /*Rotina de Calibração Estática do Giroscopio*/
    uint16_t i = calibration_size;

    DR_mpu9250_atualizar(sensor);

    sensor->max_gx = sensor->gx;
    sensor->min_gx = sensor->gx;
    sensor->max_gy = sensor->gy;
    sensor->min_gy = sensor->gy;
    sensor->max_gz = sensor->gz;
    sensor->min_gz = sensor->gz;

    while (i > 0)
    {
        DR_mpu9250_atualizar(sensor);
        // Higher and lower X-axis Value
        if (sensor->gx > sensor->max_gx)
        {
            sensor->max_gx = sensor->gx;
        }
        if (sensor->gx < sensor->min_gx)
        {
            sensor->min_gx = sensor->gx;
        }
        // Higher and lower Y-axis Value
        if (sensor->gy > sensor->max_gy)
        {
            sensor->max_gy = sensor->gy;
        }
        if (sensor->gy < sensor->min_gy)
        {
            sensor->min_gy = sensor->gy;
        }
        // Higher and lower Z-axis Value
        if (sensor->gz > sensor->max_gz)
        {
            sensor->max_gz = sensor->gz;
        }
        if (sensor->gz < sensor->min_gz)
        {
            sensor->min_gz = sensor->gz;
        }
        DR_delay_k(1);
        i--;
    }
    sensor->gyro_offset_x = (sensor->max_gx + sensor->min_gx) / 2; // Offset  x-axis
    sensor->gyro_offset_y = (sensor->max_gy + sensor->min_gy) / 2; // Offset  y-axis
    sensor->gyro_offset_z = (sensor->max_gz + sensor->min_gz) / 2; // Offset  z-axis
}
int8_t DR_magnetometer_calibrate(dr_mpu_data_t *sensor)
{/*Dynamical Calibration of Magnetometer - Set Variables max and min (x,y,z)*/
    static uint32_t contador = 0;
    if (contador == 0)
    {
        DR_mpu_readraw(0, 0x0C, 0x03, 7, leitura);
        sensor->mx = (leitura[1] << 8) | (leitura[0]); // HXL HXH
        sensor->my = (leitura[3] << 8) | (leitura[2]);
        sensor->mz = (leitura[4] << 8) | (leitura[4]);

        sensor->max_mx = sensor->mx;
        sensor->min_mx = sensor->mx;
        sensor->max_my = sensor->my;
        sensor->min_my = sensor->my;
        sensor->max_mz = sensor->mz;
        sensor->min_mz = sensor->mz;
        contador++;
        return 1;
    }
    else if (contador < (SIZE_MAGNETOMETER_CALIBRATION - 1))
    {
        DR_mpu_readraw(0, 0x0C, 0x03, 7, leitura);
        sensor->mx = (leitura[1] << 8) | (leitura[0]); // HXL HXH
        sensor->my = (leitura[3] << 8) | (leitura[2]);
        sensor->mz = (leitura[4] << 8) | (leitura[4]);
        // Higher and lower X-axis Value
        if (sensor->mx > sensor->max_mx)
        {
            sensor->max_mx = sensor->mx;
        }
        if (sensor->mx < sensor->min_mx)
        {
            sensor->min_mx = sensor->mx;
        }
        // Higher and lower Y-axis Value
        if (sensor->my > sensor->max_my)
        {
            sensor->max_my = sensor->my;
        }
        if (sensor->my < sensor->min_my)
        {
            sensor->min_my = sensor->my;
        }
        // Higher and lower Z-axis Value
        if (sensor->mz > sensor->max_mz)
        {
            sensor->max_mz = sensor->mz;
        }
        if (sensor->mz < sensor->min_mz)
        {
            sensor->min_mz = sensor->mz;
        }
        contador++;
        return 1;
    }
    else
    {
        sensor->mag_offset_x = (sensor->max_mx + sensor->min_mx) / 2; // Offset  x-axis
        sensor->mag_offset_y = (sensor->max_my + sensor->min_my) / 2; // Offset  y-axis
        sensor->mag_offset_z = (sensor->max_mz + sensor->min_mz) / 2; // Offset  z-axis
        contador++;
        return 0;
    }
}

int DR_mpu9250_init(dr_mpu_data_t *sensor)
{
// Set accelerometers low pass filter at 5Hz (ACCEL_CONFIG 2)
    DR_i2c_write(sensor->I2C, sensor->address, 29, 0x06);
// Set gyroscope low pass filter at 5Hz (CONFIG)
    DR_i2c_write(sensor->I2C, sensor->address, 26, 0x06);
// Configure gyroscope range(GYRO_FS_SEL)
    DR_i2c_write(sensor->I2C, sensor->address, 27, 0x10);
// Configure accelerometers range (ACCEL_FS_SEL)
    DR_i2c_write(sensor->I2C, sensor->address, 28, 0x08);
// Set by pass mode for the magnetometers
    DR_i2c_write(sensor->I2C, sensor->address, 0x37, 0x02);
// Request continuous magnetometer measurements in 16 bits !!!
    DR_i2c_write(sensor->I2C, 0x0C, 0x0A, 0x16);
    return 1;
}
int DR_mpu9250_atualizar(dr_mpu_data_t *sensor)
{
// Leitura do AcelerÃªometro e Giroscopio
    uint8_t leitura[14];
    DR_mpu_readraw(0, 0x68, 0x3B, 14, leitura);
    sensor->ax = (leitura[0] << 8) | (leitura[1]); // AccXH XL
    sensor->ay = (leitura[2] << 8) | (leitura[3]);
    sensor->az = (leitura[4] << 8) | (leitura[5]);
    sensor->temp = (leitura[6] << 8) | (leitura[7]);
    sensor->gx = (leitura[8] << 8) | (leitura[9]);
    sensor->gy = (leitura[10] << 8) | (leitura[11]);
    sensor->gz = (leitura[12] << 8) | (leitura[13]);
// Leitura do Magnetometro
    DR_mpu_readraw(0, 0x0C, 0x03, 7, leitura);
    sensor->mx = (leitura[1] << 8) | (leitura[0]); // HXL HXH
    sensor->my = (leitura[3] << 8) | (leitura[2]);
    sensor->mz = (leitura[4] << 8) | (leitura[4]);
//leitura[7];	// Dado pronto
    return 1;
}
void Dr_mag_signal_update(dr_mpu_data_t *sensor)
{
    float Mag_Num;
    float Mag_Den;
    int8_t axis_num = sensor->Axis_Magnetometer[0];
    int8_t axis_den = sensor->Axis_Magnetometer[1];
    switch (axis_num)
    {
    case -3: // - axis Z in arc tangent numerator
        Mag_Num = -(sensor->mz - sensor->mag_offset_z);
        break;
    case -2: // - axis Y in arc tangent numerator
        Mag_Num = -(sensor->my - sensor->mag_offset_y);
        break;
    case -1: // - axis X in arc tangent numerator
        Mag_Num = -(sensor->mx - sensor->mag_offset_x);
        break;
    case 1: // axis X in arc tangent numerator
        Mag_Num = sensor->mx - sensor->mag_offset_x;
        break;
    case 2: // Axis Y in arc tangent numerator
        Mag_Num = sensor->my - sensor->mag_offset_y;
        break;
    case 3: // Axis Z in arc tangent numerator
        Mag_Num = sensor->mz - sensor->mag_offset_z;
        break;
    default:
        Mag_Num = -1;
        break;
    }

    switch (axis_den)
    {
    case -3: // - axis Z in arc tangent denominator
        Mag_Den = -(sensor->mz - sensor->mag_offset_z);
        break;
    case -2: // - axis Y in arc tangent denominator
        Mag_Den = -(sensor->my - sensor->mag_offset_y);
        break;
    case -1: // - axis X in arc tangent denominator
        Mag_Den = -(sensor->mx - sensor->mag_offset_x);
        break;
    case 1: // axis X in arc tangent denominator
        Mag_Den = sensor->mx - sensor->mag_offset_x;
        break;
    case 2: // Axis Y in arc tangent denominator
        Mag_Den = sensor->my - sensor->mag_offset_y;
        break;
    case 3: // Axis Z in arc tangent denominator
        Mag_Den = sensor->mz - sensor->mag_offset_z;
        break;
    default:
        Mag_Den = -1;
        break;
    }
    sensor->mag_Signal = atan2f(Mag_Num, Mag_Den)
            - sensor->calibration_offset_mag;
}
void Dr_acc_signal_update(dr_mpu_data_t *sensor)
{
    float Acc_Num;
    float Acc_Den;
    int8_t axis_num = sensor->Axis_Accelerometer[0];
    int8_t axis_den = sensor->Axis_Accelerometer[1];
    switch (axis_num)
    {
    case -3: // - axis Z in arc tangent numerator
        Acc_Num = -(sensor->az - sensor->acc_offset_z) * ACC_RESOLUTION;
        break;
    case -2: // - axis Y in arc tangent numerator
        Acc_Num = -(sensor->ay - sensor->acc_offset_y) * ACC_RESOLUTION;
        break;
    case -1: // - axis X in arc tangent numerator
        Acc_Num = -(sensor->ax - sensor->acc_offset_x) * ACC_RESOLUTION;
        break;
    case 1: // axis X in arc tangent numerator
        Acc_Num = (sensor->ax - sensor->acc_offset_x) * ACC_RESOLUTION;
        break;
    case 2: // Axis Y in arc tangent numerator
        Acc_Num = (sensor->ay - sensor->acc_offset_y) * ACC_RESOLUTION;
        break;
    case 3: // Axis Z in arc tangent numerator
        Acc_Num = (sensor->az - sensor->acc_offset_z) * ACC_RESOLUTION;
        break;
    default:
        Acc_Num = -1;
        break;
    }

    switch (axis_den)
    {
    case -3: // - axis Z in arc tangent denominator
        Acc_Den = -(sensor->az - sensor->acc_offset_z) * ACC_RESOLUTION;
        break;
    case -2: // - axis Y in arc tangent denominator
        Acc_Den = -(sensor->ay - sensor->acc_offset_y) * ACC_RESOLUTION;
        break;
    case -1: // - axis X in arc tangent denominator
        Acc_Den = -(sensor->ax - sensor->acc_offset_x) * ACC_RESOLUTION;
        break;
    case 1: // axis X in arc tangent denominator
        Acc_Den = (sensor->ax - sensor->acc_offset_x) * ACC_RESOLUTION;
        break;
    case 2: // Axis Y in arc tangent denominator
        Acc_Den = (sensor->ay - sensor->acc_offset_y) * ACC_RESOLUTION;
        break;
    case 3: // Axis Z in arc tangent denominator
        Acc_Den = (sensor->az - sensor->acc_offset_z) * ACC_RESOLUTION;
        break;
    default:
        Acc_Den = -1;
        break;
    }
    sensor->acc_Signal = atan2f(Acc_Num, Acc_Den);
}
void Dr_gyro_signal_update(dr_mpu_data_t *sensor)
{
    int8_t axis = sensor->Axis_Gyroscope;
    switch (axis)
    {
    case -3: // - axis Z
        sensor->gyro_Signal = -(sensor->gz - sensor->gyro_offset_z)
                * GYRO_RESOLUTION;
        break;
    case -2: // - axis Y
        sensor->gyro_Signal = -(sensor->gy - sensor->gyro_offset_y)
                * GYRO_RESOLUTION;
        break;
    case -1: // - axis X
        sensor->gyro_Signal = -(sensor->gx - sensor->gyro_offset_x)
                * GYRO_RESOLUTION;
        break;
    case 1: // axis X
        sensor->gyro_Signal = (sensor->gx - sensor->gyro_offset_x)
                * GYRO_RESOLUTION;
        break;
    case 2: // Axis Y
        sensor->gyro_Signal = (sensor->gy - sensor->gyro_offset_y)
                * GYRO_RESOLUTION;
        break;
    case 3: // Axis Z
        sensor->gyro_Signal = (sensor->gz - sensor->gyro_offset_z)
                * GYRO_RESOLUTION;
        break;
    default:
        sensor->gyro_Signal = -1;
        break;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMPLEMENTAR FUTURAMENTE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void initAK8963(float *destination)
//{
//// First extract the factory calibration for each magnetometer axis
//    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
//    DR_i2c_write(0, 0x0C, 0x0A, 0x00); // Power down magnetometer
//    DR_delay_k(1);
//    DR_i2c_write(0, 0x0C, 0x0A, 0x0F); // Enter Fuse ROM access mode
//    DR_delay_k(1);
//    DR_i2c_readraw(0, 0x0C, 0X10, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
//    destination[0] = (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
//    destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
//    destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
//    DR_i2c_write(0, 0x0C, 0x0A, 0x00); // Power down magnetometer
//    DR_delay_k(1);
//// Configure the magnetometer for continuous read and highest resolution
//// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
//// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
//    DR_i2c_write(0, 0x0C, 0x0A, 1 << 4 | 0x02); // Set magnetometer data resolution and sample ODR
//    DR_delay_k(1);
//}

