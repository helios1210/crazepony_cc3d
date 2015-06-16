#include "stm32f10x.h"
#include "MPU6000.h"
#include "UART1.h"
#include "config.h"
// Registers
#define MPU6000_PRODUCT_ID      	0x0C
#define MPU6000_SMPLRT_DIV	    	0x19
#define MPU6000_GYRO_CONFIG	    	0x1B
#define MPU6000_ACCEL_CONFIG  		0x1C
#define MPU6000_FIFO_EN		    	0x23
#define MPU6000_INT_PIN_CFG	    	0x37
#define MPU6000_INT_ENABLE	    	0x38
#define MPU6000_INT_STATUS	    	0x3A
#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48
#define MPU6000_USER_CTRL	    	0x6A
#define MPU6000_SIGNAL_PATH_RESET   0x68
#define MPU6000_PWR_MGMT_1	    	0x6B
#define MPU6000_PWR_MGMT_2	    	0x6C
#define MPU6000_FIFO_COUNTH	    	0x72
#define MPU6000_FIFO_COUNTL	    	0x73
#define MPU6000_FIFO_R_W		   	0x74
#define MPU6000_WHOAMI		    	0x75

// Bits
#define BIT_SLEEP				    0x40
#define BIT_H_RESET				    0x80
#define BITS_CLKSEL				    0x07
#define MPU_CLK_SEL_PLLGYROX	    0x01
#define MPU_CLK_SEL_PLLGYROZ	    0x03
#define MPU_EXT_SYNC_GYROX		    0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN			    0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA		    0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A
#if 1		//CC3D MPU6000 Specific by zhangxch
#define MPU6000_CS_GPIO GPIOA  
#define MPU6000_CS_PIN GPIO_Pin_4 
#define MPU6000_SPI_INSTANCE  SPI1
#define SPI_0_5625MHZ_CLOCK_DIVIDER 128
#define SPI_18MHZ_CLOCK_DIVIDER     2
#define SPI_9MHZ_CLOCK_DIVIDER      4
#define MPU6000_WHO_AM_I_CONST              (0x68)
#define MPU6000_CONFIG		    	0x1A

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define NULL ((void *)0)  
#endif 
#define DISABLE_MPU6000       GPIO_SetBits(MPU6000_CS_GPIO,   MPU6000_CS_PIN)
#define ENABLE_MPU6000        GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN)
#define true 1
#define false 0
static int mpuSpi6000InitDone = false;

static volatile uint16_t spi1ErrorCount = 0;
static volatile uint16_t spi2ErrorCount = 0;
uint16_t acc_1G = 256;

//typedef signed char int8_t;
//typedef unsigned char uint8_t;
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
void mpu6000SpiAccRead(int16_t *gyroData);
void mpu6000SpiGyroRead(int16_t *gyroData);
int spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);
uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);

void MPU6000_CS_init(void)
{
				GPIO_InitTypeDef GPIO_InitStructure;
	
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);
        //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
				//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
      
				GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;
				GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
				GPIO_ResetBits(GPIOA,GPIO_Pin_4);
				printf("MPU6000_CS_init done...\r\n");
}

void SPIIinit()
{
		RCC->APB2ENR |= 1<<12;//使能SPI1 时钟
    RCC->APB2ENR |= 1<<2;
    //配置服用功能输出 
    GPIOA->CRL&=0X000FFFFF;  
    GPIOA->CRL|=0XBBB00000;//PA5.6.7 复用，推挽输出 50M时钟 （不能配置成开漏，否则输出为锯齿波）    
    GPIOA->ODR|=0X7<<5; 
    SPI1->CR1|=0<<11;//8bit数据格式  
    SPI1->CR1|=0<<10;//全双工模式  
    SPI1->CR1|=1<<9; //软件nss 管理 
    SPI1->CR1|=1<<8;
    SPI1->CR1|=0<<7; //MSBfirst  
    SPI1->CR1|=7<<3; //设置时钟Fsck=Fcpu/256 
    SPI1->CR1|=1<<2; //SPI 主机 
    SPI1->CR1|=1<<1; //空闲模式下 SCK为 1 CPOL=1 
    SPI1->CR1|=1<<0; //数据采样从第二个时间边沿开始
    SPI1->CR1|=1<<6; //使能  SPI 
}

static void mpu6000WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, reg);
    spiTransferByte(MPU6000_SPI_INSTANCE, data);
    DISABLE_MPU6000;
}

static void mpu6000ReadRegister(uint8_t reg, uint8_t *data, int length)
{
    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, reg | 0x80); // read transaction
		//printf("func mpu6000ReadRegister spiTransferByte done \n");
    spiTransfer(MPU6000_SPI_INSTANCE, data, NULL, length);
		//printf("func mpu6000ReadRegister spiTransfer done \n");
    DISABLE_MPU6000;
}

void mpu6000SpiGyroInit(void)
{
}

void mpu6000SpiAccInit(void)
{
    acc_1G = 512 * 8;
}

int mpu6000SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;
    if (mpuSpi6000InitDone) {
			//printf("func mpu6000SpiDetect mpuSpi6000InitDone error \n");
        return true;
    }

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    mpu6000WriteRegister(MPU6000_PWR_MGMT_1, BIT_H_RESET);

    do {
        delay(150);

        mpu6000ReadRegister(MPU6000_WHOAMI, &in, 1);
			//printf("func mpu6000SpiDetect mpu6000ReadRegister in = %d \n",in);
        if (in == MPU6000_WHO_AM_I_CONST) {
					//printf("func mpu6000SpiDetect in == MPU6000_WHO_AM_I_CONST done! break \n");
            break;
        }
        if (!attemptsRemaining) {
					//printf("func mpu6000SpiDetect !attemptsRemaining error \n");
            return false;
        }
    } while (attemptsRemaining--);


    mpu6000ReadRegister(MPU6000_PRODUCT_ID, &in, 1);
		//printf("func mpu6000SpiDetect MPU6000_PRODUCT_ID in =%d \n",in);
    /* look for a product ID we recognise */

    // verify product revision
    switch (in) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return false;
}

void mpu6000AccAndGyroInit() {

    if (mpuSpi6000InitDone) {
        return;
    }

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    // Device Reset
    mpu6000WriteRegister(MPU6000_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    mpu6000WriteRegister(MPU6000_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    mpu6000WriteRegister(MPU6000_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(1);

    // Disable Primary I2C Interface
    mpu6000WriteRegister(MPU6000_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(1);

    mpu6000WriteRegister(MPU6000_PWR_MGMT_2, 0x00);
    delayMicroseconds(1);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    mpu6000WriteRegister(MPU6000_SMPLRT_DIV, 0x00);
    delayMicroseconds(1);

    // Accel +/- 8 G Full Scale
    mpu6000WriteRegister(MPU6000_ACCEL_CONFIG, BITS_FS_8G);
    delayMicroseconds(1);

    // Gyro +/- 1000 DPS Full Scale
    mpu6000WriteRegister(MPU6000_GYRO_CONFIG, BITS_FS_2000DPS);
    delayMicroseconds(1);

    mpuSpi6000InitDone = true;
}

int mpu6000SpiAccDetect(acc_t *acc)
{
		uint8_t buf0,buf1;
		int16_t gyroData[3];
    if (!mpu6000SpiDetect()) {
			printf("Acc mpu6000SpiDetect error \n");
        return false;
    }

    spiResetErrorCounter(MPU6000_SPI_INSTANCE);
		//printf("Acc spiResetErrorCounter ok \n");
    mpu6000AccAndGyroInit();
		//printf("Acc mpu6000AccAndGyroInit ok \n");
    //acc->init = mpu6000SpiAccInit;
		//??????????????acc_1G = 512 * 8;
    //acc->read = mpu6000SpiAccRead;


    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    mpu6000ReadRegister(MPU6000_ACCEL_XOUT_H, &buf0, 1);
		mpu6000ReadRegister(MPU6000_ACCEL_XOUT_L, &buf1, 1);
    gyroData[X] = (int16_t)((buf0 << 8) | buf1);
		//printf("MPU6000 ACC : gyroData[X] = %d;\n",gyroData[X]);
		
		mpu6000ReadRegister(MPU6000_ACCEL_YOUT_H, &buf0, 1);
		mpu6000ReadRegister(MPU6000_ACCEL_YOUT_L, &buf1, 1);
    gyroData[Y] = (int16_t)((buf0 << 8) | buf1);
		//printf("MPU6000 ACC : gyroData[Y] = %d;\n",gyroData[Y]);
		
		mpu6000ReadRegister(MPU6000_ACCEL_ZOUT_H, &buf0, 1);
		mpu6000ReadRegister(MPU6000_ACCEL_ZOUT_L, &buf1, 1);
    gyroData[Z] = (int16_t)((buf0 << 8) | buf1);
		//printf("MPU6000 ACC : gyroData[Z] = %d;\n",gyroData[Z]);
	  printf("MPU6000 ACC : gyroData[X] = %d;gyroData[Y] = %d;gyroData[Z] = %d;\n",gyroData[X],gyroData[Y],gyroData[Z]);
		//printf("Acc init read ok \n");
    delay(100);
		//printf("Acc delay ok \n");
    return 0;
}

int mpu6000SpiGyroDetect(gyro_t *gyro)
{
		uint8_t mpuLowPassFilter = BITS_DLPF_CFG_42HZ;
    int16_t data[3];
    if (!mpu6000SpiDetect()) {
			printf("Gyro mpu6000SpiDetect error \n");
        return false;
    }

    spiResetErrorCounter(MPU6000_SPI_INSTANCE);

    mpu6000AccAndGyroInit();

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    // Accel and Gyro DLPF Setting
    mpu6000WriteRegister(MPU6000_CONFIG, mpuLowPassFilter);
    delayMicroseconds(1);

    mpu6000SpiGyroRead(data);

    if ((((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) || spiGetErrorCounter(MPU6000_SPI_INSTANCE) != 0) {
        spiResetErrorCounter(MPU6000_SPI_INSTANCE);
        return false;
    }
    gyro->init = mpu6000SpiGyroInit;
    gyro->read = mpu6000SpiGyroRead;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    //gyro->scale = (4.0f / 16.4f) * (M_PIf / 180.0f) * 0.000001f;
    delay(100);
		printf("MPU6000 GYRO : data[0] = %d;data[1] = %d;data[2] = %d;\n",data[0],data[1],data[2]);
    return 0;
}

void mpu6000SpiGyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    mpu6000ReadRegister(MPU6000_GYRO_XOUT_H, buf, 6);

    gyroData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[Z] = (int16_t)((buf[4] << 8) | buf[5]);
}

void mpu6000SpiAccRead(int16_t *gyroData)
{
    uint8_t buf[6];

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    mpu6000ReadRegister(MPU6000_ACCEL_XOUT_H, buf, 6);

    gyroData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[Z] = (int16_t)((buf[4] << 8) | buf[5]);
	  printf("MPU6000 ACC : data[X] = %d;data[Y] = %d;data[Z] = %d;\n",gyroData[X],gyroData[Y],gyroData[Z]);
}


// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    uint16_t spiTimeout = 1000;

    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

    SPI_I2S_SendData(instance, data);
    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

    return ((uint8_t)SPI_I2S_ReceiveData(instance));
}
uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        spi1ErrorCount++;
    } else if (instance == SPI2) {
        spi2ErrorCount++;
    }
    return -1;
}
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(instance, DISABLE);

    tempRegister = instance->CR1;

    switch (divisor) {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }
		
    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);
}
int spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    uint16_t spiTimeout = 1000;

    uint8_t b;
    instance->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
        SPI_I2S_SendData(instance, b);
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
        b = SPI_I2S_ReceiveData(instance);
        if (out)
            *(out++) = b;
    }

    return true;
}
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}
uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        return spi1ErrorCount;
    } else if (instance == SPI2) {
        return spi2ErrorCount;
    }
    return 0;
}
void spiResetErrorCounter(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        spi1ErrorCount = 0;
    } else if (instance == SPI2) {
        spi2ErrorCount = 0;
    }
}
