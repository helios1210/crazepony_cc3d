typedef void (*sensorInitFuncPtr)(void);                    // sensor init prototype
typedef void (*sensorReadFuncPtr)(int16_t *data);           // sensor read and align prototype
typedef struct acc_s {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;
typedef struct gyro_s {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor
} gyro_t;

int mpu6000SpiAccDetect(acc_t *acc);
int mpu6000SpiGyroDetect(gyro_t *gyro);

void mpu6000SpiGyroRead(int16_t *gyroADC);
void mpu6000SpiAccRead(int16_t *gyroADC);
void MPU6000_CS_init(void);
void SPIIinit(void);