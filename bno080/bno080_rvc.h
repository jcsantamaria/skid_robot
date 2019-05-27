#ifndef BNO080_RVC_H
#define BNO080_RVC_H

#define UART_DEVICE    "/dev/ttyAMA0"   // default UART device
#define RSTN_GPIO_PIN  27               // wiringPi numbering convention for BCM GPIO 16

typedef struct
{
    uint32_t timestamp;     // ms
    uint8_t index;          // sequence
    float yaw;              // deg
    float pitch;            // deg
    float roll;             // deg
    float acc_x;            // m/s2
    float acc_y;            // m/s2
    float acc_z;            // m/s2
} RvcReport_t;

typedef void (RvcReport_Callback_t)(RvcReport_t *pReport);


#ifdef __cplusplus
extern "C" {
#endif

    bool bno080_rvc_init(char const *device, int resetPin, RvcReport_Callback_t *callback);

    void bno080_rvc_debugging(bool debug);

#ifdef __cplusplus
}    // end of extern "C"
#endif

#endif  // end of include guard