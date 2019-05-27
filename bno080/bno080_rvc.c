#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "bno080_rvc.h"

#define RESET_DELAY    (10) // [mS]
#define RSTN_GPIO_PIN  27

#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif

static const float G2MS2 = 9.80665f;

// --- Forward declarations -------------------------------------------

PI_THREAD(rxThread);
static void bno080_reset(bool state);
static char * bno080_readHeaderLine(char* line, int size);
static bool bno080_rx();

typedef PACKED_STRUCT
{
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t checksum;
} RvcRawReport_t;

// --- Private data ---------------------------------------------------

static int                   _uartfd = -1;      // UART file descriptor
static bool                  _debug = false;    // debug flag
static RvcReport_Callback_t *_callback = NULL;  // callback

void bno080_rvc_debugging(bool debug)
{
    _debug = debug;
}

bool bno080_rvc_init(char const *device, int resetPin, RvcReport_Callback_t *callback)
{
    // initialize wiringPI
    wiringPiSetup();

    // set RSTN pin
    pinMode(RSTN_GPIO_PIN, OUTPUT);

    // open the UART port
    _uartfd = serialOpen(device, 115200);
    if (_uartfd < 0)
    {
        if (_debug)
            printf("bno080_rvc_init: unable to setup UART device: '%s':  %s\n", device, strerror(errno));
        return false;
    }
    if ( _debug)
        printf("bno080_rvc_init: UART device: '%s' success!\n", device);

    // set callback
    _callback = callback;

    // Create receive thread
    piThreadCreate(rxThread);

    // Assert reset
    bno080_reset(false);

    // Wait for reset to take effect
    delay(RESET_DELAY);

    // Deassert reset
    bno080_reset(true);

    return true;
}

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

PI_THREAD(rxThread)
{
    if (_debug)
        printf("rxThread: starting\n");

    if (!(_uartfd < 0))
    {
        // get three header lines
        char line[256];
        bno080_readHeaderLine(line, sizeof(line));
        if (_debug)
            printf("rx: %s", line);
        bno080_readHeaderLine(line, sizeof(line));
        if (_debug)
            printf("rx: %s", line);
        bno080_readHeaderLine(line, sizeof(line));
        if (_debug)
            printf("rx: %s\n", line);
    }

    RvcRawReport_t buffer;
    RvcReport_t    report;
    while(bno080_rx(&buffer))
    {
        report.timestamp = millis();
        report.index     = buffer.index;
        report.yaw       = buffer.yaw / 100.0f;
        report.pitch     = buffer.pitch / 100.0f;
        report.roll      = buffer.roll / 100.0f;
        report.acc_x     = buffer.acc_x * G2MS2 / 1000.0f;
        report.acc_y     = buffer.acc_y * G2MS2 / 1000.0f;
        report.acc_z     = buffer.acc_z * G2MS2 / 1000.0f;

        if (_debug)
        {
            printf("ts: %6d index: %02X yaw: %7.2f pitch: %7.2f roll: %7.2f acc_x: %7.2f acc_y: %7.2f acc_z: %7.2f\n",
                report.timestamp,
                report.index,
                report.yaw,
                report.pitch,
                report.roll,
                report.acc_x,
                report.acc_y,
                report.acc_z
            );
        }

        if (_callback != NULL)
            _callback(&report);
    }

    if (_debug)
        printf("rxThread: ending\n");
}

static void bno080_reset(bool state)
{
    digitalWrite(RSTN_GPIO_PIN, state ? 1 : 0);
    int r = digitalRead(RSTN_GPIO_PIN);
    if (_debug)
        printf("bno080_reset: %d\n", r);
}

static char * bno080_readHeaderLine(char* line, int size)
{
    memset(line, 0, size);
    // hunt for start char
    int count = 256;
    while (serialGetchar(_uartfd) != '%' && --count > 0)
        ;
    line[0] = '%';
    for (int i = 1; i < (size - 1); i++)
    {
        line[i] = serialGetchar(_uartfd);
        if (line[i] == '\n' )
            break;
    }
    return line;
}

static bool bno080_rx(RvcRawReport_t *pReport)
{
    bool result = false;
    if (_uartfd > 0)
    {
        // hunt for header: 0xAAAA
        uint8_t found = 0;
        uint8_t data = 0x00;
        while (found < 2)
        {
            int count = read(_uartfd, &data, 1);
            if (count < 0)
                return false; //Error
            switch (found)
            {
            case 0:
                if (data == 0xAA)
                    found = 1;
                break;
            case 1:
                found = data = 0xAA ? 2 : 0;
                break;
            }
        }
    }

    // we got header!
    // read the rest
    uint8_t *buffer = (uint8_t*)pReport;
    memset(buffer, 0, sizeof(RvcRawReport_t));
    int toread = sizeof(RvcRawReport_t);
    int readin = 0;
    int count  = 0;
    while (readin < toread)
    {
        count = read(_uartfd, buffer + readin, toread - readin);
        if (count < 0)
            return false; //Error
        readin += count;
    }

    //printf("read[%d]: data:", toread);
    //for (int i = 0; i < toread; i++)
    //    printf("%02X ", buffer[i]);
    //printf("\n");

    return true;
}
