//rotary_encoder.c
#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rotary_encoder.h"

static int numberofencoders = 0;

void update_encoders()
{
    struct Encoder *encoder = encoders;
    for (; encoder < encoders + numberofencoders; encoder++)
    {
        int MSB = digitalRead(encoder->pin_a);
        int LSB = digitalRead(encoder->pin_b);

        int encoded = (MSB << 1) | LSB;
        int sum = (encoder->lastEncoded << 2) | encoded;

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder->value++;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder->value--;

        encoder->lastEncoded = encoded;
    }
}

struct Encoder *setup_encoder(int pin_a, int pin_b)
{
    if (numberofencoders > MAX_ENCODERS)
    {
        printf("Maximum number of encodered exceded: %i\n", MAX_ENCODERS);
        return NULL;
    }

    struct Encoder *newencoder = encoders + numberofencoders++;
    newencoder->pin_a = pin_a;
    newencoder->pin_b = pin_b;
    newencoder->value = 0;
    newencoder->lastEncoded = 0;

    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    pullUpDnControl(pin_a, PUD_UP);
    pullUpDnControl(pin_b, PUD_UP);
    wiringPiISR(pin_a, INT_EDGE_BOTH, update_encoders);
    wiringPiISR(pin_b, INT_EDGE_BOTH, update_encoders);

    return newencoder;
}