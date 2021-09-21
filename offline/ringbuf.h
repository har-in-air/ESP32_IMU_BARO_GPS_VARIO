#ifndef RINGBUF_H_
#define RINGBUF_H_

#define RINGBUF_SIZE    20

typedef struct RINGBUF_ {
   int head;
   float buffer[RINGBUF_SIZE];
} RINGBUF;

void ringbuf_init();
void ringbuf_addSample(float sample);
float ringbuf_averageOldestSamples(int numSamples);
float ringbuf_averageNewestSamples(int numSamples);

#endif
