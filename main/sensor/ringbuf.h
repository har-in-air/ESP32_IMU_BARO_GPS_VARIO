#ifndef RINGBUF_H_
#define RINGBUF_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RINGBUF_SIZE    15

typedef struct RINGBUF_ {
   int head;
   float buffer[RINGBUF_SIZE];
} RINGBUF;

void ringbuf_init();
void ringbuf_addSample(float sample);
float ringbuf_averageOldestSamples(int numSamples);

#ifdef __cplusplus
}
#endif
#endif
