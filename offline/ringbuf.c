//#include "common.h"
//#include "config.h"
#include <string.h>
#include "ringbuf.h"

static RINGBUF RingBuf;

void ringbuf_init() {
   memset(RingBuf.buffer,0,RINGBUF_SIZE);
   RingBuf.head = RINGBUF_SIZE-1;
   }

void ringbuf_addSample(float sample) {
   RingBuf.head++;
   if (RingBuf.head >= RINGBUF_SIZE) RingBuf.head = 0;
   RingBuf.buffer[RingBuf.head] = sample;
   }


float ringbuf_averageOldestSamples(int numSamples) {
   int index = RingBuf.head+1; // oldest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index >= RINGBUF_SIZE) index = 0;
      accum += RingBuf.buffer[index];
      index++;
      }
   return accum/numSamples;
   }   

