#ifndef AUDIO_H_
#define AUDIO_H_

void audio_config( int pnDacChan);
void audio_setFrequency(int frequencyHz);
void audio_generateTone(int freqHz, int milliseconds);

#endif
