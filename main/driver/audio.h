#ifndef AUDIO_H_
#define AUDIO_H_

#ifdef __cplusplus
extern "C" {
#endif

void audio_config( int pnDacChan);
void audio_setFrequency(int frequencyHz);
void audio_generateTone(int freqHz, int milliseconds);
#ifdef __cplusplus
}
#endif

#endif
