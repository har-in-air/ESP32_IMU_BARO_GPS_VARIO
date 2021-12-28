#ifndef AUDIO_H_
#define AUDIO_H_

void audio_config( int pnDacChan);
void audio_set_frequency(int frequencyHz);
void audio_generate_tone(int freqHz, int milliseconds);

#endif
