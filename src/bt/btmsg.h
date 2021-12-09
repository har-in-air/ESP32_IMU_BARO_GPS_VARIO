#ifndef BTMSG_H_
#define BTMSG_H_

bool btmsg_init();
void btmsg_genLK8EX1(char* szmsg, int32_t altm, int32_t cps, float batVoltage);
void btmsg_genXCTRC(char* szmsg);
void btmsg_tx_message(const char* szmsg);

#endif
