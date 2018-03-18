#ifndef BTN_H_
#define BTN_H_


extern volatile uint16_t Btn0State;
extern volatile uint16_t BtnLState;
extern volatile uint16_t BtnMState;
extern volatile uint16_t BtnRState;
extern volatile bool Btn0Pressed;
extern volatile bool BtnLPressed;
extern volatile bool BtnMPressed;
extern volatile bool BtnRPressed;

void btn_clear();
void btn_debounce();

#endif
