#include "common.h"
#include "config.h"
#include "btn.h"


volatile uint16_t Btn0State;
volatile uint16_t BtnLState;
volatile uint16_t BtnMState;
volatile uint16_t BtnRState;
volatile bool Btn0Pressed = false;
volatile bool BtnLPressed = false;
volatile bool BtnMPressed = false;
volatile bool BtnRPressed = false;

static const char* TAG = "btn";

void btn_clear() {
   Btn0Pressed = BtnLPressed = BtnRPressed = BtnMPressed = false;
   }

void btn_debounce() {
   Btn0State = ((Btn0State<<1) | ((uint16_t)BTN0()) );
   if ((Btn0State | 0xFFF0) == 0xFFF8) {
	   Btn0Pressed = true;
	   }    
   BtnLState = ((BtnLState<<1) | ((uint16_t)BTNL()) );
   if ((BtnLState | 0xFFF0) == 0xFFF8) {
	   BtnLPressed = true;
	   }    
   BtnMState = ((BtnMState<<1) | ((uint16_t)BTNM()) );
   if ((BtnMState | 0xFFF0) == 0xFFF8) {
	   BtnMPressed = true;
	   }    
   BtnRState = ((BtnRState<<1) | ((uint16_t)BTNR()) );
   if ((BtnRState | 0xFFF0) == 0xFFF8) {
	   BtnRPressed = true;
	   }
   }
