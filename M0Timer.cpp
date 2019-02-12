#include "M0Timer.h"
#include <Arduino.h>

M0TimerClass M0Timer;

// Place holder values for the callback functions
void (* M0TimerClass::_TC3Callback)(uint8_t t) = 0;
void (* M0TimerClass::_TC4Callback)(uint8_t t) = 0;
void (* M0TimerClass::_TC5Callback)(uint8_t t) = 0;

// Start with fired values as false. The user can choose to use these
// rather than assign a callback
volatile boolean M0TimerClass::_TC3Fired = false;
volatile boolean M0TimerClass::_TC4Fired = false;
volatile boolean M0TimerClass::_TC5Fired = false;

// Whether or not these timers are single use (automatically
// stopped and reset when they trigger for the first time)
boolean M0TimerClass::_TC3SingleUse = false;
boolean M0TimerClass::_TC4SingleUse = false;
boolean M0TimerClass::_TC5SingleUse = false;

// Goal reps is the number of times the actual timer needs to fire before the
// library version of the timer fires. This is to support timers times greater
// than one second
uint16_t M0TimerClass::_goalReps[] = {1,1,1};

// Cur reps is the number of times the actual timer has fired so far.
uint16_t M0TimerClass::_curReps[] = {0,0,0};

// 1- 1 / 10001

// n / (n + 1-n%1)

// 0.9461538462

// Start a timer with the given period. Values for t can be 3, 4, or 5
void M0TimerClass::startTimer(double period, uint8_t t){
  TcCount16* tc = getTimer(t);

  period = period / 1000;
  if (tc != 0) {

    // Do some calculations to account for the fact that the period can be
    // greater than 1
    double p = period;

    // If the period is greater than 1...
    if (period > 1) {

        // If the user specified a time that will overflow the counter and
        // goal vars, then return now.
        if (period > 65000) {
          return;
        }

        // Set the period based on the following formula:
        // n / (n + 1-n%1)
        // This ensures that the period is always below 1 (but as close to it
        // as possible)
        p = period / (period + 1.0 - fmod(period,1.0));

        // Set the integer number of times we need to count to our newly
        // calculated period to reach the actual user's goal. For example,
        // if period = 1.5, p = 0.75 and _goalReps = 2 (IE we need to count
        // up to 0.75s twice to reach the goal of 1.5s)
        _goalReps[t - 3] = (int) (period + 1.0 - fmod(period,1.0));
    } else {

      // If the user's period was not greater than 1, then we "disable" the
      // goal counting system by setting the counting goal to 1
      _goalReps[t - 3] = 1;
    }

    // Note that since we are starting this timer now, it is currently at rep
    // number 0
    _curReps[t - 3] = 0;

    // Formally start the timer
    _startTimer(p, tc);
  }
}

// Stop the given timer number. Values can be 3, 4, or 5
void M0TimerClass::stopTimer(uint8_t t) {
  TcCount16* tc = getTimer(t);
  if (tc != 0) {
    _tcReset(tc);
    _tcDisable(tc);
  }
}

// Get the TcCount16* value for the given timer number. Can be 3, 4, or 5
TcCount16* M0TimerClass::getTimer(uint8_t t) {
  switch (t) {
    case 3:
      return (TcCount16*) TC3;
    case 4:
      return (TcCount16*) TC4;
    case 5:
      return (TcCount16*) TC5;
    default:
      return 0;
  }
}

// 48000000 / (1024 * 1/0.01)

// Internal helper function for setting the timer's frequency
void M0TimerClass::_setTimerFrequency(double period, TcCount16 * TC) {
  // Calculate the frequency to use based on the period given
  double frequencyHz = (int) 1 / period;
  int compareValue = (int) (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz));
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// Internal helper function for starting a timer with a given period
void M0TimerClass::_startTimer(double period, TcCount16 * TC) {

  if (TC == (TcCount16*) TC3) {
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  } else if (TC == (TcCount16*) TC4 || TC == (TcCount16*) TC5) {
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5) ;
  } else {
    Serial.println("ERR");
    return;
  }

  //const uint8_t GCLK_SRC = 4;
  // // Set GCLK 4 to 1 times divider
  // GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_SRC) | GCLK_GENDIV_DIV(4);
  // while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  //
  // // Set GCLK 4 To use the 32KHz low power internal clock and to run in stdby
  // GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |
  //         GCLK_GENCTRL_SRC_OSC8M | // OXCULP32K
  //         GCLK_GENCTRL_ID(GCLK_SRC) //|
  //         //GCLK_GENCTRL_RUNSTDBY;
  // while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  //
  // // Enable TC3 (via the Power Manager) [TODO Not needed?]
  // PM->APBCMASK.reg |= PM_APBCMASK_TC3;
  //
  // // Set to use GCLK 4 for TCC2 and TC3
  // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
  //         GCLK_CLKCTRL_GEN(GCLK_SRC) |
  //         GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
  // while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  while (GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  // Disable the timer
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  // IMPORTANT - this automatically resets the timer when we reach the compare
  // register (and prevents an overrun)
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024 (32)
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  _setTimerFrequency(period, TC);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  IRQn_Type irq;

  if (TC == (TcCount16*) TC3) {
    irq = TC3_IRQn;
  }else if (TC == (TcCount16*) TC4) {
    irq = TC4_IRQn;
  }else if (TC == (TcCount16*) TC5) {
    irq = TC5_IRQn;
  }else {
    Serial.println("ERR");
    irq = TC3_IRQn;
  }
  NVIC_DisableIRQ(irq);
  NVIC_ClearPendingIRQ(irq);
  NVIC_SetPriority(irq, 0);
  NVIC_EnableIRQ(irq);

  // Enable the timer
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// Reset a timer
void M0TimerClass::_tcReset(TcCount16* tc)
{
  tc->CTRLA.reg = TC_CTRLA_SWRST;
  while (tc->STATUS.reg & TC_STATUS_SYNCBUSY);
  while (tc->CTRLA.bit.SWRST);
}

// Disable a timer (pause or stop)
void M0TimerClass::_tcDisable(TcCount16* tc)
{
  tc->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc->STATUS.reg & TC_STATUS_SYNCBUSY);
}

// Return whether or not TC3 has fired. If it has, then reset the
// flag
boolean M0TimerClass::getTC3Fired() {
  if (_TC3Fired) {
    _TC3Fired = false;
    return true;
  } else {
    return false;
  }
}

// Return whether or not TC4 has fired. If it has, then reset the
// flag
boolean M0TimerClass::getTC4Fired() {
  if (_TC4Fired) {
    _TC4Fired = false;
    return true;
  } else {
    return false;
  }
}

// Return whether or not TC5 has fired. If it has, then reset the
// flag
boolean M0TimerClass::getTC5Fired() {
  if (_TC5Fired) {
    _TC5Fired = false;
    return true;
  } else {
    return false;
  }
}

// Timer handlers
// Also could use INTFLAG.bit.MC1 (if we set "TC->CC[1].reg" in _setTimerFrequency)
// instead.
// Could use INTFLAG.bit.OVF to detect a timer overflow (which should never
// happen because we are using a 1024 predivider with a 16 bit timer limited to
// one second or less)
void TC3_Handler() {
  TcCount16* tc = (TcCount16*) TC3;
  if (tc->INTFLAG.bit.MC0 == 1) {
    tc->INTFLAG.bit.MC0 = 1;

    // Count up the number of reps this timer has taken
    M0Timer._curReps[0] ++;

    // If we have reached our rep goal, then proceeed
    if (M0Timer._curReps[0] >= M0Timer._goalReps[0]) {
      // reset the counter for this timer
      M0Timer._curReps[0] = 0;

      // Note that this timer has been fired
      M0Timer._TC3Fired = true;

      // Execute the user's callback if they defined one
      if(M0Timer._TC3Callback != 0) {
        (*M0Timer._TC3Callback)(3);
      }

      // If this timer was a single use timer, then stop it
      if (M0Timer._TC3SingleUse) {
        M0Timer.stopTimer(3);
      }
    }
  }
}

void TC4_Handler() {
  TcCount16* tc = (TcCount16*) TC4;
  if (tc->INTFLAG.bit.MC0 == 1) {
    tc->INTFLAG.bit.MC0 = 1;

    // Count up the number of reps this timer has taken
    M0Timer._curReps[1] ++;

    // If we have reached our rep goal, then proceeed
    if (M0Timer._curReps[1] >= M0Timer._goalReps[1]) {
      // reset the counter for this timer
      M0Timer._curReps[1] = 0;

      // Note that this timer has been fired
      M0Timer._TC4Fired = true;

      // Execute the user's callback if they defined one
      if(M0Timer._TC4Callback != 0) {
        (*M0Timer._TC4Callback)(4);
      }

      // If this timer was a single use timer, then stop it
      if (M0Timer._TC4SingleUse) {
        M0Timer.stopTimer(4);
      }
    }
  }
}

void TC5_Handler() {
  TcCount16* tc = (TcCount16*) TC5;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (tc->INTFLAG.bit.MC0 == 1) {
    tc->INTFLAG.bit.MC0 = 1;

    // Count up the number of reps this timer has taken
    M0Timer._curReps[2] ++;

    // If we have reached our rep goal, then proceeed
    if (M0Timer._curReps[2] >= M0Timer._goalReps[2]) {
      // reset the counter for this timer
      M0Timer._curReps[2] = 0;

      // Note that this timer has been fired
      M0Timer._TC5Fired = true;

      // Execute the user's callback if they defined one
      if(M0Timer._TC5Callback != 0) {
        (*M0Timer._TC5Callback)(5);
      }

      // If this timer was a single use timer, then stop it
      if (M0Timer._TC5SingleUse) {
        M0Timer.stopTimer(5);
      }
    }
  }
}
