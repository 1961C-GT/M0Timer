#include "M0Timer.h"
#include <Arduino.h>

M0TimerClass M0Timer;

// Place holder values for the callback functions
void (* M0TimerClass::_TC3Callback)(uint8_t t) = 0;
void (* M0TimerClass::_TC4Callback)(uint8_t t) = 0;
void (* M0TimerClass::_TC5Callback)(uint8_t t) = 0;

// Start with fired values as false. The user can choose to use these
// rather than assign a callback
volatile boolean M0TimerClass::_fired[] = {0,0,0};

// Whether or not these timers are single use (automatically
// stopped and reset when they trigger for the first time)
boolean M0TimerClass::_singleUse[] = {0,0,0};

// Goal reps is the number of times the actual timer needs to fire before the
// library version of the timer fires. This is to support timers times greater
// than one second
uint16_t M0TimerClass::_goalReps[] = {1,1,1};

// Cur reps is the number of times the actual timer has fired so far.
uint16_t M0TimerClass::_curReps[] = {0,0,0};

// Record of exactly when an interrupt was fired
volatile uint32_t M0TimerClass::_intTime[] = {0,0,0};

// Counter for setting offsets
uint32_t _internalCounter = 0;

// Constant for making frequency calculations easier
const double timerCoeff = (double) CPU_HZ / (double) TIMER_PRESCALER_DIV / 1000000.0;


// If val is true, set the given timer to true. If val is false, set the given
// timer to muili-use
void M0TimerClass::setSingleUse(uint8_t t, boolean val) {
  _singleUse[t] = val;
}

// 1- 1 / 10001

// n / (n + 1-n%1)

// 0.9461538462

boolean M0TimerClass::startms(int period, uint8_t t, boolean calcOffset) {
  return start(period * 1000, t, calcOffset);
}
boolean M0TimerClass::startms(int period, uint8_t t, uint32_t offset) {
  return start(period * 1000, t, offset);
}

boolean M0TimerClass::start(int period, uint8_t t, boolean calcOffset) {
  return start(period, t, (calcOffset ? micros() : 0));
}

// Start a timer with the given period. Values for t can be 3, 4, or 5
boolean M0TimerClass::start(int period, uint8_t t, uint32_t offset){

  boolean calcOffset = offset > 0;

  // Serial.println(calcOffset ? "Calculating Offset" : "No Offset");

  _internalCounter = offset;

  TcCount16* TC = getTimer(t);
  // period = period / 1000;
  if (TC != 0) {

    // Do some calculations to account for the fact that the period can be
    // greater than 1
    int p = period + TIMER_CALAB;

    // If the period is greater than 1...
    if (period > 1000000) {

        // If the user specified a time that will overflow the counter and
        // goal vars, then return now.
        if (period > 65000000000) {
          return false;
        }

        // Set the period based on the following formula:
        // n / (n + 1-n%1)
        // This ensures that the period is always below 1 (but as close to it
        // as possible)
        p = (int) ((double) period / ((double) period + 1000000.0 - fmod((double) period,1000000.0)) * 1000000.0);

        // S    mS       uS
        // 1 -> 1,000 -> 1,000,000
        //

        // Set the integer number of times we need to count to our newly
        // calculated period to reach the actual user's goal. For example,
        // if period = 1.5, p = 0.75 and _goalReps = 2 (IE we need to count
        // up to 0.75s twice to reach the goal of 1.5s)
        _goalReps[t] = (int) (((double) period + 1000000.0 - fmod((double) period,1000000.0)) / 1000000.0);

        if (calcOffset) {
          calcOffset = false;
          Serial.println("Calculate Offset not supported for periods of more than one second");
        }
    } else {

      // If the user's period was not greater than 1, then we "disable" the
      // goal counting system by setting the counting goal to 1
      _goalReps[t] = 1;
    }

    // Automatically set offset adjusted timers to single use
    if (calcOffset) {
      _singleUse[t] = true;
    }

    // Note that since we are starting this timer now, it is currently at rep
    // number 0
    _curReps[t] = 0;

    // ===== Formally start the timer ===== //
    _configureTimer(p, TC, calcOffset);

    // Enable the timer
    _tcEnable(TC);

    // uint32_t now = micros();
    // Serial.print("Time to set: "); Serial.println(now - _internalCounter);

    return true;
  }

  return false;
}

// Start the timer using the previously configured period settings. This does
// not reset the timer. To do so, use stop first
// boolean M0TimerClass::start(uint8_t t){
//   TcCount16* tc = getTimer(t);
//   if (tc != 0) {
//     _tcEnable(tc);
//     return true;
//   }
// }

// Stop the given timer number. Values can be 3, 4, or 5
boolean M0TimerClass::stop(uint8_t t) {
  TcCount16* tc = getTimer(t);
  if (tc != 0) {
    _tcDisable(tc);
    _tcReset(tc);
    return true;
  }
  return false;
}

boolean M0TimerClass::setup(uint8_t t) {
  TcCount16* TC = getTimer(t);
  if (TC != 0) {
    _configureIRQ(TC);
    return true;
  }
  return false;
}

// Get the TcCount16* value for the given timer number. Can be 3, 4, or 5
TcCount16* M0TimerClass::getTimer(uint8_t t) {
  switch (t) {
    case 0:
      return (TcCount16*) TC3;
    case 1:
      return (TcCount16*) TC4;
    case 2:
      return (TcCount16*) TC5;
    default:
      return 0;
  }
}

// 48000000 / (1024 * 1/0.01)

// Internal helper function for setting the timer's frequency
void M0TimerClass::_setTimerPeriod(int period, TcCount16 * TC, boolean calcOffset) {
  // Calculate the frequency to use based on the period given
  if (calcOffset) {
    // Calibrate the selected period based on how long it has been since the
    // user defined offset. This will ensure that the timer fires exactly
    // `period` microseconds after the given offset time (in micros).
    uint32_t since = micros() - _internalCounter;
    period -= (since + OFFSET_PERIOD_ENABLE_MICROS);
  }

  // If period ends up being less than TIMER_THRESH, then we are late!
  // Set period to TIMER_THRESH anyway and let the timer execute.
  if (period < TIMER_THRESH) {
    period = TIMER_THRESH;
  }

  int compareValue = (int) (timerCoeff * period);

  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void M0TimerClass::_configureIRQ(TcCount16 * TC) {
  if (TC == (TcCount16*) TC3) {
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  } else if (TC == (TcCount16*) TC4 || TC == (TcCount16*) TC5) {
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5) ;
  } else {
    Serial.println("ERR");
    return;
  }
  while (GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

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

}

// Internal helper function for starting a timer with a given period
void M0TimerClass::_configureTimer(int period, TcCount16 * TC, boolean calcOffset) {
  // Disable the timer
  _tcDisable(TC);

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

  _setTimerPeriod(period, TC, calcOffset);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

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

void M0TimerClass::_tcEnable(TcCount16* TC)
{
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// Return whether or not given timer has fired. If it has, then reset the
// flag
boolean M0TimerClass::getFired(uint8_t t) {
  if (_fired[t]) {
    _fired[t] = false;
    return true;
  }
  return false;
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
      // First set the time at which this one fired;
      M0Timer._intTime[M0Timer.T3] = micros() - OFFSET_INT_TIME;

      // reset the counter for this timer
      M0Timer._curReps[0] = 0;

      // Note that this timer has been fired
      M0Timer._fired[M0Timer.T3] = true;

      // If this timer was a single use timer, then stop it
      if (M0Timer._singleUse[M0Timer.T3]) {
        M0Timer.stop(M0Timer.T3);
      }

      // Execute the user's callback if they defined one
      if(M0Timer._TC3Callback != 0) {
        (*M0Timer._TC3Callback)(M0Timer.T3);
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
      // First set the time at which this one fired;
      M0Timer._intTime[M0Timer.T4] = micros() - OFFSET_INT_TIME;

      // reset the counter for this timer
      M0Timer._curReps[1] = 0;

      // Note that this timer has been fired
      M0Timer._fired[M0Timer.T4] = true;

      // If this timer was a single use timer, then stop it
      if (M0Timer._singleUse[M0Timer.T4]) {
        M0Timer.stop(M0Timer.T4);
      }

      // Execute the user's callback if they defined one
      if(M0Timer._TC4Callback != 0) {
        (*M0Timer._TC4Callback)(M0Timer.T4);
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
      // First set the time at which this one fired;
      M0Timer._intTime[M0Timer.T5] = micros() - OFFSET_INT_TIME;

      // reset the counter for this timer
      M0Timer._curReps[2] = 0;

      // Note that this timer has been fired
      M0Timer._fired[M0Timer.T5] = true;

      // If this timer was a single use timer, then stop it
      if (M0Timer._singleUse[M0Timer.T5]) {
        M0Timer.stop(M0Timer.T5);
      }

      // Execute the user's callback if they defined one
      if(M0Timer._TC5Callback != 0) {
        (*M0Timer._TC5Callback)(M0Timer.T5);
      }
    }
  }
}
