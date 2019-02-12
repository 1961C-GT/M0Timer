#ifndef _M0TIMER_H_INCLUDED
#define _M0TIMER_H_INCLUDED

#define CPU_HZ 48000000 //1000000 //48000000

// This system uses 16 bit timers. The typical clock of the M0 is
// 48,000,000 Hz, which means that when divided by 1024:
//   48,000,000 / 1,024 = 46,875
// This is less that 2^16 (65,536) and therefore will never overflow before
// the timer is reset
#define TIMER_PRESCALER_DIV 1024

#include <Arduino.h>

class M0TimerClass {
public:

  static const uint8_t T3 = 0;
  static const uint8_t T4 = 1;
  static const uint8_t T5 = 2;

  static boolean setup(uint8_t);

  static boolean start(double period, uint8_t t);
  static boolean stop(uint8_t t);

  static TcCount16* getTimer(uint8_t t);

  static void attachTC3Handler(void (* handleNewCallback)(uint8_t t)) { _TC3Callback = handleNewCallback; };
  static void attachTC4Handler(void (* handleNewCallback)(uint8_t t)) { _TC4Callback = handleNewCallback; };
  static void attachTC5Handler(void (* handleNewCallback)(uint8_t t)) { _TC5Callback = handleNewCallback; };

  static void setTC3SingleUse(boolean val = true) { _TC3SingleUse = val; };
  static void setTC4SingleUse(boolean val = true) { _TC4SingleUse = val; };
  static void setTC5SingleUse(boolean val = true) { _TC5SingleUse = val; };

  static boolean getTC3Fired();
  static boolean getTC4Fired();
  static boolean getTC5Fired();

  // The following are only public so that the TC3, TC4, and TC5 handles can respond
  static void (* _TC3Callback)(uint8_t t);
  static void (* _TC4Callback)(uint8_t t);
  static void (* _TC5Callback)(uint8_t t);

  static volatile boolean _TC3Fired;
  static volatile boolean _TC4Fired;
  static volatile boolean _TC5Fired;

  static boolean _TC3SingleUse;
  static boolean _TC4SingleUse;
  static boolean _TC5SingleUse;

  static uint16_t _goalReps[3];
  static uint16_t _curReps[3];


private:

  static void _setTimerPeriod(double period, TcCount16 * TC);
  static void _configureTimer(double period, TcCount16 * TC);
  static void _configureIRQ(TcCount16 * TC);
  static void _tcReset(TcCount16 * TC);
  static void _tcDisable(TcCount16 * TC);
  static void _tcEnable(TcCount16* TC);

};
extern M0TimerClass M0Timer;
#endif
