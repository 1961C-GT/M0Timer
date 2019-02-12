#ifndef _M0TIMER_H_INCLUDED
#define _M0TIMER_H_INCLUDED

#define CPU_HZ 48000000 //1000000 //48000000

// This system uses 16 bit timers. The typical clock of the M0 is
// 48,000,000 Hz, which means that when divided by 1024:
//   48,000,000 / 1,024 = 46,875
// This is less that 2^16 (65,536) and therefore will never overflow before
// the timer is reset
#define TIMER_PRESCALER_DIV 1024

// Time between when we calculate the final timer period and when the timer
// actually starts
#define OFFSET_PERIOD_ENABLE_MICROS 22

// Time between the start of the interrupt and when we record the interrupt
// fire time
#define OFFSET_INT_TIME 3

// Minimum allowed period in us
#define TIMER_THRESH 15

// Final calibration of the timer period (added automatically)
#define TIMER_CALAB -6

#include <Arduino.h>

class M0TimerClass {
public:

  static const uint8_t T3 = 0;
  static const uint8_t T4 = 1;
  static const uint8_t T5 = 2;

  static boolean setup(uint8_t);

  static boolean start(int period, uint8_t t, boolean calcOffset = false);
  static boolean start(int period, uint8_t t, uint32_t offset);

  static boolean startms(int period, uint8_t t, boolean calcOffset = false);
  static boolean startms(int period, uint8_t t, uint32_t offset);
  static boolean stop(uint8_t t);

  static TcCount16* getTimer(uint8_t t);

  static void attachTC3Handler(void (* handleNewCallback)(uint8_t t)) { _TC3Callback = handleNewCallback; };
  static void attachTC4Handler(void (* handleNewCallback)(uint8_t t)) { _TC4Callback = handleNewCallback; };
  static void attachTC5Handler(void (* handleNewCallback)(uint8_t t)) { _TC5Callback = handleNewCallback; };

  static void setSingleUse(uint8_t t, boolean val = true);
  static boolean getFired(uint8_t t);


  // The following are only public so that the TC3, TC4, and TC5 handles can respond
  static void (* _TC3Callback)(uint8_t t);
  static void (* _TC4Callback)(uint8_t t);
  static void (* _TC5Callback)(uint8_t t);

  static volatile boolean _fired[];
  static boolean _singleUse[];

  static uint16_t _goalReps[3];
  static uint16_t _curReps[3];

  static volatile uint32_t _intTime[];


private:

  static void _setTimerPeriod(int period, TcCount16 * TC, boolean calcOffset);
  static void _configureTimer(int period, TcCount16 * TC, boolean calcOffset);
  static void _configureIRQ(TcCount16 * TC);
  static void _tcReset(TcCount16 * TC);
  static void _tcDisable(TcCount16 * TC);
  static void _tcEnable(TcCount16* TC);

};
extern M0TimerClass M0Timer;
#endif
