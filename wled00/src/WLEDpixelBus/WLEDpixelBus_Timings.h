/*-------------------------------------------------------------------------
WLEDpixelBus - Lightweight LED driver library for WLED

written by Damian Schneider @dedehai 2026
-------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "../../const.h"

namespace WLEDpixelBus {

// Note on timings regarding I2S/LCD/SPI buses (4 step cadence):
// the drivers will always use a 1/4 - 3/4 duty cycle for '0' and '1' bits (e.g. 250ns high + 750ns low for a 1us period)
// since the t0h timing is the most critical (according to my testing) make it accurate and derive the rest

/**
 * LED timing parameters in nanoseconds
 */
struct LedTiming {
  uint16_t t0h_ns;    // '0' bit high time
  uint16_t t0l_ns;    // '0' bit low time
  uint16_t t1h_ns;    // '1' bit high time
  uint16_t t1l_ns;    // '1' bit low time
  uint32_t reset_us;  // Reset/latch time in microseconds

  constexpr LedTiming(uint16_t t0h, uint16_t t0l, uint16_t t1h, uint16_t t1l, uint32_t reset)
    : t0h_ns(t0h), t0l_ns(t0l), t1h_ns(t1h), t1l_ns(t1l), reset_us(reset) {}

  // Calculate bit period in ns: used in hw-parallel buses only. since t0h is most critical and we do 4-step encoding, make the period 4*t0h
  // we do not have the flexibility of RMT or BB outputs: t1h is fixed at t0h*3
  constexpr uint32_t bitPeriod() const {
    return t0h_ns * 4;
  }
};

static constexpr uint32_t WLEDPB_TRANSFER_TIMEOUT_MS = 1000; // max allowed time for a full transfer, recovery is triggered if exceeded

// ---------------------------------------------------------------------------
// 4-step cadence helpers for parallel buses (I2S, Parallel SPI, PARLIO, ESP8266)
// ---------------------------------------------------------------------------

// Compute target clock frequency in Hz for 4-step cadence (4 clock cycles per bit period).
inline uint32_t calc4StepClockHz(const LedTiming& timing) {
  uint32_t period = timing.bitPeriod();
  if (period == 0) period = 1250;
  return 4000000000UL / period;
}

// Compute number of DMA bytes for the reset period appended to a frame.
// dmaBytesPerLedBit = bytes consumed in DMA for one full 4-step LED bit i.e. one bitPeriod
inline size_t calc4StepResetDmaBytes(const LedTiming& timing, uint8_t dmaBytesPerLedBit) {
  uint32_t resetNs = timing.reset_us * 1000;
  uint32_t bitPeriodNs = timing.bitPeriod() + 1; // +1 to prevent div by zero
  uint32_t zeroCycles = resetNs / bitPeriodNs;
  return (size_t)zeroCycles * dmaBytesPerLedBit;
}

// Compute DMA buffer size aligned to 4 bytes and clamped to bounds.
inline size_t calc4StepBufferSize(size_t maxSrcBytes, uint8_t dmaBytesPerSrcByte, uint8_t bufCount, size_t minSize, size_t maxSize) {
  if (bufCount == 0) bufCount = 1;
  size_t sz = (dmaBytesPerSrcByte * maxSrcBytes) / bufCount;
  sz = (sz + 3) & ~3; // 4-byte align
  if (sz > maxSize) sz = maxSize;
  if (sz < minSize) sz = minSize;
  return sz;
}

/**
 * Scale LED timing parameters by a floating point factor (percent expressed as factor, e.g. 1.2 for +20%)
 * Only t* timings (ns) are scaled; reset_us is preserved.
 */
inline LedTiming scaleTiming(const LedTiming& timing, float factor) {
  auto s = [&](uint32_t v)->uint16_t {
    uint32_t r = (uint32_t)(v * factor + 0.5f);
    if (r < 1) r = 1;
    if (r > 0xFFFF) r = 0xFFFF;
    return (uint16_t)r;
  };
  return LedTiming(s(timing.t0h_ns), s(timing.t0l_ns), s(timing.t1h_ns), s(timing.t1l_ns), timing.reset_us);
}

} // namespace WLEDpixelBus
