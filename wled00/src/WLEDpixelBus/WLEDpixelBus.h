/*-------------------------------------------------------------------------
WLEDpixelBus - Lightweight LED driver library for WLED

written by Damian Schneider @dedehai 2026

-------------------------------------------------------------------------*/

#pragma once

#include <Arduino.h>
#include <cstring>

#if defined(ARDUINO_ARCH_ESP32)

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"

// I2S support: targets where the LCD Intel 8080 bus is present (ESP32, ESP32-S2, ESP32-S3)
#if SOC_LCD_I80_BUSES
  #define WLEDPB_I2S_SUPPORT
#endif

// SPI parallel support (C3 - uses SPI quad mode with GDMA)
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define WLEDPB_PARALLEL_SPI_SUPPORT
#endif

// ESP_HAS_HIGH_GPIO_BANK — defined when the target has GPIO > 31 and therefore
// needs the second GPIO output register bank (GPIO_OUT1_W1TS/TC_REG).
#if SOC_GPIO_PIN_COUNT > 32
#  define ESP_HAS_HIGH_GPIO_BANK 1
#endif

#ifdef WLEDPB_PARALLEL_SPI_SUPPORT
#include "soc/spi_struct.h"
#include "soc/gdma_struct.h"
#include "hal/gdma_ll.h"
#include "soc/gdma_reg.h"
#include "rom/lldesc.h"
#endif

#elif defined(ESP8266)
// nothing to do here

#else
#error "WLEDpixelBus only supports ESP32 and ESP8266 platforms"
#endif

#include "WLEDpixelBus_Timings.h"
#include "WLEDpixelBus_Features.h"
#include "../../colors.h"

namespace WLEDpixelBus {

using ::CctPixel;

//==============================================================================
// CCT (Warm White / Cool White) Support
//==============================================================================

//==============================================================================
// Driver State
//==============================================================================

enum class DriverState : uint8_t {
  Idle = 0,
  Sending = 1
};

//==============================================================================
// DMA Buffer Configuration
//==============================================================================

constexpr size_t MIN_DMA_BUFFER_SIZE = 256;
constexpr size_t MAX_DMA_BUFFER_SIZE = 4092; // 12bit in DMA descriptor
#ifdef WLEDPB_PARALLEL_SPI_SUPPORT // C3 only
  constexpr size_t DEFAULT_DMA_BUFFER_SIZE = (1024*2); // must be a multiple of 16 (16 DMA bytes per source byte), clocked out at ~2.6MHz, 4 bits per clock (2k per buffer means about 1ms interrupt intervals with 4 step cadence)
#else
  #if SOC_RMT_TX_CANDIDATES_PER_GROUP > 4 // supports 8 RMT
  constexpr size_t DEFAULT_DMA_BUFFER_SIZE = MAX_DMA_BUFFER_SIZE; // note: 3k is enough except if using 8RMT (ESP32 classic), also requires triple buffering
  #else
  constexpr size_t DEFAULT_DMA_BUFFER_SIZE = (1024*3);
  #endif
#endif
static_assert(DEFAULT_DMA_BUFFER_SIZE <= 4092, "DEFAULT_DMA_BUFFER_SIZE exceeds the maximum possible size");

//==============================================================================
// Color Encoding Helpers
//==============================================================================

// Upper-nibble flags packed into ColorEncoder::_pixelFormat.
// Lower nibble = logical channel count.  Upper nibble = combination of these flags.
static constexpr uint8_t NCHF_16BIT  = 0x10; // 16-bit chip (UCS8903/8904, SM16825, G1151): wire bytes = logCh * 2
static constexpr uint8_t NCHF_CUSTOM = 0x40; // custom channel map (duplicate sources or inverted channels): _channelMap[] drives encoding
// 0x20, 0x80 reserved

// Maximum number of wire-byte entries in a custom channel map (ColorEncoder::_channelMap[]).
static constexpr uint8_t MAX_CUSTOM_CHANNELS = 6; // note: changing this wont propaget to the UI, currently ICs with more than 6 channels are not supported

// Chip-specific prefix/suffix byte lengths reserved in the encode buffer (see createBus()).
static constexpr uint8_t TM1814_PREFIX_LEN  = 8; // C1+C2 current-config prefix
static constexpr uint8_t TM1914_PREFIX_LEN  = 6; // mode-setting prefix
static constexpr uint8_t SM16825_SUFFIX_LEN = 4; // per-frame configuration suffix

/**
 * White-channel swap selector, decoded from the upper nibble of WLED's color-order byte.
 * For 4-channel (RGBW) buses, WSWAP_B/G/R swaps the W wire position with B/G/R.
 * For 5+ channel (CCT) buses, WSWAP_WWCW swaps the warm-white and cool-white wire positions.
 */
enum WSwap : uint8_t {
  WSWAP_NONE = 0,
  WSWAP_B    = 1, // swap W & B
  WSWAP_G    = 2, // swap W & G
  WSWAP_R    = 3, // swap W & R
  WSWAP_WWCW = 4, // swap WW & CW
};

static constexpr uint8_t MAX_PREFIX_BYTES = 8;
static constexpr uint8_t MAX_SUFFIX_BYTES = 8;

// Wire description of a one-wire digital bus. User color order is not a field:
// BusDigital bakes it into channelColors once, before the encoder is built.
// Clocked 2-pin chips are not described by this struct.
struct BusProperties {
  uint8_t  numChannels = 3; // logical channels; is16bit doubles wire bytes
  uint8_t  channelColors[MAX_CUSTOM_CHANNELS] = {2, 1, 3, 0, 0, 0};
  uint8_t  invertMask = 0;
  bool     is16bit = false;
  bool     invertOutput = false;
  LedTiming timing = LedTiming(300, 900, 700, 500, 300);
  uint8_t  prefixLen = 0;
  uint8_t  prefixData[MAX_PREFIX_BYTES] = {};
  uint8_t  suffixLen = 0;
  uint8_t  suffixData[MAX_SUFFIX_BYTES] = {};
  uint8_t  gapBits[MAX_CUSTOM_CHANNELS] = {}; // zero bits sent before each wire channel, pixels are bit-packed when any is > 0 (Govee G1151: 2,2,2,2,2)
  uint8_t  gapTotal() const { uint8_t t = 0; for (uint8_t g : gapBits) t += g; return t; }
};

/**
 * Custom channel map color source, used by ColorEncoder::_channelMap[] and the
 * custom-channel-map ColorEncoder constructor. Selects which color source feeds a given wire byte.
 */
enum ChannelSource : uint8_t {
  CH_UNUSED = 0,
  CH_R      = 1,
  CH_G      = 2,
  CH_B      = 3,
  CH_W      = 4,
  CH_WW     = 5,
  CH_CW     = 6,
};

/**
 * Pixel encoder: maps RGBW uint32_t to a per-LED byte stream according to color order.
 *
 * _pixelFormat packs two things:
 *   bits[3:0]  wire bytes per pixel (= logical channels for 8-bit; logical channels * 2 for 16-bit)
 *   bits[7:4]  NCHF_* flags (16BIT | CUSTOM)
 *
 * Encoding: all 16-bit types use encode16(); 8-bit types 0x03/04/05 use the fast RGB / RGBW / CCT paths,
 * custom maps (duplicate sources, inverted channels) encodeGeneric(). Decoding: fast paths for 0x03/04/05 and
 * their 16-bit variants, decodeGeneric() for custom maps.
 *
 * getPixelBytes() returns wire bytes: logCh * 2 for 16-bit types, logCh otherwise.
 */
class ColorEncoder {
public:
  ColorEncoder() : _pixelFormat(0x03), _invertMask(0), _idxR(0), _idxG(1), _idxB(2), _idxW(3), _idxCW(4) { memset(_channelMap, 0, sizeof(_channelMap)); }
  // Custom channel map constructor for custom color orders
  // channelMap[i]: ChannelSource value (CH_UNUSED/CH_R/CH_G/CH_B/CH_W/CH_WW/CH_CW)
  ColorEncoder(const uint8_t channelMap[MAX_CUSTOM_CHANNELS], uint8_t numChannels, uint8_t invertMask, bool is16bit);

  // -------------------------------------------------------------------------
  // Fast encode — standard 8-bit types (no invert)
  // -------------------------------------------------------------------------

  inline void encodeRGB(uint32_t c, uint8_t* out) const {
    out[_idxR] = R(c); out[_idxG] = G(c); out[_idxB] = B(c);
  }
  inline void encodeRGBW(uint32_t c, uint8_t* out) const {
    out[_idxR] = R(c); out[_idxG] = G(c); out[_idxB] = B(c); out[_idxW] = W(c);
  }
  inline void encodeCCT(uint32_t c, const CctPixel& cct, uint8_t* out) const {
    out[_idxR] = R(c); out[_idxG] = G(c); out[_idxB] = B(c);
    out[_idxW] = cct.ww; out[_idxCW] = cct.cw;
  }

  // -------------------------------------------------------------------------
  // Fast decode — standard 8-bit types (no invert)
  // -------------------------------------------------------------------------

  inline uint32_t decodeRGB(const uint8_t* in) const {
    return RGBW32(in[_idxR], in[_idxG], in[_idxB], 0);
  }
  inline uint32_t decodeRGBW(const uint8_t* in) const {
    return RGBW32(in[_idxR], in[_idxG], in[_idxB], in[_idxW]);
  }
  inline uint32_t decodeCCT(const uint8_t* in) const {
    return RGBW32(in[_idxR], in[_idxG], in[_idxB], in[_idxW]); // WW→W, CW dropped (lossy)
  }

  // -------------------------------------------------------------------------
  // Fast decode — 16-bit types
  // -------------------------------------------------------------------------

  inline uint32_t decodeRGB16(const uint8_t* in) const {
    return RGBW32(readU16Hi(in, _idxR), readU16Hi(in, _idxG), readU16Hi(in, _idxB), 0);
  }
  inline uint32_t decodeRGBW16(const uint8_t* in) const {
    return RGBW32(readU16Hi(in, _idxR), readU16Hi(in, _idxG), readU16Hi(in, _idxB), readU16Hi(in, _idxW));
  }
  inline uint32_t decodeCCT16(const uint8_t* in) const {
    return RGBW32(readU16Hi(in, _idxR), readU16Hi(in, _idxG), readU16Hi(in, _idxB), readU16Hi(in, _idxW));
  }

  // -------------------------------------------------------------------------
  // Generic slow encode/decode for custom channel maps (duplicate sources, inverted channels); defined in WLEDpixelBus.cpp.
  // encodeGeneric() is 8-bit only: all 16-bit types encode with encode16()
  // -------------------------------------------------------------------------

  void     encodeGeneric(uint32_t c, const CctPixel& cct, uint8_t* out) const;

  // all 16-bit types: encode from a 16-bit value per color source (v[ChannelSource], v[CH_UNUSED] = 0), scaled by bri
  inline void encode16(const uint16_t* v, uint8_t* out, uint8_t bri) const {
    const uint8_t n = getColorChannels();
    for (uint8_t i = 0; i < n; i++) {
      uint32_t x = _channelMap[i] <= CH_CW ? v[_channelMap[i]] : 0;
      if (_invertMask & (1u << i)) x ^= 0xFFFF;
      x = (x * bri + 127) / 255;
      out[i*2]   = x >> 8;
      out[i*2+1] = x & 0xFF;
    }
  }
  uint32_t decodeGeneric(const uint8_t* in) const;

  // Accessors
  uint8_t getPixelFormat()     const { return _pixelFormat; }   // lower nibble=wire bytes, upper=NCHF_* flags
  uint8_t getColorChannels()   const { return (_pixelFormat & NCHF_16BIT) ? (_pixelFormat & 0x0F) / 2 : (_pixelFormat & 0x0F); } // logical color channels
  uint8_t getPixelBytes()      const { return _pixelFormat & 0x0F; } // wire bytes per pixel
  bool    is16bit()            const { return (_pixelFormat & NCHF_16BIT) != 0; } // 16-bit types (UCS8903/8904, SM16825, G1151)
  bool    isCustom()           const { return (_pixelFormat & NCHF_CUSTOM) != 0; } // true for non-permutation custom channel maps

private:
  uint8_t _pixelFormat; // lower nibble = bytes per pixel, upper nibble = NCHF_ flags (invert, 16-bit, custom)
  uint8_t _invertMask;  // bitmask: bit i = invert wire channel i (custom channel maps)
  uint8_t _idxR, _idxG, _idxB;  // wire byte index for R, G, B
  uint8_t _idxW, _idxCW;        // wire byte index for W (or WW) and CW (CCT types)
  uint8_t _channelMap[MAX_CUSTOM_CHANNELS]; // custom channel map: _channelMap[i] = ChannelSource for wire byte i

  // 16-bit decode helper: high byte of channel idx
  static inline uint8_t readU16Hi(const uint8_t* in, uint8_t idx) { return in[idx*2]; }
};

//==============================================================================
// Base Pixel Bus Interface
//==============================================================================

class PixelBus {
protected:
  uint8_t* _encodeBuffer = nullptr;   // encoded pixel data ready for hardware transmission
  size_t   _encodeBufferSize = 0;     // allocated size in bytes
  uint16_t _numPixels = 0;
  uint8_t  _prefixLen = 0;            // byte length of chip prefix at start of _encodeBuffer
  uint8_t  _prefixData[MAX_PREFIX_BYTES] = {};
  ColorEncoder _encoder;              // color encoder, set by derived class constructors
  uint8_t  _ledType   = 0;            // LED chip type (e.g. 31=TM1814); 0 = generic
  uint8_t* _pixelData = nullptr;      // _encodeBuffer + _prefixLen, cached to avoid per-call addition
  uint8_t  _suffixLen = 0;            // byte length of chip suffix appended after pixel data
  uint8_t  _suffixData[MAX_SUFFIX_BYTES] = {};
  bool     _inverted  = false;        // physical output signal inversion (polarity)
  uint8_t  _busBri = 255;  // brightness for color_fade() in setPixelColor() (software scaling, can be on top of hardware scaling)
  uint8_t  _encBri = 255;  // encoder brightness for 16-bit types (SM16825/UCS8903/UCS8904):
  uint8_t  _padBits = 0;   // zero bits per pixel (sum of _gapBits), pixels are bit-packed in _pixelData when > 0
  uint8_t  _gapBits[MAX_CUSTOM_CHANNELS] = {}; // zero bits before each wire channel

  // AI: below section was generated by an AI
  // Bit-packed pixels (_padBits > 0): a pixel occupies getPixelBytes()*8 + _padBits bits, MSB first, and is not byte aligned;
  // each wire channel is preceded by _gapBits[channel] zero bits. setPixelColor()/getPixelColor() encode to/decode from a
  // byte aligned scratch copy and move its bits in/out of _pixelData (packPixel()/unpackPixel()).
  static constexpr uint8_t PACKED_SCRATCH_BYTES = 0x0F + 1; // max pixel bytes (lower nibble of pixel format) + 1 byte writeBits() may read ahead

  // copy nBits from src (MSB first, starting at bit 0) into dst starting at bit dstBit, other dst bits are kept
  static constexpr void writeBits(uint8_t* dst, size_t dstBit, const uint8_t* src, uint16_t nBits) {
    for (uint16_t i = 0; i < nBits; ) {
      const size_t   d     = dstBit + i;
      const uint8_t  shift = d & 7;
      const uint16_t left  = nBits - i;           // bits still to copy
      const uint8_t  room  = 8 - shift;           // bits left in the current dst byte
      const uint8_t  take  = (left < room) ? left : room;
      const uint16_t word  = ((uint16_t)src[i >> 3] << 8) | src[(i >> 3) + 1];
      const uint8_t  bits  = (word >> (16 - (i & 7) - take)) & ((1u << take) - 1);
      const uint8_t  pos   = 8 - shift - take;
      const uint8_t  mask  = ((1u << take) - 1) << pos;
      dst[d >> 3] = (dst[d >> 3] & ~mask) | (bits << pos);
      i += take;
    }
  }
  // copy nBits from src starting at bit srcBit into dst (MSB first, starting at bit 0), one byte per step;
  // only source bytes that hold requested bits are read
  static constexpr void readBits(uint8_t* dst, const uint8_t* src, size_t srcBit, uint16_t nBits) {
    const uint8_t  shift = srcBit & 7;
    const uint8_t  room  = 8 - shift;            // bits used from the first source byte
    const uint8_t* in    = src + (srcBit >> 3);
    for (uint16_t i = 0; i < nBits; i += 8) {
      const uint16_t left = nBits - i;           // bits still to copy
      uint8_t b = in[i >> 3] << shift;
      if (shift && left > room) b |= in[(i >> 3) + 1] >> room;
      if (left < 8) b &= 0xFF << (8 - left);     // last partial byte
      dst[i >> 3] = b;
    }
  }
  // copy nch byte aligned channels of chBytes each from src into dst starting at bit, skipping gaps[c] bits before
  // channel c (gap bits are not written: they stay 0 as the buffer is zeroed when allocated and cleared)
  static constexpr void packChannels(uint8_t* dst, size_t bit, const uint8_t* src, uint8_t chBytes, uint8_t nch, const uint8_t* gaps) {
    for (uint8_t c = 0; c < nch; c++) {
      bit += gaps[c];
      writeBits(dst, bit, src + c * chBytes, chBytes * 8);
      bit += chBytes * 8;
    }
  }
  // inverse of packChannels()
  static constexpr void unpackChannels(uint8_t* dst, const uint8_t* src, size_t bit, uint8_t chBytes, uint8_t nch, const uint8_t* gaps) {
    for (uint8_t c = 0; c < nch; c++) {
      bit += gaps[c];
      readBits(dst + c * chBytes, src, bit, chBytes * 8);
      bit += chBytes * 8;
    }
  }
  inline uint16_t packedPixelBits() const { return (uint16_t)_encoder.getPixelBytes() * 8 + _padBits; }
  // copy a byte aligned encoded pixel into the bit-packed buffer at pixel pos (checked in WLEDpixelBus_PackTest.cpp)
  void packPixel(size_t pos, const uint8_t* scratch) {
    const uint8_t chBytes = _encoder.is16bit() ? 2 : 1;
    packChannels(_pixelData, pos * packedPixelBits(), scratch, chBytes, _encoder.getPixelBytes() / chBytes, _gapBits);
  }
  // inverse of packPixel(): copy pixel pos out of the bit-packed buffer into a byte aligned scratch buffer
  void unpackPixel(size_t pos, uint8_t* scratch) const {
    const uint8_t chBytes = _encoder.is16bit() ? 2 : 1;
    unpackChannels(scratch, _pixelData, pos * packedPixelBits(), chBytes, _encoder.getPixelBytes() / chBytes, _gapBits);
  }
  // AI: end

public:
  virtual ~PixelBus() {
    if (_encodeBuffer) { free(_encodeBuffer); _encodeBuffer = nullptr; }
  }

  /**
   * Reserve prefix bytes before pixel data. Must be called BEFORE begin().
   * allocateEncodeBuffer() reserves the prefix region; updatePrefix() fills or updates it.
   * @param len  Number of prefix bytes to reserve
   */
  void setPrefixLen(uint8_t len) {
    _prefixLen = len > MAX_PREFIX_BYTES ? MAX_PREFIX_BYTES : len;
  }

  void setPrefixData(const uint8_t* data, uint8_t len) {
    if (!data) return;
    if (len > MAX_PREFIX_BYTES) len = MAX_PREFIX_BYTES;
    memcpy(_prefixData, data, len);
  }

  /**
   * Update the prefix data in the transmit buffer (e.g. per-frame for current control).
   * Must be called AFTER begin() (i.e. after allocateEncodeBuffer()). No-op if buffer not ready.
   * @param data  New prefix bytes
   * @param len   Must be <= _prefixLen (will be clamped)
   */
  virtual void updatePrefix(const uint8_t* data, uint8_t len) {
    if (!_encodeBuffer || len == 0) return;
    if (len > _prefixLen) len = _prefixLen;
    memcpy(_encodeBuffer, data, len);
  }

  /**
   * Set bus-level brightness applied during pixel encoding for all LED types.
   * For 8-bit types: applied using color_fade() on pixel color
   * For 16-bit types (SM16825, UCS8903, UCS8904): applied as `channel * bri` → full 16-bit value.
   * For TM1814/TM1815: set to the fine residual scale after hardware current-step selection.
   * @param b  brightness 0–255
   */
  /**
   * Set the zero bits sent after each wire channel (MAX_CUSTOM_CHANNELS entries). Pixels are bit-packed when any is > 0.
   * Must be called BEFORE begin().
   * Only supported by drivers that transmit _encodeBufferSize bytes (RMT, I2S, PARLIO, parallel SPI), see createBus().
   */
  void setGapBits(const uint8_t* gaps) {
    _padBits = 0;
    for (uint8_t c = 0; c < MAX_CUSTOM_CHANNELS; c++) { _gapBits[c] = gaps[c] > 16 ? 16 : gaps[c]; _padBits += _gapBits[c]; }
  }
  // bytes of encoded pixel data (without prefix/suffix) for numPixels pixels
  size_t getPixelDataBytes(size_t numPixels) const {
    if (_padBits) return (numPixels * packedPixelBits() + 7) / 8;
    return numPixels * _encoder.getPixelBytes();
  }

  void setBusBri(uint8_t b) { _busBri = b; } // TODO: brightness scaling/parameters may need some refinement
  void setEncBri(uint8_t b) { _encBri = b; }
  inline uint8_t getBusBri() const { return _busBri; }

  /**
   * Set the APA102 5-bit per-pixel hardware brightness step (0–31).
   * Default implementation is a no-op; overridden by SpiBus for TYPE_APA102.
   * Called by BusDigital::setBrightness() as part of the two-stage brightness scheme:
   * coarse control via hardware current step, fine control via color_fade() residual.
   */
  virtual void setApa102HwBri(uint8_t /*v*/) {}

  /**
   * physical output signal inversion (polarity).
   * must be implemented on bus driver level
   */
  virtual void setInverted(bool inv) { _inverted = inv; }
  bool isInverted() const { return _inverted; }

  /**
   * Replace the color encoder (e.g. for curstom bus types after bus creation).
   * Must be called after construction but before begin().
   */
  void setEncoder(const ColorEncoder& enc) { _encoder = enc; }

  bool hasPrefix() const { return _prefixLen > 0; }
  uint8_t getPrefixLen() const { return _prefixLen; }

  /**
   * Reserve suffix bytes after pixel data. Must be called BEFORE begin().
   * allocateEncodeBuffer() initialises the suffix region; updateSuffix() overwrites it.
   * @param len  Number of suffix bytes to reserve
   */
  void setSuffixLen(uint8_t len) {
    _suffixLen = len > MAX_SUFFIX_BYTES ? MAX_SUFFIX_BYTES : len;
  }

  void setSuffixData(const uint8_t* data, uint8_t len) {
    if (!data) return;
    if (len > MAX_SUFFIX_BYTES) len = MAX_SUFFIX_BYTES;
    memcpy(_suffixData, data, len);
  }

  /**
   * Overwrite the suffix bytes in _encodeBuffer.
   * Must be called AFTER begin() (i.e. after allocateEncodeBuffer()). No-op if buffer not ready.
   * @param data  New suffix bytes
   * @param len   Must be <= _suffixLen (will be clamped)
   */
  virtual void updateSuffix(const uint8_t* data, uint8_t len) {
    if (!_pixelData || _suffixLen == 0 || len == 0) return;
    if (len > _suffixLen) len = _suffixLen;
    memcpy(_pixelData + getPixelDataBytes(_numPixels), data, len);
  }

  bool hasSuffix() const { return _suffixLen > 0; }
  uint8_t getSuffixLen() const { return _suffixLen; }

  virtual bool begin() = 0;
  virtual void end() = 0;
  // show() sends the pre-encoded _encodeBuffer to hardware.
  virtual bool show() = 0;
  virtual bool canShow() const = 0;
#ifdef WLED_DEBUG_BUS
  virtual const char* getTypeStr() const = 0;
#endif

  /**
   * Encode one pixel into _encodeBuffer at pos.
   * c and ww/cw must already be brightness-scaled by the caller (BusDigital::setPixelColor
   * applies color_fade() via _busBri before calling here). For 16-bit LED types the encoder
   * multiplies each channel by _encBri for full 16-bit wire precision.
   * @param pos  pixel index (0-based, hardware index including skip)
   * @param c    RGBW color (brightness-scaled for 8-bit types; raw for 16-bit)
   * @param wwcw  warm-white/cool-white combined (CCT calculation result, brightness-scaled)
   */
  // Preconditions (guaranteed by BusDigital calling path):
  //   _encodeBuffer != nullptr  (_valid == true implies begin() succeeded)
  //   pos < _numPixels          (setNumPixels = lenToCreate + _skip, pix bounded by both)
  // note: using O2 optimization seems to make it slower
  // TODO: on ESP32, do not put this in IRAM on C3 it works in IRAM, need to test if there is any speed benefit from IRAM (note: this may have all changed in IDF V5)
  virtual bool setPixelColor(uint16_t pos, uint32_t c, uint16_t wwcw) {
    storePixel(pos, [&](uint8_t* out) { encodePixel(out, c, wwcw); });
    return true;
  }

  // 16-bit types only: set pixel pos from a 16-bit value per color source (see ColorEncoder::encode16())
  void setPixelColor16(uint16_t pos, const uint16_t* v) {
    storePixel(pos, [&](uint8_t* out) { _encoder.encode16(v, out, _encBri); });
  }

  // encode one pixel into out (byte aligned, getPixelBytes() bytes)
  inline void encodePixel(uint8_t* out, uint32_t c, uint16_t wwcw) const {
    const CctPixel cct{wwcw};
    if (_encoder.is16bit()) { // 8-bit values scaled to the full 16-bit range, times the encoder brightness
      const uint16_t v[CH_CW + 1] = {0, uint16_t(R(c) * 257), uint16_t(G(c) * 257), uint16_t(B(c) * 257),
                                     uint16_t(W(c) * 257), uint16_t(cct.ww * 257), uint16_t(cct.cw * 257)};
      _encoder.encode16(v, out, _encBri);
      return;
    }
    switch (_encoder.getPixelFormat()) {
      case 3:  _encoder.encodeRGB(c, out);           break; // 3ch RGB
      case 4:  _encoder.encodeRGBW(c, out);          break; // 4ch RGBW
      case 5:  _encoder.encodeCCT(c, cct, out);      break; // 5ch CCT
      default: _encoder.encodeGeneric(c, cct, out);  break; // custom channel maps: duplicate sources, inverted channels
    }
  }

  // encode into pixel pos: in place, or for bit-packed buses into a byte aligned scratch copy that packPixel() moves into place
  template <typename Encode> void storePixel(uint16_t pos, Encode encode) {
    if (!_padBits) { encode(_pixelData + (size_t)pos * _encoder.getPixelBytes()); return; }
    uint8_t scratch[PACKED_SCRATCH_BYTES] = {0};
    encode(scratch);
    packPixel(pos, scratch);
  }

  /**
   * Decode one pixel back from _encodeBuffer (used for read-modify-write and getPixelColor).
   * Returns RGBW32 color. WW/CW are NOT encoded separately so CCT round-trips are lossy.
   * Override only if the bus uses non-linear encoding (e.g. Esp8266DmaBus 4-step).
   */
  virtual uint32_t getPixelColor(uint16_t pix) const {
    if (!_padBits) return decodePixel(_pixelData + (size_t)pix * _encoder.getPixelBytes());
    uint8_t scratch[PACKED_SCRATCH_BYTES] = {0}; // bit-packed: decode a byte aligned copy
    unpackPixel(pix, scratch);
    return decodePixel(scratch);
  }

  // decode one byte aligned pixel (getPixelBytes() bytes)
  inline uint32_t decodePixel(const uint8_t* in) const {
    const uint8_t pixelFormat = _encoder.getPixelFormat();
    switch (pixelFormat) {
      case 3:                    return _encoder.decodeRGB(in);
      case 4:                    return _encoder.decodeRGBW(in);
      case 5:                    return _encoder.decodeCCT(in);
      case (3*2) | NCHF_16BIT:   return _encoder.decodeRGB16(in);
      case (4*2) | NCHF_16BIT:   return _encoder.decodeRGBW16(in);
      case (5*2) | NCHF_16BIT:   return _encoder.decodeCCT16(in);
      default:                   return _encoder.decodeGeneric(in);
    }
  }

  /**
   * Zero the encode buffer (set all pixels to black), preserving the prefix.
   * Derived classes may override if their encoding is non-trivial (e.g. I2S 4-step).
   * For all standard RGB/RGBW protocols, all-zero bytes encode as black.
   */
  virtual void clearEncodeBuffer() {
    if (_pixelData && _numPixels > 0) {
      memset(_pixelData, 0, getPixelDataBytes(_numPixels));
    }
  }

  /**
   * Proportionally scale all encoded channel bytes by scale/256 (video scale).
   * Used by BusDigital::applyBriLimit() for ABL. No-op if scale == 255.
   * Note: buses with non-linear encoding (e.g. Esp8266DmaBus 4-step) must override this.
   */
  virtual void scaleAll(uint8_t scale) {
    if (scale == 255 || !_pixelData || _numPixels == 0) return;
    if (_padBits) { // bit-packed: bytes do not line up with channels, scale each pixel's channel bytes in a byte aligned copy
      const uint8_t pixelBytes = _encoder.getPixelBytes();
      for (size_t p = 0; p < _numPixels; p++) {
        uint8_t scratch[PACKED_SCRATCH_BYTES] = {0};
        unpackPixel(p, scratch);
        for (uint8_t i = 0; i < pixelBytes; i++) scratch[i] = ((uint16_t)(scratch[i] + 1) * scale) >> 8;
        packPixel(p, scratch);
      }
      return;
    }
    const size_t pixelBytes = (size_t)_numPixels * _encoder.getPixelBytes();
    for (size_t i = 0; i < pixelBytes; i++) {
      _pixelData[i] = ((uint16_t)(_pixelData[i] + 1) * scale) >> 8;
    }
  }

  /**
   * Allocate encode buffer. Called from begin() after hardware init.
   * @param numPixels  hardware pixel count (may include skipped pixels)
   * @param numChannels bytes per pixel in the encoded stream
   */
  virtual bool allocateEncodeBuffer(uint16_t numPixels, uint8_t numChannels) {
    const size_t pixelBytes = _padBits ? getPixelDataBytes(numPixels) : (size_t)numPixels * numChannels; // bit-packed pixels need fewer, non multiple bytes
    // note: if suffix is used, pixelBytes must align with physical bus requirements, we assume pixels*channels matches a physically possible layout
    size_t needed = _prefixLen + pixelBytes + _suffixLen;
    if (_encodeBuffer && _encodeBufferSize >= needed) return true;
    if (_encodeBuffer) { free(_encodeBuffer); _encodeBuffer = nullptr; }
    if (needed == 0) return true;
#if defined(ARDUINO_ARCH_ESP32)
    _encodeBuffer = (uint8_t*)heap_caps_malloc(needed, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // _encodeBuffer is accessed from ISR (but not from DMA), do not use PSRAM
#else
    _encodeBuffer = (uint8_t*)malloc(needed);
#endif
    if (!_encodeBuffer) { _encodeBufferSize = 0; return false; }
    memset(_encodeBuffer, 0, needed);
    _encodeBufferSize = needed;
    _pixelData = _encodeBuffer + _prefixLen;
    if (_prefixLen > 0) memcpy(_encodeBuffer, _prefixData, _prefixLen);
    if (_suffixLen > 0) memcpy(_pixelData + pixelBytes, _suffixData, _suffixLen);
    return true;
  }

  size_t   getEncodeBufferSize() const { return _encodeBufferSize; }
  const uint8_t* getEncodeBuffer() const { return _encodeBuffer; } // diagnostics (/json/info)
  virtual uint16_t getNumPixels() const { return _numPixels; }
  void setNumPixels(uint16_t n) { _numPixels = n; }
};

//==============================================================================
// Forward Declarations
//==============================================================================

class RmtBus;

#ifdef WLEDPB_I2S_SUPPORT
class I2sBus;
class I2sBusContext;
#endif

#ifdef WLEDPB_PARLIO_SUPPORT
class ParlioBus;
class ParlioBusContext;
#endif

//==============================================================================
// Bus Factory - Create appropriate bus for platform
//==============================================================================

enum class BusDriver : uint8_t {
  RMT     = 0,
  I2S     = 1, // I2S on ESP32 and S2, LCD on S3
  SPI     = 2, // parallel SPI output (C3)
  UART    = 3,
  DMA     = 4,
  BitBang = 5,
  PARLIO  = 6  // PARLIO TX parallel output (C6, H2, C5, P4 — IDF >= 5.3)
};

/**
 * Get the maximum number of RMT TX channels for the current platform
 */
constexpr uint8_t getRmtMaxChannels() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return 8;   // ESP32 original: 8 RMT channels
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
  return 4;   // ESP32-S2/S3: 4 RMT TX channels
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C61) || defined(CONFIG_IDF_TARGET_ESP32H2)
  return 2;   // ESP32-C3/C6/C5/C61/H2: 2 RMT TX channels
#else
  return 0;
#endif
}

/**
 * Create a bus instance
 * @param driver Bus driver type
 * @param pin GPIO pin
 * @param properties Complete one-wire protocol, channel, and framing properties
 * @param ledType Legacy type hint retained by low-level driver implementations
 * @param bufferSize DMA buffer size (for I2S/LCD)
 * @return Bus instance (caller owns, delete when done)
 */
PixelBus* createBus(BusDriver driver, int8_t pin, const BusProperties& properties,
  uint8_t ledType = 0, size_t bufferSize = DEFAULT_DMA_BUFFER_SIZE);

} // namespace WLEDpixelBus

#include "WLEDpixelBus_SPI.h"

#if defined(ESP32)
#include "WLEDpixelBus_RMT.h"
#include "WLEDpixelBus_I2S.h"
#include "WLEDpixelBus_ParallelSpi.h"
#include "WLEDpixelBus_PARLIO.h"
#include "WLEDpixelBus_BitBang.h"
#elif defined(ESP8266)
#include "WLEDpixelBus_ESP8266.h"
#include "WLEDpixelBus_BitBang.h"
#endif
