// -*-c++-*--------------------------------------------------------------------
// Copyright 2026 NRV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EVENT_CAMERA_CODECS__GROUP_AER_DECODER_H_
#define EVENT_CAMERA_CODECS__GROUP_AER_DECODER_H_

#include <stdint.h>

#include <memory>
#include <string>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/event_processor.h"

namespace event_camera_codecs
{
namespace group_aer
{
// Decoder for the NRV Delta (S5KRC1S) grouped-AER wire format.
//
// The stream is a sequence of big-endian 32-bit words. Bit 31 marks an event
// group; otherwise bits [30:26] select the packet type:
//
//   0x04  Column       [23] mirror, [21] first-column flag, [10:0] x
//   0x08  Timestamp    [23] 1 = sub (10 bit), 0 = reference (22 bit)
//   0x0C  Frame end
//
// An event group carries two groups of 8 rows each. The second group address is
// the first one plus (or minus, in mirror mode) the offset in bits [30:26], so
// a single-group packet is just an event group with offset zero.
//
//   [24:18] group 1 address      [17] group 2 polarity   [16] group 1 polarity
//   [15:8]  group 2 event mask   [7:0] group 1 event mask
//
// Sensor polarity is 0 = ON, 1 = OFF, which is inverted relative to the
// convention used by event_camera_codecs.
template <class MsgT, class EventProcT>
class Decoder : public event_camera_codecs::Decoder<MsgT, EventProcT>
{
public:
  using timestamp_t = uint64_t;

  size_t decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) override
  {
    struct NoTimeLimit
    {
      static bool isInFuture(uint64_t, uint64_t) { return (false); }
    };
    size_t numConsumed{0};
    doDecode<NoTimeLimit>(buf, bufSize, processor, 0, &numConsumed, nullptr);
    processor->finished();
    return (numConsumed);
  }

  size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) override
  {
    struct TimeLimit
    {
      static bool isInFuture(uint64_t t, uint64_t limit) { return (t >= limit); }
    };
    size_t numConsumed{0};
    doDecode<TimeLimit>(buf, bufSize, processor, timeLimit, &numConsumed, nextTime);
    processor->finished();
    return (numConsumed);
  }

  bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) override
  {
    bool hasValidTime(false);
    for (size_t i = 0; i + 4 <= size; i += 4) {
      const uint8_t * w = buf + i;
      if (w[0] & 0x80) {
        if (numEventsOnOff && state_ == State::Running) {
          const uint8_t p1 = static_cast<uint8_t>(w[1] & 0x01);
          const uint8_t p2 = static_cast<uint8_t>((w[1] & 0x02) >> 1);
          numEventsOnOff[p1 == 0] += popCount(w[3]);
          numEventsOnOff[p2 == 0] += popCount(w[2]);
        }
        continue;
      }
      const uint8_t header = static_cast<uint8_t>(w[0] & 0x7C);
      if (header == 0x04) {
        applyColumnWord(w);
        if (state_ == State::WaitOrigin && haveOrigin_) {
          state_ = State::Running;
        }
      } else if (header == 0x08) {
        applyTimestampWord(w);
        if (state_ == State::WaitOrigin && haveOrigin_ && posX_ >= 0) {
          state_ = State::Running;
        }
        if (haveOrigin_) {
          const uint64_t t = sensorTime();
          if (!hasValidTime) {
            *firstTS = t;
            hasValidTime = true;
          }
          *lastTS = t;
        }
      } else if (header == 0x0C) {
        if (state_ == State::WaitFrameEnd) {
          state_ = State::WaitOrigin;
        }
      }
    }
    return (hasValidTime);
  }

  // Times reported here are the sensor's own clock. Consumers pair it with the
  // message header to relate sensor time to ROS time, so time_base is unused.
  void setTimeBase(const uint64_t) override {}

  // Callers may ask this of a freshly constructed decoder, so the answer comes
  // from the buffer rather than from accumulated state.
  bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) override
  {
    uint32_t ref = 0;
    bool haveRef = false;
    for (size_t i = 0; i + 4 <= size; i += 4) {
      const uint8_t * w = buf + i;
      if (w[0] & 0x80 || (w[0] & 0x7C) != 0x08) {
        continue;
      }
      if (w[1] & 0x80) {
        if (haveRef) {
          const uint32_t sub =
            (static_cast<uint32_t>(w[2] & 0x03) << 8) | static_cast<uint32_t>(w[3]);
          *firstTS = static_cast<uint64_t>(composeUs(ref, sub)) * timeMult_;
          return (true);
        }
      } else {
        ref = (static_cast<uint32_t>(w[1] & 0x3F) << 16) |
              (static_cast<uint32_t>(w[2]) << 8) | static_cast<uint32_t>(w[3]);
        haveRef = true;
      }
    }
    if (haveRef) {
      *firstTS = static_cast<uint64_t>(composeUs(ref, 0)) * timeMult_;
      return (true);
    }
    return (false);
  }

  bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) override
  {
    return (findFirstSensorTime(msg.events.data(), msg.events.size(), firstTS));
  }

  void setTimeMultiplier(uint32_t mult) override { timeMult_ = mult; }

  void setGeometry(uint16_t width, uint16_t height) override
  {
    this->width_ = width;
    this->height_ = height;
  }

  uint16_t getWidth() const override { return (this->width_); }
  uint16_t getHeight() const override { return (this->height_); }
  uint32_t getTimeMultiplier() const final { return (timeMult_); }
  bool hasSensorTimeSinceEpoch() const final { return (false); }

private:
  // Startup is only complete once a frame boundary has been seen, a timestamp
  // has established the time origin, and a column word has established x.
  // Column state carries across frame ends in the sensor stream, so a frame end
  // alone does not tell us where in the image the following groups belong.
  enum class State { WaitFrameEnd, WaitOrigin, Running };

  // [DELTA01] Timestamp periods corrected for 24 MHz -> 19.2 MHz.
  static constexpr double kSubPeriodUs = 1.2539808917197452;
  static constexpr double kRefPeriodUs = 1253.9808917197452;

  // [DELTA10]
  // static constexpr double kSubPeriodUs = 1.003184713375796;
  // static constexpr double kRefPeriodUs = 1003.184713375796;

  static constexpr uint32_t kRefMax = 0x3FFFFF;

  static size_t popCount(uint8_t v)
  {
    size_t n = 0;
    while (v) {
      v &= static_cast<uint8_t>(v - 1);
      ++n;
    }
    return (n);
  }

  static uint32_t composeUs(uint32_t refTS, uint32_t subTS)
  {
    const double us = refTS * kRefPeriodUs + subTS * kSubPeriodUs;
    return (static_cast<uint32_t>(static_cast<uint64_t>(us + 0.5) & 0xFFFFFFFFULL));
  }

  // Absolute sensor time, accumulated across the 32-bit microsecond rollover.
  uint64_t sensorTime() const { return ((wrapAccumUs_ + fullTS_) * timeMult_); }

  void applyColumnWord(const uint8_t * w)
  {
    mirrorFlag_ = (w[1] & 0x80) != 0;
    posX_ = (static_cast<int32_t>(w[2] & 0x07) << 8) | static_cast<int32_t>(w[3]);
  }

  void applyTimestampWord(const uint8_t * w)
  {
    if (w[1] & 0x80) {
      const uint32_t subTS =
        (static_cast<uint32_t>(w[2] & 0x03) << 8) | static_cast<uint32_t>(w[3]);
      uint32_t newTs = composeUs(refTS_, subTS);
      // The reference word is not resent on every tick, so a sub timestamp that
      // fails to advance means the reference counter rolled over.
      if (newTs <= fullTS_) {
        if (lastRefTS_ == refTS_) {
          refTS_ = (refTS_ >= kRefMax) ? 0 : (refTS_ + 1);
        }
        newTs = composeUs(refTS_, subTS);
      }
      if (newTs < fullTS_) {
        wrapAccumUs_ += (1ULL << 32);
      }
      fullTS_ = newTs;
      lastRefTS_ = refTS_;
    } else {
      refTS_ = (static_cast<uint32_t>(w[1] & 0x3F) << 16) |
               (static_cast<uint32_t>(w[2]) << 8) | static_cast<uint32_t>(w[3]);
      if (state_ == State::WaitOrigin && !haveOrigin_) {
        fullTS_ = composeUs(refTS_, 0);
        lastRefTS_ = refTS_;
        haveOrigin_ = true;
      }
    }
  }

  template <typename TimeLimitT>
  void doDecode(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    size_t * numConsumed, uint64_t * nextTime)
  {
    size_t i = 0;
    for (; i + 4 <= bufSize; i += 4) {
      const uint8_t * w = buf + i;

      if (w[0] & 0x80) {
        if (state_ != State::Running || posX_ < 0) {
          continue;
        }
        const uint64_t t = sensorTime();
        if (TimeLimitT::isInFuture(t, timeLimit)) {
          if (nextTime) {
            *nextTime = t;
          }
          *numConsumed = i;
          return;
        }
        emitEventGroup(w, t, processor);
        continue;
      }

      const uint8_t header = w[0] & 0x7C;
      if (header == 0x04) {
        applyColumnWord(w);
        if (state_ == State::WaitOrigin && haveOrigin_) {
          state_ = State::Running;
        }
      } else if (header == 0x08) {
        applyTimestampWord(w);
        if (state_ == State::WaitOrigin && haveOrigin_ && posX_ >= 0) {
          state_ = State::Running;
        }
      } else if (header == 0x0C) {
        if (state_ == State::WaitFrameEnd) {
          state_ = State::WaitOrigin;
        }
      }
    }
    *numConsumed = i;
  }

  void emitEventGroup(const uint8_t * w, uint64_t t, EventProcT * processor)
  {
    const uint8_t offset = static_cast<uint8_t>((w[0] & 0x7C) >> 2);
    int32_t grpAddr =
      (static_cast<int32_t>(w[1] & 0xFC) >> 2) | (static_cast<int32_t>(w[0] & 0x01) << 6);

    emitGroup(grpAddr, w[3], w[1] & 0x01, t, processor);

    grpAddr = mirrorFlag_ ? (grpAddr - offset) : (grpAddr + offset);
    emitGroup(grpAddr, w[2], (w[1] & 0x02) >> 1, t, processor);
  }

  void emitGroup(
    int32_t grpAddr, uint8_t mask, uint8_t sensorPolarity, uint64_t t, EventProcT * processor)
  {
    if (mask == 0 || grpAddr < 0) {
      return;
    }
    const int32_t y0 = grpAddr << 3;
    if (posX_ >= static_cast<int32_t>(this->width_) ||
        (y0 + 7) >= static_cast<int32_t>(this->height_)) {
      return;
    }
    // event_camera_codecs uses 0 = OFF, 1 = ON; the sensor uses the opposite.
    const uint8_t polarity = sensorPolarity ? 0 : 1;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (mask & (1u << bit)) {
        processor->eventCD(
          t, static_cast<uint16_t>(posX_), static_cast<uint16_t>(y0 + bit), polarity);
      }
    }
  }

  // --------------------- variables
  State state_{State::WaitFrameEnd};
  bool haveOrigin_{false};
  int32_t posX_{-1};
  bool mirrorFlag_{false};
  uint32_t refTS_{0};
  uint32_t lastRefTS_{0};
  uint32_t fullTS_{0};
  uint64_t wrapAccumUs_{0};
  uint32_t timeMult_{1000};
};
}  // namespace group_aer
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__GROUP_AER_DECODER_H_
