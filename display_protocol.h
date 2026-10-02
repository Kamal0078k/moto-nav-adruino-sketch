#ifndef MOTORCYCLE_DISPLAY_PROTOCOL_H
#define MOTORCYCLE_DISPLAY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace navproto {

constexpr uint8_t PROTOCOL_VERSION = 3;
constexpr uint8_t FRAGMENT_MAGIC = 0xA5;
constexpr size_t MAX_POINTS = 32;
constexpr size_t FRAME_HEADER_SIZE = 15;
constexpr size_t POINT_SIZE = 5;
constexpr size_t CRC_SIZE = 2;
// Optional text block: two length-prefixed printable-ASCII lines (e.g. From / To place names).
// 40 characters so a song title can scroll; the phone keeps place and caller names to 20.
constexpr size_t MAX_TEXT_LENGTH = 40;
constexpr size_t MAX_TEXT_BLOCK_SIZE = 2 * (1 + MAX_TEXT_LENGTH);
// Optional call block: phase byte, then caller and status lines in the same text format.
constexpr size_t MAX_CALL_BLOCK_SIZE = 1 + 2 * (1 + MAX_TEXT_LENGTH);
// Optional clock block: hour (0-23), minute, second, options (bit 0 = 24-hour format).
constexpr size_t CLOCK_BLOCK_SIZE = 4;
// Optional whole minutes remaining, combined with the synced clock to display ETA.
constexpr size_t REMAINING_DURATION_BLOCK_SIZE = 2;
constexpr size_t MIN_FRAME_SIZE = FRAME_HEADER_SIZE + CRC_SIZE;
constexpr size_t MAX_FRAME_SIZE =
  FRAME_HEADER_SIZE + MAX_POINTS * POINT_SIZE + MAX_TEXT_BLOCK_SIZE + MAX_CALL_BLOCK_SIZE +
  CLOCK_BLOCK_SIZE + REMAINING_DURATION_BLOCK_SIZE + CRC_SIZE;
constexpr size_t FRAGMENT_HEADER_SIZE = 8;
constexpr uint32_t ASSEMBLY_TIMEOUT_MS = 2000;

constexpr uint8_t FRAME_FLAG_REMAINING_DISTANCE = 0x01;
constexpr uint8_t FRAME_FLAG_GOOGLE_MAPS = 0x02;
constexpr uint8_t FRAME_FLAG_TEXT = 0x04;
constexpr uint8_t FRAME_FLAG_CALL = 0x08;
constexpr uint8_t FRAME_FLAG_CLOCK = 0x10;
constexpr uint8_t FRAME_FLAG_REMAINING_DURATION = 0x20;
constexpr uint8_t CLOCK_OPTION_24_HOUR = 0x01;

enum CallPhase : uint8_t {
  CALL_NONE = 0,
  CALL_RINGING = 1,
  CALL_ACTIVE = 2,
  // A call that rang out unanswered; the phone shows it briefly, then clears it.
  CALL_MISSED = 3
};
constexpr uint8_t POINT_FLAG_START = 0x01;
constexpr uint8_t POINT_FLAG_DESTINATION = 0x02;

enum NavigationState : uint8_t {
  DISCONNECTED = 0,
  ACQUIRING_GPS = 1,
  NAVIGATING = 2,
  REROUTING = 3,
  ARRIVED = 4,
  GPS_DEGRADED = 5,
  WAITING_PERMISSION = 6,
  LOCATION_DISABLED = 7,
  // Phone linked, no route chosen yet: greet the rider.
  READY = 8,
  // Route chosen, ride not started: show From / To and distance.
  ROUTE_PREVIEW = 9
};

enum Maneuver : uint8_t {
  STRAIGHT = 0,
  LEFT = 1,
  RIGHT = 2,
  ARRIVE = 3
};

struct RoutePoint {
  uint16_t x;
  uint16_t y;
  uint8_t flags;
};

struct DisplayFrame {
  uint16_t sequence;
  NavigationState state;
  Maneuver maneuver;
  uint16_t nextTurnMetres;
  bool hasRemainingDistance;
  uint32_t remainingDistanceMetres;
  uint16_t mapHeadingDegrees;
  bool googleMapsContent;
  uint8_t pointCount;
  RoutePoint points[MAX_POINTS];
  bool hasText;
  char primaryText[MAX_TEXT_LENGTH + 1];
  char secondaryText[MAX_TEXT_LENGTH + 1];
  CallPhase callPhase;
  char callerText[MAX_TEXT_LENGTH + 1];
  char callStatusText[MAX_TEXT_LENGTH + 1];
  bool hasClock;
  uint8_t clockHour;
  uint8_t clockMinute;
  uint8_t clockSecond;
  uint8_t clockOptions;
  bool hasRemainingDuration;
  uint16_t remainingDurationMinutes;
};

enum DecodeStatus : uint8_t {
  DECODE_OK,
  DECODE_TOO_SHORT,
  DECODE_TOO_LONG,
  DECODE_UNSUPPORTED_VERSION,
  DECODE_INVALID_STATE,
  DECODE_INVALID_MANEUVER,
  DECODE_INVALID_FLAGS,
  DECODE_TOO_MANY_POINTS,
  DECODE_LENGTH_MISMATCH,
  DECODE_INVALID_POINT_FLAGS,
  DECODE_FIRST_POINT_NOT_START,
  DECODE_CRC_MISMATCH,
  DECODE_INVALID_TEXT,
  DECODE_OUTPUT_TOO_SMALL
};

enum FragmentResult : uint8_t {
  FRAGMENT_IN_PROGRESS,
  FRAGMENT_COMPLETE,
  FRAGMENT_INVALID,
  FRAGMENT_STALE,
  FRAGMENT_FRAME_INVALID
};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length);
DecodeStatus encodeFrame(
  const DisplayFrame& frame,
  uint8_t* output,
  size_t outputCapacity,
  size_t& outputLength
);
DecodeStatus decodeFrame(
  const uint8_t* data,
  size_t length,
  DisplayFrame& output
);
const char* decodeStatusName(DecodeStatus status);

class FragmentAssembler {
 public:
  FragmentAssembler();
  void resetSession();
  bool expire(uint32_t nowMs);
  FragmentResult acceptFragment(
    const uint8_t* fragment,
    size_t fragmentLength,
    uint32_t nowMs,
    DisplayFrame& output
  );
  DecodeStatus lastDecodeStatus() const;
  size_t receivedCount() const;
  size_t expectedLength() const;

 private:
  void clearAssembly();
  void startAssembly(uint16_t sequence, uint16_t totalLength, uint32_t nowMs);
  static bool isNewerSequence(uint16_t candidate, uint16_t reference);

  bool active_;
  bool hasLastSequence_;
  uint16_t activeSequence_;
  uint16_t lastSequence_;
  uint16_t totalLength_;
  size_t receivedCount_;
  uint32_t lastFragmentAtMs_;
  DecodeStatus lastDecodeStatus_;
  uint8_t buffer_[MAX_FRAME_SIZE];
  bool received_[MAX_FRAME_SIZE];
};

}  // namespace navproto

#endif
