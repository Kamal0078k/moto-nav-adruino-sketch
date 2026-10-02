#include "display_protocol.h"

#include <string.h>

namespace navproto {
namespace {

uint16_t readU16(const uint8_t* data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint32_t readU32(const uint8_t* data) {
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

void writeU16(uint8_t* data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)(value >> 8);
}

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
  data[2] = (uint8_t)((value >> 16) & 0xFF);
  data[3] = (uint8_t)((value >> 24) & 0xFF);
}

bool isPrintableText(const char* text, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if ((uint8_t)text[i] < 0x20 || (uint8_t)text[i] > 0x7E) return false;
  }
  return true;
}

size_t boundedLength(const char* text) {
  size_t length = 0;
  while (length <= MAX_TEXT_LENGTH && text[length] != '\0') length++;
  return length;
}

size_t textBlockSize(const DisplayFrame& frame) {
  if (!frame.hasText) return 0;
  return 2 + boundedLength(frame.primaryText) + boundedLength(frame.secondaryText);
}

size_t callBlockSize(const DisplayFrame& frame) {
  if (frame.callPhase == CALL_NONE) return 0;
  return 3 + boundedLength(frame.callerText) + boundedLength(frame.callStatusText);
}

bool validLine(const char* text) {
  size_t length = boundedLength(text);
  return length <= MAX_TEXT_LENGTH && isPrintableText(text, length);
}

// Reads one length-prefixed line; returns false if it runs past `end` or is not printable.
bool readLine(const uint8_t* data, size_t& offset, size_t end, char* output) {
  if (offset >= end) return false;
  size_t length = data[offset++];
  if (length > MAX_TEXT_LENGTH || offset + length > end) return false;
  if (!isPrintableText((const char*)data + offset, length)) return false;
  memcpy(output, data + offset, length);
  output[length] = '\0';
  offset += length;
  return true;
}

DecodeStatus validateFrame(const DisplayFrame& frame) {
  if ((uint8_t)frame.state > (uint8_t)ROUTE_PREVIEW) return DECODE_INVALID_STATE;
  if ((uint8_t)frame.callPhase > (uint8_t)CALL_MISSED) return DECODE_INVALID_TEXT;
  if (frame.hasClock && (frame.clockHour > 23 || frame.clockMinute > 59 || frame.clockSecond > 59 ||
      (frame.clockOptions & ~CLOCK_OPTION_24_HOUR) != 0)) {
    return DECODE_INVALID_FLAGS;
  }
  if (frame.callPhase != CALL_NONE && (!validLine(frame.callerText) || !validLine(frame.callStatusText))) {
    return DECODE_INVALID_TEXT;
  }
  if (frame.hasText) {
    size_t primary = boundedLength(frame.primaryText);
    size_t secondary = boundedLength(frame.secondaryText);
    if (primary > MAX_TEXT_LENGTH || secondary > MAX_TEXT_LENGTH) return DECODE_INVALID_TEXT;
    if (!isPrintableText(frame.primaryText, primary) || !isPrintableText(frame.secondaryText, secondary)) {
      return DECODE_INVALID_TEXT;
    }
  }
  if ((uint8_t)frame.maneuver > (uint8_t)ARRIVE) return DECODE_INVALID_MANEUVER;
  if (frame.mapHeadingDegrees > 359) return DECODE_INVALID_FLAGS;
  if (frame.pointCount > MAX_POINTS) return DECODE_TOO_MANY_POINTS;
  for (size_t i = 0; i < frame.pointCount; i++) {
    if ((frame.points[i].flags & ~(POINT_FLAG_START | POINT_FLAG_DESTINATION)) != 0) {
      return DECODE_INVALID_POINT_FLAGS;
    }
  }
  if (frame.pointCount > 0 && (frame.points[0].flags & POINT_FLAG_START) == 0) {
    return DECODE_FIRST_POINT_NOT_START;
  }
  return DECODE_OK;
}

}  // namespace

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

DecodeStatus encodeFrame(
  const DisplayFrame& frame,
  uint8_t* output,
  size_t outputCapacity,
  size_t& outputLength
) {
  outputLength = 0;
  DecodeStatus validation = validateFrame(frame);
  if (validation != DECODE_OK) return validation;

  size_t required = FRAME_HEADER_SIZE + frame.pointCount * POINT_SIZE + textBlockSize(frame) +
      callBlockSize(frame) + (frame.hasClock ? CLOCK_BLOCK_SIZE : 0) +
      (frame.hasRemainingDuration ? REMAINING_DURATION_BLOCK_SIZE : 0) + CRC_SIZE;
  if (outputCapacity < required) return DECODE_OUTPUT_TOO_SMALL;

  output[0] = PROTOCOL_VERSION;
  output[1] = (uint8_t)frame.state;
  output[2] = (uint8_t)frame.maneuver;
  output[3] = (frame.hasRemainingDistance ? FRAME_FLAG_REMAINING_DISTANCE : 0) |
      (frame.googleMapsContent ? FRAME_FLAG_GOOGLE_MAPS : 0) |
      (frame.hasText ? FRAME_FLAG_TEXT : 0) |
      (frame.callPhase != CALL_NONE ? FRAME_FLAG_CALL : 0) |
      (frame.hasClock ? FRAME_FLAG_CLOCK : 0) |
      (frame.hasRemainingDuration ? FRAME_FLAG_REMAINING_DURATION : 0);
  writeU16(output + 4, frame.sequence);
  writeU16(output + 6, frame.nextTurnMetres);
  writeU32(output + 8, frame.hasRemainingDistance ? frame.remainingDistanceMetres : 0);
  writeU16(output + 12, frame.mapHeadingDegrees);
  output[14] = frame.pointCount;

  size_t offset = FRAME_HEADER_SIZE;
  for (size_t i = 0; i < frame.pointCount; i++) {
    output[offset] = frame.points[i].flags;
    writeU16(output + offset + 1, frame.points[i].x);
    writeU16(output + offset + 3, frame.points[i].y);
    offset += POINT_SIZE;
  }
  if (frame.hasText) {
    const char* lines[2] = {frame.primaryText, frame.secondaryText};
    for (const char* line : lines) {
      size_t length = boundedLength(line);
      output[offset++] = (uint8_t)length;
      memcpy(output + offset, line, length);
      offset += length;
    }
  }
  if (frame.callPhase != CALL_NONE) {
    output[offset++] = (uint8_t)frame.callPhase;
    const char* lines[2] = {frame.callerText, frame.callStatusText};
    for (const char* line : lines) {
      size_t length = boundedLength(line);
      output[offset++] = (uint8_t)length;
      memcpy(output + offset, line, length);
      offset += length;
    }
  }
  if (frame.hasClock) {
    output[offset++] = frame.clockHour;
    output[offset++] = frame.clockMinute;
    output[offset++] = frame.clockSecond;
    output[offset++] = frame.clockOptions;
  }
  if (frame.hasRemainingDuration) {
    writeU16(output + offset, frame.remainingDurationMinutes);
    offset += REMAINING_DURATION_BLOCK_SIZE;
  }

  uint16_t crc = crc16CcittFalse(output, required - CRC_SIZE);
  writeU16(output + required - CRC_SIZE, crc);
  outputLength = required;
  return DECODE_OK;
}

DecodeStatus decodeFrame(
  const uint8_t* data,
  size_t length,
  DisplayFrame& output
) {
  if (length < MIN_FRAME_SIZE) return DECODE_TOO_SHORT;
  if (length > MAX_FRAME_SIZE) return DECODE_TOO_LONG;
  if (data[0] != PROTOCOL_VERSION) return DECODE_UNSUPPORTED_VERSION;
  if (data[1] > (uint8_t)ROUTE_PREVIEW) return DECODE_INVALID_STATE;
  if (data[2] > (uint8_t)ARRIVE) return DECODE_INVALID_MANEUVER;
  if ((data[3] & ~(FRAME_FLAG_REMAINING_DISTANCE | FRAME_FLAG_GOOGLE_MAPS | FRAME_FLAG_TEXT | FRAME_FLAG_CALL |
      FRAME_FLAG_CLOCK | FRAME_FLAG_REMAINING_DURATION)) != 0) {
    return DECODE_INVALID_FLAGS;
  }

  uint16_t mapHeadingDegrees = readU16(data + 12);
  if (mapHeadingDegrees > 359) return DECODE_INVALID_FLAGS;
  uint8_t pointCount = data[14];
  if (pointCount > MAX_POINTS) return DECODE_TOO_MANY_POINTS;
  size_t pointsEnd = FRAME_HEADER_SIZE + pointCount * POINT_SIZE;
  bool hasText = (data[3] & FRAME_FLAG_TEXT) != 0;
  bool hasCall = (data[3] & FRAME_FLAG_CALL) != 0;
  if (length < pointsEnd + CRC_SIZE) return DECODE_LENGTH_MISMATCH;
  // Optional blocks are length-prefixed lines; walk them to find where the CRC must start.
  size_t bodyEnd = length - CRC_SIZE;
  size_t cursor = pointsEnd;
  char scratch[MAX_TEXT_LENGTH + 1];
  if (hasText && (!readLine(data, cursor, bodyEnd, scratch) || !readLine(data, cursor, bodyEnd, scratch))) {
    return DECODE_INVALID_TEXT;
  }
  if (hasCall) {
    if (cursor >= bodyEnd || data[cursor] < CALL_RINGING || data[cursor] > CALL_MISSED) return DECODE_INVALID_TEXT;
    cursor++;
    if (!readLine(data, cursor, bodyEnd, scratch) || !readLine(data, cursor, bodyEnd, scratch)) return DECODE_INVALID_TEXT;
  }
  bool hasClock = (data[3] & FRAME_FLAG_CLOCK) != 0;
  size_t clockOffset = cursor;
  if (hasClock) {
    if (cursor + CLOCK_BLOCK_SIZE > bodyEnd) return DECODE_LENGTH_MISMATCH;
    if (data[cursor] > 23 || data[cursor + 1] > 59 || data[cursor + 2] > 59 ||
        (data[cursor + 3] & ~CLOCK_OPTION_24_HOUR) != 0) {
      return DECODE_INVALID_FLAGS;
    }
    cursor += CLOCK_BLOCK_SIZE;
  }
  bool hasRemainingDuration = (data[3] & FRAME_FLAG_REMAINING_DURATION) != 0;
  size_t remainingDurationOffset = cursor;
  if (hasRemainingDuration) {
    if (cursor + REMAINING_DURATION_BLOCK_SIZE > bodyEnd) return DECODE_LENGTH_MISMATCH;
    cursor += REMAINING_DURATION_BLOCK_SIZE;
  }
  if (cursor != bodyEnd) return DECODE_LENGTH_MISMATCH;

  uint16_t receivedCrc = readU16(data + length - CRC_SIZE);
  uint16_t computedCrc = crc16CcittFalse(data, length - CRC_SIZE);
  if (receivedCrc != computedCrc) return DECODE_CRC_MISMATCH;

  output.sequence = readU16(data + 4);
  output.state = (NavigationState)data[1];
  output.maneuver = (Maneuver)data[2];
  output.nextTurnMetres = readU16(data + 6);
  output.hasRemainingDistance = (data[3] & FRAME_FLAG_REMAINING_DISTANCE) != 0;
  output.remainingDistanceMetres = output.hasRemainingDistance ? readU32(data + 8) : 0;
  output.mapHeadingDegrees = mapHeadingDegrees;
  output.googleMapsContent = (data[3] & FRAME_FLAG_GOOGLE_MAPS) != 0;
  output.pointCount = pointCount;

  size_t offset = FRAME_HEADER_SIZE;
  for (size_t i = 0; i < pointCount; i++) {
    uint8_t flags = data[offset];
    if ((flags & ~(POINT_FLAG_START | POINT_FLAG_DESTINATION)) != 0) {
      return DECODE_INVALID_POINT_FLAGS;
    }
    output.points[i] = {
      readU16(data + offset + 1),
      readU16(data + offset + 3),
      flags
    };
    offset += POINT_SIZE;
  }

  if (pointCount > 0 && (output.points[0].flags & POINT_FLAG_START) == 0) {
    return DECODE_FIRST_POINT_NOT_START;
  }

  output.hasText = hasText;
  output.primaryText[0] = '\0';
  output.secondaryText[0] = '\0';
  output.callPhase = CALL_NONE;
  output.callerText[0] = '\0';
  output.callStatusText[0] = '\0';
  if (hasText) {
    readLine(data, offset, bodyEnd, output.primaryText);
    readLine(data, offset, bodyEnd, output.secondaryText);
  }
  if (hasCall) {
    output.callPhase = (CallPhase)data[offset++];
    readLine(data, offset, bodyEnd, output.callerText);
    readLine(data, offset, bodyEnd, output.callStatusText);
  }
  output.hasClock = hasClock;
  output.clockHour = hasClock ? data[clockOffset] : 0;
  output.clockMinute = hasClock ? data[clockOffset + 1] : 0;
  output.clockSecond = hasClock ? data[clockOffset + 2] : 0;
  output.clockOptions = hasClock ? data[clockOffset + 3] : 0;
  output.hasRemainingDuration = hasRemainingDuration;
  output.remainingDurationMinutes = hasRemainingDuration ? readU16(data + remainingDurationOffset) : 0;
  return DECODE_OK;
}

const char* decodeStatusName(DecodeStatus status) {
  switch (status) {
    case DECODE_OK: return "ok";
    case DECODE_TOO_SHORT: return "too short";
    case DECODE_TOO_LONG: return "too long";
    case DECODE_UNSUPPORTED_VERSION: return "unsupported version";
    case DECODE_INVALID_STATE: return "invalid state";
    case DECODE_INVALID_TEXT: return "invalid text";
    case DECODE_INVALID_MANEUVER: return "invalid maneuver";
    case DECODE_INVALID_FLAGS: return "invalid frame flags";
    case DECODE_TOO_MANY_POINTS: return "too many points";
    case DECODE_LENGTH_MISMATCH: return "length mismatch";
    case DECODE_INVALID_POINT_FLAGS: return "invalid point flags";
    case DECODE_FIRST_POINT_NOT_START: return "first point is not a start";
    case DECODE_CRC_MISMATCH: return "CRC mismatch";
    case DECODE_OUTPUT_TOO_SMALL: return "output too small";
    default: return "unknown";
  }
}

FragmentAssembler::FragmentAssembler() {
  resetSession();
}

void FragmentAssembler::clearAssembly() {
  active_ = false;
  activeSequence_ = 0;
  totalLength_ = 0;
  receivedCount_ = 0;
  lastFragmentAtMs_ = 0;
  memset(received_, 0, sizeof(received_));
}

void FragmentAssembler::resetSession() {
  clearAssembly();
  hasLastSequence_ = false;
  lastSequence_ = 0;
  lastDecodeStatus_ = DECODE_OK;
}

void FragmentAssembler::startAssembly(
  uint16_t sequence,
  uint16_t totalLength,
  uint32_t nowMs
) {
  clearAssembly();
  active_ = true;
  activeSequence_ = sequence;
  totalLength_ = totalLength;
  lastFragmentAtMs_ = nowMs;
}

bool FragmentAssembler::isNewerSequence(uint16_t candidate, uint16_t reference) {
  return (int16_t)(candidate - reference) > 0;
}

bool FragmentAssembler::expire(uint32_t nowMs) {
  if (!active_ || (uint32_t)(nowMs - lastFragmentAtMs_) <= ASSEMBLY_TIMEOUT_MS) {
    return false;
  }
  clearAssembly();
  return true;
}

FragmentResult FragmentAssembler::acceptFragment(
  const uint8_t* fragment,
  size_t fragmentLength,
  uint32_t nowMs,
  DisplayFrame& output
) {
  expire(nowMs);
  if (fragmentLength <= FRAGMENT_HEADER_SIZE ||
      fragment[0] != FRAGMENT_MAGIC ||
      fragment[1] != PROTOCOL_VERSION) {
    return FRAGMENT_INVALID;
  }

  uint16_t sequence = readU16(fragment + 2);
  uint16_t offset = readU16(fragment + 4);
  uint16_t totalLength = readU16(fragment + 6);
  size_t payloadLength = fragmentLength - FRAGMENT_HEADER_SIZE;

  if (totalLength < MIN_FRAME_SIZE || totalLength > MAX_FRAME_SIZE ||
      offset >= totalLength || payloadLength > totalLength - offset) {
    return FRAGMENT_INVALID;
  }

  if (active_ && sequence != activeSequence_) {
    if (!isNewerSequence(sequence, activeSequence_)) return FRAGMENT_STALE;
    startAssembly(sequence, totalLength, nowMs);
  } else if (!active_) {
    if (hasLastSequence_ && !isNewerSequence(sequence, lastSequence_)) {
      return FRAGMENT_STALE;
    }
    startAssembly(sequence, totalLength, nowMs);
  } else if (totalLength != totalLength_) {
    clearAssembly();
    return FRAGMENT_INVALID;
  }

  lastFragmentAtMs_ = nowMs;
  for (size_t i = 0; i < payloadLength; i++) {
    size_t destination = offset + i;
    buffer_[destination] = fragment[FRAGMENT_HEADER_SIZE + i];
    if (!received_[destination]) {
      received_[destination] = true;
      receivedCount_++;
    }
  }

  if (receivedCount_ != totalLength_) return FRAGMENT_IN_PROGRESS;

  lastDecodeStatus_ = decodeFrame(buffer_, totalLength_, output);
  if (lastDecodeStatus_ != DECODE_OK || output.sequence != activeSequence_) {
    clearAssembly();
    return FRAGMENT_FRAME_INVALID;
  }

  lastSequence_ = activeSequence_;
  hasLastSequence_ = true;
  clearAssembly();
  return FRAGMENT_COMPLETE;
}

DecodeStatus FragmentAssembler::lastDecodeStatus() const {
  return lastDecodeStatus_;
}

size_t FragmentAssembler::receivedCount() const {
  return receivedCount_;
}

size_t FragmentAssembler::expectedLength() const {
  return totalLength_;
}

}  // namespace navproto
