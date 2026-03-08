#include "esp32-hal-gpio.h"
#include "Arduino.h"
#include <TAMC_GT911.h>

#include <Wire.h>

TAMC_GT911::TAMC_GT911(uint8_t _int, uint16_t _width, uint16_t _height)
  : pinInt(_int), width(_width), height(_height) {
}

void TAMC_GT911::begin(TwoWire *wire) {
  _wire = wire;

  _wire->beginTransmission(GT911_ADDR1);
  if (_wire->endTransmission() == 0) {
    addr = GT911_ADDR1;
    Serial.println("detected ADDR1");
  } else {
    _wire->beginTransmission(GT911_ADDR2);
    if (_wire->endTransmission() == 0) {
      addr = GT911_ADDR2;
      Serial.print("detected ADDR2");
    } else {
      Serial.print("not detected");
      return;
    }
  }
  reset();
}

void TAMC_GT911::reset() {
  pinMode(pinInt, INPUT);
  //attachInterrupt(pinInt, onTouch, FALLING);
  readBlockData(GT911_CONFIG_START, configBuf, GT911_CONFIG_SIZE);
  setResolution(width, height);
}

void TAMC_GT911::calculateChecksum() {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < GT911_CONFIG_SIZE; i++) {
    checksum += configBuf[i];
  }
  checksum = (~checksum) + 1;
  configBuf[GT911_CONFIG_CHKSUM - GT911_CONFIG_START] = checksum;
}

void TAMC_GT911::reflashConfig() {
  calculateChecksum();
  writeByteData(GT911_CONFIG_CHKSUM, configBuf[GT911_CONFIG_CHKSUM - GT911_CONFIG_START]);
  writeByteData(GT911_CONFIG_FRESH, 1);
}

void TAMC_GT911::setRotation(uint8_t rot) {
  rotation = rot;
}

void TAMC_GT911::setResolution(uint16_t _width, uint16_t _height) {
  configBuf[GT911_X_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_width);
  configBuf[GT911_X_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_width);
  configBuf[GT911_Y_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_height);
  configBuf[GT911_Y_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_height);
  reflashConfig();
}

bool TAMC_GT911::read() {

  uint8_t data[8];

  readBlockData(GT911_POINT_INFO, data, 8);

  uint8_t pointInfo = data[0];
  uint8_t bufferStatus = (pointInfo >> 7) & 1;
  if (!bufferStatus) {
    return false;  // ❗ do NOT read points if buffer invalid.
  }

  writeByteData(GT911_POINT_INFO, 0);

  touches = pointInfo & 0x0F;

  isLargeDetect = (pointInfo >> 6) & 1;

  if (touches > 0) {
    down = true;
    points[0] = readPoint(data + 1);
    x = points[0].x;
    y = points[0].y;
    size = points[0].size;
  } else {
    down = false;
    x = last_x;
    y = last_y;
    size = last_size;
  }

  bool changed =
    x != last_x || y != last_y || size != last_size || down != last_down;

  last_x = x;
  last_y = y;
  last_size = size;
  last_down = down;

  return changed;
}


TP_Point TAMC_GT911::readPoint(uint8_t *data) {
  uint16_t temp;
  uint8_t id = data[0];
  uint16_t x = data[1] + (data[2] << 8);
  uint16_t y = data[3] + (data[4] << 8);
  uint16_t size = data[5] + (data[6] << 8);

  switch (rotation) {
    case ROTATION_NORMAL:
      x = width - x;
      y = height - y;
      break;

    case ROTATION_LEFT:
      temp = x;
      x = width - y;
      y = temp;
      break;

    case ROTATION_INVERTED:
      break;

    case ROTATION_RIGHT:
      temp = x;
      x = y;
      y = height - temp;
      break;

    default:
      break;
  }

  return TP_Point(id, x, y, size);
}

void TAMC_GT911::writeByteData(uint16_t reg, uint8_t val) {
  _wire->beginTransmission(addr);
  _wire->write(highByte(reg));
  _wire->write(lowByte(reg));
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t TAMC_GT911::readByteData(uint16_t reg) {
  _wire->beginTransmission(addr);
  _wire->write(highByte(reg));
  _wire->write(lowByte(reg));
  _wire->endTransmission(false);  // repeated start

  if (_wire->requestFrom(addr, (uint8_t)1) != 1) {
    return 0;
  }
  return _wire->read();
}

void TAMC_GT911::readBlockData(uint16_t reg, uint8_t *buf, uint8_t size) {
  _wire->beginTransmission(addr);
  _wire->write(highByte(reg));
  _wire->write(lowByte(reg));
  _wire->endTransmission(false);  // repeated start

  uint8_t received = _wire->requestFrom(addr, size);
  if (received != size) {
    return;
  }

  for (uint8_t i = 0; i < size && _wire->available(); i++) {
    buf[i] = _wire->read();
  }
}


// TP_Point implementation

TP_Point::TP_Point()
  : id(0), x(0), y(0), size(0) {
}

TP_Point::TP_Point(uint8_t _id, uint16_t _x, uint16_t _y, uint16_t _size)
  : id(_id), x(_x), y(_y), size(_size) {
}

bool TP_Point::operator==(TP_Point point) {
  return (point.x == x) && (point.y == y) && (point.size == size);
}

bool TP_Point::operator!=(TP_Point point) {
  return (point.x != x) || (point.y != y) || (point.size != size);
}