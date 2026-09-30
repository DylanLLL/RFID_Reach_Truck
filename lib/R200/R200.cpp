#include <Arduino.h>
#include "R200.h"

// Constructor
R200::R200(){};

bool R200::begin(HardwareSerial* serial, int baud, uint8_t RxPin, uint8_t TxPin)
{
  _serial = serial;
  _serial->begin(baud, SERIAL_8N1, RxPin, TxPin);
  return true;
};

void printHexByte(const char* name, uint8_t value)
{
  Serial.print(name);
  Serial.print(":");
  Serial.print(value < 0x10 ? "0x0" : "0x");
  Serial.println(value, HEX);
}

void printHexBytes(const char* name, uint8_t* value, uint8_t len)
{
  Serial.print(name);
  Serial.print(":");
  Serial.print("0x");
  for (int i = 0; i < len; i++)
  {
    Serial.print(value[i] < 0x10 ? "0" : "");
    Serial.print(value[i], HEX);
  }
  Serial.println("");
}

void printHexWord(const char* name, uint8_t MSB, uint8_t LSB)
{
  Serial.print(name);
  Serial.print(":");
  Serial.print(MSB < 0x10 ? "0x0" : "0x");
  Serial.print(MSB, HEX);  // was println, which split every word across two lines
  Serial.print(LSB < 0x10 ? "0" : "");
  Serial.println(LSB, HEX);
}

R200::R200_Event R200::loop()
{
  R200_Event event = EVT_None;

  // Has any new data been received?
  if (dataAvailable())
  {
    // Attempt to receive a full frame of data
    if (receiveData())
    {
      if (dataIsValid())
      {
        event = EVT_Other;
        // If a full frame of data has been received, parse it
        // TODO For reasons that I absolutely cannot fathom, this section does not work if moved into
        // a separate function....
        // parseReceivedData();
        switch (_buffer[R200_CommandPos])
        {
          case CMD_GetModuleInfo:
            for (uint8_t i = 0; i < RX_BUFFER_LENGTH - 8; i++)
            {
              Serial.print((char)_buffer[6 + i]);
              // Stop when then only two bytes left are the CRC and FrameEnd marker
              if (_buffer[8 + i] == R200_FrameEnd)
              {
                break;
              }
            }
            Serial.println("");
            break;
          case CMD_SinglePollInstruction:
// Example successful response
// AA 02 22 00 11 C7 30 00 E2 80 68 90 00 00 50 0E 88 C6 A4 A7 11 9B 29 DD
// AA:Frame Header
// 02:Instruction Code
// 22:Command Parameter
// 00 11:Instruction data length (0x11 = 17 bytes)
// C7：RSSI Signal Strength
// 30 00: Label PC code (factory reg code)
// E2 80 68 90 00 00 50 0E 88 C6 A4 A7：EPC code
// 11 9B:CRC check
// 29: Verification
// DD: End of frame
// FIELD OFFSETS CORRECTED. The frame above annotates C7 as RSSI, and C7 sits at
// index 5 -- params begin immediately after the 2-byte length at [3][4]. The
// original code read RSSI from [6] (the PC code's MSB, a constant 0x30 on
// standard tags) and the EPC from [9], one byte late, so the stored uid dropped
// the leading EPC byte and picked up the first CRC byte instead. It looked
// plausible because it was still deterministic per tag.
// Note R200::parseReceivedData() below already used the correct offset 8.
            // A tag frame is RSSI(1) + PC(2) + EPC + CRC(2). Everything downstream
            // assumes a 96-bit EPC, so check the length the tag itself declares in
            // the PC word (bits 15-11, in 16-bit words: 0x30 >> 3 = 6 = 12 bytes)
            // rather than silently misparsing a tag that differs.
            if (arrayToUint16(&_buffer[R200_ParamLengthMSBPos]) != 17 || (_buffer[6] >> 3) != 6)
            {
              break;
            }
            rssi = (int8_t)_buffer[5];
            event = EVT_TagRead;
#ifdef DEBUG
            printHexByte("RSSI", _buffer[5]);
            printHexWord("PC", _buffer[6], _buffer[7]);
            printHexBytes("EPC(", &_buffer[8], 12);
#endif
            if (memcmp(uid, &_buffer[8], 12) != 0)
            {
              memcpy(uid, &_buffer[8], 12);
#ifdef DEBUG
              Serial.print("New card detected : ");
              dumpUIDToSerial();
              Serial.println("");
#endif
            }
            else
            {
#ifdef DEBUG
              Serial.print("Same card still present : ");
              dumpUIDToSerial();
              Serial.println("");
#endif
            }
#ifdef DEBUG
            printHexWord("CRC", _buffer[20], _buffer[21]);
#endif
            break;
          case CMD_ExecutionFailure:
            switch (_buffer[R200_ParamPos])
            {
              case ERR_CommandError:
                Serial.println("Command error");
                break;
              case ERR_InventoryFail:
                // This is not necessarily a "failure" - it just means that there are no cards in range
                // Serial.print("No card detected!");
                event = EVT_NoTag;
                // If there was previously a uid
                if (memcmp(uid, blankUid, sizeof uid) != 0)
                {
#ifdef DEBUG
                  Serial.print("Card removed : ");
                  dumpUIDToSerial();
                  Serial.println("");
#endif
                  memset(uid, 0, sizeof uid);
                }
                break;
              case ERR_AccessFail:
                // Serial.println("Access Fail");
                break;
              case ERR_ReadFail:
                // Serial.println("Read fail");
                break;
              case ERR_WriteFail:
                // Serial.println("Write fail");
                break;
              default:
                // Serial.print("Fail code ");
                // Serial.println(_buffer[R200_ParamPos], HEX);
                break;
            }
            break;
        }
      }
    }
  }
  return event;
}

// Has any data been received from the reader?
bool R200::dataIsValid()
{
  // Serial.println("Checking Data Valid");
  // dumpReceiveBufferToSerial();
  // NOTE
  // You can't just be smart and do this in one line, because
  // the pointer reference f*cks up.
  // uint16_t paramLength = _buffer[3]<<8 + _buffer[4];
  uint16_t paramLength = _buffer[3];
  paramLength <<= 8;
  paramLength += _buffer[4];

  // Bounds check before indexing. paramLength comes off the wire, so a single
  // corrupted byte can claim up to 65535 params. CRCpos was also a uint8_t,
  // which silently wrapped mod 256 and then indexed a 64-byte buffer.
  uint16_t CRCpos = 5u + paramLength;
  if (CRCpos >= RX_BUFFER_LENGTH)
  {
    return false;
  }

  uint8_t CRC = calculateCheckSum(_buffer);
  return (CRC == _buffer[CRCpos]);
}

// Has any data been received from the reader?
bool R200::dataAvailable()
{
  // Serial.println("Checking Data Available");
  return _serial->available() > 0;
}

/*
 * Dumps the most recently read UID to the serial output
 */
void R200::dumpUIDToSerial()
{
  // Serial.print("Dumping UID...");
  Serial.print("0x");
  for (uint8_t i = 0; i < 12; i++)
  {
    Serial.print(uid[i] < 0x10 ? "0" : "");
    Serial.print(uid[i], HEX);
  }
  // Serial.println(". Done.");
}

void R200::dumpReceiveBufferToSerial()
{
  // Serial.print("Dumping buffer...");
  Serial.print("0x");
  for (uint8_t i = 0; i < RX_BUFFER_LENGTH; i++)
  {
    Serial.print(_buffer[i] < 0x10 ? "0" : "");
    Serial.print(_buffer[i], HEX);
  }
  Serial.println(". Done.");
}

// Parse data that has been placed in the receive buffer
bool R200::parseReceivedData()
{
  switch (_buffer[R200_CommandPos])
  {
    case CMD_GetModuleInfo:
      break;
    case CMD_SinglePollInstruction:
      for (uint8_t i = 8; i < 20; i++)
      {
        uid[i - 8] = _buffer[i];
      };
      // memcpy(uid, _buffer+9, 12);
      break;
    case CMD_MultiplePollInstruction:
      for (uint8_t i = 8; i < 20; i++)
      {
        uid[i - 8] = _buffer[i];
      };
      // memcpy(uid, _buffer+9, 12);
      break;
    case CMD_ExecutionFailure:
      break;
    default:
      return false;
  }
  return true;
}

/*
 * Note that Arduino Serial.flush() method does not clear the incoming serial buffer - only the outgoing!
 */
uint8_t R200::flush()
{
  uint8_t bytesDiscarded = 0;
  while (_serial->available())
  {
    _serial->read();
    bytesDiscarded++;
  }
  return bytesDiscarded;
}

// Read incoming serial data sent by the reader
// This could either be a response to a command sent, or a notification
// (e.g. when set to automatic polling mode)
// Returns true if a complete frame of data is read within the allotted timeout
// Wait for a single byte, giving up once timeOut has elapsed since startTime.
// Unsigned arithmetic on millis() makes the comparison overflow-safe.
bool R200::readByte(unsigned long startTime, unsigned long timeOut, uint8_t& out)
{
  while ((millis() - startTime) < timeOut)
  {
    if (_serial->available())
    {
      out = (uint8_t)_serial->read();
      return true;
    }
    // Don't starve the idle task while spinning.
    yield();
  }
  return false;
}

// Read one complete frame from the reader.
//
// Rewritten to be length-driven. The original scanned for the 0xDD frame-end
// byte, which fails two ways:
//
//  1. 0xDD occurs inside EPC and CRC payloads -- about 5% of random 12-byte
//     EPCs contain one -- and truncated the frame mid-EPC.
//  2. Its `break` on frame-end only left the inner loop, so the outer loop kept
//     spinning until the full timeout on EVERY call, and any bytes arriving in
//     the meantime were appended past the frame end, invalidating a good frame.
//
// Now the declared parameter length says exactly how many bytes to consume, and
// the function returns as soon as the frame is complete.
bool R200::receiveData(unsigned long timeOut)
{
  const unsigned long startTime = millis();

  for (int i = 0; i < RX_BUFFER_LENGTH; i++)
  {
    _buffer[i] = 0;
  }

  // Hunt for the frame header. Anything ahead of it is noise or the tail of a
  // frame we already abandoned, so discarding it resynchronises the stream.
  uint8_t b = 0;
  do
  {
    if (!readByte(startTime, timeOut, b))
    {
      return false;
    }
  } while (b != R200_FrameHeader);
  _buffer[R200_HeaderPos] = R200_FrameHeader;

  // Fixed part: type, command, and the two length bytes.
  for (uint8_t i = 1; i <= 4; i++)
  {
    if (!readByte(startTime, timeOut, _buffer[i]))
    {
      return false;
    }
  }

  uint16_t paramLength = _buffer[R200_ParamLengthMSBPos];
  paramLength <<= 8;
  paramLength += _buffer[R200_ParamLengthLSBPos];

  // header + type + command + 2 length bytes + params + checksum + frame end
  const uint16_t frameLength = 7u + paramLength;
  if (frameLength > RX_BUFFER_LENGTH)
  {
    // A corrupt length byte can claim up to 65535 params. Bail and let the
    // header hunt above resynchronise on the next call.
    return false;
  }

  // Consume exactly the declared remainder. A 0xDD inside the EPC or CRC is
  // now just another payload byte.
  for (uint16_t i = 5; i < frameLength; i++)
  {
    if (!readByte(startTime, timeOut, _buffer[i]))
    {
      return false;
    }
  }

  // The frame must end where its own length field said it would.
  return (_buffer[frameLength - 1] == R200_FrameEnd);
}

void R200::dumpModuleInfo()
{
  uint8_t commandFrame[8] = { 0 };
  commandFrame[0] = R200_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_GetModuleInfo;
  commandFrame[3] = 0x00;  // ParamLen MSB
  commandFrame[4] = 0x01;  // ParamLen LSB
  commandFrame[5] = 0x00;  // Param
  commandFrame[6] = 0x04;  // LSB of commandFrame[2] + commandFrame[3] + commandFrame[4] + commandFrame[5]
  commandFrame[7] = R200_FrameEnd;
  _serial->write(commandFrame, 8);
}

/**
 * Send single poll command to the reader
 */
void R200::poll()
{
  uint8_t commandFrame[7] = { 0 };
  commandFrame[0] = R200_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_SinglePollInstruction;
  commandFrame[3] = 0x00;  // ParamLen MSB
  commandFrame[4] = 0x00;  // ParamLen LSB
  commandFrame[5] = 0x22;  // Checksum
  commandFrame[6] = R200_FrameEnd;
  _serial->write(commandFrame, 7);
}

void R200::setMultiplePollingMode(bool enable)
{
  if (enable)
  {
    uint8_t commandFrame[10] = { 0 };
    commandFrame[0] = R200_FrameHeader;
    commandFrame[1] = FrameType_Command;            //(0x00)
    commandFrame[2] = CMD_MultiplePollInstruction;  // 0x27
    commandFrame[3] = 0x00;                         // ParamLen MSB
    commandFrame[4] = 0x03;                         // ParamLen LSB
    commandFrame[5] = 0x22;                         // Param (Reserved? Always 0x22 for this command)
    commandFrame[6] = 0xFF;                         // Param (Count of polls, MSB)
    commandFrame[7] = 0xFF;                         // Param (Count of polls, LSB)
    commandFrame[8] = 0x4A;  // LSB of commandFrame[2] + commandFrame[3] + commandFrame[4] + commandFrame[5] +
                             // commandFrame[6] + commandFrame[7] (full value is 0x024A)
    commandFrame[9] = R200_FrameEnd;
    _serial->write(commandFrame, 10);
  }
  else
  {
    uint8_t commandFrame[7] = { 0 };
    commandFrame[0] = R200_FrameHeader;
    commandFrame[1] = FrameType_Command;     //(0x00)
    commandFrame[2] = CMD_StopMultiplePoll;  // 0x28
    commandFrame[3] = 0x00;                  // ParamLen MSB
    commandFrame[4] = 0x00;                  // ParamLen LSB
    commandFrame[5] = 0x28;                  // LSB of commandFrame[2] + commandFrame[3] + commandFrame[4]
    commandFrame[6] = R200_FrameEnd;
    _serial->write(commandFrame, 7);
  }
}

uint8_t R200::calculateCheckSum(uint8_t* buffer)
{
  // Extract how many parameters there are in the buffer
  uint16_t paramLength = buffer[3];
  paramLength <<= 8;
  paramLength += buffer[4];

  // Clamp to the buffer. Unbounded, a corrupt length field walks up to 64 KB
  // past the end of a 64-byte array -- the most likely source of unexplained
  // crashes once this is running against real tags in a noisy environment.
  uint16_t lastIndex = paramLength + 4;
  if (lastIndex >= RX_BUFFER_LENGTH)
  {
    lastIndex = RX_BUFFER_LENGTH - 1;
  }

  // Checksum is calculated as the sum of all parameter bytes
  // added to four control bytes at the start (type, command, and the 2-byte parameter length)
  // Start from 1 to exclude frame header
  uint16_t check = 0;
  for (uint16_t i = 1; i <= lastIndex; i++)
  {
    check += buffer[i];
  }
  // Now only return LSB
  return (check & 0xff);

  /*
  // This is an alternative checksum calculation sometimes used
  uint16_t paramLength = *(buffer+3);
  paramLength <<=8;
  paramLength += *(buffer+4);

  uint16_t sum = 0;
  for (int i=1; i<4+paramLength; i++) {
    sum += buffer[i];
  }
  return -sum;
  */
}

uint16_t R200::arrayToUint16(uint8_t* array)
{
  uint16_t value = *array;
  value <<= 8;
  value += *(array + 1);
  return value;
}