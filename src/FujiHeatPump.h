#include <HardwareSerial.h>
#include <Arduino.h>


const byte kModeIndex = 3;
const byte kModeMask = 0b00001110;
const byte kModeOffset = 1;

const byte kFanIndex = 3;
const byte kFanMask = 0b01110000;
const byte kFanOffset = 4;

const byte kEnabledIndex = 3;
const byte kEnabledMask = 0b00000001;
const byte kEnabledOffset = 0;

const byte kErrorIndex = 3;
const byte kErrorMask = 0b10000000;
const byte kErrorOffset = 7;

const byte kEconomyIndex = 4;
const byte kEconomyMask = 0b10000000;
const byte kEconomyOffset = 7;

const byte kTemperatureIndex = 4;
const byte kTemperatureMask = 0b01111111;
const byte kTemperatureOffset = 0;

const byte kUpdateMagicIndex = 5;
const byte kUpdateMagicMask = 0b11110000;
const byte kUpdateMagicOffset = 4;

const byte kSwingIndex = 5;
const byte kSwingMask = 0b00000100;
const byte kSwingOffset = 2;

const byte kSwingStepIndex = 5;
const byte kSwingStepMask = 0b00000010;
const byte kSwingStepOffset = 1;

const byte kControllerPresentIndex = 6;
const byte kControllerPresentMask = 0b00000001;
const byte kControllerPresentOffset = 0;

const byte kControllerTempIndex = 6;
const byte kControllerTempMask = 0b01111110;
const byte kControllerTempOffset = 1;

// Zone message constants
const byte kZoneGroupIndex = 4;
const byte kZoneGroupMask = 0b00011000;
const byte kZoneGroupOffset = 3;

const byte kZoneWriteBitIndex = 4;
const byte kZoneWriteBitMask = 0b10000000;
const byte kZoneWriteBitOffset = 7;

const byte kZoneStateIndex = 3;
const byte kZoneStateMask = 0b00000011;
const byte kZoneStateOffset = 0;

const size_t kZoneCount = 8;

// Zone message signature (bytes 0-2 pattern)
const byte kZoneMessageSignature0 = 0x20;
const byte kZoneMessageSignature1 = 0xA1;
const byte kZoneMessageSignature2 = 0x58;

// LIN bus timing constants
const unsigned long kMinFrameDelay = 50;  // Minimum delay after receiving frame before sending (ms)
const unsigned long kMaxFrameDelay = 60;  // Maximum delay after receiving frame before sending (ms)
const unsigned long kMinRetryDelay = 100; // Minimum delay between retry attempts (ms) - prevents rapid-fire retries


typedef struct FujiFrames  {
    byte onOff = 0;
    byte temperature = 16;
    byte acMode = 0;
    byte fanMode = 0;
    byte acError = 0;
    byte economyMode = 0;
    byte swingMode = 0;
    byte swingStep = 0;
    byte controllerPresent = 0;
    byte updateMagic = 0; // unsure what this value indicates
    byte controllerTemp = 16;

    bool writeBit = false;
    bool loginBit = false;
    bool unknownBit = false; // unsure what this bit indicates

    byte messageType = 0;
    byte messageSource = 0;
    byte messageDest = 0;
} FujiFrame;

enum class FujiZoneGroup : byte {
    NONE = 0x0,
    DAY = 0x1,
    NIGHT = 0x2,
    ALL = 0x3,
};

typedef struct ZoneFrames {
    bool zones[kZoneCount];
    FujiZoneGroup zoneGroup;
    bool zoneWriteBit;
    bool dayZones[kZoneCount];
    bool nightZones[kZoneCount];

    bool writeBit = false;
    bool loginBit = false;
    bool unknownBit = false;

    byte messageType = 0;
    byte messageSource = 0;
    byte messageDest = 0;
} ZoneFrame;

class FujiHeatPump
{
  private:
    HardwareSerial *_serial;
    byte            readBuf[8];
    byte            writeBuf[8];
    byte            zoneWriteBuf[8];
    
    byte            controllerAddress;  
    bool            controllerIsPrimary = true;  
    bool            seenSecondaryController = false;  
    bool            controllerLoggedIn = false; 
    unsigned long   lastFrameReceived;
    unsigned long   lastFrameSent;
    unsigned long   lastRetryAttempt;  // Track when we last attempted to send (for retry backoff)
    unsigned long   updateFirstSent;   // Track when we first sent the update (for confirmation delay)
    
    byte            updateFields;
    FujiFrame       updateState;
    FujiFrame       currentState;
    byte            updateRetryCount = 0;  // Track retry attempts for pending updates
    
    ZoneFrame       currentZoneState;
    ZoneFrame       zoneUpdateState;
    bool            initialZoneStateReceived = false;
    bool            pendingZoneFrame = false;

    FujiFrame decodeFrame();
    ZoneFrame decodeZoneFrame();
    void encodeFrame(FujiFrame ff);
    void encodeZoneFrame(ZoneFrame zf);
    void printFrame(byte buf[8], FujiFrame ff);
    bool isZoneMessage(byte buf[8]);
    
    bool pendingFrame = false;
  public:
    void connect(HardwareSerial *serial, bool secondary);
    void connect(HardwareSerial *serial, bool secondary, int rxPin, int txPin);

    bool waitForFrame();
    void sendPendingFrame();
    bool sendPendingZoneFrame();
    bool isBound();
    bool updatePending();
    
    void setOnOff(bool o);
    void setTemp(byte t);
    void setMode(byte m);
    void setFanMode(byte fm);
    void setEconomyMode(byte em);
    void setSwingMode(byte sm);
    void setSwingStep(byte ss);
    
    void setZoneGroup(FujiZoneGroup zoneGroup);
    void setZoneOnOff(int zone, bool on);
    
    bool getOnOff();
    byte getTemp();
    byte getMode();
    byte getFanMode();
    byte getEconomyMode();
    byte getSwingMode();
    byte getSwingStep();
    byte getControllerTemp();
    
    FujiZoneGroup getZoneGroup();
    bool getZoneOnOff(int zone);
    
    FujiFrame *getCurrentState();
    FujiFrame *getUpdateState();
    ZoneFrame *getCurrentZoneState();
    byte getUpdateFields();
    
    bool debugPrint = false;
    
};

enum class FujiMode : byte {
  UNKNOWN=0,
  FAN=1,
  DRY=2,
  COOL=3,
  HEAT=4,
  AUTO=5
};

enum class FujiMessageType : byte {
  STATUS  = 0,
  ERROR   = 1,
  LOGIN   = 2,
  UNKNOWN = 3,
  ZONE    = 5,
};

enum class FujiAddress : byte {
  START       = 0,
  UNIT        = 1,
  PRIMARY     = 32,
  SECONDARY   = 33,
};

enum class FujiFanMode : byte {
  FAN_AUTO      = 0,
  FAN_QUIET     = 1,
  FAN_LOW       = 2,
  FAN_MEDIUM    = 3,
  FAN_HIGH      = 4
};

const byte kOnOffUpdateMask       = 0b10000000;
const byte kTempUpdateMask        = 0b01000000;
const byte kModeUpdateMask        = 0b00100000;
const byte kFanModeUpdateMask     = 0b00010000;
const byte kEconomyModeUpdateMask = 0b00001000;
const byte kSwingModeUpdateMask   = 0b00000100;
const byte kSwingStepUpdateMask   = 0b00000010;

const byte kMaxUpdateRetries = 15;  // Maximum number of retry attempts before giving up
