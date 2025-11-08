    #include "FujiHeatPump.h"
    #include <string.h>



bool FujiHeatPump::isZoneMessage(byte buf[8]) {
    // Zone messages have a specific signature: 20 A1 58 in first 3 bytes
    return (buf[0] == kZoneMessageSignature0 && 
            buf[1] == kZoneMessageSignature1 && 
            buf[2] == kZoneMessageSignature2);
}

FujiFrame FujiHeatPump::decodeFrame() {
    FujiFrame ff;

    ff.messageSource       =  readBuf[0];
    ff.messageDest         =  readBuf[1] & 0b01111111;
    ff.messageType         = (readBuf[2] & 0b00110000) >> 4;

    ff.acError             = (readBuf[kErrorIndex] & kErrorMask) >> kErrorOffset;
    ff.temperature         = (readBuf[kTemperatureIndex] & kTemperatureMask) >> kTemperatureOffset;
    ff.acMode              = (readBuf[kModeIndex] & kModeMask) >> kModeOffset;
    ff.fanMode             = (readBuf[kFanIndex] & kFanMask) >> kFanOffset;
    ff.economyMode         = (readBuf[kEconomyIndex] & kEconomyMask) >> kEconomyOffset;
    ff.swingMode           = (readBuf[kSwingIndex] & kSwingMask) >> kSwingOffset;
    ff.swingStep           = (readBuf[kSwingStepIndex] & kSwingStepMask) >> kSwingStepOffset;
    ff.controllerPresent   = (readBuf[kControllerPresentIndex] & kControllerPresentMask) >> kControllerPresentOffset;
    ff.updateMagic         = (readBuf[kUpdateMagicIndex] & kUpdateMagicMask) >> kUpdateMagicOffset;
    ff.onOff               = (readBuf[kEnabledIndex] & kEnabledMask) >> kEnabledOffset;
    ff.controllerTemp      = (readBuf[kControllerTempIndex] & kControllerTempMask) >> kControllerTempOffset; // there is one leading bit here that is unknown - probably a sign bit for negative temps?

    ff.writeBit =   (readBuf[2] & 0b00001000) != 0;
    ff.loginBit =   (readBuf[1] & 0b00100000) != 0;
    ff.unknownBit = (readBuf[1] & 0b10000000)  > 0;

    return ff;
}

ZoneFrame FujiHeatPump::decodeZoneFrame() {
    ZoneFrame zf;
    
    // Initialize zones array
    for(int i = 0; i < kZoneCount; i++) {
        zf.zones[i] = false;
        zf.dayZones[i] = false;
        zf.nightZones[i] = false;
    }
    
    zf.messageSource = readBuf[0];
    zf.messageDest = readBuf[1] & 0b01111111;
    zf.messageType = static_cast<byte>(FujiMessageType::ZONE);
    
    // Decode zone state from byte 3 (bits 0-1)
    byte zoneState = (readBuf[kZoneStateIndex] & kZoneStateMask) >> kZoneStateOffset;
    
    // Map zone state to individual zones
    // Based on debug data:
    // 1 = zone 0 on, zone 1 off (ground on, upstairs off)
    // 2 = zone 0 off, zone 1 on (ground off, upstairs on)
    // 3 = both zones on (ground on, upstairs on)
    // 0 = both zones off (likely)
    if(zoneState == 1) {
        zf.zones[0] = true;
        zf.zones[1] = false;
    } else if(zoneState == 2) {
        zf.zones[0] = false;
        zf.zones[1] = true;
    } else if(zoneState == 3) {
        zf.zones[0] = true;
        zf.zones[1] = true;
    } else {
        zf.zones[0] = false;
        zf.zones[1] = false;
    }
    
    // Decode zone group from byte 4 (bits 3-4)
    byte zoneGroupBits = (readBuf[kZoneGroupIndex] & kZoneGroupMask) >> kZoneGroupOffset;
    zf.zoneGroup = static_cast<FujiZoneGroup>(zoneGroupBits);
    
    // Decode zone write bit from byte 4 (bit 7)
    zf.zoneWriteBit = (readBuf[kZoneWriteBitIndex] & kZoneWriteBitMask) >> kZoneWriteBitOffset;
    
    zf.writeBit = (readBuf[2] & 0b00001000) != 0;
    zf.loginBit = (readBuf[1] & 0b00100000) != 0;
    zf.unknownBit = (readBuf[1] & 0b10000000) > 0;
    
    return zf;
}

void FujiHeatPump::encodeFrame(FujiFrame ff){

    memset(writeBuf, 0, 8);

    writeBuf[0] = ff.messageSource;

    writeBuf[1] &= 0b10000000;
    writeBuf[1] |= ff.messageDest & 0b01111111;

    writeBuf[2] &= 0b11001111;
    writeBuf[2] |= ff.messageType << 4;

    if(ff.writeBit){
        writeBuf[2] |= 0b00001000;
    } else {
        writeBuf[2] &= 0b11110111;
    }

    writeBuf[1] &= 0b01111111;
    if(ff.unknownBit) {
        writeBuf[1] |= 0b10000000;
    }

    if(ff.loginBit){
        writeBuf[1] |= 0b00100000;
    } else {
        writeBuf[1] &= 0b11011111;
    }

    writeBuf[kModeIndex] =              (writeBuf[kModeIndex]              & ~kModeMask)              | (ff.acMode << kModeOffset);
    writeBuf[kModeIndex] =              (writeBuf[kEnabledIndex]           & ~kEnabledMask)           | (ff.onOff << kEnabledOffset);
    writeBuf[kFanIndex] =               (writeBuf[kFanIndex]               & ~kFanMask)               | (ff.fanMode << kFanOffset);
    writeBuf[kErrorIndex] =             (writeBuf[kErrorIndex]             & ~kErrorMask)             | (ff.acError << kErrorOffset);
    writeBuf[kEconomyIndex] =           (writeBuf[kEconomyIndex]           & ~kEconomyMask)           | (ff.economyMode << kEconomyOffset);
    writeBuf[kTemperatureIndex] =       (writeBuf[kTemperatureIndex]       & ~kTemperatureMask)       | (ff.temperature << kTemperatureOffset);
    writeBuf[kSwingIndex] =             (writeBuf[kSwingIndex]             & ~kSwingMask)             | (ff.swingMode << kSwingOffset);
    writeBuf[kSwingStepIndex] =         (writeBuf[kSwingStepIndex]         & ~kSwingStepMask)         | (ff.swingStep << kSwingStepOffset);
    writeBuf[kControllerPresentIndex] = (writeBuf[kControllerPresentIndex] & ~kControllerPresentMask) | (ff.controllerPresent << kControllerPresentOffset);
    writeBuf[kUpdateMagicIndex] =       (writeBuf[kUpdateMagicIndex]       & ~kUpdateMagicMask)       | (ff.updateMagic << kUpdateMagicOffset);
    writeBuf[kControllerTempIndex] =    (writeBuf[kControllerTempIndex]    & ~kControllerTempMask)    | (ff.controllerTemp << kControllerTempOffset);

}

void FujiHeatPump::encodeZoneFrame(ZoneFrame zf) {
    memset(zoneWriteBuf, 0, 8);
    
    // Set source and destination addresses
    // Zone messages use the same address pattern as regular frames
    zoneWriteBuf[0] = zf.messageSource;  // Our controller address
    
    // Byte 1: destination address with flags
    zoneWriteBuf[1] = zf.messageDest & 0b01111111;  // Destination address (bits 0-6)
    if(zf.unknownBit) {
        zoneWriteBuf[1] |= 0b10000000;  // Bit 7: unknown bit
    }
    if(zf.loginBit) {
        zoneWriteBuf[1] |= 0b00100000;  // Bit 5: login bit
    }
    
    // Byte 2: message type and flags
    // Zone messages appear to use message type 1 (ERROR type) but with 0x58 pattern
    // Actually, looking at received zone messages: 20 A1 58
    // Byte 2 = 0x58 = 01011000 = message type 1 (bits 4-5) | write bit (bit 3) | other bits
    // For zone messages, we use a special pattern in byte 2
    zoneWriteBuf[2] = 0x58;  // Zone message identifier
    if(zf.writeBit) {
        zoneWriteBuf[2] |= 0b00001000;  // Bit 3: write bit
    }
    
    // Set remaining bytes based on debug data pattern
    // From debug: 20 A1 58 X Y 9 0 41
    // Bytes 5-7 seem to be constant or related to other state
    zoneWriteBuf[5] = 0x09;
    zoneWriteBuf[6] = 0x00;
    zoneWriteBuf[7] = 0x41;
    
    // If zone group is set, use group-based encoding
    if(zf.zoneGroup != FujiZoneGroup::NONE) {
        // Zone group encoding
        // Day group (0x89): zone state = 1, zone group = 1, write bit = 1
        // Night group (0x90): zone state = 2, zone group = 2, write bit = 1
        // All group (0x98): zone state = 3, zone group = 3, write bit = 1
        byte zoneGroupValue = static_cast<byte>(zf.zoneGroup);
        byte zoneState = 0;
        
        // Set zone state based on group
        if(zf.zoneGroup == FujiZoneGroup::DAY) {
            zoneState = 1;
        } else if(zf.zoneGroup == FujiZoneGroup::NIGHT) {
            zoneState = 2;
        } else if(zf.zoneGroup == FujiZoneGroup::ALL) {
            zoneState = 3;
        }
        
        zoneWriteBuf[kZoneStateIndex] = zoneState;
        
        // Encode zone group and write bit in byte 4
        // Based on debug data analysis:
        // Day (0x89):  0x80 | (1<<3) | 0x01 = 0x80 | 0x08 | 0x01 = 0x89
        // Night (0x90): 0x80 | (2<<3) | 0x00 = 0x80 | 0x10 | 0x00 = 0x90
        // All (0x98):  0x80 | (3<<3) | 0x00 = 0x80 | 0x18 | 0x00 = 0x98
        // Pattern: byte 4 = write bit | (zoneGroup << 3) | (zoneState == 1 ? 0x01 : 0x00)
        byte zoneStateBit = (zoneState == 1) ? 0x01 : 0x00;
        zoneWriteBuf[kZoneGroupIndex] = kZoneWriteBitMask | (zoneGroupValue << kZoneGroupOffset) | zoneStateBit;
    } else {
        // Individual zone control
        // Convert zone states to byte value:
        // zone 0 on, zone 1 off = 1
        // zone 0 off, zone 1 on = 2
        // both zones on = 3
        // both zones off = 0
        byte zoneState = 0;
        if(zf.zones[0] && !zf.zones[1]) {
            zoneState = 1;
        } else if(!zf.zones[0] && zf.zones[1]) {
            zoneState = 2;
        } else if(zf.zones[0] && zf.zones[1]) {
            zoneState = 3;
        } else {
            zoneState = 0;
        }
        
        zoneWriteBuf[kZoneStateIndex] = zoneState;
        
        // Individual zone control: zone group = 0, write bit = 1
        zoneWriteBuf[kZoneGroupIndex] = kZoneWriteBitMask;
    }
}

void FujiHeatPump::connect(HardwareSerial *serial, bool secondary){
    return this->connect(serial, secondary, -1, -1);
}

void FujiHeatPump::connect(HardwareSerial *serial, bool secondary, int rxPin=-1, int txPin=-1){
    _serial = serial;
    if(rxPin != -1 && txPin != -1) {
#ifdef ESP32
        _serial->begin(500, SERIAL_8E1, rxPin, txPin);
#else
        Serial.print("Setting RX/TX pin unsupported, using defaults.\n");
        _serial->begin(500, SERIAL_8E1);
#endif
    } else {
        _serial->begin(500, SERIAL_8E1);
    }
    _serial->setTimeout(200);
    
    if(secondary) {
        controllerIsPrimary = false;
        controllerAddress = static_cast<byte>(FujiAddress::SECONDARY);
    } else {
        controllerIsPrimary = true;
        controllerAddress = static_cast<byte>(FujiAddress::PRIMARY);
    }
    
    lastFrameReceived = 0;
    lastFrameSent = 0;
    updateRetryCount = 0;
}

void printBinary(byte value) {
    for (int i = 7; i >= 0; i--) {
        Serial.print((value >> i) & 1);
    }
    Serial.print(" ");
} // added to print in binary for zone discovery


void FujiHeatPump::printFrame(byte buf[8], FujiFrame ff) {
  for (int i = 0; i < 8; i++) {
        printBinary(buf[i]);
  } // added to print in binary for zone discovery
  
  Serial.printf("%X %X %X %X %X %X %X %X  ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  Serial.printf(" mSrc: %d mDst: %d mType: %d write: %d login: %d unknown: %d onOff: %d temp: %d, mode: %d cP:%d uM:%d cTemp:%d acError:%d \n", ff.messageSource, ff.messageDest, ff.messageType, ff.writeBit, ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode, ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);

}

void FujiHeatPump::sendPendingFrame() {
    // Check if we have updates to send
    if(!updateFields) {
        // No updates pending, but check if we have a pre-encoded frame
        if(pendingFrame && (millis() - lastFrameReceived) > 60) {
            _serial->write(writeBuf, 8);
            _serial->flush();
            pendingFrame = false;
            lastFrameSent = millis();
            _serial->readBytes(writeBuf, 8); // read back our own frame so we dont process it again
        }
        return;
    }
    
    // Check retry limit to prevent DOSing the controller
    if(updateRetryCount >= kMaxUpdateRetries) {
        // Max retries exceeded, give up to prevent overwhelming the controller
        if(debugPrint) {
            Serial.printf("WARNING: Max retries (%d) exceeded for updates, clearing pending updates\n", kMaxUpdateRetries);
        }
        updateFields = 0;
        updateRetryCount = 0;
        return;
    }
    
    // We have updates to send - construct a frame even if pendingFrame is false
    // This handles the case where we received a zone message but have regular updates pending
    // Increase timing window to 60ms for more reliability
    unsigned long timeSinceLastFrame = (lastFrameSent > 0) ? (millis() - lastFrameSent) : (millis() - lastFrameReceived);
    if(timeSinceLastFrame < 60 && lastFrameReceived != 0) {
        // Not enough time has passed, wait for next cycle
        // DON'T clear updateFields - keep them pending
        return;
    }
    
    FujiFrame ff;
    
    // Start with current state
    memcpy(&ff, &currentState, sizeof(FujiFrame));
    
    // Set up frame structure based on controller state
    ff.messageSource = controllerAddress;
    
    if(seenSecondaryController) {
        ff.messageDest = static_cast<byte>(FujiAddress::SECONDARY);
        ff.loginBit = true;
        ff.controllerPresent = 0;
    } else {
        ff.messageDest = static_cast<byte>(FujiAddress::UNIT);
        ff.loginBit = false;
        ff.controllerPresent = 1;
    }
    
    ff.updateMagic = 0;
    ff.unknownBit = true;
    ff.writeBit = 1;  // We have updates, so set write bit
    ff.messageType = static_cast<byte>(FujiMessageType::STATUS);
    
    // Apply pending updates (ff already has currentState copied, so we just override updated fields)
    if(updateFields & kOnOffUpdateMask) {
        ff.onOff = updateState.onOff;
    }
    if(updateFields & kTempUpdateMask) {
        ff.temperature = updateState.temperature;
    }
    if(updateFields & kModeUpdateMask) {
        ff.acMode = updateState.acMode;
    }
    if(updateFields & kFanModeUpdateMask) {
        ff.fanMode = updateState.fanMode;
    }
    if(updateFields & kSwingModeUpdateMask) {
        ff.swingMode = updateState.swingMode;
    }
    if(updateFields & kSwingStepUpdateMask) {
        ff.swingStep = updateState.swingStep;
    }
    if(updateFields & kEconomyModeUpdateMask) {
        ff.economyMode = updateState.economyMode;
    }
    
    // Encode and send
    encodeFrame(ff);
    
    if(debugPrint) {
        Serial.printf("--> (constructed) ");
        printFrame(writeBuf, ff);
    }
    
    for(int i=0; i<8; i++) {
        writeBuf[i] ^= 0xFF;
    }
    
    _serial->write(writeBuf, 8);
    _serial->flush();
    pendingFrame = false;
    // DON'T clear updateFields here - wait for confirmation from unit
    // Store what we sent so we can verify it was applied
    updateRetryCount++;  // Increment retry counter
    lastFrameSent = millis();
    
    if(debugPrint) {
        Serial.printf("Sent update frame (retry %d/%d)\n", updateRetryCount, kMaxUpdateRetries);
    }
    
    _serial->readBytes(writeBuf, 8); // read back our own frame so we dont process it again
    
    // DON'T update currentState here - wait for confirmation from the unit
    // This ensures currentState reflects what the unit actually has, not what we sent
}

bool FujiHeatPump::sendPendingZoneFrame() {
    if(pendingZoneFrame) {
        // Ensure enough time has passed since last frame was sent/received
        // This prevents sending zone frames too close to regular frames
        // Increase to 60ms for more reliability
        unsigned long timeSinceLastFrame = (lastFrameSent > 0) ? (millis() - lastFrameSent) : (millis() - lastFrameReceived);
        if(timeSinceLastFrame < 60) {
            // Not enough time has passed, wait for next cycle
            return false;
        }
        
        // Encode the zone frame from zoneUpdateState
        encodeZoneFrame(zoneUpdateState);
        
        if(debugPrint) {
            Serial.printf("--> ZONE (before XOR): %X %X %X %X %X %X %X %X\n",
                zoneWriteBuf[0], zoneWriteBuf[1], zoneWriteBuf[2], zoneWriteBuf[3],
                zoneWriteBuf[4], zoneWriteBuf[5], zoneWriteBuf[6], zoneWriteBuf[7]);
        }
        
        // XOR encode before sending (same as regular frames)
        byte sendBuf[8];
        for(int i=0; i<8; i++) {
            sendBuf[i] = zoneWriteBuf[i] ^ 0xFF;
        }
        
        if(debugPrint) {
            Serial.printf("--> ZONE (after XOR): %X %X %X %X %X %X %X %X\n",
                sendBuf[0], sendBuf[1], sendBuf[2], sendBuf[3],
                sendBuf[4], sendBuf[5], sendBuf[6], sendBuf[7]);
        }
        
        _serial->write(sendBuf, 8);
        _serial->flush();
        pendingZoneFrame = false;
        lastFrameSent = millis();
        
        // Read back our own frame so we don't process it again
        _serial->readBytes(sendBuf, 8);
        return true;
    }
    return false;
}


bool FujiHeatPump::waitForFrame() {
    FujiFrame ff;
    
    if(_serial->available()) {

        memset(readBuf, 0, 8);
        int bytesRead = _serial->readBytes(readBuf,8);

        if(bytesRead < 8) {
            // skip incomplete frame
            return false;
        }
        
        for(int i=0;i<8;i++) {
            readBuf[i] ^= 0xFF;
        }
    
        // Check if this is a zone message before decoding as regular frame
        if(isZoneMessage(readBuf)) {
            ZoneFrame zf = decodeZoneFrame();
            
            if(debugPrint) {
                Serial.printf("<-- ZONE: %X %X %X %X %X %X %X %X  ", 
                    readBuf[0], readBuf[1], readBuf[2], readBuf[3], 
                    readBuf[4], readBuf[5], readBuf[6], readBuf[7]);
                Serial.printf("zoneGroup: %d, zones[0]: %d, zones[1]: %d\n",
                    static_cast<byte>(zf.zoneGroup), zf.zones[0], zf.zones[1]);
            }
            
            // Update current zone state
            memcpy(&currentZoneState, &zf, sizeof(ZoneFrame));
            initialZoneStateReceived = true;
            
            // Update lastFrameReceived so timing checks work for regular frames
            // Zone messages are still valid frames for timing purposes
            lastFrameReceived = millis();
            
            // Don't send a reply for zone messages - they're informational
            return true;
        }
    
        ff = decodeFrame();

        if(debugPrint) {
            Serial.printf("<-- ");
            printFrame(readBuf, ff);
        }
        
        if(ff.messageDest == controllerAddress) {
            lastFrameReceived = millis();
            
            if(ff.messageType == static_cast<byte>(FujiMessageType::STATUS)){

                if(ff.controllerPresent == 1) {
                    // we have logged into the indoor unit
                    // this is what most frames are
                    ff.messageSource     = controllerAddress;
                    
                    if(seenSecondaryController) {
                        ff.messageDest       = static_cast<byte>(FujiAddress::SECONDARY);
                        ff.loginBit          = true;
                        ff.controllerPresent = 0;
                    } else {
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        ff.loginBit          = false;
                        ff.controllerPresent = 1;
                    }
                    
                    ff.updateMagic       = 0;
                    ff.unknownBit        = true;
                    ff.writeBit          = 0;
                    ff.messageType       = static_cast<byte>(FujiMessageType::STATUS);
                    
                } else {
                    if(controllerIsPrimary) {
                        // if this is the first message we have received, announce ourselves to the indoor unit
                        ff.messageSource     = controllerAddress;
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        ff.loginBit          = false;
                        ff.controllerPresent = 0;
                        ff.updateMagic       = 0;
                        ff.unknownBit        = true;
                        ff.writeBit          = 0;
                        ff.messageType       = static_cast<byte>(FujiMessageType::LOGIN);
                        
                        ff.onOff             = 0;
                        ff.temperature       = 0;
                        ff.acMode            = 0;
                        ff.fanMode           = 0;
                        ff.swingMode         = 0;
                        ff.swingStep         = 0;
                        ff.acError           = 0;
                    } else {
                        // secondary controller never seems to get any other message types, only status with controllerPresent == 0
                        // the secondary controller seems to send the same flags no matter which message type
                        
                        ff.messageSource     = controllerAddress;
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        ff.loginBit          = false;
                        ff.controllerPresent = 1;
                        ff.updateMagic       = 2;
                        ff.unknownBit        = true;
                        ff.writeBit          = 0;
                    }
                    
                }
                
                // Check if the unit has applied our updates by comparing received state with what we sent
                // Only clear updateFields if the unit has confirmed the changes
                bool updatesConfirmed = true;
                if(updateFields & kOnOffUpdateMask) {
                    if(ff.onOff != updateState.onOff) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kTempUpdateMask) {
                    if(ff.temperature != updateState.temperature) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kModeUpdateMask) {
                    if(ff.acMode != updateState.acMode) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kFanModeUpdateMask) {
                    if(ff.fanMode != updateState.fanMode) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kSwingModeUpdateMask) {
                    if(ff.swingMode != updateState.swingMode) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kSwingStepUpdateMask) {
                    if(ff.swingStep != updateState.swingStep) {
                        updatesConfirmed = false;
                    }
                }
                if(updateFields & kEconomyModeUpdateMask) {
                    if(ff.economyMode != updateState.economyMode) {
                        updatesConfirmed = false;
                    }
                }
                
                // If updates are confirmed, clear the flags and reset retry counter
                // Otherwise, keep them set so sendPendingFrame() will retry
                if(updatesConfirmed) {
                    if(debugPrint && updateRetryCount > 0) {
                        Serial.printf("Updates confirmed after %d retries\n", updateRetryCount);
                    }
                    updateFields = 0;
                    updateRetryCount = 0;
                }
                
                // if we have any remaining updates, set the flags in the reply
                if(updateFields) {
                    ff.writeBit = 1;
                }
                
                if(updateFields & kOnOffUpdateMask) {
                    ff.onOff = updateState.onOff;
                }
                
                if(updateFields & kTempUpdateMask) {
                    ff.temperature = updateState.temperature;
                }
                
                if(updateFields & kModeUpdateMask) {
                    ff.acMode = updateState.acMode;
                }
                
                if(updateFields & kFanModeUpdateMask) {
                    ff.fanMode = updateState.fanMode;
                }
                
                if(updateFields & kSwingModeUpdateMask) {
                    ff.swingMode = updateState.swingMode;
                }
                
                if(updateFields & kSwingStepUpdateMask) {
                    ff.swingStep = updateState.swingStep;
                }

                if(updateFields & kEconomyModeUpdateMask) {
                    ff.economyMode = updateState.economyMode;
                }
                
                // Update currentState with what the unit actually has
                memcpy(&currentState, &ff, sizeof(FujiFrame));

            }
            else if(ff.messageType == static_cast<byte>(FujiMessageType::LOGIN)){
                // received a login frame OK frame
                // the primary will send packet to a secondary controller to see if it exists
                ff.messageSource     = controllerAddress;
                ff.messageDest       = static_cast<byte>(FujiAddress::SECONDARY);
                ff.loginBit          = true;
                ff.controllerPresent = 1;
                ff.updateMagic       = 0;
                ff.unknownBit        = true;
                ff.writeBit          = 0;
                
                ff.onOff             = currentState.onOff;
                ff.temperature       = currentState.temperature;
                ff.acMode            = currentState.acMode;
                ff.fanMode           = currentState.fanMode;
                ff.swingMode         = currentState.swingMode;
                ff.swingStep         = currentState.swingStep;
                ff.acError           = currentState.acError;
            } else if(ff.messageType == static_cast<byte>(FujiMessageType::ERROR)) {
                // Check if this is actually a zone message being misidentified as ERROR
                // Zone messages sometimes appear as ERROR type but have the zone signature
                if(isZoneMessage(readBuf)) {
                    ZoneFrame zf = decodeZoneFrame();
                    
                    if(debugPrint) {
                        Serial.printf("<-- ZONE (misidentified as ERROR): %X %X %X %X %X %X %X %X  ", 
                            readBuf[0], readBuf[1], readBuf[2], readBuf[3], 
                            readBuf[4], readBuf[5], readBuf[6], readBuf[7]);
                        Serial.printf("zoneGroup: %d, zones[0]: %d, zones[1]: %d\n",
                            static_cast<byte>(zf.zoneGroup), zf.zones[0], zf.zones[1]);
                    }
                    
                    // Update current zone state
                    memcpy(&currentZoneState, &zf, sizeof(ZoneFrame));
                    initialZoneStateReceived = true;
                    
                    // Don't send a reply for zone messages
                    return true;
                } else {
                    Serial.printf("AC ERROR RECV: ");
                    printFrame(readBuf, ff);
                    // handle errors here
                    return false;
                }
            }
            
            encodeFrame(ff);

            if(debugPrint) {
                Serial.printf("--> ");
                printFrame(writeBuf, ff);
            }

            for(int i=0;i<8;i++) {
                writeBuf[i] ^= 0xFF;
            }
                    
            pendingFrame = true;
                        

        } else if (ff.messageDest == static_cast<byte>(FujiAddress::SECONDARY)) {
            seenSecondaryController = true;
            currentState.controllerTemp = ff.controllerTemp; // we dont have a temp sensor, use the temp reading from the secondary controller
        }
        
        return true;
    }
    
    return false;
}

bool FujiHeatPump::isBound() {
    if(millis() - lastFrameReceived < 1000) {
        return true;
    }
    return false;
}

bool FujiHeatPump::updatePending() {
    if(updateFields) {
        return true;
    }
    return false;
}

void FujiHeatPump::setOnOff(bool o){
    updateFields |= kOnOffUpdateMask;
    updateState.onOff = o ? 1 : 0;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setTemp(byte t){
    updateFields |= kTempUpdateMask;
    updateState.temperature = t;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setMode(byte m){
    updateFields |= kModeUpdateMask;
    updateState.acMode = m;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setFanMode(byte fm){
    updateFields |= kFanModeUpdateMask;
    updateState.fanMode = fm;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setEconomyMode(byte em){
    updateFields |= kEconomyModeUpdateMask;
    updateState.economyMode = em;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setSwingMode(byte sm){
    updateFields |= kSwingModeUpdateMask;
    updateState.swingMode = sm;
    updateRetryCount = 0;  // Reset retry counter for new update
}
void FujiHeatPump::setSwingStep(byte ss){
    updateFields |= kSwingStepUpdateMask;
    updateState.swingStep = ss;
    updateRetryCount = 0;  // Reset retry counter for new update
}

bool FujiHeatPump::getOnOff(){
    return currentState.onOff == 1 ? true : false;
}
byte FujiHeatPump::getTemp(){
    return currentState.temperature;
}
byte FujiHeatPump::getMode(){
    return currentState.acMode;
}
byte FujiHeatPump::getFanMode(){
    return currentState.fanMode;
}
byte FujiHeatPump::getEconomyMode(){
    return currentState.economyMode;
}
byte FujiHeatPump::getSwingMode(){
    return currentState.swingMode;
}
byte FujiHeatPump::getSwingStep(){
    return currentState.swingStep;
}
byte FujiHeatPump::getControllerTemp(){
    return currentState.controllerTemp;
}

FujiFrame *FujiHeatPump::getCurrentState(){
    return &currentState;
}

FujiFrame *FujiHeatPump::getUpdateState(){
    return &updateState;
}

byte FujiHeatPump::getUpdateFields(){
    return updateFields;
}

void FujiHeatPump::setZoneGroup(FujiZoneGroup zoneGroup) {
    // Initialize zoneUpdateState if needed
    zoneUpdateState.zoneGroup = zoneGroup;
    zoneUpdateState.zoneWriteBit = true;
    zoneUpdateState.messageSource = controllerAddress;
    // Zone commands use a different pattern than regular commands
    // Secondary controllers send zone commands to PRIMARY (not UNIT like regular commands)
    // This matches the pattern seen in working zone changes
    if(controllerIsPrimary) {
        zoneUpdateState.messageDest = static_cast<byte>(FujiAddress::UNIT);
        zoneUpdateState.loginBit = false;
    } else {
        // Secondary: send zone commands to PRIMARY with login bit
        // This is different from regular commands which go to UNIT
        zoneUpdateState.messageDest = static_cast<byte>(FujiAddress::PRIMARY);
        zoneUpdateState.loginBit = true;
    }
    zoneUpdateState.unknownBit = true;  // Zone messages seem to have this set
    
    // When setting zone group, clear individual zone states
    // The zone state will be set based on the group in encodeZoneFrame
    for(int i = 0; i < kZoneCount; i++) {
        zoneUpdateState.zones[i] = false;
    }
    
    pendingZoneFrame = true;
    
    if(debugPrint) {
        Serial.printf("Zone group set to: %d, pendingZoneFrame: %d\n", 
                     static_cast<byte>(zoneGroup), pendingZoneFrame);
    }
}

void FujiHeatPump::setZoneOnOff(int zone, bool on) {
    if(zone >= 0 && zone < kZoneCount) {
        // Initialize zoneUpdateState if needed
        zoneUpdateState.zones[zone] = on;
        zoneUpdateState.zoneGroup = FujiZoneGroup::NONE; // Individual zone control
        zoneUpdateState.zoneWriteBit = true;
        zoneUpdateState.messageSource = controllerAddress;
        // Zone commands use a different pattern than regular commands
        // Secondary controllers send zone commands to PRIMARY (not UNIT like regular commands)
        // This matches the pattern seen in working zone changes
        if(controllerIsPrimary) {
            zoneUpdateState.messageDest = static_cast<byte>(FujiAddress::UNIT);
            zoneUpdateState.loginBit = false;
        } else {
            // Secondary: send zone commands to PRIMARY with login bit
            // This is different from regular commands which go to UNIT
            zoneUpdateState.messageDest = static_cast<byte>(FujiAddress::PRIMARY);
            zoneUpdateState.loginBit = true;
        }
        zoneUpdateState.unknownBit = true;  // Zone messages seem to have this set
        
        pendingZoneFrame = true;
        
        if(debugPrint) {
            Serial.printf("Zone %d set to %s, pendingZoneFrame: %d\n", 
                         zone, on ? "ON" : "OFF", pendingZoneFrame);
        }
    }
}

FujiZoneGroup FujiHeatPump::getZoneGroup() {
    return currentZoneState.zoneGroup;
}

bool FujiHeatPump::getZoneOnOff(int zone) {
    if(zone >= 0 && zone < kZoneCount) {
        return currentZoneState.zones[zone];
    }
    return false;
}

ZoneFrame *FujiHeatPump::getCurrentZoneState() {
    return &currentZoneState;
}
 

