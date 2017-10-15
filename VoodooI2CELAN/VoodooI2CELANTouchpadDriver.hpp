//
//  VoodooI2CELANTouchpadDriver.hpp
//  VoodooI2CELAN
//
//  Created by Kishor Prins on 2017-10-13.
//  Copyright © 2017 Kishor Prins. All rights reserved.
//

#ifndef VOODOOI2C_ELAN_TOUCHPAD_DRIVER_HPP
#define VOODOOI2C_ELAN_TOUCHPAD_DRIVER_HPP

#include <IOKit/IOService.h>

#include "../../../VoodooI2C/VoodooI2C/VoodooI2CDevice/VoodooI2CDeviceNub.hpp"

#include "../../../Multitouch Support/VoodooI2CMultitouchInterface.hpp"
#include "../../../Multitouch Support/MultitouchHelpers.hpp"

#include "../../../Dependencies/helpers.hpp"

class VoodooI2CELANTouchpadDriver : public IOService {
    OSDeclareDefaultStructors(VoodooI2CELANTouchpadDriver);
    VoodooI2CDeviceNub* api;
    IOACPIPlatformDevice* acpiDevice;
    
    IOWorkLoop* workLoop;
    IOCommandGate* commandGate;
    IOInterruptEventSource* interruptSource;
    void interruptOccured(OSObject* owner, IOInterruptEventSource* src, int intCount);
    IOReturn writeELANCMD(uint16_t reg, uint16_t cmd);
    IOReturn readELANCMD(uint8_t reg, uint8_t* val);
    IOReturn readRawData(uint8_t reg, size_t len, uint8_t* values);
    IOReturn readRaw16Data(uint8_t reg, size_t len, uint8_t* values);
    bool initELANDevice();
    bool checkForASUSFirmware(uint8_t productId, uint8_t ic_type);
    void releaseResources();
public:
    virtual bool start(IOService* provider) override;
    virtual void stop(IOService* device) override;
    virtual VoodooI2CELANTouchpadDriver* probe(IOService* provider, SInt32* score) override;
};

#endif /* VOODOOI2C_ELAN_HID_EVENT_DRIVER_HPP */
