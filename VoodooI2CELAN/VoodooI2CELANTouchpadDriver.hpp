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
    
    bool awake;
    bool readyForInput;
    bool readInProgress;
    
    // Hardware dimensions
    int maxHWResolutionX;
    int maxHWResolutionY;
    
    // Dimensions for reported X, Y?
    int maxReportX;
    int maxReportY;
    
    int productId;
    
    VoodooI2CMultitouchInterface *multitouchInterface;
    OSArray* transducers;
    
    IOWorkLoop* workLoop;
    IOCommandGate* commandGate;
    IOInterruptEventSource* interruptSource;
    void interruptOccurred(OSObject* owner, IOInterruptEventSource* src, int intCount);
    IOReturn writeELANCMD(uint16_t reg, uint16_t cmd);
    IOReturn readELANCMD(uint16_t reg, uint8_t* val);
    IOReturn readRawData(uint8_t reg, size_t len, uint8_t* values);
    IOReturn readRaw16Data(uint16_t reg, size_t len, uint8_t* values);
    bool resetELANDevice();
    bool initELANDevice();
    bool checkForASUSFirmware(uint8_t productId, uint8_t ic_type);
    bool publishMultitouchInterface();
    void unpublishMultitouchInterface();
    IOReturn parseELANReport();
    void handleELANInput();
    void setELANSleepStatus(bool enable);
    void releaseResources();
protected:
    virtual IOReturn setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) override;
public:
    virtual bool start(IOService* provider) override;
    virtual void stop(IOService* device) override;
    virtual bool init(OSDictionary* properties) override;
    virtual void free() override;
    virtual VoodooI2CELANTouchpadDriver* probe(IOService* provider, SInt32* score) override;
};

#endif /* VOODOOI2C_ELAN_TOUCHPAD_DRIVER_HPP */
