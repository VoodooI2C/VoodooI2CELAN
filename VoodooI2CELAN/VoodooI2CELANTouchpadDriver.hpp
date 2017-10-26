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
    IOReturn writeELANCMD(UInt16 reg, UInt16 cmd);
    IOReturn readELANCMD(UInt16 reg, UInt8* val);
    IOReturn readRawData(UInt8 reg, size_t len, UInt8* values);
    IOReturn readRaw16Data(UInt16 reg, size_t len, UInt8* values);
    bool resetELANDevice();
    bool initELANDevice();
    bool checkForASUSFirmware(UInt8 productId, UInt8 ic_type);
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
