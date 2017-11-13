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
    IOACPIPlatformDevice* acpi_device;
    bool awake;
    bool ready_for_input;
    bool read_in_progress;
    int max_hw_resx;
    int max_hw_resy;
    int max_report_x;
    int max_report_y;
    int product_id;
    VoodooI2CMultitouchInterface *mt_interface;
    OSArray* transducers;
    IOWorkLoop* workLoop;
    IOCommandGate* command_gate;
    IOInterruptEventSource* interrupt_source;
    void interruptOccurred(OSObject* owner, IOInterruptEventSource* src, int intCount);
    IOReturn write_ELAN_cmd(UInt16 reg, UInt16 cmd);
    IOReturn read_ELAN_cmd(UInt16 reg, UInt8* val);
    IOReturn read_raw_data(UInt8 reg, size_t len, UInt8* values);
    IOReturn read_raw_16bit_data(UInt16 reg, size_t len, UInt8* values);
    bool reset_device();
    bool init_device();
    bool check_ASUS_firmware(UInt8 productId, UInt8 ic_type);
    bool publish_multitouch_interface();
    void unpublish_multitouch_interface();
    IOReturn parse_ELAN_report();
    void handle_input_threaded();
    void set_sleep_status(bool enable);
    void release_resources();

 protected:
        IOReturn setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) override;

 public:
    bool start(IOService* provider) override;
    void stop(IOService* device) override;
    bool init(OSDictionary* properties) override;
    void free() override;
    VoodooI2CELANTouchpadDriver* probe(IOService* provider, SInt32* score) override;
};

#endif /* VOODOOI2C_ELAN_TOUCHPAD_DRIVER_HPP */
