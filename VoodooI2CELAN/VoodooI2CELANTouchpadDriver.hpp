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

#define ELAN_NAME "elan"

/* Main class that handles all communication between macOS, VoodooI2C, and a I2C based ELAN touchpad */

class VoodooI2CELANTouchpadDriver : public IOService {
    OSDeclareDefaultStructors(VoodooI2CELANTouchpadDriver);
    
    VoodooI2CDeviceNub* api;
    IOACPIPlatformDevice* acpi_device;

 public:
    /* Initialises the VoodooI2CELANTouchpadDriver object/instance (intended as IOKit driver ctor)
     *
     * @return true if properly initialised
     */
    bool init(OSDictionary* properties) override;
    /* Frees any allocated resources, called implicitly by the kernel
     * as the last stage of the driver being unloaded
     *
     */
    void free() override;
    /* Checks if an ELAN device exists on the current system
     *
     * @return returns an instance of the current VoodooI2CELANTouchpadDriver if there is a matched ELAN device, NULL otherwise
     */
    VoodooI2CELANTouchpadDriver* probe(IOService* provider, SInt32* score) override;
    /* Starts the driver and initialises the ELAN device
     *
     * @return returns true if the driver has started
     */
    bool start(IOService* provider) override;
    /* Stops the driver and frees any allocated resource
     *
     */
    void stop(IOService* device) override;
    
protected:
    IOReturn setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) override;

private:
    bool awake;
    bool read_in_progress;
    bool ready_for_input;
    
    char device_name[10];
    char elan_name[5];
    
    int max_hw_resx;
    int max_hw_resy;
    int max_report_x;
    int max_report_y;
    
    int product_id;
    
    IOCommandGate* command_gate;
    IOInterruptEventSource* interrupt_source;
    VoodooI2CMultitouchInterface *mt_interface;
    OSArray* transducers;
    IOWorkLoop* workLoop;
    
    /* Handles any interrupts that the ELAN device generates
     * @productId product ID of the ELAN device
     * @ic_type IC type (provided by the device)
     *
     * @return returns true if this ELAN device is ASUS manufactured
     */
    bool check_ASUS_firmware(UInt8 productId, UInt8 ic_type);
    /* Handles input in a threaded manner, then
     * calls parse_ELAN_report via the command gate for synchronisation
     *
     */
    void handle_input_threaded();
    /* Sends the appropriate ELAN protocol packets to
     * initialise the device into multitouch mode
     *
     * @return true if the device was initialised properly
     */
    bool init_device();
    /* Handles any interrupts that the ELAN device generates
     * by spawning a thread that is out of the inerrupt context
     *
     */
    void interrupt_occurred(OSObject* owner, IOInterruptEventSource* src, int intCount);
    /* Reads the ELAN report (touch data) in the I2C bus and generates a VoodooI2C multitouch event
     *
     * @return returns a IOReturn status of the reads (usually a representation of I2C bus)
     */
    IOReturn parse_ELAN_report();
    /* Initialises the VoodooI2C multitouch classes
     *
     * @return true if the VoodooI2C multitouch classes were properly initialised
     */
    bool publish_multitouch_interface();
    /* Reads a ELAN command from the I2C bus
     * @reg which register to read the data from
     * @val a buffer which is large enough to hold the ELAN command data
     *
     * @return returns a IOReturn status of the reads (usually a representation of I2C bus)
     */
    IOReturn read_ELAN_cmd(UInt16 reg, UInt8* val);
    /* Reads raw data from the I2C bus
     * @reg which 16bit register to read the data from
     * @len the length of the @val buffer
     * @vaue a buffer which is large enough to hold the data being read
     *
     * @return returns a IOReturn status of the reads (usually a representation of I2C bus)
     */
    IOReturn read_raw_16bit_data(UInt16 reg, size_t len, UInt8* values);
    /* Reads raw data from the I2C bus
     * @reg which 8bit register to read the data from
     * @len the length of the @val buffer
     * @vaue a buffer which is large enough to hold the data being read
     *
     * @return returns a IOReturn status of the reads (usually a representation of I2C bus)
     */
    IOReturn read_raw_data(UInt8 reg, size_t len, UInt8* values);
    /* Releases any allocated resources (called by stop)
     *
     */
    void release_resources();
    /* Releases any allocated resources
     *
     * @return true if the ELAN device was reset succesfully
     */
    bool reset_device();
    /* Enables or disables the ELAN device for sleep
     *
     */
    void set_sleep_status(bool enable);
    /* Releases any allocated VoodooI2C multitouch device
     *
     */
    void unpublish_multitouch_interface();
    /* Writes a ELAN command formatted I2C message
     * @reg which register to write the data to
     * @cmd the command which we want to write
     *
     * @return returns a IOReturn status of the reads (usually a representation of I2C bus)
     */
    IOReturn write_ELAN_cmd(UInt16 reg, UInt16 cmd);
};

#endif /* VOODOOI2C_ELAN_TOUCHPAD_DRIVER_HPP */
