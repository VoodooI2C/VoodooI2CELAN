//
//  VoodooI2CELANTouchpadDriver.cpp
//  VoodooI2CELAN
//
//  Created by Kishor Prins on 2017-10-13.
//  Copyright © 2017 Kishor Prins. All rights reserved.
//

#include "../../../VoodooI2C/VoodooI2C/VoodooI2CController/VoodooI2CControllerDriver.hpp"

#include "VoodooI2CELANTouchpadDriver.hpp"
#include "VoodooI2CElanConstants.h"

#define super IOService
OSDefineMetaClassAndStructors(VoodooI2CELANTouchpadDriver, IOService);

bool VoodooI2CELANTouchpadDriver::check_ASUS_firmware(UInt8 productId, UInt8 ic_type) {
    if (ic_type == 0x0E) {
        switch (productId) {
            case 0x05 ... 0x07:
            case 0x09:
            case 0x13:
                return true;
        }
    } else if (ic_type == 0x08 && productId == 0x26) {
        return true;
    }
    return false;
}

void VoodooI2CELANTouchpadDriver::free() {
    IOLog("%s::%s VoodooI2CELAN resources have been deallocated\n", getName(), elan_name);
    super::free();
}

void VoodooI2CELANTouchpadDriver::handle_input_threaded() {
    if (!ready_for_input) {
        return;
    }
    command_gate->attemptAction(OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooI2CELANTouchpadDriver::parse_ELAN_report));
    read_in_progress = false;
}

bool VoodooI2CELANTouchpadDriver::init(OSDictionary *properties) {
    max_hw_resx = 0;
    max_hw_resy = 0;
    max_report_x = 0;
    max_report_y = 0;
    transducers = NULL;
    if (!super::init(properties)) {
        return false;
    }
    transducers = OSArray::withCapacity(ETP_MAX_FINGERS);
    if (!transducers) {
        return false;
    }
    DigitiserTransducuerType type = kDigitiserTransducerFinger;
    for (int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(type, NULL);
        transducers->setObject(transducer);
    }
    awake = true;
    ready_for_input = false;
    read_in_progress = false;
    strncpy(elan_name, ELAN_NAME, strlen(ELAN_NAME));
    return true;
}

bool VoodooI2CELANTouchpadDriver::init_device() {
    if (!reset_device()) {
        return false;
    }
    IOReturn retVal;
    UInt8 val[3];
    retVal = read_ELAN_cmd(ETP_I2C_FW_VERSION_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get version cmd\n", getName(), device_name);
        return false;
    }
    UInt8 version = val[0];
    retVal = read_ELAN_cmd(ETP_I2C_FW_CHECKSUM_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get checksum cmd\n", getName(), device_name);
        return false;
    }
    UInt16 csum = *reinterpret_cast<UInt16 *>(val);
    retVal = read_ELAN_cmd(ETP_I2C_IAP_VERSION_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get IAP version cmd\n", getName(), device_name);
        return false;
    }
    UInt8 iapversion = val[0];
    retVal = read_ELAN_cmd(ETP_I2C_PRESSURE_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get pressure cmd\n", getName(), device_name);
        return false;
    }
    retVal = read_ELAN_cmd(ETP_I2C_MAX_X_AXIS_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get max X axis cmd\n", getName(), device_name);
        return false;
    }
    max_report_x = (*(reinterpret_cast<UInt16 *>(val))) & 0x0fff;
    retVal = read_ELAN_cmd(ETP_I2C_MAX_Y_AXIS_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get max Y axis cmd\n", getName(), device_name);
        return false;
    }
    max_report_y = (*(reinterpret_cast<UInt16 *>(val))) & 0x0fff;
    retVal = read_ELAN_cmd(ETP_I2C_XY_TRACENUM_CMD, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get XY tracenum cmd\n", getName(), device_name);
        return false;
    }
    retVal = read_ELAN_cmd(ETP_I2C_RESOLUTION_CMD, val);
    if (retVal != kIOReturnSuccess) {
        
        return false;
    }
    max_hw_resx = val[0];
    max_hw_resy = val[1];
    IOLog("%s::%s ProdID: %d Vers: %d Csum: %d IAPVers: %d Max X: %d Max Y: %d\n", getName(), device_name, product_id, version, csum, iapversion, max_report_x, max_report_y);
    if (mt_interface != NULL) {
        mt_interface->logical_max_x = max_report_x;
        mt_interface->logical_max_y = max_report_y;
        mt_interface->physical_max_x = max_hw_resx;
        mt_interface->physical_max_x = max_hw_resy;
    }
    return true;
}

void VoodooI2CELANTouchpadDriver::interrupt_occurred(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    if (read_in_progress)
        return;
    if (!awake)
        return;
    read_in_progress = true;
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CELANTouchpadDriver::handle_input_threaded), this, &new_thread);
    if (ret != KERN_SUCCESS) {
        read_in_progress = false;
        IOLog("%s::%s Thread error while attemping to get input report\n", getName(), device_name);
    } else {
        thread_deallocate(new_thread);
    }
}

IOReturn VoodooI2CELANTouchpadDriver::parse_ELAN_report() {
    if (!api) {
        IOLog("%s::%s API is null\n", getName(), device_name);
        return kIOReturnError;
    }
    UInt8 reportData[ETP_MAX_REPORT_LEN];
    for (int i = 0; i < ETP_MAX_REPORT_LEN; i++) {
        reportData[i] = 0;
    }
    IOReturn retVal = read_raw_data(0, sizeof(reportData), reportData);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to handle input\n", getName(), device_name);
        return retVal;
    }
    if (transducers == NULL) {
        return kIOReturnBadArgument;
    }
    if (reportData[ETP_REPORT_ID_OFFSET] != ETP_REPORT_ID) {
        IOLog("%s::%s Invalid report (%d)\n", getName(), device_name, reportData[ETP_REPORT_ID_OFFSET]);
        return kIOReturnError;
    }
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    UInt8* finger_data = &reportData[ETP_FINGER_DATA_OFFSET];
    UInt8 tp_info = reportData[ETP_TOUCH_INFO_OFFSET];
    int numFingers = 0;
    for (int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(i));
        if (!transducer) {
            continue;
        }
        bool contactValid = tp_info & (1U << (3 + i));
        transducer->is_valid = contactValid;
        if (contactValid) {
            unsigned int posX = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
            unsigned int posY = ((finger_data[0] & 0x0f) << 8) | finger_data[2];
            posY *= -1;
            transducer->coordinates.x.update(posX, timestamp);
            transducer->coordinates.y.update(posY, timestamp);
            transducer->physical_button.update(tp_info & 0x01, timestamp);
            transducer->tip_switch.update(1, timestamp);
            numFingers += 1;
            finger_data += ETP_FINGER_DATA_LEN;
        }
    }
    // create new VoodooI2CMultitouchEvent
    VoodooI2CMultitouchEvent event;
    event.contact_count = numFingers;
    event.transducers = transducers;
    // send the event into the multitouch interface
    if (mt_interface) {
        mt_interface->handleInterruptReport(event, timestamp);
    }
    return kIOReturnSuccess;
}

VoodooI2CELANTouchpadDriver* VoodooI2CELANTouchpadDriver::probe(IOService* provider, SInt32* score) {
    IOLog("%s::%s Touchpad probe\n", getName(), elan_name);
    if (!super::probe(provider, score)) {
        return NULL;
    }
    acpi_device = OSDynamicCast(IOACPIPlatformDevice, provider->getProperty("acpi-device"));
    if (!acpi_device) {
        IOLog("%s::%s Could not get ACPI device\n", getName(), elan_name);
        return NULL;
    }
    // check for ELAN devices (DSDT must have ELAN* defined in the name property)
    OSData* nameData = OSDynamicCast(OSData, provider->getProperty("name"));
    if (nameData == NULL) {
        IOLog("%s::%s Unable to get 'name' property\n", getName(), elan_name);
        return NULL;
    }
    const char* deviceName = reinterpret_cast<char*>(const_cast<void*>(nameData->getBytesNoCopy()));
    if (deviceName[0] != 'E' && deviceName[1] != 'L'
        && deviceName[2] != 'A'&& deviceName[3] != 'N') {
        IOLog("%s::%s ELAN device not found, instead found %s\n", getName(), elan_name, deviceName);
        return NULL;
    }
    strncpy(device_name, deviceName, 10);
    IOLog("%s::%s ELAN device not found, instead found %s\n", getName(), elan_name, device_name);
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    if (!api) {
        IOLog("%s::%s Could not get VoodooI2C API instance\n", getName(), device_name);
        return NULL;
    }
    return this;
}

bool VoodooI2CELANTouchpadDriver::publish_multitouch_interface() {
    mt_interface = new VoodooI2CMultitouchInterface();
    if (mt_interface == NULL) {
        IOLog("%s::%s No memory to allocate VoodooI2CMultitouchInterface instance\n", getName(), device_name);
        goto multitouchExit;
    }
    if (!mt_interface->init(NULL)) {
        IOLog("%s::%s Failed to init multitouch interface\n", getName(), device_name);
        goto multitouchExit;
    }
    if (!mt_interface->attach(this)) {
        IOLog("%s::%s Failed to attach multitouch interface\n", getName(), device_name);
        goto multitouchExit;
    }
    if (!mt_interface->start(this)) {
        IOLog("%s::%s Failed to start multitouch interface\n", getName(), device_name);
        goto multitouchExit;
    }
    // Assume we are a touchpad
    mt_interface->setProperty(kIOHIDDisplayIntegratedKey, true);
    // 0x04f3 is Elan's Vendor Id
    mt_interface->setProperty(kIOHIDVendorIDKey, 0x04f3, 32);
    mt_interface->setProperty(kIOHIDProductIDKey, product_id, 32);
    mt_interface->registerService();
    return true;
multitouchExit:
    unpublish_multitouch_interface();
    return false;
}

IOReturn VoodooI2CELANTouchpadDriver::read_ELAN_cmd(UInt16 reg, UInt8* val) {
    return read_raw_16bit_data(reg, ETP_I2C_INF_LENGTH, val);
}

IOReturn VoodooI2CELANTouchpadDriver::read_raw_data(UInt8 reg, size_t len, UInt8* values) {
    IOReturn retVal = kIOReturnSuccess;
    retVal = api->writeReadI2C(&reg, 1, values, len);
    return retVal;
}

IOReturn VoodooI2CELANTouchpadDriver::read_raw_16bit_data(UInt16 reg, size_t len, UInt8* values) {
    IOReturn retVal = kIOReturnSuccess;
    UInt16 buffer[] {
        reg
    };
    retVal = api->writeReadI2C(reinterpret_cast<UInt8*>(&buffer), sizeof(buffer), values, len);
    return retVal;
}

bool VoodooI2CELANTouchpadDriver::reset_device() {
    IOReturn retVal = kIOReturnSuccess;
    retVal = write_ELAN_cmd(ETP_I2C_STAND_CMD, ETP_I2C_RESET);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to write RESET cmd\n", getName(), device_name);
        return false;
    }
    IOSleep(100);
    UInt8 val[256];
    UInt8 val2[3];
    retVal = read_raw_data(0x00, ETP_I2C_INF_LENGTH, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get reset acknowledgement\n", getName(), device_name);
        return false;;
    }
    retVal = read_raw_16bit_data(ETP_I2C_DESC_CMD, ETP_I2C_DESC_LENGTH, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s  Failed to get desc cmd\n", getName(), device_name);
        return false;
    }
    retVal = read_raw_16bit_data(ETP_I2C_REPORT_DESC_CMD, ETP_I2C_REPORT_DESC_LENGTH, val);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get report cmd\n", getName(), device_name);
        return false;
    }
    // get the product ID
    retVal = read_ELAN_cmd(ETP_I2C_UNIQUEID_CMD, val2);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get product ID cmd\n", getName(), device_name);
        return false;
    }
    UInt8 productId = val2[0];
    retVal = read_ELAN_cmd(ETP_I2C_SM_VERSION_CMD, val2);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get IC type cmd\n", getName(), device_name);
        return false;
    }
    UInt8 ictype = val2[1];
    if (check_ASUS_firmware(productId, ictype)) {
        IOLog("%s::%s ASUS trackpad detected, applying workaround\n", getName(), device_name);
        retVal = write_ELAN_cmd(ETP_I2C_STAND_CMD, ETP_I2C_WAKE_UP);
        if (retVal != kIOReturnSuccess) {
            IOLog("%s::%s Failed to send wake up cmd (workaround)\n", getName(), device_name);
            return false;
        }
        IOSleep(200);
        retVal =  write_ELAN_cmd(ETP_I2C_SET_CMD, ETP_ENABLE_ABS);
        if (retVal != kIOReturnSuccess) {
            IOLog("%s::%s Failed to send enable cmd (workaround)\n", getName(), device_name);
            return false;
        }
    } else {
        retVal = write_ELAN_cmd(ETP_I2C_SET_CMD, ETP_ENABLE_ABS);
        if (retVal != kIOReturnSuccess) {
            IOLog("%s::%s Failed to send enable cmd\n", getName(), device_name);
            return false;
        }
        retVal = write_ELAN_cmd(ETP_I2C_STAND_CMD, ETP_I2C_WAKE_UP);
        if (retVal != kIOReturnSuccess) {
            IOLog("%s::%s Failed to send wake up cmd\n", getName(), device_name);
            return false;
        }
    }
    return true;
}

void VoodooI2CELANTouchpadDriver::release_resources() {
    if (command_gate) {
        workLoop->removeEventSource(command_gate);
        command_gate->release();
        command_gate = NULL;
    }
    if (interrupt_source != NULL) {
        interrupt_source->disable();
        workLoop->removeEventSource(interrupt_source);
        interrupt_source->release();
        interrupt_source = NULL;
    }
    if (workLoop) {
        workLoop->release();
        workLoop = NULL;
    }
    if (acpi_device) {
        acpi_device->release();
        acpi_device = NULL;
    }
    if (api) {
        if (api->isOpen(this)) {
            api->close(this);
        }
        api->release();
        api = NULL;
    }
    if (transducers) {
        for (int i = 0; i < transducers->getCount(); i++) {
            OSObject* object = transducers->getObject(i);
            if (object) {
                object->release();
            }
        }
        OSSafeReleaseNULL(transducers);
    }
}

IOReturn VoodooI2CELANTouchpadDriver::setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOReturnInvalid;
    if (longpowerStateOrdinal == 0) {
        if (awake) {
            awake = false;
            for (;;) {
                if (!read_in_progress) {
                    break;
                }
                IOSleep(10);
            }
            IOLog("%s::%s Going to sleep\n", getName(), device_name);
        }
    } else {
        if (!awake) {
            reset_device();
            awake = true;
            IOLog("%s::%s Woke up and reset device\n", getName(), device_name);
        }
    }
    return kIOPMAckImplied;
}

bool VoodooI2CELANTouchpadDriver::start(IOService* provider) {
    if (!super::start(provider)) {
        return false;
    }
    workLoop = this->getWorkLoop();
    if (!workLoop) {
        IOLog("%s::%s Could not get a IOWorkLoop instance\n", getName(), elan_name);
        return false;
    }
    workLoop->retain();
    command_gate = IOCommandGate::commandGate(this);
    if (!command_gate || (workLoop->addEventSource(command_gate) != kIOReturnSuccess)) {
        IOLog("%s::%s Could not open command gate\n", getName(), elan_name);
        goto start_exit;
    }
    acpi_device->retain();
    api->retain();
    if (!api->open(this)) {
        IOLog("%s::%s Could not open API\n", getName(), elan_name);
        goto start_exit;
    }
    // set interrupts AFTER device is initialised
    interrupt_source = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CELANTouchpadDriver::interrupt_occurred), api, 0);
    if (!interrupt_source) {
        IOLog("%s::%s Could not get interrupt event source\n", getName(), elan_name);
        goto start_exit;
    }
    publish_multitouch_interface();
    if (!init_device()) {
        IOLog("%s::%s Failed to init device\n", getName(), elan_name);
        return NULL;
    }
    workLoop->addEventSource(interrupt_source);
    interrupt_source->enable();
    PMinit();
    api->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    IOSleep(100);
    ready_for_input = true;
    setProperty("VoodooI2CServices Supported", OSBoolean::withBoolean(true));
    IOLog("%s::%s VoodooI2CELAN has started\n", getName(), elan_name);
    return true;
start_exit:
    release_resources();
    return false;
}

void VoodooI2CELANTouchpadDriver::stop(IOService* provider) {
    release_resources();
    unpublish_multitouch_interface();
    PMstop();
    IOLog("%s::%s VoodooI2CELAN has stopped\n", getName(), elan_name);
    super::stop(provider);
}

void VoodooI2CELANTouchpadDriver::unpublish_multitouch_interface() {
    if (mt_interface) {
        mt_interface->stop(this);
        mt_interface->release();
        mt_interface = NULL;
    }
}

// Linux equivalent of elan_i2c_write_cmd function
IOReturn VoodooI2CELANTouchpadDriver::write_ELAN_cmd(UInt16 reg, UInt16 cmd) {
    UInt16 buffer[] {
        reg,
        cmd
    };
    IOReturn retVal = kIOReturnSuccess;
    retVal = api->writeI2C(reinterpret_cast<UInt8*>(&buffer), sizeof(buffer));
    return retVal;
}
