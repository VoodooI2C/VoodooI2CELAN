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

bool VoodooI2CELANTouchpadDriver::init(OSDictionary *properties) {
    if (!super::init(properties))
        return false;

    interrupt_source = NULL;
    interrupt_simulator = NULL;

    // Allocate finger transducers
    transducers = OSArray::withCapacity(ETP_MAX_FINGERS);
    if (!transducers)
        return false;

    DigitiserTransducerType type = kDigitiserTransducerFinger;
    for (int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(type, NULL);
        transducers->setObject(transducer);
        OSSafeReleaseNULL(transducer);
    }

    // Allocate the multitouch interface
    mt_interface = OSTypeAlloc(VoodooI2CMultitouchInterface);
    if (!mt_interface) {
        IOLog("%s::%s No memory to allocate VoodooI2CMultitouchInterface instance\n", getName(), device_name);
        return false;
    }

    awake = true;
    ready_for_input = false;
    strlcpy(elan_name, ELAN_NAME, sizeof(elan_name));
    return true;
}

void VoodooI2CELANTouchpadDriver::free() {
    OSSafeReleaseNULL(transducers);

    OSSafeReleaseNULL(mt_interface);

    IOLog("%s::%s VoodooI2CELAN resources have been deallocated\n", getName(), elan_name);
    super::free();
}

bool VoodooI2CELANTouchpadDriver::init_device() {
    if (!reset_device()) {
        return false;
    }
    IOReturn retVal;
    UInt8 val[3];

    UInt32 max_report_x = 0;
    UInt32 max_report_y = 0;

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
    if ((val[0] >> 4) & 0x1)
        pressure_adjustment = 0;
    else
        pressure_adjustment = ETP_PRESSURE_OFFSET;
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

    UInt32 x_traces = val[0];
    UInt32 y_traces = val[1];

    if (x_traces == 0 || y_traces == 0) {
        IOLog("%s::%s Traces == 0\n", getName(), device_name);
        return false;
    }

    retVal = read_ELAN_cmd(ETP_I2C_RESOLUTION_CMD, val);
    if (retVal != kIOReturnSuccess) {
        return false;
    }
    UInt32 hw_res_x = val[0];
    UInt32 hw_res_y = val[1];

    // Resolution in dots per mm
    hw_res_x = (hw_res_x * 10 + 790) * 10 / 254;
    hw_res_y = (hw_res_y * 10 + 790) * 10 / 254;

    if (hw_res_x == 0 || hw_res_y == 0) {
        IOLog("%s::%s HW resolution == 0\n", getName(), device_name);
        return false;
    }

    UInt32 hw_phys_x = max_report_x * 100 / hw_res_x;
    UInt32 hw_phys_y = max_report_y * 100 / hw_res_y;
    width_per_trace_x = hw_phys_x / x_traces / 100;
    width_per_trace_y = hw_phys_y / y_traces / 100;

    IOLog("%s::%s ProdID: %d Vers: %d Csum: %d IAPVers: %d Max X: %d Max Y: %d\n", getName(), device_name, product_id, version, csum, iapversion, max_report_x, max_report_y);
    if (mt_interface) {
        mt_interface->physical_max_x = hw_phys_x;
        mt_interface->physical_max_y = hw_phys_y;
        mt_interface->logical_max_x = max_report_x;
        mt_interface->logical_max_y = max_report_y;
    }
    return true;
}

void VoodooI2CELANTouchpadDriver::interrupt_occurred(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    if (!ready_for_input || !awake)
        return;

    parse_ELAN_report();
}

IOReturn VoodooI2CELANTouchpadDriver::parse_ELAN_report() {
    if (!api) {
        IOLog("%s::%s API is null\n", getName(), device_name);
        return kIOReturnError;
    }

    UInt8 reportData[ETP_MAX_REPORT_LEN];
    memset(&reportData, 0, sizeof(reportData));

    IOReturn retVal = api->readI2C(reportData, sizeof(reportData));
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to handle input\n", getName(), device_name);
        return retVal;
    }

    if (!transducers)
        return kIOReturnBadArgument;

    UInt8 report_id = reportData[ETP_REPORT_ID_OFFSET];
    if (report_id != ETP_REPORT_ID) {
        // Ignore 0xFF reports
        if (report_id == 0xFF)
            return kIOReturnSuccess;

        IOLog("%s::%s Invalid report (%d)\n", getName(), device_name, report_id);
        return kIOReturnError;
    }

    // Check if input is disabled via ApplePS2Keyboard request
    if (ignoreall)
        return kIOReturnSuccess;

    // Ignore input for specified time after keyboard usage
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    uint64_t timestamp_ns;
    absolutetime_to_nanoseconds(timestamp, &timestamp_ns);

    if (timestamp_ns - keytime < maxaftertyping)
        return kIOReturnSuccess;


    UInt8* finger_data = &reportData[ETP_FINGER_DATA_OFFSET];
    UInt8 tp_info = reportData[ETP_TOUCH_INFO_OFFSET];
    int numFingers = 0;
    for (int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(i));
        if (!transducer) {
            continue;
        }
        transducer->type = kDigitiserTransducerFinger;
        bool contactValid = tp_info & (1U << (3 + i));
        transducer->is_valid = contactValid;
        if (contactValid) {
            unsigned int posX = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
            unsigned int posY = ((finger_data[0] & 0x0f) << 8) | finger_data[2];
            // unsigned int pressure = finger_data[4] + pressure_adjustment;
            // unsigned int mk_x = (finger_data[3] & 0x0f);
            // unsigned int mk_y = (finger_data[3] >> 4);
            // unsigned int area_x = mk_x;
            // unsigned int area_y = mk_y;

            if (mt_interface) {
                transducer->logical_max_x = mt_interface->logical_max_x;
                transducer->logical_max_y = mt_interface->logical_max_y;
                posY = transducer->logical_max_y - posY;
                // area_x = mk_x * (transducer->logical_max_x - ETP_FWIDTH_REDUCE);
                // area_y = mk_y * (transducer->logical_max_y - ETP_FWIDTH_REDUCE);
            }

            // unsigned int major = max(area_x, area_y);
            // unsigned int minor = min(area_x, area_y);

            // if (pressure > ETP_MAX_PRESSURE)
            //     pressure = ETP_MAX_PRESSURE;

            transducer->coordinates.x.update(posX, timestamp);
            transducer->coordinates.y.update(posY, timestamp);
            // transducer->touch_major.update(major, timestamp);
            // transducer->touch_minor.update(minor, timestamp);
            transducer->physical_button.update(tp_info & 0x01, timestamp);
            transducer->tip_switch.update(1, timestamp);
            transducer->id = i;
            transducer->secondary_id = i;
            // transducer->pressure_physical_max = ETP_MAX_PRESSURE;
            // transducer->tip_pressure.update(pressure, timestamp);
            numFingers += 1;
            finger_data += ETP_FINGER_DATA_LEN;
        } else {
            transducer->id = i;
            transducer->secondary_id = i;
            transducer->coordinates.x.update(transducer->coordinates.x.last.value, timestamp);
            transducer->coordinates.y.update(transducer->coordinates.y.last.value, timestamp);
            transducer->physical_button.update(0, timestamp);
            transducer->tip_switch.update(0, timestamp);
            // transducer->pressure_physical_max = ETP_MAX_PRESSURE;
            // transducer->tip_pressure.update(0, timestamp);
        }
    }

    // create new VoodooI2CMultitouchEvent
    VoodooI2CMultitouchEvent event;
    event.contact_count = numFingers;
    event.transducers = transducers;

    // send the event into the multitouch interface
    if (mt_interface)
        mt_interface->handleInterruptReport(event, timestamp);

    return kIOReturnSuccess;
}

VoodooI2CELANTouchpadDriver* VoodooI2CELANTouchpadDriver::probe(IOService* provider, SInt32* score) {
    IOLog("%s::%s Touchpad probe\n", getName(), elan_name);
    if (!super::probe(provider, score)) {
        return NULL;
    }
    
    // check for ELAN devices (DSDT must have ELAN* defined in the name property)
    OSData* name_data = OSDynamicCast(OSData, provider->getProperty("name"));
    if (!name_data) {
        IOLog("%s::%s Unable to get 'name' property\n", getName(), elan_name);
        return NULL;
    }
    const char* acpi_name = reinterpret_cast<char*>(const_cast<void*>(name_data->getBytesNoCopy()));
    if (acpi_name[0] != 'E' || acpi_name[1] != 'L'
        || acpi_name[2] != 'A' || acpi_name[3] != 'N') {
        IOLog("%s::%s ELAN device not found, instead found %s\n", getName(), elan_name, acpi_name);
        return NULL;
    }
    strlcpy(device_name, acpi_name, sizeof(device_name));
    IOLog("%s::%s ELAN device found (%s)\n", getName(), elan_name, device_name);
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    if (!api) {
        IOLog("%s::%s Could not get VoodooI2C API instance\n", getName(), device_name);
        return NULL;
    }
    return this;
}

bool VoodooI2CELANTouchpadDriver::publish_multitouch_interface() {
    if (!mt_interface->init(NULL)) {
        IOLog("%s::%s Failed to init multitouch interface\n", getName(), device_name);
        return false;
    }
    if (!mt_interface->attach(this)) {
        IOLog("%s::%s Failed to attach multitouch interface\n", getName(), device_name);
        return false;
    }
    if (!mt_interface->start(this)) {
        IOLog("%s::%s Failed to start multitouch interface\n", getName(), device_name);
        mt_interface->detach(this);
        return false;
    }

    // Assume we are a touchpad
    mt_interface->setProperty(kIOHIDDisplayIntegratedKey, false);
    // 0x04f3 is Elan's Vendor Id
    mt_interface->setProperty(kIOHIDVendorIDKey, 0x04f3, 32);
    mt_interface->setProperty(kIOHIDProductIDKey, product_id, 32);

    return true;
}

IOReturn VoodooI2CELANTouchpadDriver::read_ELAN_cmd(UInt16 reg, UInt8* val) {
    return read_raw_16bit_data(reg, ETP_I2C_INF_LENGTH, val);
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
    retVal = api->readI2C(val, ETP_I2C_INF_LENGTH);
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
    product_id = val2[0];
    retVal = read_ELAN_cmd(ETP_I2C_SM_VERSION_CMD, val2);
    if (retVal != kIOReturnSuccess) {
        IOLog("%s::%s Failed to get IC type cmd\n", getName(), device_name);
        return false;
    }
    UInt8 ictype = val2[1];
    if (check_ASUS_firmware(product_id, ictype)) {
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
    if (interrupt_source) {
        interrupt_source->disable();
        workLoop->removeEventSource(interrupt_source);
        OSSafeReleaseNULL(interrupt_source);
    }

    if (interrupt_simulator) {
        interrupt_simulator->disable();
        workLoop->removeEventSource(interrupt_simulator);
        OSSafeReleaseNULL(interrupt_simulator);
    }

    OSSafeReleaseNULL(workLoop);

    if (api) {
        if (api->isOpen(this)) {
            api->close(this);
        }
        api = nullptr;
    }
}

IOReturn VoodooI2CELANTouchpadDriver::setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOReturnInvalid;

    if (longpowerStateOrdinal == 0) {
        if (awake) {
            if (interrupt_simulator) {
                interrupt_simulator->disable();
            } else if (interrupt_source) {
                interrupt_source->disable();
            }

            IOLog("%s::%s Going to sleep\n", getName(), device_name);
            awake = false;
        }
    } else {
        if (!awake) {
            reset_device();
            awake = true;

            if (interrupt_simulator) {
                interrupt_simulator->setTimeoutMS(200);
                interrupt_simulator->enable();
            } else if (interrupt_source) {
                interrupt_source->enable();
            }

            IOLog("%s::%s Woke up and reset device\n", getName(), device_name);
        }
    }
    return kIOPMAckImplied;
}

bool VoodooI2CELANTouchpadDriver::start(IOService* provider) {
    if (!super::start(provider))
        return false;

    // Read QuietTimeAfterTyping configuration value (if available)
    OSNumber* quietTimeAfterTyping = OSDynamicCast(OSNumber, getProperty("QuietTimeAfterTyping"));
    if (quietTimeAfterTyping != NULL)
        maxaftertyping = quietTimeAfterTyping->unsigned64BitValue() * 1000000; // Convert to nanoseconds

    workLoop = this->getWorkLoop();
    if (!workLoop) {
        IOLog("%s::%s Could not get a IOWorkLoop instance\n", getName(), elan_name);
        return false;
    }
    workLoop->retain();
    
    if (!api->open(this)) {
        IOLog("%s::%s Could not open API\n", getName(), elan_name);
        goto start_exit;
    }

    // set interrupts AFTER device is initialised
    interrupt_source = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CELANTouchpadDriver::interrupt_occurred), api, 0);

    if (!interrupt_source) {
        IOLog("%s::%s Could not get interrupt event source, trying to fallback on polling\n", getName(), elan_name);
        interrupt_simulator = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &VoodooI2CELANTouchpadDriver::simulateInterrupt));
        if (!interrupt_simulator) {
            IOLog("%s::%s Could not get timer event source\n", getName(), elan_name);
            goto start_exit;
        }
        publish_multitouch_interface();
        if (!init_device()) {
            IOLog("%s::%s Failed to init device\n", getName(), elan_name);
            goto start_exit;
        }
        workLoop->addEventSource(interrupt_simulator);
        interrupt_simulator->setTimeoutMS(200);
        IOLog("%s::%s Polling mode initialisation succeeded.", getName(), elan_name);
    } else {
        publish_multitouch_interface();
        if (!init_device()) {
            IOLog("%s::%s Failed to init device\n", getName(), elan_name);
            goto start_exit;
        }
        workLoop->addEventSource(interrupt_source);
        interrupt_source->enable();
    }
    
    PMinit();
    api->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    IOSleep(100);
    ready_for_input = true;
    setProperty("VoodooI2CServices Supported", kOSBooleanTrue);
    IOLog("%s::%s VoodooI2CELAN has started\n", getName(), elan_name);
    mt_interface->registerService();
    registerService();
    return true;
start_exit:
    release_resources();
    return false;
}

void VoodooI2CELANTouchpadDriver::simulateInterrupt(OSObject* owner, IOTimerEventSource *timer) {
    interrupt_occurred(owner, NULL, 0);
    interrupt_simulator->setTimeoutMS(INTERRUPT_SIMULATOR_TIMEOUT);
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
        mt_interface->detach(this);
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

IOReturn VoodooI2CELANTouchpadDriver::message(UInt32 type, IOService* provider, void* argument) {
    switch (type) {
        case kKeyboardGetTouchStatus:
        {
#if DEBUG
            IOLog("%s::getEnabledStatus = %s\n", getName(), ignoreall ? "false" : "true");
#endif
            bool* pResult = (bool*)argument;
            *pResult = !ignoreall;
            break;
        }
        case kKeyboardSetTouchStatus:
        {
            bool enable = *((bool*)argument);
#if DEBUG
            IOLog("%s::setEnabledStatus = %s\n", getName(), enable ? "true" : "false");
#endif
            // ignoreall is true when trackpad has been disabled
            if (enable == ignoreall) {
                // save state, and update LED
                ignoreall = !enable;
            }
            break;
        }
        case kKeyboardKeyPressTime:
        {
            //  Remember last time key was pressed
            keytime = *((uint64_t*)argument);
#if DEBUG
            IOLog("%s::keyPressed = %llu\n", getName(), keytime);
#endif
            break;
        }
    }

    return kIOReturnSuccess;
}
