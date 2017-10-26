//
//  VoodooI2CELANTouchpadDriver.cpp
//  VoodooI2CELAN
//
//  Created by Kishor Prins on 2017-10-13.
//  Copyright © 2017 Kishor Prins. All rights reserved.
//

#include "../../../VoodooI2C/VoodooI2C/VoodooI2CController/VoodooI2CControllerDriver.hpp"

#include "VoodooI2CELANTouchpadDriver.hpp"
#include "LinuxELANI2C.h"

#define super IOService
OSDefineMetaClassAndStructors(VoodooI2CELANTouchpadDriver, IOService);

bool VoodooI2CELANTouchpadDriver::init(OSDictionary *properties) {
    maxHWResolutionX = 0;
    maxHWResolutionY = 0;
    
    maxReportX = 0;
    maxReportY = 0;
    transducers = NULL;
    
    if(!super::init(properties)) {
        return false;
    }
    
    transducers = OSArray::withCapacity(ETP_MAX_FINGERS);
    if(transducers == NULL) {
        return false;
    }
    
    // initialise the N transducers we need for tracking our N fingers
    // we are a touchpad
    DigitiserTransducuerType type = kDigitiserTransducerFinger;
    for(int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(type, NULL);
        transducers->setObject(transducer);
    }
    
    awake = true;
    readyForInput = false;
    readInProgress = false;
    
    return true;
}

void VoodooI2CELANTouchpadDriver::free() {
    IOLog("ELAN: free called\n");
    super::free();
}

bool VoodooI2CELANTouchpadDriver::start(IOService* provider) {
    if(!super::start(provider)) {
        return false;
    }
    
    workLoop = this->getWorkLoop();
    if(workLoop == NULL) {
        IOLog("ELAN: Could not get a IOWorkLoop instance\n");
        return false;
    }
    
    workLoop->retain();
    
    commandGate = IOCommandGate::commandGate(this);
    if (commandGate == NULL || (workLoop->addEventSource(commandGate) != kIOReturnSuccess)) {
        IOLog("ELAN: Could not open command gate\n");
        goto startExit;
    }
    
    acpiDevice->retain();
    api->retain();
    
    if (!api->open(this)) {
        IOLog("ELAN: Could not open API\n");
        goto startExit;
    }
    
    // set interrupts AFTER device is initialised
    interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CELANTouchpadDriver::interruptOccurred), api, 0);
    
    if(interruptSource == NULL) {
        IOLog("ELAN: Could not get interrupt event source\n");
        goto startExit;
    }
    
    publishMultitouchInterface();
   
    if(!initELANDevice()) {
        IOLog("ELAN: Failed to init device\n");
        return NULL;
    }

    workLoop->addEventSource(interruptSource);
    interruptSource->enable();
    
    
    PMinit();
    api->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    
    IOSleep(100);
    
    readyForInput = true;
    
    IOLog("ELAN: Started \n");
    
    return true;
    
startExit:
    releaseResources();
    return false;
}

// Linux equivalent of elan_i2c_write_cmd function
IOReturn VoodooI2CELANTouchpadDriver::writeELANCMD(UInt16 reg, UInt16 cmd) {
    UInt16 buffer[] {
        reg,
        cmd
    };
    
    IOReturn retVal = kIOReturnSuccess;
    retVal = api->writeI2C((UInt8 *)&buffer, sizeof(buffer));
    
    return retVal;
}

// Linux equivalent of elan_i2c_read_cmd
IOReturn VoodooI2CELANTouchpadDriver::readELANCMD(UInt16 reg, UInt8* val) {
    return readRaw16Data(reg, ETP_I2C_INF_LENGTH, val);
}

IOReturn VoodooI2CELANTouchpadDriver::readRawData(UInt8 reg, size_t len, UInt8* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    retVal = api->writeReadI2C(&reg, 1, values, len);
    
    return retVal;
}

IOReturn VoodooI2CELANTouchpadDriver::readRaw16Data(UInt16 reg, size_t len, UInt8* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    UInt16 buffer[] {
        reg
    };
   
    retVal = api->writeReadI2C((UInt8*)buffer, sizeof(buffer), values, len);
    
    return retVal;
}

bool VoodooI2CELANTouchpadDriver::checkForASUSFirmware(UInt8 productId, UInt8 ic_type) {
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

bool VoodooI2CELANTouchpadDriver::resetELANDevice() {
    IOReturn retVal = kIOReturnSuccess;
    retVal = writeELANCMD(ETP_I2C_STAND_CMD, ETP_I2C_RESET);
    
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to write RESET cmd\n");
        return false;
    }
    
    // Wait for the device to reset
    IOSleep(100);
    
    // get reset acknowledgement 0000
    UInt8 val[256];
    UInt8 val2[3];
    retVal = readRawData(0x00, ETP_I2C_INF_LENGTH, val);
    
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get reset acknowledgement\n");
        return false;;
    }
    
    retVal = readRaw16Data(ETP_I2C_DESC_CMD, ETP_I2C_DESC_LENGTH, val);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get desc cmd\n");
        return false;
    }
    
    retVal = readRaw16Data(ETP_I2C_REPORT_DESC_CMD, ETP_I2C_REPORT_DESC_LENGTH, val);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get report cmd\n");
        return false;
    }
    
    // get the product ID
    retVal = readELANCMD(ETP_I2C_UNIQUEID_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get product ID cmd\n");
        return false;
    }
    
    UInt8 productId = val2[0];
    
    retVal = readELANCMD(ETP_I2C_SM_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get IC type cmd\n");
        return false;
    }
    
    UInt8 smvers = val2[0];
    UInt8 ictype = val2[1];
    
    if(checkForASUSFirmware(productId, ictype)) {
        IOLog("ELAN: Buggy ASUS trackpad detected\n");
        
        retVal = writeELANCMD(ETP_I2C_STAND_CMD, ETP_I2C_WAKE_UP);
        if(retVal != kIOReturnSuccess) {
            IOLog("ELAN: Failed to send wake up cmd (workaround)\n");
            return false;
        }
        
        IOSleep(200);
        
        retVal =  writeELANCMD(ETP_I2C_SET_CMD, ETP_ENABLE_ABS);
        if(retVal != kIOReturnSuccess) {
            IOLog("ELAN: Failed to send enable cmd (workaround)\n");
            return false;
        }
    } else {
        retVal = writeELANCMD(ETP_I2C_SET_CMD, ETP_ENABLE_ABS);
        if(retVal != kIOReturnSuccess) {
            IOLog("ELAN: Failed to send enable cmd\n");
            return false;
        }
        
        retVal = writeELANCMD(ETP_I2C_STAND_CMD, ETP_I2C_WAKE_UP);
        if(retVal != kIOReturnSuccess) {
            IOLog("ELAN: Failed to send wake up cmd\n");
            return false;
        }
    }
    
    return true;
}

bool VoodooI2CELANTouchpadDriver::initELANDevice() {
    if(!resetELANDevice()) {
        return false;
    }
    
    IOReturn retVal;
    UInt8 val[256];
    UInt8 val2[3];
    
    retVal = readELANCMD(ETP_I2C_FW_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get version cmd\n");
        return false;
    }
    UInt8 version = val2[0];
    
    retVal = readELANCMD(ETP_I2C_FW_CHECKSUM_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get checksum cmd\n");
        return false;
    }
    UInt16 csum = *((UInt16 *)val2);
    
    retVal = readELANCMD(ETP_I2C_IAP_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get IAP version cmd\n");
        return false;
    }
    UInt8 iapversion = val2[0];
    
    retVal = readELANCMD(ETP_I2C_PRESSURE_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get pressure cmd\n");
        return false;
    }
    
    retVal = readELANCMD(ETP_I2C_MAX_X_AXIS_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get max X axis cmd\n");
        return false;
    }
    maxReportX = (*((UInt16 *)val2)) & 0x0fff;
    
    retVal = readELANCMD(ETP_I2C_MAX_Y_AXIS_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get max Y axis cmd\n");
        return false;
    }
    
    maxReportY = (*((UInt16 *)val2)) & 0x0fff;
    
    retVal = readELANCMD(ETP_I2C_XY_TRACENUM_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get XY tracenum cmd\n");
        return false;
    }
    
    retVal = readELANCMD(ETP_I2C_RESOLUTION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get touchpad resolution cmd\n");
        return false;
    }
    
    maxHWResolutionX = val2[0];
    maxHWResolutionY = val2[1];
    
    IOLog("ELAN: ProdID: %d Vers: %d Csum: %d IAPVers: %d Max X: %d Max Y: %d\n", productId, version, csum, iapversion, maxReportX, maxReportY);
    
    if(multitouchInterface != NULL) {
        multitouchInterface->logical_max_x = maxReportX;
        multitouchInterface->logical_max_y = maxReportY;
        
        multitouchInterface->physical_max_x = maxHWResolutionX;
        multitouchInterface->physical_max_x = maxHWResolutionY;
    }

    return true;
}

void VoodooI2CELANTouchpadDriver::interruptOccurred(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    if(readInProgress)
        return;
    
    if (!awake)
        return;
    
    readInProgress = true;
    
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CELANTouchpadDriver::handleELANInput), this, &new_thread);
    if (ret != KERN_SUCCESS){
        readInProgress = false;
        IOLog("ELAN: Thread error while attempint to get input report\n");
    } else {
        thread_deallocate(new_thread);
    }
}

VoodooI2CELANTouchpadDriver* VoodooI2CELANTouchpadDriver::probe(IOService* provider, SInt32* score) {
    IOLog("ELAN: Touchpad probe!\n");
    if(!super::probe(provider, score)) {
        return NULL;
    }
    
    acpiDevice = OSDynamicCast(IOACPIPlatformDevice, provider->getProperty("acpi-device"));
    
    if (acpiDevice == NULL) {
        IOLog("ELAN: Could not get ACPI device\n");
        return NULL;
    }
    
    // check for ELAN devices (DSDT must have ELAN* defined in the name property)
    OSData* nameData = OSDynamicCast(OSData, provider->getProperty("name"));
    if(nameData == NULL) {
        IOLog("ELAN: Unable to get 'name' property\n");
        return NULL;
    }
    
    char* deviceName = (char*)nameData->getBytesNoCopy();
    if(deviceName[0] != 'E' && deviceName[1] != 'L'
       && deviceName[2] != 'A'&& deviceName[3] != 'N') {
        IOLog("ELAN: ELAN device not found, instead found %s\n", deviceName);
        return NULL;
    }
    
    IOLog("ELAN: Found device name %s\n", deviceName);
    
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    
    if(api == NULL) {
        IOLog("ELAN: Could not get VoodooI2C API instance\n");
        return NULL;
    }
    
    return this;
}

bool VoodooI2CELANTouchpadDriver::publishMultitouchInterface() {
    multitouchInterface = new VoodooI2CMultitouchInterface();
   
    if(multitouchInterface == NULL) {
        IOLog("ELAN: No memory to allocate VoodooI2CMultitouchInterface instance\n");
        goto multitouchExit;
    }
    
    if(!multitouchInterface->init(NULL)) {
        IOLog("ELAN: Failed to init multitouch interface\n");
        goto multitouchExit;
    }
    
    if (!multitouchInterface->attach(this)) {
        IOLog("ELAN: Failed to attach multitouch interface\n");
        goto multitouchExit;
    }
    
    if (!multitouchInterface->start(this)) {
        IOLog("ELAN: Failed to start multitouch interface\n");
        goto multitouchExit;
    }
    
    // Assume we are a touchpad
    multitouchInterface->setProperty(kIOHIDDisplayIntegratedKey, true);
    // 0x04f3 is Elan's Vendor Id
    multitouchInterface->setProperty(kIOHIDVendorIDKey, 0x04f3, 32);
    multitouchInterface->setProperty(kIOHIDProductIDKey, productId, 32);
    
    multitouchInterface->registerService();
    
    return true;
    
multitouchExit:
    unpublishMultitouchInterface();
    
    return false;
}

void VoodooI2CELANTouchpadDriver::unpublishMultitouchInterface() {
    if(multitouchInterface != NULL) {
        multitouchInterface->stop(this);
        multitouchInterface->release();
        multitouchInterface = NULL;
    }
}

IOReturn VoodooI2CELANTouchpadDriver::parseELANReport() {
    if(api == NULL) {
        IOLog("ELAN: API is null\n");
        return kIOReturnError;
    }
    
    UInt8 reportData[ETP_MAX_REPORT_LEN];
    for(int i = 0; i < ETP_MAX_REPORT_LEN; i++) {
        reportData[i] = 0;
    }
    
    IOReturn retVal = readRawData(0, sizeof(reportData), reportData);
    
    if(retVal == kIOReturnBadArgument) {
        IOLog("ELAN: Bad argument when reading input\n");
    } else if(retVal == kIOReturnAborted) {
        IOLog("ELAN: Aborted when reading input\n");
    } else if(retVal == kIOReturnCannotLock) {
        IOLog("ELAN: Cannot acquire command gate lock when reading input\n");
    } else if(retVal == kIOReturnNotPermitted) {
        IOLog("ELAN: Aborted when reading input\n");
    }

    
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to handle input\n");
        return retVal;
    }
    
    if(transducers == NULL) {
        return kIOReturnBadArgument;
    }
    
    if(reportData[ETP_REPORT_ID_OFFSET] != ETP_REPORT_ID) {
        IOLog("ELAN: Invalid report (%d)", reportData[ETP_REPORT_ID_OFFSET]);
        return kIOReturnError;
    }
    
    // Get the current timestamp
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    UInt8* finger_data = &reportData[ETP_FINGER_DATA_OFFSET];
    UInt8 tp_info = reportData[ETP_TOUCH_INFO_OFFSET];
   // UInt8 hover_info = reportData[ETP_HOVER_INFO_OFFSET];
    
    int numFingers = 0;
    
    for (int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(i));
        
        if(transducer == NULL) {
            continue;
        }
        
        bool contactValid = tp_info & (1U << (3 + i));
        transducer->is_valid = contactValid;
        
        if(contactValid) {
            unsigned int posX = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
            unsigned int posY = ((finger_data[0] & 0x0f) << 8) | finger_data[2];
            
            // switch to relative coords
            posX = posX - maxReportX;
            posY = maxReportY - posY;
            
            posX *= 10;
            posY *= 10;
            
            posX /= maxHWResolutionX;
            posY /= maxHWResolutionY;
            
            transducer->coordinates.x.update(posX, timestamp);
            transducer->coordinates.y.update(posY, timestamp);
            transducer->physical_button.update(tp_info & 0x01, timestamp);
            
            // fake HID-ness (for CSGesture)
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
    if(multitouchInterface != NULL) {
        multitouchInterface->handleInterruptReport(event, timestamp);
    };
    
    return kIOReturnSuccess;
}

void VoodooI2CELANTouchpadDriver::handleELANInput() {
    if(!readyForInput) {
        return;
    }
    
    commandGate->attemptAction(OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooI2CELANTouchpadDriver::parseELANReport));
    readInProgress = false;
}

void VoodooI2CELANTouchpadDriver::releaseResources() {
    if (commandGate != NULL) {
        workLoop->removeEventSource(commandGate);
        commandGate->release();
        commandGate = NULL;
    }
    
    if (interruptSource != NULL) {
        interruptSource->disable();
        workLoop->removeEventSource(interruptSource);
        interruptSource->release();
        interruptSource = NULL;
    }
    
    if (workLoop != NULL) {
        workLoop->release();
        workLoop = NULL;
    }
    
    if (acpiDevice != NULL) {
        acpiDevice->release();
        acpiDevice = NULL;
    }
    
    if (api != NULL) {
        if (api->isOpen(this)) {
            api->close(this);
        }
        api->release();
        api = NULL;
    }
    
    if(transducers != NULL) {
        for(int i = 0; i < transducers->getCount(); i++) {
            OSObject* object = transducers->getObject(i);
            if(object != NULL) {
                object->release();
            }
        }
        //transducers->flushCollection();
        OSSafeReleaseNULL(transducers);
    }
}

void VoodooI2CELANTouchpadDriver::stop(IOService* provider) {
    releaseResources();
    unpublishMultitouchInterface();
    PMstop();
    IOLog("ELAN: Stop called\n");
    super::stop(provider);

}

IOReturn VoodooI2CELANTouchpadDriver::setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOReturnInvalid;
    if (longpowerStateOrdinal == 0){
        if (awake){
            awake = false;
            
            for(;;) {
                if(!readInProgress) {
                    break;
                }
                
                IOSleep(10);
            }
            
            // Off
            //setELANSleepStatus(false);
            
            IOLog("ELAN: Going to sleep\n");
        }
    } else {
        if (!awake){
           // On
            //setELANSleepStatus(true);
            resetELANDevice();
            
            awake = true;
            IOLog("ELAN: Woke up\n");
        }
    }
    
    return kIOPMAckImplied;
}

