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

#define VOODOOI2C_ELAN_CTL "me.kishorprins.VoodooI2CELANTouchpadDriver"

#define super IOService
OSDefineMetaClassAndStructors(VoodooI2CELANTouchpadDriver, IOService);

bool VoodooI2CELANTouchpadDriver::init(OSDictionary *properties) {
    ctlRef = NULL;
    mallocTag = NULL;
    lockGroup = NULL;
    handleReportLock = NULL;
    
    if(!super::init(properties)) {
        return false;
    }
    
    // allocate memory for the lock
    mallocTag = OSMalloc_Tagalloc(VOODOOI2C_ELAN_CTL, OSMT_DEFAULT);
    
    // sanity checks
    if(mallocTag == NULL) {
        return false;
    }
    
    lockGroup = lck_grp_alloc_init(VOODOOI2C_ELAN_CTL, LCK_GRP_ATTR_NULL);
    if(lockGroup == NULL) {
        return false;
    }
    
    handleReportLock = lck_mtx_alloc_init(lockGroup, LCK_ATTR_NULL);
    if(handleReportLock == NULL) {
        return false;
    }
    
    awake = true;
    readyForInput = false;
    
    return true;
}

void VoodooI2CELANTouchpadDriver::free() {
    IOLog("ELAN: free called\n");
    super::free();
    
    // clean up memory for locks
    if(handleReportLock) {
        lck_mtx_free(handleReportLock, lockGroup);
        handleReportLock = NULL;
    }
    
    if(lockGroup) {
        lck_grp_free(lockGroup);
        lockGroup = NULL;
    }
    
    if (mallocTag) {
        OSMalloc_Tagfree(mallocTag);
        mallocTag = NULL;
    }
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
    
    workLoop->addEventSource(interruptSource);
    interruptSource->enable();
    
    api->joinPMtree(this);
    
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    
    IOSleep(100);
    
    publishMultitouchInterface();
    
    readyForInput = true;
    
    return true;
    
startExit:
    releaseResources();
    return false;
}

// Linux equivalent of elan_i2c_write_cmd function
IOReturn VoodooI2CELANTouchpadDriver::writeELANCMD(uint16_t reg, uint16_t cmd) {
    uint16_t buffer[] {
        reg,
        cmd
    };
    
    IOReturn retVal = kIOReturnSuccess;
    retVal = api->writeI2C((uint8_t *)&buffer, sizeof(buffer));
    
    return retVal;
}

// Linux equivalent of elan_i2c_read_cmd
IOReturn VoodooI2CELANTouchpadDriver::readELANCMD(uint16_t reg, uint8_t* val) {
    return readRaw16Data(reg, ETP_I2C_INF_LENGTH, val);
}

IOReturn VoodooI2CELANTouchpadDriver::readRawData(uint8_t reg, size_t len, uint8_t* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    retVal = api->writeReadI2C(&reg, 1, values, len);
    
    return retVal;
}

IOReturn VoodooI2CELANTouchpadDriver::readRaw16Data(uint16_t reg, size_t len, uint8_t* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    uint16_t buffer[] {
        reg
    };
   
    retVal = api->writeReadI2C((uint8_t*)buffer, sizeof(buffer), values, len);
    
    return retVal;
}

bool VoodooI2CELANTouchpadDriver::checkForASUSFirmware(uint8_t productId, uint8_t ic_type) {
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

bool VoodooI2CELANTouchpadDriver::initELANDevice() {
    IOReturn retVal = kIOReturnSuccess;
    retVal = writeELANCMD(ETP_I2C_STAND_CMD, ETP_I2C_RESET);
    
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to write RESET cmd\n");
        return false;
    }
    
    // Wait for the device to reset
    IOSleep(100);
    
    // get reset acknowledgement 0000
    uint8_t val[256];
    uint8_t val2[3];
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
    
    uint8_t productId = val2[0];
    
    retVal = readELANCMD(ETP_I2C_SM_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get IC type cmd\n");
        return false;
    }
    
    uint8_t smvers = val2[0];
    uint8_t ictype = val2[1];
    
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
    
    retVal = readELANCMD(ETP_I2C_FW_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get version cmd\n");
        return false;
    }
    uint8_t version = val2[0];
    
    retVal = readELANCMD(ETP_I2C_FW_CHECKSUM_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get checksum cmd\n");
        return false;
    }
    uint16_t csum = *((uint16_t *)val2);
    
    retVal = readELANCMD(ETP_I2C_IAP_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get IAP version cmd\n");
        return false;
    }
    uint8_t iapversion = val2[0];
    
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
    uint16_t max_x = (*((uint16_t *)val2)) & 0x0fff;
    
    retVal = readELANCMD(ETP_I2C_MAX_Y_AXIS_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get max Y axis cmd\n");
        return false;
    }
    
    uint16_t max_y = (*((uint16_t *)val2)) & 0x0fff;
    
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
    
    this->productId = productId;
    
    IOLog("ELAN: ProdID: %d Vers: %d Csum: %d SmVers: %d ICType: %d IAPVers: %d Max X: %d Max Y: %d\n", productId, version, ictype, csum, smvers, iapversion, max_x, max_y);

    return true;
}

void VoodooI2CELANTouchpadDriver::interruptOccurred(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    IOLog("ELAN: Interrupt occurred!\n");
    if (!awake)
        return;
    
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CELANTouchpadDriver::handleELANInput), this, &new_thread);
    if (ret != KERN_SUCCESS){
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
    
    PMinit();
    
    if(!initELANDevice()) {
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

int VoodooI2CELANTouchpadDriver::filloutMultitouchEvent(uint8_t* reportData, OSArray* transducers) {
    if(transducers == NULL) {
        return 0;
    }
    
    if(reportData[0] == 0xff) {
        return 0;
    }
    
    // Get the current timestamp
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    uint8_t *fingerData = &reportData[ETP_FINGER_DATA_OFFSET];
    int numFingers = 0;
    
    uint8_t touchInfo = reportData[ETP_TOUCH_INFO_OFFSET];
    uint8_t hoverInfo = reportData[ETP_HOVER_INFO_OFFSET];
    
   // bool hoverEvent = hoverInfo & 0x40;
    for(int i = 0; i < ETP_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, transducers->getObject(i));
        
        if(transducer == NULL) {
            continue;
        }
        
        bool contactValid = touchInfo & (1U << (3 + i));
        if(!contactValid) {
            continue;
        }
        
        transducer->coordinates.x.update(((fingerData[0] & 0xf0) << 4) | fingerData[1], timestamp);
        transducer->coordinates.x.update(((fingerData[0] & 0x0f) << 8) | fingerData[2], timestamp);
        
        transducer->physical_button.update(touchInfo & 0x01, timestamp);
        
        numFingers += 1;
        fingerData += ETP_FINGER_DATA_LEN;
        
    }
    
    return numFingers;
}

void VoodooI2CELANTouchpadDriver::handleELANInput() {
    lck_mtx_lock(handleReportLock);
    if(!readyForInput) {
        lck_mtx_unlock(handleReportLock);
        return;
    }
    
    if(api == NULL) {
        IOLog("ELAN: API is null\n");
        lck_mtx_unlock(handleReportLock);
        return;
    }
    
   
    uint8_t reportData[ETP_MAX_REPORT_LEN];
    for(int i = 0; i < ETP_MAX_REPORT_LEN; i++) {
        reportData[i] = 0;
    }
    
    IOReturn retVal = api->readI2C(reportData, ETP_MAX_REPORT_LEN);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to handle input\n");
        lck_mtx_unlock(handleReportLock);
        return;
    }
    
    // Get the current timestamp
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    OSArray* transducers = OSArray::withCapacity(ETP_MAX_FINGERS);
    int numFingers =  filloutMultitouchEvent(reportData, transducers);
    
    // create new VoodooI2CMultitouchEvent
    VoodooI2CMultitouchEvent event;
    event.contact_count = numFingers;
    event.transducers = transducers;
    
    //TODO: fill out the event (???)
    
    // send the event into the multitouch interface
    if(multitouchInterface != NULL) {
        //lck_spin_lock(handleReportLock);
        multitouchInterface->handleInterruptReport(event, timestamp);
      //  lck_spin_unlock(handleReportLock);
    }
    
    IOLog("ELAN: Handled data!\n");
    
    lck_mtx_unlock(handleReportLock);
    
    OSSafeReleaseNULL(transducers);
}

void VoodooI2CELANTouchpadDriver::setELANSleepStatus(bool enable) {
    /*uint8_t val[2];
    uint16_t reg;
    
    IOReturn error = readELANCMD((uint8_t)ETP_I2C_POWER_CMD, val);
    if(error != kIOReturnSuccess) {
        IOLog("ELAN: Failed to read power state\n");
        return;
    }
    
    //le16_to_cpup((__le16 *)val);
    // Nopt sure if this is correct (ignoring little endianess here)
    reg = val[0];
    
    // enable = True means turn on power
    if (enable) {
        reg &= ~ETP_DISABLE_POWER;
    } else {
        reg |= ETP_DISABLE_POWER;
    }
    
    error = writeELANCMD(reg, ETP_I2C_POWER_CMD);
    if(error != kIOReturnSuccess) {
        IOLog("ELAN: Failed to set power state to %d\n", enable);
    }*/
    
    IOReturn retVal = writeELANCMD(ETP_I2C_STAND_CMD, enable ? ETP_I2C_SLEEP : ETP_I2C_WAKE_UP);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to set sleep status(%d)\n", enable);
    }
    
    IOLog("ELAN: Set sleep status to %d\n", enable);
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
}

void VoodooI2CELANTouchpadDriver::stop(IOService* provider) {
        releaseResources();
    unpublishMultitouchInterface();
    PMstop();
    IOLog("ELAN: Stop called\n");
    super::stop(provider);

}

IOReturn VoodooI2CELANTouchpadDriver::setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) {
    /*if (whatDevice != this)
        return kIOReturnInvalid;
    if (longpowerStateOrdinal == 0){
        if (awake){
            awake = false;
            
            // Off
            setELANSleepStatus(false);
            
            IOLog("ELAN: Going to sleep\n");
        }
    } else {
        if (!awake){
           // On
            setELANSleepStatus(true);
            
            awake = true;
            IOLog("ELAN: Woke up\n");
        }
    } */
    IOLog("ELAN: set power state called\n");
    return super::setPowerState(longpowerStateOrdinal, whatDevice);
}

