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

bool VoodooI2CELANTouchpadDriver::start(IOService* provider) {
    if(!IOService::start(provider)) {
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

    interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CELANTouchpadDriver::interruptOccured), api, 0);
    
    if(interruptSource == NULL) {
        IOLog("ELAN: Could not get interrupt event source\n");
        goto startExit;
    }
    
    workLoop->addEventSource(interruptSource);
    interruptSource->enable();
    
    PMinit();
    if(!initELANDevice()) {
        goto startExit;
    }
    
    api->joinPMtree(this);
    
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    
    IOSleep(100);
    
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
IOReturn VoodooI2CELANTouchpadDriver::readELANCMD(uint8_t reg, uint8_t* val) {
    return readRaw16Data(reg, ETP_I2C_INF_LENGTH, val);
}

IOReturn VoodooI2CELANTouchpadDriver::readRawData(uint8_t reg, size_t len, uint8_t* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    uint8_t buffer[] {
        reg
    };
    
    retVal = api->writeReadI2C(buffer, sizeof(buffer), values, len);
    
    return retVal;
}

IOReturn VoodooI2CELANTouchpadDriver::readRaw16Data(uint8_t reg, size_t len, uint8_t* values) {
    IOReturn retVal= kIOReturnSuccess;
    
    uint16_t buffer[] {
        reg
    };
    
    retVal = api->writeReadI2C((uint8_t *)&buffer, sizeof(buffer), values, len);
    
    return retVal;
}

bool VoodooI2CELANTouchpadDriver::checkForASUSFirmware(uint8_t productId, uint8_t ic_type) {
    if (ic_type == 0x0E) {
        switch (productId) {
            case 0x05 ... 0x07: break;
            case 0x09: break;
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
    
    retVal = readRawData(ETP_I2C_DESC_CMD, ETP_I2C_DESC_LENGTH, val);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get desc cmd\n");
        return false;
    }
    
    retVal = readRawData(ETP_I2C_REPORT_DESC_CMD, ETP_I2C_REPORT_DESC_LENGTH, val);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get report cmd\n");
        return false;
    }
    
    // get the product ID
    retVal = readELANCMD((uint8_t)ETP_I2C_UNIQUEID_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get product ID cmd\n");
        return false;
    }
    uint8_t productId = val2[0];
    
    retVal = readELANCMD((uint8_t)ETP_I2C_SM_VERSION_CMD, val2);
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
    
    retVal = readELANCMD((uint8_t)ETP_I2C_FW_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get version cmd\n");
        return false;
    }
    uint8_t version = val2[0];
    
    retVal = readELANCMD((uint8_t)ETP_I2C_FW_CHECKSUM_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get checksum cmd\n");
        return false;
    }
    uint16_t csum = *((uint16_t *)val2);
    
    retVal = readELANCMD((uint8_t)ETP_I2C_IAP_VERSION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get IAP version cmd\n");
        return false;
    }
    uint8_t iapversion = val2[0];
    
    retVal = readELANCMD((uint8_t)ETP_I2C_PRESSURE_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get pressure cmd\n");
        return false;
    }
    
    retVal = readELANCMD((uint8_t)ETP_I2C_MAX_X_AXIS_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get max X axis cmd\n");
        return false;
    }
    uint16_t max_x = (*((uint16_t *)val2)) & 0x0fff;
    
    retVal = readELANCMD((uint8_t)ETP_I2C_MAX_Y_AXIS_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get max Y axis cmd\n");
        return false;
    }
    uint16_t max_y = (*((uint16_t *)val2)) & 0x0fff;
    
    retVal = readELANCMD((uint8_t)ETP_I2C_XY_TRACENUM_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get XY tracenum cmd\n");
        return false;
    }
    
    uint8_t x_traces = val2[0];
    uint8_t y_traces = val2[1];
    
    retVal = readELANCMD((uint8_t)ETP_I2C_RESOLUTION_CMD, val2);
    if(retVal != kIOReturnSuccess) {
        IOLog("ELAN: Failed to get touchpad resolution cmd\n");
        return false;
    }
    
    IOLog("ELAN: ProdID: %d Vers: %d Csum: %d SmVers: %d ICType: %d IAPVers: %d Max X: %d Max Y: %d\n", productId, version, ictype, csum, smvers, iapversion, max_x, max_y);

    
    return true;
}

void VoodooI2CELANTouchpadDriver::interruptOccured(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    /*if (read_in_progress)
        return;
    if (!awake)
        return;
    
    read_in_progress = true;
    
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CHIDDevice::getInputReport), this, &new_thread);
    if (ret != KERN_SUCCESS){
        read_in_progress = false;
        IOLog("%s::%s Thread error while attempint to get input report\n", getName(), name);
    } else {
        thread_deallocate(new_thread);
    } */
}

VoodooI2CELANTouchpadDriver* VoodooI2CELANTouchpadDriver::probe(IOService* provider, SInt32* score) {
    IOLog("ELAN: Touchpad probe!\n");
    if(!IOService::probe(provider, score)) {
        return NULL;
    }
    
    acpiDevice = OSDynamicCast(IOACPIPlatformDevice, provider->getProperty("acpi-device"));
    
    if (acpiDevice == NULL) {
        IOLog("ELAN: Could not get ACPI device\n");
        return NULL;
    }
    
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    
    if(api == NULL) {
        IOLog("ELAN: Could not get VoodooI2C API instance\n");
        return NULL;
    }
   
    return this;
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
