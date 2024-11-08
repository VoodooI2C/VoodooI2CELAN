# VoodooI2CELAN
VoodooI2C ELAN is a macOS kernel extension for a limited subset of ELAN based touchpads. The codebase based on the Linux kernel mode extensions and as such the codebase is released under the GPL v2 license.

## Installation and Usage
Please refer to https://voodooi2c.github.io/#Installation/Installation.

## Supported Touchpads
* ELAN0000 (Found in Chromebooks)
* ELAN1000
* ELAN0000
* ELAN0100
* ELAN0651
* ELAN0626 (use Force Polling with polling interval set to 5 recommended, see Polling Configuration)

## Polling Configuration
Some trackpads may not work properly or at all with interrupt mode, even though it gets initialized successfully, that's why you may consider forcing polling. To force polling you need to open `VoodooI2CELAN.kext/Contents/Info.plist` in any editor and change `ForcePolling` to true.
You may also want to change the polling interval, for that you'll need to edit `PollingInterval`, you can determine yours by trying out a couple different ones.

*Note: Newer versions of the ELAN touchpads supports another protocol called Precision Touchpad (PTP). Touchpads implementing this protocol need to use VoodooI2CHID. An example of a ELAN based touchpad PTP is the ELAN1200.*

## Support
Please make sure you have the read https://voodooi2c.github.io/#Troubleshooting/Troubleshooting. If you are still facing troubles please contact me on Gitter.
