#include "stmp3770_usbphy.h"
#include "registers/regsclkctrl.h"
#include "registers/regsdigctl.h"
#include "registers/regspower.h"
#include "registers/regsusbctrl.h"
#include "registers/regsusbphy.h"

void usb_phy_clkctrl_enable(bool enable) {
    //PLL_enable(1);

    if (enable)
        BF_SET(CLKCTRL_PLLCTRL0, EN_USB_CLKS);
    else
        BF_CLR(CLKCTRL_PLLCTRL0, EN_USB_CLKS);
}

bool usb_phy_clkctrl_is_usb_enabled(void) {
    return BF_RD(CLKCTRL_PLLCTRL0, EN_USB_CLKS);
}

void usb_phy_enable_usb_controller(bool enable) {
    if (enable)
        BF_CLR(DIGCTL_CTRL, USB_CLKGATE);
    else
        BF_SET(DIGCTL_CTRL, USB_CLKGATE);
}

void usb_phy_pwr_enable(bool enable) {
    if (enable) {

        BF_CLR(POWER_CTRL, CLKGATE);
        BF_SET(POWER_DEBUG, VBUSVALIDPIOLOCK);
        BF_SET(POWER_DEBUG, AVALIDPIOLOCK);
        BF_SET(POWER_DEBUG, BVALIDPIOLOCK);

        BF_SET(POWER_STS, BVALID);
        BF_SET(POWER_STS, AVALID);
        BF_SET(POWER_STS, VBUSVALID);

        BF_CLR(USBPHY_CTRL, SFTRST);
        BF_CLR(USBPHY_CTRL, CLKGATE);
        //HW_USBPHY_PWD_CLR(0xffffffff);
    } else {
        //HW_USBPHY_PWD_SET(0xffffffff);
        BF_SET(USBPHY_CTRL, SFTRST);
        BF_SET(USBPHY_CTRL, CLKGATE);
    }
}

bool usb_phy_power_usb_detect(void) {
    return BF_RD(POWER_STS, VDD5V_GT_VDDIO);
}



void usb_phy_enable(bool on) {
    if (on) {
        usb_phy_clkctrl_enable(true);
        usb_phy_pwr_enable(true);
        usb_phy_enable_usb_controller(true);
        //usb_core_init();
    } else {
        //usb_core_exit();
        usb_phy_enable_usb_controller(false);
        usb_phy_pwr_enable(false);
        usb_phy_clkctrl_enable(false);
    }
}
