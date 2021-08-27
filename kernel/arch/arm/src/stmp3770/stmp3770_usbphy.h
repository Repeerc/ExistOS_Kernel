
#ifndef STMP3770_USBPHY_H
#define STMP3770_USBPHY_H
#include <stdbool.h>

void usb_phy_enable(bool on);
void usb_phy_clkctrl_enable(bool enable);
bool usb_phy_clkctrl_is_usb_enabled(void) ;




#endif