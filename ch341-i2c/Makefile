#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ch341-i2c
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ch341-i2c
  SUBMENU:=USB Support
  TITLE:=CH341 USB-to-I2C SPI UART converter
  FILES:=$(PKG_BUILD_DIR)/ch341-i2c.ko
  DEPENDS:=+kmod-i2c-core +kmod-usb-core
  AUTOLOAD:=$(call AutoLoad,30,ch341-i2c,1)
  KCONFIG:=
endef

define KernelPackage/ch341-i2c/description
	Drives ch341 based usb dongles. 
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,ch341-i2c))
