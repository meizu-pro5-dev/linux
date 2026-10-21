# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(srctree)/$(src) -DCONFIG_UMTS_MODEM_SS333 -DCONFIG_LINK_DEVICE_LLI \
	-DCONFIG_LINK_POWER_MANAGEMENT_WITH_FSM -DCONFIG_BOOT_DEVICE_SPI

obj-$(CONFIG_SEC_MODEM_IF_V1)	+= modem_if.o modem_spi_boot.o
modem_if-y := modem_main.o ipc_common.o modem_io_device.o modem_utils.o \
	modem_modemctl_device_ss333.o modem_link_device_lli.o \
	modem_link_device_memory.o modem_link_device_pm.o
modem_spi_boot-y := boot_device_spi.o
