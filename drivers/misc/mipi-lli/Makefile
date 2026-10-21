# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_EXYNOS_MIPI_LLI_MPHY)	+= exynos-mipi-lli-mphy.o
obj-$(CONFIG_MIPI_LLI)			+= mipi-lli.o
mipi-lli-y				:= mipi-lli-core.o exynos-mipi-lli.o
ccflags-y			+= -DCONFIG_UMTS_MODEM_SS333 -DCONFIG_LINK_DEVICE_LLI
