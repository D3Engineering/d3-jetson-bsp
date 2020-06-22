# The static build ID
obj-$(CONFIG_D3_BUILDID) += build-id/

# The driver to read configuration from the EEPROM.
# List before config-eeprom so config-eeprom will be able to
# access this EEPROM.
# Note: initialization order is link order within an initcall level.
obj-$(CONFIG_D3_EARLY_AT24) += early-at24/

# The driver to read configuration from the EEPROM.
# List before overlay-loader so overlay-loader will be able to
# retrieve the EEPROM information.
# Note: initialization order is link order within an initcall level.
obj-$(CONFIG_D3_CONFIG_EEPROM) += config-eeprom/

# The overlay loader to activate the configured camera drivers.
# List after config-eeprom; recommend listing before any other drivers
# so that the device tree is fully populated when the drivers load.
obj-$(CONFIG_D3_OVERLAY_LOADER) += overlay-loader/

# The drivers that can be activated by the overlay loader
obj-$(CONFIG_D3_UB960) += ub960/
obj-$(CONFIG_D3_UB953) += ub953/
obj-$(CONFIG_D3_MAX9296) += max9296/
obj-$(CONFIG_D3_IMX390) += imx390/
obj-$(CONFIG_D3_AR0820) += ar0820/
obj-$(CONFIG_D3_VG6768) += vg6768/
obj-$(CONFIG_D3_OV10640) += ov10640/
