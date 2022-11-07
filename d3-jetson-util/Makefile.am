udevdir = /etc/udev/rules.d/
dist_udev_DATA = \
	target-overlay/etc/udev/rules.d/99-d3-rsp.rules

extlinuxdir = /boot/extlinux/
extlinux_DATA = \
	target-overlay/boot/extlinux/extlinux.conf

aptprefsdir = /etc/apt/preferences.d
dist_aptprefs_DATA = \
	target-overlay/etc/apt/preferences.d/hold-nvidia-repo


# tegralibdir = /usr/lib/aarch64-linux-gnu/tegra/
# dist_tegralib_DATA = \
# 	target-overlay/usr/lib/aarch64-linux-gnu/tegra/libnvscf.so \
# 	target-overlay/usr/lib/aarch64-linux-gnu/tegra/libnvodm_imager.so

scriptsdir = /usr/bin/
dist_scripts_DATA = \
	scripts/d3-install-dtb \
	scripts/d3-select-cameras-boot \
	scripts/d3-eeprom-select-cameras \
	scripts/d3-build-tegra-media-api \
	scripts/d3-patch-jp-44-jmm-api
tmplibdir = /usr/lib/d3-jetson-utils
dist_tmplib_DATA = \
	target-overlay/usr/lib/d3-jetson-utils/libnvscf.so
