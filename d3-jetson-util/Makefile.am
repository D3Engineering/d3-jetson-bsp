udevdir = /etc/udev/rules.d/
dist_udev_DATA = \
	target-overlay/etc/udev/rules.d/99-d3-rsp.rules

ispdir = /var/nvidia/nvcam/settings/
dist_isp_DATA = \
	target-overlay/var/nvidia/nvcam/settings/imx390.isp \
	target-overlay/var/nvidia/nvcam/settings/ov10640.isp \
	target-overlay/var/nvidia/nvcam/settings/ar0234velocity_wide.isp \
	target-overlay/var/nvidia/nvcam/settings/ar0234velocity_medium.isp \
	target-overlay/var/nvidia/nvcam/settings/ar0234velocity_narrow.isp \
	target-overlay/var/nvidia/nvcam/settings/imx390discovery_wide.isp \
	target-overlay/var/nvidia/nvcam/settings/imx390discovery_medium.isp \
	target-overlay/var/nvidia/nvcam/settings/imx390discovery_narrow.isp

extlinuxdir = /boot/extlinux/
extlinux_DATA = \
	target-overlay/boot/extlinux/extlinux.conf

aptprefsdir = /etc/apt/preferences.d
dist_aptprefs_DATA = \
	target-overlay/etc/apt/preferences.d/hold-nvidia-repo

tmpfanctrldir = /etc/d3-jetson-utils
dist_tmpfanctrl_DATA = \
	target-overlay/etc/d3-jetson-utils/nvfancontrol.conf

# tegralibdir = /usr/lib/aarch64-linux-gnu/tegra/
# dist_tegralib_DATA = \
# 	target-overlay/usr/lib/aarch64-linux-gnu/tegra/libnvscf.so \
# 	target-overlay/usr/lib/aarch64-linux-gnu/tegra/libnvodm_imager.so

scriptsdir = /usr/bin/
dist_scripts_DATA = \
	scripts/d3-install-dtb \
	scripts/d3-select-cameras-boot \
	scripts/d3-eeprom-select-cameras \
	scripts/d3-select-isp
tmplibdir = /usr/lib/d3-jetson-utils
dist_tmplib_DATA = \
	target-overlay/usr/lib/d3-jetson-utils/libnvscf.so

install-data-hook:
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ov10640.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ar0234velocity_wide.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ar0234velocity_medium.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ar0234velocity_narrow.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390discovery_wide.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390discovery_medium.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390discovery_narrow.isp
