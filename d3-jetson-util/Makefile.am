udevdir = /etc/udev/rules.d/
dist_udev_DATA = \
	target-overlay/etc/udev/rules.d/99-d3-rsp.rules

ispdir = /var/nvidia/nvcam/settings/
dist_isp_DATA = \
	target-overlay/var/nvidia/nvcam/settings/imx390.isp \
	target-overlay/var/nvidia/nvcam/settings/ov10640.isp

bootdir = /boot/
boot_DATA = \
	target-overlay/boot/boot-logo.blob

extlinuxdir = /boot/extlinux/
extlinux_DATA = \
	target-overlay/boot/extlinux/extlinux.conf


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

install-data-hook:
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ov10640.isp
