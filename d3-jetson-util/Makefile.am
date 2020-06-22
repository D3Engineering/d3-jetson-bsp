udevdir = /etc/udev/rules.d/
dist_udev_DATA = \
	target-overlay/etc/udev/rules.d/99-d3-rsp.rules

ispdir = /var/nvidia/nvcam/settings/
dist_isp_DATA = \
	target-overlay/var/nvidia/nvcam/settings/imx390.isp \
	target-overlay/var/nvidia/nvcam/settings/ov10640.isp

bootdir = /boot/
dist_boot_DATA = \
	target-overlay/boot/boot-logo.blob

extlinuxdir = /boot/extlinux/
dist_extlinux_DATA = \
	target-overlay/boot/extlinux/extlinux.conf

scriptsdir = /usr/bin/
dist_scripts_DATA = \
	scripts/d3-install-dtb \
	scripts/d3-select-cameras \
	scripts/d3-build-tegra-media-api

install-data-hook:
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/imx390.isp
	chmod 644 $(DESTDIR)/var/nvidia/nvcam/settings/ov10640.isp

