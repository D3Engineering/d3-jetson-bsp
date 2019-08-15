##
# @author Greg Rowe <growe@d3engineering.com> based on work from Josh
# Watts <jwatts@d3engineering.com)
#
# A Makefile for working with the Jetson platform from Nvidia.
#
# `make help` for documentation

# Do not build docs by default to save developers the hassle of the
# pandoc and latex dependencies.
#SUBDIRS = doc

# This is used so that the custom rules are silent unless V=1 is
# used. $(v) will expand to @ when silent and to nothing when not
# silent.
v = $(v_@AM_V@)
v_ = $(v_@AM_DEFAULT_V@)
v_0 = @
vecho = $(v)echo "  "

linux_ZIMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/zImage
linux_IMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/Image
linux_SIGNED_IMAGE=@DEPLOY_BUILD_PATH@/boot/boot_sigheader.img.encrypt
BIN_RELEASE_DIR=d3-jetson-bsp-bin-$(VERSION_NAME)
BIN_RELEASE_FILE=artifacts/d3-jetson-bsp-bin-$(VERSION_NAME).tar.gz
SRC_RELEASE_FILE=artifacts/d3-jetson-bsp-src-$(VERSION_NAME).tar.gz
RSYNC_OPTS=-ar --exclude=.git --no-owner --no-group $(if $V,--progress)
HASH_FILE=d3-jetson-bsp-bin-$(VERSION_NAME).sha1
DTB_BUILD_PATH=@LINUX_BUILD_PATH@/arch/arm64/boot/dts/

# Custom kernel configuration
KCONFIG_ASSIGNMENT := $(if @KCONFIG_FILE@,KCONFIG_CONFIG='@KCONFIG_FILE@')

# Signed DTB target with path
SIGNED_DTB=$(DEPLOY_BUILD_PATH)/boot/$(patsubst %.dtb,%_sigheader.dtb.encrypt,$(notdir @DTB_FILE@))

# Chip ID passed into Tegra flash tools when signing DTB
if XAVIER
FLASH_CHIP_ID=0x19
SYNC_TARGETS=sync-modules sync-boot sync-debug sync-dtb sync-kernel
else
FLASH_CHIP_ID=0x18
SYNC_TARGETS=sync-modules sync-boot sync-debug sync-dtb
endif


# We need a SINGLE build id to be embedded in the DTB and the
# kernel. Using ccflags-y would result in just about everything being
# rebuilt every single time. Instead we'll only rebuild a single
# module, build-id.


# Make all of the directories we need exist.
$(shell  mkdir -p "@DEPLOY_BUILD_PATH@" "@LINUX_BUILD_PATH@" "@DEPLOY_BUILD_PATH@/boot")


.PHONY: linux-defconfig linux-dtbs linux-clean
.PHONY: reboot shutdown
.PHONY: deploy deploy-clean flash-dtb
.PHONY: sync sync-boot sync-modules sync-debug sync-dtb
.PHONY: release release-src release-bin bin-kernel doc
.PHONY: sign-dtb
.PHONY: show-config clean-local


define HELP_TEXT
	all             - alias for deploy which builds everything
	clean           - deletes build artifacts in the kernel and deployment trees
	deploy          - builds kernel and DTB
	deploy-clean    - deletes build artifacts in the deployment tree
	sync            - rsyncs kernel, kernel modules, and dtb to target
	sync-debug      - rsyncs debug/ to target
	sync-kernel     - (Xavier only) copies signed kernel image to device and dd's it to the kernel memory partition
	sync-modules    - rsyncs kernel modules to target
	sync-dtb        - copies signed dtb to device and dd's it to the kernel-dtb memory partition
	flash-dtb       - flashes dtb to target via USB (device in recovery mode)
	flash-kernel    - (Xavier only) flashes kernel to target via USB (device in recovery mode)
	linux-defconfig - prepares kernel with default D3 configuration
	linux-menuconfig- launches curses based kernel config editor
	linux-dtbs      - builds dtbs
	linux-dtbs-install- installs dtbs
	linux-clean     - deletes build artifacts in the kernel tree
	reboot          - reboots target
	shutdown        - halts target
	sign-dtb        - signs dtb file with L4T
	sign-kernel     - (Xavier only) signs kernel image with L4T
	bin-kernel      - creates kernel .deb package
	show-config     - show the configuration supplied to configure
endef
export HELP_TEXT
help:
	$(v)echo "$$HELP_TEXT"



clean-local: linux-clean deploy-clean


# Probably need different config based on target type.
# Can we build all supported?
linux-defconfig:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ tegra_defconfig

linux-dtbs-install:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ dtbs_install

linux-dtbs:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ dtbs

linux-menuconfig:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ menuconfig


linux-clean:
	$(vecho) "RM     @LINUX_BUILD_PATH@"
	$(v)rm -Rf "@LINUX_BUILD_PATH@"


# Just compile the kernel.  Provided for use by Makefiles in /mk .
_compile_kernel:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ \
		$(if ${COMPILE_KERNEL_ARGS}, ${COMPILE_KERNEL_ARGS}, \
		zImage Image modules dtbs)

# Default target
deploy:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) _compile_kernel
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ modules_install headers_install
	$(v)install -D "$(linux_ZIMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D "$(linux_IMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D @LINUX_BUILD_PATH@/arch/arm64/boot/dts/*.dtb "@DEPLOY_BUILD_PATH@/boot/"
	$(v)echo "Done"

deploy-clean:
	$(vecho) "RM     @DEPLOY_BUILD_PATH@"
	$(v)rm -Rf "@DEPLOY_BUILD_PATH@"


reboot:
	$(vecho) "REBOOT   @TARGET_HOST@"
	$(v)ssh @TARGET_HOST@ sudo shutdown -r now || true


shutdown:
	$(vecho) "HALT     @TARGET_HOST@"
	$(v)ssh @TARGET_HOST@ sudo shutdown -P now || true


sync-debug:
	$(vecho) "SYNC     debug/"
	$(v)rsync debug/ @TARGET_HOST@:debug/ $(RSYNC_OPTS)


sync-boot:
	$(vecho) "SYNC     boot/"
	$(v)rsync @DEPLOY_BUILD_PATH@/boot/ $(TARGET_HOST):/boot/ $(RSYNC_OPTS)

sync-modules:
	$(vecho) "SYNC     lib/modules/"
	$(v)for d in @DEPLOY_BUILD_PATH@/lib/modules/*; do \
		[ "$(V)" ] && echo "         $${d}"; \
		rsync $${d}/ @TARGET_HOST@:/lib/modules/$$(basename $${d}) \
			$(RSYNC_OPTS) $(if ${RSYNC_NO_DELETE},,--delete); \
	done
	$(v)ssh @TARGET_HOST@ sync

sync: $(SYNC_TARGETS)
	$(v)echo "Run 'make reboot' when ready"

#This builds all dtbs using kernel make target "dtbs"
%.dtb:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ dtbs

$(DEPLOY_BUILD_PATH)/boot/%_sigheader.dtb.encrypt: $(DTB_BUILD_PATH)/%.dtb
	$(vecho) "SIGN      $^"
	$(v)cd $(DEPLOY_BUILD_PATH)/boot; \
		@L4TDIR@/bootloader/tegraflash.py --chip $(FLASH_CHIP_ID) \
		--cmd "sign $^" --skipuid;

sign-dtb: $(SIGNED_DTB)

sync-dtb: sign-dtb
	$(vecho) "SYNC     $(SIGNED_DTB)"
	$(v)scp $(SIGNED_DTB) @TARGET_HOST@:~/device_tree.dtb.encrypt
	$(v)ssh @TARGET_HOST@ sudo dd if=device_tree.dtb.encrypt of=/dev/disk/by-partlabel/kernel-dtb

if XAVIER
sign-kernel:
	$(vecho) "SIGN     $(linux_IMAGE)"
	$(v)cd @L4TDIR@/bootloader; \
		./mkbootimg --kernel $(linux_IMAGE) --ramdisk initrd --board mmcblk0p1 --output $(DEPLOY_BUILD_PATH)/boot/boot.img;
	$(v)cd $(DEPLOY_BUILD_PATH)/boot; \
		@L4TDIR@/bootloader/tegraflash.py --chip $(FLASH_CHIP_ID) \
		--cmd "sign boot.img" --skipuid > /dev/null;

sync-kernel: sign-kernel
	$(vecho) "SYNC     $(linux_SIGNED_IMAGE)"
	$(v)scp $(linux_SIGNED_IMAGE) @TARGET_HOST@:~/boot.img.encrypt > /dev/null
	$(v)ssh @TARGET_HOST@ sudo dd bs=10MB if=boot.img.encrypt of=/dev/disk/by-partlabel/kernel > /dev/null 2>&1
# On Tx2 the kernel is loaded from /boot on the rootfs.
# On Xavier (only) the kernel is written to a raw partition.
flash-kernel:
	$(vecho) "FLASH     $(linux_IMAGE)"
	$(v)(cd "@L4TDIR@"; sudo ./flash.sh -k kernel -K "$(linux_IMAGE)" jetson-@SYSTEM_TYPE@ mmcblk0p1 > /dev/null; )
#On Tx2, the kernel cmdline is loaded from extlinux.conf
#On Xavier, kernel cmdline is baked into the device tree by flash.sh
flash-dtb:
	$(vecho) "XAVIER FLASH     @DTB_FILE@"
	$(vecho) "CMDLINE          $${XAVIER_K_CMDLINE}"
	$(v)(cd "@L4TDIR@"; sudo ./flash.sh -k kernel -C "$${XAVIER_K_CMDLINE}" -k kernel-dtb -d "@DTB_FILE@" jetson-@SYSTEM_TYPE@ mmcblk0p1 > /dev/null; )
else
flash-dtb:
	$(vecho) "FLASH     @DTB_FILE@"
	$(v)(cd "@L4TDIR@"; sudo ./flash.sh -k kernel-dtb -d "@DTB_FILE@" jetson-@SYSTEM_TYPE@ mmcblk0p1 > /dev/null; )
endif


show-config:
	$(vecho) "     -Configuration-"
	$(vecho) "System Type      : @SYSTEM_TYPE@"
	$(vecho) "Target Host      : @TARGET_HOST@"
	$(vecho) "Kernel config    : $(if @KCONFIG_FILE@,@KCONFIG_FILE@,<default>)"
	$(vecho) "DTB File         : @DTB_FILE@"
	$(vecho) "L4T Directory    : @L4TDIR@"

# Set the main target - automake automatically adds 'all: all-local'
all-local: deploy
