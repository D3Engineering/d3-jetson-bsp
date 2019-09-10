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

VERSION_NAME=@D3_JETSON_BSP_VERSION@@VERSION_SUFFIX@

linux_ZIMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/zImage
linux_IMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/Image
LINUX_PACKAGE_FILE=kernel/linux-image-4.9.140+@D3_JETSON_BSP_VERSION@-@CUSTOMER@+_1_arm64.deb
BIN_RELEASE_DIR=d3-jetson-bsp-bin-$(VERSION_NAME)
BIN_RELEASE_FILE=artifacts/d3-jetson-bsp-bin-$(VERSION_NAME).tar.gz
SRC_RELEASE_FILE=artifacts/d3-jetson-bsp-src-$(VERSION_NAME).tar.gz
RSYNC_OPTS=-ar --exclude=.git --no-owner --no-group
UTIL_PACKAGE_FILE=d3-jetson-util_$(VERSION_NAME)-1_arm64.deb
HASH_FILE=d3-jetson-bsp-bin-$(VERSION_NAME).sha1
DTB_BUILD_PATH=@LINUX_BUILD_PATH@/arch/arm64/boot/dts/


# DTBs to Sign
TX2_DTB_BASENAMES=d3-ncic-ov10640.dtb d3-ncic-imx390.dtb d3-ncic-imx390rcm.dtb \
	d3-rsp-fpdlink-ov10640.dtb d3-rsp-fpdlink-imx390.dtb d3-rsp-fpdlink-imx390rcm.dtb
XAVIER_DTB_BASENAMES=d3-xavier-2xocto-ov10640.dtb d3-xavier-2xocto-imx390.dtb d3-xavier-2xocto-imx390rcm.dtb d3-xavier-ncic-ov10640.dtb d3-xavier-ncic-imx390.dtb d3-xavier-ncic-imx390rcm.dtb
# Signed DTB targets with path
TX2_SIGNED_DTBS=$(addprefix $(DEPLOY_BUILD_PATH)/boot/,$(patsubst %.dtb,%.dtb.encrypt,$(TX2_DTB_BASENAMES)))
XAVIER_SIGNED_DTBS=$(addprefix $(DEPLOY_BUILD_PATH)/boot/,$(patsubst %.dtb,%.dtb.encrypt,$(XAVIER_DTB_BASENAMES)))



# We need a SINGLE build id to be embedded in the DTB and the
# kernel. Using ccflags-y would result in just about everything being
# rebuilt every single time. Instead we'll only rebuild a single
# module, build-id.
D3_JETSON_BSP_BUILDID := '\"$(shell uuidgen -r)\"'
export D3_JETSON_BSP_BUILDID


# Make all of the directories we need exist.
$(shell  mkdir -p "$(BIN_RELEASE_DIR)/doc" "$(BIN_RELEASE_DIR)/dtb" "@DEPLOY_BUILD_PATH@" "@LINUX_BUILD_PATH@" "@DEPLOY_BUILD_PATH@/boot" artifacts)


.PHONY: linux-defconfig linux-dtbs linux-clean linux-dtbs-install
.PHONY: reboot shutdown
.PHONY: deploy deploy-clean flash-dtb
.PHONY: sync sync-boot sync-modules sync-debug
.PHONY: release release-src release-bin bin-kernel doc
.PHONY: sign-dtbs
.PHONY: $(UTIL_PACKAGE_FILE) $(BIN_RELEASE_FILE) $(LINUX_PACKAGE_FILE)
.PHONY: $(SRC_RELEASE_FILE)
.PHONY: show-config


define HELP_TEXT
	all             - alias for deploy which builds everything
	deploy          - builds kernel and DTB
	deploy-clean    - deletes kernel, DTB, and object code
	sync            - rsyncs kernel and kernel modules to target
	sync-debug      - rsyncs debug/ to target
	module-sync     - rsyncs kernel modules to target
	flash-dtb       - flashes dtb to target via USB (device in recovery mode)
	flash-kernel    - (Xavier only) flashes kernel to target via USB (device in recovery mode)
	linux-defconfig - prepares kernel with default D3 configuration
	linux-menuconfig- launches curses based kernel config editor
	linux-dtbs      - builds dtbs
	linux-dtbs-install- installs dtbs
	linux-clean     - cleans kernel
	reboot          - reboots target
	shutdown        - halts target
	sign-dtbs       - signs all built dtb files with L4T
	release         - creates a release (binary and source)
	release-src     - prepares a source release tarball
	release-bin     - builds a binary release
	bin-kernel      - creates kernel .deb package
	doc             - builds documentation (requires pandoc and latex)
	show-config     - show the configuration supplied to configure
endef
export HELP_TEXT
help:
	$(v)echo "$$HELP_TEXT"

doc:
	$(v)$(MAKE) -C doc


$(UTIL_PACKAGE_FILE):
	$(AM_V_GEN)
	$(v)(cd d3-jetson-util; dpkg-buildpackage -b -us -uc -aarm64;)
	$(v)cp $(UTIL_PACKAGE_FILE) "$(BIN_RELEASE_DIR)"
d3-jetson-util: $(UTIL_PACKAGE_FILE)


$(BIN_RELEASE_FILE): bin-kernel doc d3-jetson-util
	$(AM_V_GEN)
	$(vecho) "ID      $(D3_JETSON_BSP_BUILDID)"
	$(vecho) "CP      PDFs"
	$(v)cp doc/D3-Jetson-BSP-Release-Notes.pdf "$(BIN_RELEASE_DIR)"/doc/
	$(v)cp doc/D3-Jetson-BSP-Release-Notes.pdf artifacts/
	$(vecho) "CP      DTBs"
	$(v)find kernel/kernel-4.9/arch/arm64 -type f -name '*.dtb' -exec cp {} "$(BIN_RELEASE_DIR)"/dtb/ ';'
	$(vecho) "CP      SIGNED DTBs"
	$(v)find kernel/kernel-4.9/arch/arm64 -type f -name '*_sigheader.dtb.encrypt' -exec cp {} "$(BIN_RELEASE_DIR)"/dtb/ ';'
	$(vecho) "CP      kernel package"
	$(v)cp "$(LINUX_PACKAGE_FILE)" "$(BIN_RELEASE_DIR)"
	$(vecho) "GEN     $(HASH_FILE)"
	$(v)find "$(BIN_RELEASE_DIR)" -type f -exec sha1sum {} ';' >> "$(HASH_FILE)"
	$(v)mv "$(HASH_FILE)" "$(BIN_RELEASE_DIR)"
	$(vecho) "TGZ     $(BIN_RELEASE_DIR)"
	$(v)tar czf "$(BIN_RELEASE_FILE)" "$(BIN_RELEASE_DIR)"
release-bin: $(BIN_RELEASE_FILE)


$(LINUX_PACKAGE_FILE):
	$(AM_V_GEN)
	$(v)bin/kernel-deb @CUSTOMER@ '@D3_JETSON_BSP_VERSION@'
	$(v)cp "$(LINUX_PACKAGE_FILE)" "@DEPLOY_BUILD_PATH@/"
bin-kernel: $(LINUX_PACKAGE_FILE)


$(SRC_RELEASE_FILE):
	$(AM_V_GEN)
	$(v)bin/git-archive-all.sh \
	--verbose \
	--format tar.gz \
	--prefix d3-jetson-bsp-src-$(VERSION_NAME)/ \
	--tree-ish $(GIT_BRANCH) \
	$(SRC_RELEASE_FILE)
release-src: $(SRC_RELEASE_FILE)


release: release-bin release-src


# Probably need different config based on target type.
# Can we build all supported?
linux-defconfig:
	$(v)+$(MAKE) @LINUX_ARGS@ d3_defconfig

linux-dtbs-install:
	$(v)+$(MAKE) @LINUX_ARGS@ dtbs_install


linux-dtbs:
	$(v)+$(MAKE) @LINUX_ARGS@ dtbs


linux-menuconfig:
	$(v)+$(MAKE) @LINUX_ARGS@ menuconfig


linux-clean:
	$(vecho) "RM     @LINUX_BUILD_PATH@"
	$(v)rm -Rf "@LINUX_BUILD_PATH@"


deploy:
	touch kernel/d3/drivers/d3/build-id/build-id.c
	$(v)+$(MAKE) @LINUX_ARGS@ CFLAGS_build-id.o=-DD3_JETSON_BSP_BUILDID='\"$(D3_JETSON_BSP_BUILDID)\"' zImage Image modules dtbs
	$(v)+$(MAKE) @LINUX_ARGS@ modules_install headers_install
	$(v)install -D "$(linux_ZIMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D "$(linux_IMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D @LINUX_BUILD_PATH@/arch/arm64/boot/dts/*.dtb "@DEPLOY_BUILD_PATH@/boot/"

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
		rsync $${d}/ @TARGET_HOST@:/lib/modules/$$(basename $${d}) $(RSYNC_OPTS) --delete; \
	done
	$(v)ssh @TARGET_HOST@ sync

sync: sync-modules sync-boot sync-debug

#This builds all dtbs using kernel make target "dtbs"
%.dtb:
	$(v)+$(MAKE) @LINUX_ARGS@ dtbs

$(DEPLOY_BUILD_PATH)/boot/%.dtb.encrypt: $(DTB_BUILD_PATH)/%.dtb
	$(vecho) "SIGN      $^"
	$(v)cd $(DEPLOY_BUILD_PATH)/boot; \
	if [[ " ${XAVIER_DTB_BASENAMES} " =~ " $$(basename $^) " ]]; then \
		@L4TDIR@/bootloader/tegraflash.py --chip 0x19 \
			--cmd "sign $^" --skipuid; \
	fi; \
	if [[ " ${TX2_DTB_BASENAMES} " =~ " $$(basename $^) " ]]; then \
		@L4TDIR@/bootloader/tegraflash.py --chip 0x18 \
			--cmd "sign $^" --skipuid; \
	fi

sign-dtbs: $(TX2_SIGNED_DTBS) $(XAVIER_SIGNED_DTBS)

flash-dtb:
	$(vecho) "FLASH     @DTB_FILE@"
	$(v)(cd "@L4TDIR@"; sudo ./flash.sh -k kernel-dtb -d "@DTB_FILE@" jetson-@SYSTEM_TYPE@ mmcblk0p1 > /dev/null; )

# On Tx2 the kernel is loaded from /boot on the rootfs.
# On Xavier (only) the kernel is written to a raw partition.
if XAVIER
flash-kernel:
	$(vecho) "FLASH     $(linux_IMAGE)"
	$(v)(cd "@L4TDIR@"; sudo ./flash.sh -k kernel -K "$(linux_IMAGE)" jetson-@SYSTEM_TYPE@ mmcblk0p1 > /dev/null; )
endif


show-config:
	$(vecho) "     -D3 Configuration-"
	$(vecho) "System Type      : @SYSTEM_TYPE@"
	$(vecho) "Target Host      : @TARGET_HOST@"
	$(vecho) "DTB File         : @DTB_FILE@"
	$(vecho) "L4T Directory    : @L4TDIR@"


all-local: deploy
