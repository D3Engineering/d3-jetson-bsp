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

# $(suppress_unless_v) will redirect stdout to /dev/null unless V=1.
suppress_unless_v = $(suppress_unless_v_@AM_V@)
suppress_unless_v_ = $(suppress_unless_v_@AM_DEFAULT_V@)
suppress_unless_v_0 = > /dev/null

LINUX_BUILD_CONFIG=@LINUX_BUILD_PATH@/.config
D3_DEFAULT_CONFIG=kernel/d3/d3_defconfig
linux_ZIMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/zImage
linux_IMAGE=@LINUX_BUILD_PATH@/arch/arm64/boot/Image
linux_SIGNED_IMAGE=@DEPLOY_BUILD_PATH@/boot/boot_sigheader.img.encrypt
LINUX_PACKAGE_FILE=kernel/linux-image-4.9.140+@D3_JETSON_BSP_VERSION@-@CUSTOMER@+_1_arm64.deb
BIN_RELEASE_DIR=d3-jetson-bsp-bin-@VERSION_NAME@
BIN_RELEASE_FILE=@DEPLOY_BUILD_PATH@/d3-jetson-bsp-bin-@VERSION_NAME@.tar.gz
SRC_RELEASE_FILE=@DEPLOY_BUILD_PATH@/d3-jetson-bsp-src-@VERSION_NAME@.tar.gz
ZIP_RELEASE_FILE=artifacts/d3-jetson-bsp-@VERSION_NAME@.zip
ZIP_RELEASE_STAGING_DIR=d3-jetson-bsp-@VERSION_NAME@
RELNOTES_PDF_FILE=doc/D3-Jetson-BSP-Release-Notes.pdf
RSYNC_OPTS=-ar --exclude=.git --no-owner --no-group $(if $V,--progress)
# Filename of the deb for d3-jetson-util.  Cannot be changed
UTIL_PACKAGE_FILE=d3-jetson-util_@VERSION_NAME@-1_arm64.deb
HASH_FILE=d3-jetson-bsp-bin-@VERSION_NAME@.sha1
DTB_BUILD_PATH=@LINUX_BUILD_PATH@/arch/arm64/boot/dts/
NVLIBS_PATH=debug/nvlibs
BOOT_LOGO_FILE=d3-jetson-util/target-overlay/boot/boot-logo.blob
TARGET_NVLIBS_PATH=/usr/lib/aarch64-linux-gnu/tegra/

# Custom kernel configuration
KCONFIG_ASSIGNMENT := $(if @KCONFIG_FILE@,KCONFIG_CONFIG='@KCONFIG_FILE@')

# Signed DTB target with path
SIGNED_DTB=$(DEPLOY_BUILD_PATH)/boot/$(patsubst %.dtb,%_sigheader.dtb.encrypt,$(notdir @DTB_FILE@))

# Chip ID passed into Tegra flash tools when signing DTB
if XAVIER
FLASH_CHIP_ID=0x19
SYNC_TARGETS=sync-modules sync-boot sync-logo sync-nvlibs sync-debug sync-dtb sync-kernel
else
FLASH_CHIP_ID=0x18
SYNC_TARGETS=sync-modules sync-boot sync-logo sync-nvlibs sync-debug sync-dtb
endif


# We need a SINGLE build id to be embedded in the DTB and the
# kernel. Using ccflags-y would result in just about everything being
# rebuilt every single time. Instead we'll only rebuild a single
# module, build-id.
#
# Leading spaces required to hide this from automake --- see
# https://patchwork.kernel.org/patch/6709921/
#
# Can't use make's ?= because it maps to =, not :=.
 ifeq ($(origin D3_JETSON_BSP_BUILDID), undefined)
 D3_JETSON_BSP_BUILDID := $(shell uuidgen -r)
 endif
export D3_JETSON_BSP_BUILDID

$(warning ---------------- Build ID is $(D3_JETSON_BSP_BUILDID))

D3_IMX390_HDR_ENABLE := $(shell if [ $$(@KERNEL_DIR@/scripts/config --file $(LINUX_BUILD_PATH)/.config -s d3_imx390_hdr_enable) == 'y' ]; then echo 1; else echo 0; fi)
D3_OV10640_HDR_ENABLE := $(shell if [ $$(@KERNEL_DIR@/scripts/config --file $(LINUX_BUILD_PATH)/.config -s d3_ov10640_hdr_enable) == 'y' ]; then echo 1; else echo 0; fi)

# These flags are passed to cpp when building the dtb
export D3_DTCCPP_FLAGS=-DD3_JETSON_BSP_BUILDID='"$(D3_JETSON_BSP_BUILDID)"' -DD3_IMX390_HDR_ENABLE=$(D3_IMX390_HDR_ENABLE) -DD3_OV10640_HDR_ENABLE=$(D3_OV10640_HDR_ENABLE)

# Make all of the directories we need exist.
$(shell  mkdir -p "$(BIN_RELEASE_DIR)/doc" "$(BIN_RELEASE_DIR)/dtb" "@DEPLOY_BUILD_PATH@" "@LINUX_BUILD_PATH@" "@DEPLOY_BUILD_PATH@/boot" artifacts)


.PHONY: linux-defconfig linux-dtbs linux-clean
.PHONY: reboot shutdown
.PHONY: deploy deploy-clean flash-dtb
.PHONY: sync sync-boot sync-modules sync-debug sync-dtb
.PHONY: release release-src release-bin release-zip bin-kernel doc
.PHONY: sign-dtb
.PHONY: show-config clean-local

# distribution files: listed as phony so we don't have to
# expressly track their prerequisites.
.PHONY: $(UTIL_PACKAGE_FILE) $(BIN_RELEASE_FILE) $(LINUX_PACKAGE_FILE)
.PHONY: $(SRC_RELEASE_FILE)


define HELP_TEXT
	all             - alias for deploy which builds everything
	clean           - deletes build artifacts in the kernel and deployment trees
	deploy          - builds kernel and DTB
	deploy-clean    - deletes build artifacts in the deployment tree
	sync            - rsyncs kernel, kernel modules, and dtb to target
	sync-debug      - rsyncs debug/ to target
	sync-kernel     - (Xavier only) copies signed kernel image to device and writes it to the kernel partition
	sync-modules    - rsyncs kernel modules to target
	sync-nvlibs     - copies nvidia libraries that have been modified from release to target
	sync-dtb        - copies signed dtb to device and writes it to the kernel-dtb partition
	sync-logo       - copies boot logo to device and writes it to the BMP partition
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
	release         - creates a release (binary and source)
	release-src     - prepares a source release tarball
	release-bin     - builds a binary release
	release-zip     - build a ZIP file of a release
	bin-kernel      - creates kernel .deb package
	doc             - builds documentation (requires pandoc and latex)
	show-config     - show the configuration supplied to configure
endef
export HELP_TEXT
help:
	$(v)echo "$$HELP_TEXT"

doc:
	$(v)+$(MAKE) -C doc
# '+' for consistency with the other sub-make invocations.  Note that in
# GNU Make, use of '$(MAKE)' implies '+'.
# See https://www.gnu.org/software/make/manual/html_node/Testing-Flags.html#Testing-Flags
# and https://www.gnu.org/software/make/manual/html_node/MAKE-Variable.html#MAKE-Variable

# Build the .deb package of d3-jetson-util.  The CC=true suppresses a
# spurious warning - see
# https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894965#40
$(UTIL_PACKAGE_FILE):
	$(AM_V_GEN)
	$(v)( cd d3-jetson-util; CC=true dpkg-buildpackage -b -us -uc -a arm64 \
			-t aarch64-linux-gnu > /dev/null )
	$(v)cp $(UTIL_PACKAGE_FILE) "$(BIN_RELEASE_DIR)"
d3-jetson-util: $(UTIL_PACKAGE_FILE)


$(BIN_RELEASE_FILE): bin-kernel doc d3-jetson-util
	$(AM_V_GEN)
	$(vecho) "ID      $(D3_JETSON_BSP_BUILDID)"
	$(vecho) "CP      PDFs"
	$(v)cp "$(RELNOTES_PDF_FILE)" "$(BIN_RELEASE_DIR)"/doc/
	$(vecho) "CP      DTBs"
	$(v)find kernel/kernel-4.9/arch/arm64 -type f -name '*.dtb' -exec cp {} "$(BIN_RELEASE_DIR)"/dtb/ ';'
	$(vecho) "CP      SIGNED DTBs"
	$(v)find kernel/kernel-4.9/arch/arm64 -type f -name '*_sigheader.dtb.encrypt' -exec cp {} "$(BIN_RELEASE_DIR)"/dtb/ ';'
	$(vecho) "CP      kernel package"
	$(v)cp "./l4t-submodules/$(LINUX_PACKAGE_FILE)" "$(BIN_RELEASE_DIR)"
	$(vecho) "GEN     $(HASH_FILE)"
	$(v)find "$(BIN_RELEASE_DIR)" -type f -exec sha1sum {} ';' >> "$(HASH_FILE)"
	$(v)mv "$(HASH_FILE)" "$(BIN_RELEASE_DIR)"
	$(vecho) "TGZ     $(BIN_RELEASE_DIR)"
	$(v)tar czf "$(BIN_RELEASE_FILE)" "$(BIN_RELEASE_DIR)"
release-bin: $(BIN_RELEASE_FILE)


$(LINUX_PACKAGE_FILE): bin/kernel-deb
	$(AM_V_GEN)
	$(v)bin/kernel-deb @CUSTOMER@ '@D3_JETSON_BSP_VERSION@'
	$(v)cp "./l4t-submodules/$(LINUX_PACKAGE_FILE)" "@DEPLOY_BUILD_PATH@/"
bin-kernel: $(LINUX_PACKAGE_FILE)


$(SRC_RELEASE_FILE):
	$(AM_V_GEN)
	$(v)bin/git-archive-all.sh \
	--verbose \
	--format tar.gz \
	--prefix d3-jetson-bsp-src-@VERSION_NAME@/ \
	--tree-ish $(GIT_BRANCH) \
	$(SRC_RELEASE_FILE)
release-src: $(SRC_RELEASE_FILE)


release: release-bin release-src

release-zip: release
	$(vecho) "ZIP     $(ZIP_RELEASE_FILE)"
	$(v)zip -j "$(ZIP_RELEASE_FILE)" \
		"$(BIN_RELEASE_FILE)" "$(SRC_RELEASE_FILE)" \
		"$(RELNOTES_PDF_FILE)"

clean-local: linux-clean deploy-clean


# Probably need different config based on target type.
# Can we build all supported?
linux-defconfig:
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ d3_defconfig

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
		CFLAGS_build-id.o=-DD3_JETSON_BSP_BUILDID='\"$(D3_JETSON_BSP_BUILDID)\"' zImage Image modules dtbs)

# Default target
deploy:
	touch kernel/d3/drivers/d3/build-id/build-id.c
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) _compile_kernel
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) @LINUX_ARGS@ modules_install headers_install
	$(v)install -D "$(linux_ZIMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D "$(linux_IMAGE)" "@DEPLOY_BUILD_PATH@/boot/"
	$(v)install -D @LINUX_BUILD_PATH@/arch/arm64/boot/dts/*.dtb "@DEPLOY_BUILD_PATH@/boot/"
	$(v)echo "Done - build ID $(D3_JETSON_BSP_BUILDID)"

deploy-clean:
	$(vecho) "RM     @DEPLOY_BUILD_PATH@"
	$(v)rm -Rf "@DEPLOY_BUILD_PATH@"


reboot:
	$(vecho) "REBOOT   @TARGET_HOST@"
	$(v)ssh @TARGET_HOST@ sudo shutdown -r now& exit || true


shutdown:
	$(vecho) "HALT     @TARGET_HOST@"
	$(v)ssh @TARGET_HOST@ sudo shutdown -P now& exit || true


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

sync-nvlibs:
	$(vecho) "SYNC     $(NVLIBS_PATH)"
	$(v) if [ -e $(NVLIBS_PATH) ]; then scp $(NVLIBS_PATH)/* $(TARGET_HOST):$(TARGET_NVLIBS_PATH); fi
	$(v)ssh @TARGET_HOST@ sync

sync-logo: $(BOOT_LOGO_FILE)
	$(vecho) "SYNC     $(BOOT_LOGO_FILE)"
	$(v)scp $(BOOT_LOGO_FILE) $(TARGET_HOST):debug/boot-logo.blob
	$(v)ssh @TARGET_HOST@ 'sudo dd if=debug/boot-logo.blob of=/dev/disk/by-partlabel/BMP && sync'

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
	$(vecho) "     -D3 Configuration-"
	$(vecho) "Version          : @VERSION_NAME@"
	$(vecho) "System Type      : @SYSTEM_TYPE@"
	$(vecho) "Target Host      : @TARGET_HOST@"
	$(vecho) "Kernel config    : $(if @KCONFIG_FILE@,@KCONFIG_FILE@,<default>)"
	$(vecho) "DTB File         : @DTB_FILE@"
	$(vecho) "L4T Directory    : @L4TDIR@"
	$(v) ( id="$$(strings build/kernel/drivers/d3/build-id/build-id.o 2>/dev/null \
			| grep -Ei '^([0-9a-f]{4,8}-){4}[0-9a-f]{12}' 2>/dev/null)" && \
		echo "   Build ID of most recent build: $${id}" ) ; \
		true

# Set the main target - automake automatically adds 'all: all-local'
all-local: deploy
