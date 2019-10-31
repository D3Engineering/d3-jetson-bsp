# single_module.mk: Make and sync only a single kernel module.
# For development use only.  Invoke from the BSP top-level directory as:
#
#     make -f mk/single_module.mk M=module/directory
#
# NOTES:
#     - the make does not change the build ID
#     - the rsync omits the --delete option for speed.

AM_DEFAULT_VERBOSITY = 0
v = $(v_$(V))
v_ = $(v_$(AM_DEFAULT_VERBOSITY))
v_0 = @
vecho = $(v)echo "  "

.PHONY: all build sync

all: build sync

build:
	@# Require the M=... assignment
	$(v)$(if ${M},true,echo "Usage: make -f mk/single_module.mk M=module/directory [build] [sync]"; false)

	@# Build the specified module
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) _compile_kernel \
		COMPILE_KERNEL_ARGS="M='`realpath '${M}'`' modules modules_install"

sync:
	@# Require the M=... assignment
	$(v)$(if ${M},true,echo "Usage: make -f mk/single_module.mk M=module/directory [build] [sync]"; false)

	@# rsync just the changed files over to the board
	$(v)+$(KCONFIG_ASSIGNMENT) $(MAKE) sync-modules RSYNC_NO_DELETE=1
