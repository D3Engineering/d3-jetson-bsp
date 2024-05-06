# NOTICE

The D3 Jetson BSP has been replaced by D3 Jetson Modules, which is a package that contains loadable kernel modules for D3's cameras and device tree overlays for D3's interface and carrier boards.  The supported Jetpack version is 5.1.3 as of 5/1/2024.  Visit D3engineering.com to purchase D3's Jetson supported products and D3 Jetson Modules source code and documentation will be provided as a download with your purchase.  Please contact sales@d3engineering.com with any questions.  Thank you!!

# Overview

This project serves two purposes:

1. To make building and developing for the Nvidia Jetson kernel easier.

2. To publish D3 Jetson modifications to the public in a clean and
   easy-to-use manner.

Please note that this hasn't been used extensively with Nano and
likely has some issues.

# Getting the Code

This project uses git submodules to pull in repositories publish by
Nvidia. The use of git submodules requires extra steps that are not
needed for git repositories that do not use submodules.

After the initial clone choose a branch corresponding to the release
you would like to build. The master branch only provides this
documentation; all working code is contained in branches.  D3 specific
code will be added on different branches in the near future. The
following will checkout the nvidia branch with Linux for Tegra r32.2.1
submodules.

```
git checkout nvidia/r32.2.1
git submodule update --init
```

The `--init` flag is only needed one time.

Anytime a branch is changed be sure to update the submodules as git
does not do this automatically by default!

```
git submodule update
```

Use of git-prompt is recommended as it will alert you when your
submodules need updating.

# Dependencies

 * Ubuntu 16.04 or 18.04.

 * Linux For Tegra host-side tools as installed by Nvidia's SDK Manager

 * rsync: for efficiently transferring files to the Jetson target

 * TODO: track down the required Ubuntu packages


# Building

Autoconf is utilized to make it easier to choose different systems and
various other options. The basic steps required are below.

```
./bootstrap
./configure
make linux-defconfig
make -j8
```

There are options for customizing the build. If, for example, you are
developing for Xavier you would execute:

```
./bootstrap
./configure --with-system-type=xavier
make linux-defconfig
make -j8
```


## Configure Options

--with-system-type

The choices are xavier, tx2, or nano. The default is tx2. This option
influences the L4T directory. If you have installed the L4T host
utilities in the default locations from Nvidia's SDK Manager then this
will choose the correct path and there is no need to specify
--with-l4t.

--with-dtb

Specifies a path to a device tree blob. This is used for the make
target 'flash-dtb'. Normally a path to the build directory is used:

```
./configure --with-dtb=$(realpath build/deploy/boot/tegra186-quill-e3313-1000-a00-00-e2598.dtb)
```

--with-target-host

Normally this is not required. This specifies an IP address or
hostname that is used when sending files to the target Jetson
device. The default is 192.168.55.1 which is the default IP address
for the USB connection to the Jetson.

--with-kconfig

Optionally specify a non-default location for the kernel configuration
file.

--with-l4t

Normally this is not required as a suitable default is chosen based on
the system type (see --with-system-type). If you have installed L4T in
a non-standard location you can use this to direct the build tools to
the correct location. This situation might arise, for example, on a
dedicated build server.

# Installing

The only step required to install the kernel, modules, and dtb is
`make sync`. This target copies the required files to the target
device, taking into account differences between Xavier, Tx2, and Nano.


# Makefile Targets

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

