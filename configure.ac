dnl A configure.ac for working with the Jetson platform from Nvidia.
dnl See Makefile.am or README.md for more information
dnl Copyright (c) 2019--2022 D3 Engineering, LLC.

dnl === Load version info ====================================================

m4_define(d3_jetson_bsp_triplet, m4_esyscmd_s([cat version.txt]))
m4_define(d3_jetson_bsp_list, m4_split(d3_jetson_bsp_triplet, [[.]]))
m4_define(d3_jetson_bsp_major, m4_argn(1, d3_jetson_bsp_list))
m4_define(d3_jetson_bsp_minor, m4_argn(2, d3_jetson_bsp_list))
m4_define(d3_jetson_bsp_patch, m4_argn(3, d3_jetson_bsp_list))
m4_define(customer, m4_esyscmd_s([cat customer.txt]))

dnl === Load up Linux Kernel version info ====================================

m4_define(d3_linux_kernel_version, m4_esyscmd_s([cat kernel/kernel-*/Makefile | grep VERSION | head -n 1 | sed 's/.* = //g']))
m4_define(d3_linux_kernel_patch, m4_esyscmd_s([cat kernel/kernel-*/Makefile | grep PATCHLEVEL | head -n 1 | sed 's/.* = //g']))
m4_define(d3_linux_kernel_sublevel, m4_esyscmd_s([cat kernel/kernel-*/Makefile | grep SUBLEVEL | head -n 1 | sed 's/.* = //g']))
m4_define(d3_linux_kernel_triplet, d3_linux_kernel_version.d3_linux_kernel_patch.d3_linux_kernel_sublevel)

dnl === Basics ===============================================================

AC_INIT([D3 Jetson BSP], [d3_jetson_bsp_triplet])
AC_PREREQ(2.69)
AC_COPYRIGHT([Copyright (C) 2019--2022 D3 Engineering])

AC_SUBST([D3_JETSON_BSP_MAJOR], d3_jetson_bsp_major)
AC_SUBST([D3_JETSON_BSP_MINOR], d3_jetson_bsp_minor)
AC_SUBST([D3_JETSON_BSP_PATCH], d3_jetson_bsp_patch)
AC_SUBST([D3_JETSON_BSP_VERSION], d3_jetson_bsp_triplet)

AC_DEFINE([D3_JETSON_BSP_MAJOR], d3_jetson_bsp_major, [Major Version])
AC_DEFINE([D3_JETSON_BSP_MINOR], d3_jetson_bsp_minor, [Minor Version])
AC_DEFINE([D3_JETSON_BSP_PATCH], d3_jetson_bsp_version_patch, [Patch Version])
AC_DEFINE([D3_JETSON_BSP_VERSION], d3_jetson_bsp_triplet, [Version])

AC_DEFINE([D3_LINUX_KERNEL_TRIPLET], d3_linux_kernel_triplet, [Linux Version Triplet])
AC_SUBST([D3_LINUX_KERNEL_TRIPLET], d3_linux_kernel_triplet)

AC_USE_SYSTEM_EXTENSIONS
AM_PROG_AR
AC_CONFIG_MACRO_DIRS([m4])
AM_INIT_AUTOMAKE([foreign])
AM_SILENT_RULES([yes])

dnl === Load L4T version info ================================================

m4_define(jp_ver_file, [./l4t-submodules/jp_version.txt])
m4_define(l4t_ver_file, [./l4t-submodules/l4t_version.txt])

AC_CHECK_FILE(jp_ver_file, [],
    [AC_MSG_ERROR([Cannot find jp_ver_file --- did you run "git submodule update --init --recursive"?])])
AC_CHECK_FILE(l4t_ver_file, [],
    [AC_MSG_ERROR([Cannot find l4t_ver_file --- did you run "git submodule update --init --recursive"?])])

m4_define(jp_version_number, m4_esyscmd_s(cat "jp_ver_file"))
m4_define(l4t_version, m4_esyscmd_s(cat "l4t_ver_file"))

AC_MSG_NOTICE([JetPack jp_version_number; L4T l4t_version])

AC_SUBST([JP_VERSION], jp_version_number)
AC_SUBST([L4T_VERSION], l4t_version)

dnl === Configure-time arguments =============================================

AC_ARG_WITH(
    [target-host],
    AS_HELP_STRING([--with-target-host=host], [specify Jetson target hostname]),
    [target_host=$withval],
    [target_host=192.168.55.1])

AC_ARG_WITH(
    [username],
    AS_HELP_STRING([--with-username=NAME], [specify username on the target.  The default NAME is "nvidia".  If you do not specify --with-username, your ssh configuration will pick a username.]),
    [ dnl Argument was given
        AS_CASE(["$withval"],
            [yes], [with_username=nvidia],
            [no], [with_username=]  dnl --without-username => default behaviour
        )
    ],
    [ dnl No argument was given - default behaviour
        with_username=
    ])

AC_ARG_WITH(
    [dtb],
    AS_HELP_STRING([--with-dtb=dtb-file], [specify dtb file for flash-dtb (use absolute path)]),
    [dtb_file=`realpath -q -m $withval`],
    [dtb_file=`realpath -q -m .`/build/deploy/boot/d3-orin-6x-serdes.dtb])

AC_ARG_WITH(
    [kconfig],
    AS_HELP_STRING([--with-kconfig=kernel-config-file], [specify a custom Linux .config file (use absolute path)]),
    [kconfig_file=$withval],
    [kconfig_file=])

AC_ARG_WITH(
    [l4t],
    AS_HELP_STRING([--with-l4t=l4t directory],
    [specify path to l4t directory (defaults based on target-system-type)]),
    [
        AS_IF([test -z "$withval"],
            [AC_MSG_ERROR([Please specify a directory when using "--with-l4t=..."])])
        dnl Expand tildes
        eval "withval=$withval"
        l4t_override_dir=`realpath -q -m $withval`
    ],
    [l4t_override_dir=])

dnl === Argument processing ==================================================

# Set the L4T path based on the target system
AM_CONDITIONAL(TEGRAFLASH, false) # Set to true to use the new tegraflash
AC_SUBST([D3_EXTLINUX_APPEND], []) # Adds extra options to extlinux.conf

AM_CONDITIONAL(ORIN, false)
AM_CONDITIONAL(XAVIER, false)
AM_CONDITIONAL(NX, false)
AC_ARG_WITH(
    [system-type],
    AS_HELP_STRING([--with-system-type=system], [orin, xavier, nx]),
    [
        case "${withval}" in
          orin)
            AM_CONDITIONAL(ORIN, true)
            AM_CONDITIONAL(TEGRAFLASH, true)
            AC_SUBST([D3_EXTLINUX_APPEND], ["console=ttyTCU0,115200 console=ttyAMA0,115200 console=tty0"]) # Add serial output
            system_type=orin
            l4t_dir=`realpath ~/nvidia/nvidia_sdk/JetPack_${JP_VERSION}_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra`
          ;;
          xavier)
            AM_CONDITIONAL(XAVIER, true)
            AM_CONDITIONAL(TEGRAFLASH, true)
            system_type=xavier
            l4t_dir=`realpath ~/nvidia/nvidia_sdk/JetPack_${JP_VERSION}_Linux_JETSON_AGX_XAVIER_TARGETS/Linux_for_Tegra`
          ;;
          nx)
            AM_CONDITIONAL(NX, true)
            l4t_dir=`realpath ~/nvidia/nvidia_sdk/JetPack_${JP_VERSION}_Linux_JETSON_XAVIER_NX_TARGETS/Linux_for_Tegra`
            system_type=xavier-nx-devkit-emmc
          ;;
          *)
            AC_MSG_ERROR([bad value ${withval} for --with-system-type (valid:orin, xavier, nx)])
          ;;
    esac],
    [
        jv="jp_version"
        AM_CONDITIONAL(ORIN, true)
        AM_CONDITIONAL(TEGRAFLASH, true)
        AC_SUBST([D3_EXTLINUX_APPEND], ["console=ttyTCU0,115200 console=ttyAMA0,115200 console=tty0"]) # Add serial output
        system_type=orin
        l4t_dir=`realpath ~/nvidia/nvidia_sdk/JetPack_${JP_VERSION}_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra`
    ])

# Change the L4T directory from the one inferred by the system type if
# it has been supplied.
if [ [ -n "$l4t_override_dir" ] ]; then
    l4t_dir="$l4t_override_dir"
fi

# Verify L4T directory exists, whether inferred or provided.
AC_MSG_CHECKING([L4T directory $l4t_dir])
if [ [ -z "$l4t_dir" ] ] ; then
      AC_MSG_ERROR([L4T dir is not defined --- try specifying "--with-l4t=..."])
elif [ [ ! -d "$l4t_dir" ] ] ; then
      AC_MSG_ERROR([L4T directory $l4t_dir does not exist --- did you specify an existent directory using "--with-l4t=..."?])
fi
AC_MSG_RESULT([found])

AC_CHECK_FILE([${l4t_dir}/flash.sh], [ ],
    [AC_MSG_WARN([flash.sh not found --- signing and syncing may not work])])

AC_SUBST([CUSTOMER], customer)
version_suffix=''
version_prefix=''
if [ [ "$CUSTOMER" != "d3" ] ]; then
    version_suffix="-customer"
    version_prefix="customer-"
fi
AC_SUBST([VERSION_SUFFIX], [$version_suffix])
AC_SUBST([VERSION_PREFIX], [$version_prefix])
AC_SUBST([VERSION_NAME], [$D3_JETSON_BSP_VERSION$VERSION_SUFFIX])

# Paths
kernel_dir=`realpath ./kernel/kernel-5.10`
m4_define(top_build_path, "$PWD/build")
AC_SUBST([TOP_BUILD_PATH], $top_build_path)
m4_define(linux_build_path, "$PWD/build/kernel")
AC_SUBST([LINUX_BUILD_PATH], $linux_build_path)
m4_define(deploy_build_path, "$PWD/build/deploy")
AC_SUBST([DEPLOY_BUILD_PATH], $deploy_build_path)
m4_define(artifact_path, "$PWD/artifacts")
AC_SUBST([ARTIFACT_PATH], $artifact_path)

# Compilation setup
m4_define(linux_cross_tool_prefix, "aarch64-buildroot-linux-gnu-")
AC_SUBST([LINUX_CROSS_TOOL_PREFIX], $linux_cross_tool_prefix)
m4_define(linux_cross32cc, "arm-linux-gnueabihf-gcc-5")
m4_define(linux_args, "-C "$kernel_dir" "O=$linux_build_path" "CROSS_COMPILE=$linux_cross_tool_prefix" "CROSS32CC=$linux_cross32cc" "ARCH=arm64" "INSTALL_MOD_PATH=$deploy_build_path" "INSTALL_HDR_PATH=$deploy_build_path/linux-headers"")
AC_SUBST([LINUX_ARGS], $linux_args)
AC_SUBST([KERNEL_DIR], $kernel_dir)

AC_SUBST([NVIDIA_DISPLAY_DRIVER_DIR], ["l4t-submodules/tegra/kernel-src/nv-kernel-display-driver"])

# Config variables
AC_SUBST([TARGET_HOST], $target_host)
AC_SUBST([DTB_FILE], $dtb_file)
AC_SUBST([KCONFIG_FILE], $kconfig_file)

AC_SUBST([SYSTEM_TYPE], $system_type)
AC_SUBST([L4TDIR], $l4t_dir)

AS_IF([test "x$with_username" != "x"],
    [ dnl Use the specified username
        AC_SUBST([USERNAME_SSH], ['-l "'$with_username'"'])         dnl ssh parameter
        AC_SUBST([USERNAME_AT], ['"'$with_username'"@'])            dnl rsync parameter
        AC_SUBST([USERNAME_SCP], ['-o "User '$with_username'"'])    dnl scp parameter
    ],
    [
        AC_SUBST([USERNAME_SSH], [ ])   dnl blank spaces required here
        AC_SUBST([USERNAME_AT], [ ])
        AC_SUBST([USERNAME_SCP], [ ])
    ]
)

dnl === Build ===============================================================

AC_CONFIG_FILES([
    Makefile
    doc/Makefile
    kernel/d3/include/d3/d3-jetson-bsp.h
    hardware/d3/d3-jetson-bsp-version.dtsi
    d3-jetson-util/debian/changelog
    d3-jetson-util/debian/preinst
    d3-jetson-util/target-overlay/boot/extlinux/extlinux.conf
    d3-jetson-util/Makefile
    doc/D3-Jetson-BSP-Release-Notes.md
    scripts/target/extlinux.conf
    supplemental/patches/install-example
])
AC_CONFIG_FILES(
    [bin/prep-new-system-from-host], [chmod +x bin/prep-new-system-from-host]
)

AM_SILENT_RULES([yes])
AC_OUTPUT

dnl === Report ==============================================================

echo
echo
echo "-D3 Configuration-"
echo "Customer         : customer"
echo "Version          : d3_jetson_bsp_triplet"
echo "System Type      : $system_type"
echo "Target Host      : $target_host"
[ [ "$kconfig_file" ] ] && echo "Kernel config    : $kconfig_file"
echo "DTB File         : $dtb_file"
echo "L4T Directory    : $l4t_dir"
echo "Linux Version    :" d3_linux_kernel_triplet
echo -n "Username on tgt  : "
AS_IF([test "$with_username"],
    [ echo "$with_username" ],
    [ echo "<default>" ]
)
