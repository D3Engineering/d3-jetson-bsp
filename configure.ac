AC_INIT([Jetson BSP], [0.0.0])
AC_PREREQ(2.69)
#AC_COPYRIGHT([Copyright (C) 2019 D3 Engineering])


AC_USE_SYSTEM_EXTENSIONS
AM_PROG_AR
AC_CONFIG_MACRO_DIRS([m4])
AM_INIT_AUTOMAKE([foreign])
AM_SILENT_RULES([yes])



AC_ARG_WITH(
    [target-host],
    AS_HELP_STRING([--with-target-host=host], [specify Jetson target hostname]),
    [target_host=$withval],
    [target_host=192.168.55.1])

AC_ARG_WITH(
    [dtb],
    AS_HELP_STRING([--with-dtb=dtb-file], [specify dtb file for flash-dtb]),
    [dtb_file=$withval],
    [dtb_file=`realpath -q .`/build/deploy/boot/TODO])

AC_ARG_WITH(
    [kconfig],
    AS_HELP_STRING([--with-kconfig=kernel-config-file], [specify a custom Linux .config file]),
    [kconfig_file=$withval],
    [kconfig_file=])

AC_ARG_WITH(
    [l4t],
    AS_HELP_STRING([--with-l4t=l4t directory],
    [specify path to l4t directory (defaults based on target-system-type)]),
    [l4t_override_dir=$withval],
    [l4t_override_dir=])

# Set the L4T path based on the target system
AM_CONDITIONAL(XAVIER, false)
AM_CONDITIONAL(TX2, false)
AM_CONDITIONAL(NANO, false)
AC_ARG_WITH(
    [system-type],
    AS_HELP_STRING([--with-system-type=system], [xavier, tx2, nano]),
    [
        case "${withval}" in
          xavier)
            AM_CONDITIONAL(XAVIER, true)
            system_type=xavier
            l4t_dir=`realpath -q ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P2888/Linux_for_Tegra`
          ;;
          tx2)
            AM_CONDITIONAL(TX2, true)
            system_type=tx2
            l4t_dir=`realpath -q ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra`
          ;;
          nano)
            AM_CONDITIONAL(NANO, true)
            system_type=nano
            l4t_dir=`realpath -q ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3448/Linux_for_Tegra`
          ;;
          *)
            AC_MSG_ERROR([bad value ${withval} for --with-system-type (valid:xavier, tx2, nano)])
          ;;
    esac],
    [
        system_type=tx2
        l4t_dir=`realpath -q ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra`
    ])

# Change the l4t directory from the one inferred by the system type if
# it has been supplied.
if [[ -n "$l4t_override_dir" ]]; then
   l4t_dir="$l4t_override_dir"
fi

kernel_dir=`realpath ./kernel/kernel-4.9`
m4_define(linux_build_path, "$PWD/build/kernel")
AC_SUBST([LINUX_BUILD_PATH], $linux_build_path)
m4_define(deploy_build_path, "$PWD/build/deploy")
AC_SUBST([DEPLOY_BUILD_PATH], $deploy_build_path)
m4_define(linux_cross_tool_prefix, "aarch64-linux-gnu-")
m4_define(linux_cross32cc, "arm-linux-gnueabihf-gcc-5")
m4_define(linux_args, "-C "$kernel_dir" "O=$linux_build_path" "CROSS_COMPILE=$linux_cross_tool_prefix" "CROSS32CC=$linux_cross32cc" "ARCH=arm64" "INSTALL_MOD_PATH=$deploy_build_path" "INSTALL_HDR_PATH=$deploy_build_path/linux-headers"")
AC_SUBST([LINUX_ARGS], $linux_args)


AC_SUBST([TARGET_HOST], $target_host)
AC_SUBST([DTB_FILE], $dtb_file)
AC_SUBST([KCONFIG_FILE], $kconfig_file)

AC_SUBST([SYSTEM_TYPE], $system_type)
AC_SUBST([L4TDIR], $l4t_dir)


AC_CONFIG_FILES([
    Makefile
])
AM_SILENT_RULES([yes])
AC_OUTPUT

echo
echo
echo "-Configuration-"
echo "System Type      : $system_type"
echo "Target Host      : $target_host"
[ [ "$kconfig_file" ] ] && echo "Kernel config    : $kconfig_file"
echo "DTB File         : $dtb_file"
echo "L4T Directory    : $l4t_dir"

