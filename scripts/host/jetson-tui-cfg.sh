#!/bin/bash
#
# Script to accelerate jetson configure script usage
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.

set -o errexit
set -o pipefail

[[ -t 0 && -t 1 ]] || { echo "Can't run - not on a terminal" ; exit ; }

MY_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Assumes we are in ./scripts/host
BSP_DIR="$MY_DIR/../.."
cd "$BSP_DIR"

# Input DTB
dtb_array=($(awk '{if ( $1 !~ /^#/ && $3 ~ /\.dtb$/ )print $3}' ./hardware/*/Makefile))
select dtb_name in "${dtb_array[@]}"; do
	test -n $dtb_name && break
	echo "Select a valid dtb"
done
dtb_file=`realpath -q -m ."/build/deploy/boot/$dtb_name"`

# Input System Type
sys_array=("orin" "xavier" "nx")
select sys_name in "${sys_array[@]}"; do
	test -n $sys_name && break
	echo "Select a valid system type"
done

read -e -p "Provide target hostname (default '$sys_name'): " -i "$sys_name" target_hostname

./configure --with-dtb="$dtb_file" --with-system-type="$sys_name" --with-target-host="$target_hostname"
