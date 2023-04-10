# D3 Jetson BSP Supplements
This folder contains userspace patches and configurations to aid in the
installation of D3 Jetson BSP.

## Xavier NX Flash Config
Due to the large amount of supported camera combinations on our Xavier NX Carrier
Board, we need to provide a special configuration for flashing. This is provided
in the `xavier-nx-flash-config` folder and further documented in the D3 Jetson
NX Carrier Bringup document. To install the flash config, simply copy all of the
files from the folder and drop them into your Linux for Tegra folder (typically
`$HOME/nvidia/nvidia_sdk/JetPack_5.0.2_Linux_JETSON_XAVIER_NX_TARGETS/Linux_for_Tegra`).

## Patches
These are fixes provided by NVIDIA. The patches address various issues with
Argus, as documented below:

1. libnvscf.so - Fix for overzealous size check that prevents the IMX390 from streaming.

### Installation
#### Automatic (recommended)
The patches mentioned above are included in the d3-jetson-util package, which
upon installation performs the steps documented in the section below. We
recommend installing d3-jetson-util over the manual method due to its ease of
use.

#### Manual
To manually install the provided patches, overwrite the existing copies in
`/usr/lib/aarch-linux-gnu/tegra/` with the versions provided in this
release package:

```
sudo -i
for f in libnvscf.so; do
    # Save a copy of the original L4T file
    cp /usr/lib/aarch64-linux-gnu/tegra/$f{,-original}
    cp $f /usr/lib/aarch64-linux-gnu/tegra/
done
reboot
```
