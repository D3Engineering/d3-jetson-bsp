# Patches

These are "patches" provided by NVIDIA. The patches are for addressing various
bugs with Argus, as described below:

1. libnvscf.so - Fix for overzealous size check that prevents the IMX390 from streaming.

# Installation

## libnvscf.so

To install the provided patches, overwrite the existing copies in
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
