# Patches

These are "patches" provided by NVIDIA. These patches are for addressing a
bug limiting the number of cameras that can be streamed with Argus to 6.

1. libnvargus.so, libnvargus\_socketclient.so,
   libnvargus\_socketserver.so - Fix for letting sensors to be given a position
   using numbers (0-F).

# Installation

## libnvscf.so, libnvargus.so, libnvargus\_socketclient.so, libnvargus\_socketserver.so, libnvjpeg.so

To install these libraries overwrite the existing copies in
`/usr/lib/aarch-linux-gnu/tegra/` with the version provided in this
release package.

```
sudo -i
for f in libnvscf.so libnvargus.so libnvargus_socketclient.so libnvargus_socketserver.so libnvjpeg.so; do
    # Save a copy of the original L4T file
    cp /usr/lib/aarch64-linux-gnu/tegra/$f{,-original}
    cp $f /usr/lib/aarch64-linux-gnu/tegra/
done
reboot
```
