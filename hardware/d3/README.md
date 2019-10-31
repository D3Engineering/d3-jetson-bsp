# Using Overlays

The overlay-loader plugin runs early during the init process of our kernel.
It looks for nodes in /overlay-loader/ in the device tree. These overlays can
be activated using kernel parameter `active_overlays=name_name_1,node_name_2`.
Our nodes follow the convention `[camera model]_[port_index]`. Thus a mix of
cams on RSP may look like `active_overlays=imx390_0,ov10640_1,vg6768_2`.

overlay-loader has very verbose output. This can be seen by searching for
"overlay-loader" in dmesg. It will log all the params that it matched and
applied overlays for.

```
# sample tree of an overlay fragment

overlay-loader
|
|-->fragment
|   |
|   |-->param
|   |
|   |-->override@N
|   |   |
|   |   |-->target
|   |   |
|   |   |-->_overlay_
```


# Adding to the "new" device tree

All templates have been split into their respective type. They should include
their own resets and checks.

A camera consists of 3 includes which are to be read in the following order.
This should be done in d3-[platform]-[boardname]-cam[camnumber]-template.dtsi.

1. d3-des-[deserializermodel]-template.dtsi
2. d3-ser-[serializermodel]-template.dtsi
3. d3-cam-[cammodel]-template.dtsi

Please look at an existing template for a similar device to understand what
is required for a new one.


# Important notes

CIC is a deviation from this pattern, its deserializers are hard coded in
d3-[board]-cic.dtsi because it only has 1 of each deserializer. It doesn't
need templates.

All overlays are currently created by the camera templates using the include
`d3-overlays-cam.dtsi`. The overlay DTSI file is included by a camera-specific
DTSI file in the templated-cameras/ subdirectory. As the cameras are what we
are trying to activate or deactivate from a high level.
