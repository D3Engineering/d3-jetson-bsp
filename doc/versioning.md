---
urlcolor: blue
colorlinks: true
---

# Versioning scheme for the D3 Jetson BSP

`d3/` branches and customer branches are versioned differently.  If you
are reading this on GitHub, the "Customer branches" section does not
apply to you.

Versions are in `MAJOR.MINOR.PATCH` format.  Each time one component is
incremented, any components below it are set to `0`.

## Semantic Versioning

We do **not** use [Semantic Versioning] ("SemVer") due to specific
requirements of our use cases.  However, version numbers are formatted
and compared according to the lexical conventions of SemVer v2.0.0,
specification items [2], [9], [10], and [11].  For example, a release
candidate has a pre-release version number such as `2.0.0-rc`.
The full BNF for version numbers is at [SemVer BNF].

## `d3/` branches

D3 branches (including those released on [GitHub]) are numbered `x.y.z`:

- `x` is incremented when the NVIDIA [Linux for Tegra] version changes.
- `y` is incremented for a release with new features.
- `z` is incremented for a release only containing bugfixes.

## Customer branches

Customer branches are numbered `x.y.z`:

- `x` is the release number listed in the statement of work
- `y` is incremented for a release with new features.
- `z` is incremented for a release only containing bugfixes.

For example, release 2.0.0 meets all the requirements of milestone 2
in the statement of work.

# Copyright

Copyright (c) 2019 D3 Engineering, LLC.  All rights reserved.  This file
may be copied and distributed as part of the D3 Jetson BSP.

[GitHub]: https://github.com/D3Engineering/d3-jetson-bsp
[Linux for Tegra]: https://developer.nvidia.com/embedded/linux-tegra
[Semantic Versioning]: https://semver.org/spec/v2.0.0.html
[SemVer BNF]: https://semver.org/spec/v2.0.0.html#backusnaur-form-grammar-for-valid-semver-versions
[2]: https://semver.org/spec/v2.0.0.html#spec-item-2
[9]: https://semver.org/spec/v2.0.0.html#spec-item-9
[10]: https://semver.org/spec/v2.0.0.html#spec-item-10
[11]: https://semver.org/spec/v2.0.0.html#spec-item-11

[]( vi: set ts=2 sts=2 sw=2 et ai tw=72: )
