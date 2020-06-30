# AmbiqSuiteSDK
A copy of the AmbiqSuite SDK available on GitHub. Can be used to include AmbiqSuite as a submodule. Can be used to track issues with the SDK, however it is not maintained by AmbiqMicro so the issues may not be resolved upstream.

# Current Version
2.4.2

# Getting Started

First make sure that the necessary tools are available at your command line. They are:
- Git 
  - ```git```
- Make 
  - ```make```
- ARM GCC 
  - ```arm-none-eabi-xxx``` (preferably [8-2018-q4-major](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads#panel2a) - there is a known problem with 8-2019-q3-update for example)
- Python3 
  - ```python3```

Then follow these steps
```
git clone --recursive https://github.com/sparkfun/AmbiqSuiteSDK
cd AmbiqSuiteSDK
cd boards_sfe/common/examples/{example}/gcc
YOUR_BOARD=redboard_artemis_atp   # choose the bsp directory name of the board you want to use
make BOARD=$YOUR_BOARD            # build project
make BOARD=$YOUR_BOARD bootload         # equivalent to 'bootload_svl'
make BOARD=$YOUR_BOARD bootload_svl     # bootloads using the SparkFun Variable Loader
make BOARD=$YOUR_BOARD bootload_asb     # bootloads using the Ambiq Secure Bootloader - overwrites SparkFun Variable Loader
```

# Advanced Usage
All the convenient functionality that we've added to the AmbiqSuiteSDK comes from our [SparkFun AmbiqSuite Apollo3 BSPs](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs). That repo contains more [detailed documentation for advanced usage](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs#advanced-usage).

# Repository Structure
In addition to including the [SparkFun BSPs](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs)This repo catalogs information about the AmbiqSuite SDK. Various branches serve different purposes:

### Branch Purposes

Pattern | Use | Addtl. info
---|---|---
master | contains the most up-to-date version of the SDK along with all patches |
mirror | mirrors latest SDK available from AmbiqMicro | 
\*-archive | provides an archive of the SDK as released by AmbiqMicro at each version | 
\*-patch-\*description| provides a version of the SDK patched beyond AmbiqMicro release


# Submodules
Git submodules can be used to reuse code between repositories. Some special precautions can be necessary when working with them -- most notably the need to clone the contents of submodules explicitly. Here's how to do that:

- If you've already cloned a repo
  - ```git submodule update --init --recursive```
- If you are about to clone the repo
  - ```git clone --recursive <project url>```

Maintainers of this repo may also need to keep submodules updated.

Here are some more documents about submodules.
- [Working with Submodules by GitHub](https://github.blog/2016-02-01-working-with-submodules/)
- [Git Submodules by gitaarik](https://gist.github.com/gitaarik/8735255)

This repo includes the following git submodules:
- [SparkFun_Apollo3_AmbiqSuite_BSPs](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs) : provides board definitions and examples for SparkFun Apollo3 boards
