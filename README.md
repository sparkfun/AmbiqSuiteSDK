# AmbiqSuiteSDK
A copy of the AmbiqSuite SDK available on GitHub. Can be used to include AmbiqSuite as a submodule. Cannot be used to change the SDK since we are not the owners. To report issues contact Ambiq Micro

# Current Version
2.3.2

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
When compiling pretty much any example the SDK relies on the BSP (Board Support Package) definitions to specialize to that board and its capabilities. In the basic examples above we use ```BOARD=$NAME_OF_BOARD_DIR``` to specify a SparkFun board. When the ```BOARD``` variable is set the Makefile will automatically configure the path used to find the BSP directory as ```../../../../$NAME_OF_BOARD_DIR```. This path, relative to the build directory ```~/common/examples/{example}/gcc``` finds the board directory in the root of the BSP repo. 

What if you want to use a custom BSP for your own board? Firstly, you can generate BSP files by following the [bsp_pinconfig README](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs/blob/master/common/bsp_pinconfig/README.md). You should put your BSP files in a directory called ```bsp``` and enclose that in a directory named after your board. Once you have done that there are two options.

1. Instead of using ```BOARD=``` you can directly specify the path to the board directory on the make lines like this: ```make BOARDPATH=../some/relative/or/absolute/path/to/your/board/directory bootload```
1. You can hijack the system so that your baord works with the shortcut ```BOARD=```. Do this by placing your board directory next to the SparkFun boards in your copy of the repo.

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
