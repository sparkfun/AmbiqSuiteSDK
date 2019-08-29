# AmbiqSuiteSDK
A copy of the AmbiqSuite SDK available on GitHub. Can be used to include AmbiqSuite as a submodule. Cannot be used to change the SDK since we are not the owners. To report issues contact Ambiq Micro

# Current Version
2.2.0

# Submodules
Git submodules can be used to reuse code between repositories. Some special precautions can be necessary when working with them -- most notably the need to clone the contents of submodules explicitly. Here's how to do that:

- If you've already cloned a repo
  - ```git submodule update --init --recursive```
- If you are about to clone the repo
  - git clone --recursive <project url>

Maintainers of this repo may also need to keep submodules updated.

Here are some more documents about submodules.
[Working with Submodules by GitHub](https://github.blog/2016-02-01-working-with-submodules/)
[Git Submodules by gitaarik](https://gist.github.com/gitaarik/8735255)

This repo includes the following git submodules:
- [SparkFun_Apollo3_AmbiqSuite_BSPs](https://github.com/sparkfun/SparkFun_Apollo3_AmbiqSuite_BSPs) : provides board definitions and examples for SparkFun Apollo3 boards
