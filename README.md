WHAT IS THIS?
=============

Linux Kernel source code for the devices: bq edison 2, bq edison 2 3G, bq edison 2 Quad Core and bq edison 2 Quad Core 3G

BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

$ git clone git@github.com:bq/edison-2.git

After it, choose the version you would like to build:

$ cd edison-2/
$ git checkout 2.0.0_20140125-1425


Finally, build the kernel according the next defconfig files:

bq edison 2               -> edison2_defconfig
bq edison 2 3G            -> edison2_3g_defconfig
bq edison 2 Quad Core     -> edison2_qc_defconfig
bq edison 2 Quad Core 3G  -> edison2_qc_3g_defconfig

$ cd kernel/
$ make edison2_defconfig
$ make kernel.img





