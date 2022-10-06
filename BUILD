#!/usr/bin/bash

set -e

make -j 16
cp arch/arm64/boot/Image.gz /tftpboot/Image-pilatus-plus.gz
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-variscite.dtb /tftpboot

