#!/usr/bin/bash

set -e


make -j 16 HOSTCC=x86_64-fslcsdk-linux-gcc

cp arch/arm64/boot/Image.gz /tftpboot/Image-pilatus-plus.gz
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-variscite.dtb /tftpboot
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-inputech.dtb /tftpboot
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-marelcom.dtb /tftpboot

