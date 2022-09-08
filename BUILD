#!/usr/bin/bash

set -e

make -j 16
cp arch/arm64/boot/Image.gz /tftpboot/
cp arch/arm64/boot/dts/freescale/imx8mm-var-som-pilatus.dtb /tftpboot/pilatus.dtb

