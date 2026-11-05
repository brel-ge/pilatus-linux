#!/usr/bin/bash

set -e


# Only the Pilatus targets: a full `make` overflows Kbuild's dtbs-list command line.
make -j 16 HOSTCC=x86_64-fslcsdk-linux-gcc Image.gz modules \
	$(cd arch/arm64/boot/dts && ls freescale/imx8mp-pilatus-plus-*.dts | sed 's/\.dts$/.dtb/')

cp arch/arm64/boot/Image.gz /tftpboot/Image-pilatus-plus.gz
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-variscite.dtb /tftpboot
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-inputech.dtb /tftpboot
cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-marelcom.dtb /tftpboot

