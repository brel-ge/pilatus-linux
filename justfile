YOCTO_ENV := ". /opt/fslc-xwayland/pilatus-sdk-4.1/environment-setup-armv8a-fslc-linux &&"
DOCKER_RUN := "docker run --rm -u " + `id -u` + ":" + `id -g` + " -v " + `pwd` + ":/workspace -w /workspace pilatus-build"
MAKE := "make -j 16 HOSTCC=x86_64-fslcsdk-linux-gcc"

# Default target
default: build

docker_build:
  {{ YOCTO_ENV }} make -j 16

build:
  {{ DOCKER_RUN }} just docker_build


docker_configure_pilatus:
  {{ YOCTO_ENV }} make pilatus_plus_defconfig 

configure_pilatus:
  {{ DOCKER_RUN }} just docker_configure_pilatus

install:
  cp arch/arm64/boot/Image.gz /tftpboot/Image-pilatus-plus.gz
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-variscite.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-inputech.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-marelcom.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-dev-marelcom.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-test1350.dtb /tftpboot

docker_mrproper:
  {{ YOCTO_ENV }} make mrproper

mrproper:
  {{ DOCKER_RUN }} just docker_mrproper


install_modules:
  sudo make INSTALL_MOD_PATH=/exports/var-som-plus modules_install 
