YOCTO_ENV := ". /opt/fslc-xwayland/pilatus-sdk-wrynose/environment-setup-cortexa53-crypto-fsl-linux && "
DOCKER_IMAGE := "gitlabregistry.brel.ch/24_ge-team/3583_pilatus/850_pilatus_sw/build_container:wrynose"
DOCKER_RUN := "docker run --rm -u " + `id -u` + ":" + `id -g` + " -v " + `pwd` + ":/workspace -w /workspace " + DOCKER_IMAGE
DOCKER_IT := "docker run --rm -u " + `id -u` + ":" + `id -g` + " -v " + `pwd` + ":/workspace -w /workspace -it " + DOCKER_IMAGE
MAKE := "make -j 16 HOSTCC=x86_64-fslcsdk-linux-gcc"
# Build only what Pilatus ships: a full `make` builds every freescale DTB, and that list is
# too long for one shell argument (Kbuild dtbs-list fails with "Argument list too long").
PILATUS_DTBS := `cd arch/arm64/boot/dts && ls freescale/imx8mp-pilatus-plus-*.dts | sed 's/\.dts$/.dtb/' | tr '\n' ' '`
TARGETS := "Image.gz modules " + PILATUS_DTBS

# Default target
default: build

docker_build:
  {{ YOCTO_ENV }} make -j 16 {{ TARGETS }}

build:
  {{ DOCKER_RUN }} just docker_build


docker_configure_pilatus:
  {{ YOCTO_ENV }} make pilatus_plus_defconfig 

configure_pilatus:
  {{ DOCKER_RUN }} just docker_configure_pilatus

docker_configure_imx8_var_defconfig:
  {{ YOCTO_ENV }} make imx8_var_defconfig

configure_imx8_var_defconfig:
  {{ DOCKER_RUN }} just docker_configure_imx8_var_defconfig

# Kernel and modules always go out together, so the rootfs never has a stale module dir
install: install_modules
  cp arch/arm64/boot/Image.gz /tftpboot/Image-pilatus-plus.gz
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-symphony.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-variscite.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-inputech.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-marelcom.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-dev-marelcom.dtb /tftpboot
  cp arch/arm64/boot/dts/freescale/imx8mp-pilatus-plus-test1350.dtb /tftpboot

docker_mrproper:
  {{ YOCTO_ENV }} make mrproper

mrproper:
  {{ DOCKER_RUN }} just docker_mrproper

docker_menuconfig:
  {{ YOCTO_ENV }} {{MAKE}} menuconfig

menuconfig:
  {{ DOCKER_IT }} just docker_menuconfig

# NFS rootfs exports booted with the TFTP kernel; each needs the matching modules
NFS_ROOTFS := "/exports/var-som-plus /exports/var-som-plus-devply"

docker_install_modules:
  rm -rf .modules-install
  {{ YOCTO_ENV }} make INSTALL_MOD_PATH=/workspace/.modules-install modules_install

install_modules:
  {{ DOCKER_RUN }} just docker_install_modules
  # depmod is missing in the docker image, run it on the host
  depmod -b .modules-install `cat include/config/kernel.release`
  rm -f .modules-install/lib/modules/*/build
  # copy into lib/modules, as lib is a symlink to usr/lib in usrmerge rootfs
  for rootfs in {{ NFS_ROOTFS }}; do \
    sudo rm -rf $rootfs/lib/modules/`cat include/config/kernel.release` && \
    sudo mkdir -p $rootfs/lib/modules && \
    sudo cp -a --no-preserve=ownership .modules-install/lib/modules/. $rootfs/lib/modules/ || exit 1; \
  done
