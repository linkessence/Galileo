DESCRIPTION = "Packages required for a target (on-device) SDK"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COREBASE}/LICENSE;md5=3f40d7994397109285ec7b81fdeb3b58"

PR = "r1"

inherit packagegroup allarch

RPROVIDES_${PN} += "packagegroup-native-sdk task-sdk-target task-native-sdk"
RREPLACES_${PN} += "packagegroup-native-sdk task-sdk-target task-native-sdk"
RCONFLICTS_${PN} += "packagegroup-native-sdk task-sdk-target task-native-sdk"
RDEPENDS_${PN} = "gcc-symlinks g++-symlinks cpp cpp-symlinks \
                  binutils-symlinks \
                  perl-modules \
                  flex flex-dev \
                  bison \
                  gawk \
                  sed \
                  grep \
                  autoconf automake \
                  make \
                  patch diffstat diffutils \
                  libstdc++-dev \
                  libtool libtool-dev libltdl-dev \
                  pkgconfig"

# usefull, but not in oe-core/meta-oe yet: patchutils
RRECOMMENDS_${PN} = " g77-symlinks gfortran-symlinks"
