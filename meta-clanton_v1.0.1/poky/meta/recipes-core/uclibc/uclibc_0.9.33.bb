SRCREV="7810e4f8027b5c4c8ceec6fefec4eb779362ebb5"

require uclibc.inc
require uclibc-package.inc
require uclibc-${PV}.inc

STAGINGCC = "gcc-cross-initial"
STAGINGCC_class-nativesdk = "gcc-crosssdk-initial"

DEPENDS = "virtual/${TARGET_PREFIX}binutils \
           virtual/${TARGET_PREFIX}gcc-initial \
           virtual/${TARGET_PREFIX}libc-initial \
           linux-libc-headers ncurses-native"

PROVIDES += "virtual/libc virtual/${TARGET_PREFIX}libc-for-gcc"
RDEPENDS_${PN}-dev = "linux-libc-headers-dev"
RPROVIDES_${PN}-dev += "libc-dev virtual-libc-dev"
# uclibc does not really have libsegfault but then using the one from glibc is also not
# going to work. So we pretend that we have it to make bitbake not pull other recipes
# to satisfy this dependency for the images/tasks

RPROVIDES_${PN} += "libsegfault rtld(GNU_HASH)"
