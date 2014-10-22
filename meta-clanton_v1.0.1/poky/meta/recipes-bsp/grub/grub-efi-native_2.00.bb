SUMMARY = "GRUB2 is the next-generation GRand Unified Bootloader"

DESCRIPTION = "GRUB2 is the next generaion of a GPLed bootloader \
intended to unify bootloading across x86 operating systems. In \
addition to loading the Linux kernel, it implements the Multiboot \
standard, which allows for flexible loading of multiple boot images. \
This recipe builds an EFI binary for the target. It does not install \
or package anything, it only deploys a target-arch GRUB EFI image."

HOMEPAGE = "http://www.gnu.org/software/grub/"
SECTION = "bootloaders"

LICENSE = "GPLv3"
LIC_FILES_CHKSUM = "file://COPYING;md5=d32239bcb673463ab874e80d47fae504"

# FIXME: We should be able to optionally drop freetype as a dependency
DEPENDS = "autogen-native"
RDEPENDS_${PN} = "diffutils freetype"
PR = "r2"

# Native packages do not normally rebuild when the target changes.
# Ensure this is built once per HOST-TARGET pair.
PN := "grub-efi-${TRANSLATED_TARGET_ARCH}-native"

SRC_URI = "ftp://ftp.gnu.org/gnu/grub/grub-${PV}.tar.gz \
           file://grub-2.00-fpmath-sse-387-fix.patch \
	   file://grub-2.00-fix-enable_execute_stack-check.patch \
           file://grub-2.00-disable-help2man.patch \
           file://check-if-liblzma-is-disabled.patch \
	   file://grub-no-unused-result.patch \
	   file://grub-2.00-ignore-gnulib-gets-stupidity.patch \
          "
SRC_URI[md5sum] = "e927540b6eda8b024fb0391eeaa4091c"
SRC_URI[sha256sum] = "65b39a0558f8c802209c574f4d02ca263a804e8a564bc6caf1cd0fd3b3cc11e3"

COMPATIBLE_HOST = '(x86_64.*|i.86.*)-(linux|freebsd.*)'

S = "${WORKDIR}/grub-${PV}"

# Determine the target arch for the grub modules before the native class
# clobbers TARGET_ARCH.
ORIG_TARGET_ARCH := "${TARGET_ARCH}"
python __anonymous () {
    import re
    target = d.getVar('ORIG_TARGET_ARCH', True)
    if target == "x86_64":
        grubtarget = 'x86_64'
        grubimage = "bootx64.efi"
    elif re.match('i.86', target):
        grubtarget = 'i386'
        grubimage = "bootia32.efi"
    else:
        raise bb.parse.SkipPackage("grub-efi is incompatible with target %s" % target)
    d.setVar("GRUB_TARGET", grubtarget)
    d.setVar("GRUB_IMAGE", grubimage)
}

inherit autotools
inherit gettext
inherit native
inherit deploy

EXTRA_OECONF = "--with-platform=efi --disable-grub-mkfont \
                --target=${GRUB_TARGET} --enable-efiemu=no --program-prefix='' \
                --enable-liblzma=no --enable-device-mapper=no --enable-libzfs=no"

do_mkimage() {
	./grub-mkimage -p /EFI/BOOT -d ./grub-core/ \
		       -O ${GRUB_TARGET}-efi -o ./${GRUB_IMAGE} \
	               boot linux ext2 fat serial part_msdos part_gpt normal efi_gop
}
addtask mkimage after do_compile before do_install

do_deploy() {
	install -m 644 ${B}/${GRUB_IMAGE} ${DEPLOYDIR}
}
addtask deploy after do_install before do_build

do_install[noexec] = "1"
do_populate_sysroot[noexec] = "1"
