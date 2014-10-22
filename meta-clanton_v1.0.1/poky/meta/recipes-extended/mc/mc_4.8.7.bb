DESCRIPTION = "Midnight Commander is an ncurses based file manager."
HOMEPAGE = "http://www.midnight-commander.org/"
LICENSE = "GPLv3"
LIC_FILES_CHKSUM = "file://COPYING;md5=270bbafe360e73f9840bd7981621f9c2"
SECTION = "console/utils"
DEPENDS = "ncurses glib-2.0"
RDEPENDS_${PN} = "ncurses-terminfo"

PR = "r0"

SRC_URI = "http://www.midnight-commander.org/downloads/${BPN}-${PV}.tar.bz2"

SRC_URI[md5sum] = "b113e50682293a953141134969dba785"
SRC_URI[sha256sum] = "4e9c45925b47650dac58d7556a89c3e9b4275e48976b6c13b63c2f8eed3a383b"

inherit autotools gettext

EXTRA_OECONF = "--with-screen=ncurses --without-gpm-mouse --without-x --without-samba"

FILES_${PN}-dbg += "${libexecdir}/mc/.debug/"

do_install_append () {
	sed -i -e '1s,#!.*perl,#!${bindir}/env perl,' ${D}${libexecdir}/mc/extfs.d/*
	rm -rf ${D}${libdir}
}
