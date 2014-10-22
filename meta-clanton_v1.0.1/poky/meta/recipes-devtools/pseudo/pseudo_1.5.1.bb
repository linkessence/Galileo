require pseudo.inc

PR = "r3"

SRC_URI = "http://www.yoctoproject.org/downloads/${BPN}/${BPN}-${PV}.tar.bz2"

SRC_URI[md5sum] = "5ec67c7bff5fe68c56de500859c19172"
SRC_URI[sha256sum] = "3b896f592f4d568569bd02323fad2d6b8c398e16ca36ee5a8947d2ff6c1d3d52"

PSEUDO_EXTRA_OPTS ?= "--enable-force-async"
