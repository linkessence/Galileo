LICENSE = "GPLv2"
SECTION = "x11/gnome/libs"

PR = "r2"

DEPENDS = "libxml2 libgnomecups glib-2.0 pango libart-lgpl fontconfig popt gnome-common"

LIC_FILES_CHKSUM = "file://COPYING;md5=94d55d512a9ba36caa9b7df079bae19f"

inherit pkgconfig gnome

SRC_URI += "file://fix.includes.patch"

SRC_URI[archive.md5sum] = "d66b81ee8aa3b3a33f5efc9a47ce07e3"
SRC_URI[archive.sha256sum] = "8b34f81599423ef3da9c43775495da602f83cbbee069c9e760ffeae6aa4768e6"

FILES_${PN}-dbg += "\
    ${libdir}/${PN}/${PV}/modules/.debug \
    ${libdir}/${PN}/${PV}/modules/*/.debug \
"
FILES_${PN}-staticdev += "\
    ${libdir}/${PN}/${PV}/modules/*.a \
    ${libdir}/${PN}/${PV}/modules/*/*.a \
"
