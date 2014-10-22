DESCRIPTION = "GEGL (Generic Graphics Library) is a graph based image processing framework."
LICENSE = "LGPL-3.0"
LIC_FILES_CHKSUM = "file://COPYING;md5=d32239bcb673463ab874e80d47fae504"
DEPENDS = "babl librsvg glib-2.0 gtk+ pango cairo expat zlib libpng jpeg virtual/libsdl"

inherit gnome

EXTRA_OECONF = "--disable-docs "

SRC_URI = "ftp://ftp.gimp.org/pub/${PN}/0.2/${PN}-${PV}.tar.bz2"
SRC_URI[md5sum] = "32b00002f1f1e316115c4ed922e1dec8"
SRC_URI[sha256sum] = "df2e6a0d9499afcbc4f9029c18d9d1e0dd5e8710a75e17c9b1d9a6480dd8d426"

FILES_${PN} += "${libdir}/gegl-*/*.so"
FILES_${PN}-dev += "${libdir}/gegl-*/*.la"
FILES_${PN}-dbg += "${libdir}/gegl-*/.debug"
