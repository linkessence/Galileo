SUMMARY = "MKVToolNix -- Cross-platform tools for Matroska"
HOMEPAGE = "http://www.bunkus.org/videotools/mkvtoolnix/source.html"

LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=b234ee4d69f5fce4486a80fdaf4a4263"

DEPENDS = "curl boost expat zlib libebml libmatroska libogg libvorbis bzip2 lzo file ruby-native"

SRC_URI = "http://www.bunkus.org/videotools/mkvtoolnix/sources/mkvtoolnix-${PV}.tar.bz2"
SRC_URI[md5sum] = "033621461ef8eb922fc1366e0a9a6f16"
SRC_URI[sha256sum] = "d913f531331c3332d2fb334c872ea19bfea7293dfedc4bf33ae7162e4efcbde1"

inherit autotools

EXTRA_OECONF = " --with-boost-libdir=${STAGING_LIBDIR} \
"

# remove some hardcoded searchpaths
do_configure_prepend() {
    sed -i -e s:/usr/local/lib:${STAGING_LIBDIR}:g -e s:/usr/local/include:${STAGING_INCDIR}:g ac/ebml.m4
}

# Yeah, no makefile
do_compile() {
    ./drake
}

do_install() {
    ./drake install DESTDIR=${D}
}
