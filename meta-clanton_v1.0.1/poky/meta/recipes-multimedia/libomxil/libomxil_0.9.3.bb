DESCRIPTION = "Bellagio OpenMAX Integration Layer"
HOMEPAGE = "http://omxil.sourceforge.net/"
LICENSE = "LGPLv2.1+"
LICENSE_FLAGS = "commercial"
LIC_FILES_CHKSUM = "file://COPYING;md5=ae6f0f4dbc7ac193b50f323a6ae191cb \
                    file://src/omxcore.h;beginline=1;endline=27;md5=806b1e5566c06486fe8e42b461e03a90"
DEPENDS = "libvorbis libogg alsa-lib libmad"

PR = "r0"

SRC_URI = "${SOURCEFORGE_MIRROR}/omxil/libomxil-bellagio-${PV}.tar.gz \
           file://configure-fix.patch \
           file://parallel-make.patch \
           file://makefile-docdir-fix.patch"

SRC_URI[md5sum] = "a1de827fdb75c02c84e55f740ca27cb8"
SRC_URI[sha256sum] = "593c0729c8ef8c1467b3bfefcf355ec19a46dd92e31bfc280e17d96b0934d74c"

S = "${WORKDIR}/${BPN}-bellagio-${PV}"

inherit autotools

EXTRA_OECONF += "--disable-ffmpegcomponents --disable-Werror"

FILES_${PN} += "${libdir}/bellagio/*${SOLIBS} \
                ${libdir}/omxloaders/*${SOLIBS}"
FILES_${PN}-staticdev += "${libdir}/bellagio/*.a \
                          ${libdir}/omxloaders/*.a"
FILES_${PN}-dev += "${libdir}/bellagio/*.la \
                    ${libdir}/bellagio/*${SOLIBSDEV} \
                    ${libdir}/omxloaders/*.la \
                    ${libdir}/omxloaders/*${SOLIBSDEV}"
FILES_${PN}-dbg += "${libdir}/bellagio/.debug/ \
                    ${libdir}/omxloaders/.debug/"
