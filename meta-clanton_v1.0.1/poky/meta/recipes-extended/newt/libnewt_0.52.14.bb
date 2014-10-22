SUMMARY = "A library for text mode user interfaces"

DESCRIPTION = "Newt is a programming library for color text mode, widget based user \
interfaces.  Newt can be used to add stacked windows, entry widgets, \
checkboxes, radio buttons, labels, plain text fields, scrollbars, \
etc., to text mode user interfaces.  This package also contains the \
shared library needed by programs built with newt, as well as a \
/usr/bin/dialog replacement called whiptail.  Newt is based on the \
slang library."

HOMEPAGE = "https://fedorahosted.org/newt/"
SECTION = "libs"

LICENSE = "LGPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=55ca817ccb7d5b5b66355690e9abc605"

# slang needs to be >= 2.2
DEPENDS = "slang popt python"

PR = "r2"

SRC_URI = "https://fedorahosted.org/releases/n/e/newt/newt-${PV}.tar.gz \
           file://remove_slang_include.patch \
           file://fix_SHAREDDIR.patch \
           file://cross_ar.patch \
           file://fix_python_fpic.patch"

SRC_URI[md5sum] = "eb78c6bb658b92ec7198908b5b8d0e37"
SRC_URI[sha256sum] = "f70f4f58baa60388ddf2e39249ffb00898fb40f2b2767e42e2ab51fe4b40978e"

S = "${WORKDIR}/newt-${PV}"

EXTRA_OECONF = "--without-tcl"

inherit autotools pythonnative python-dir

EXTRA_OEMAKE = "PYTHONVERS=${PYTHON_DIR}"

export STAGING_INCDIR
export STAGING_LIBDIR

export BUILD_SYS
export HOST_SYS

PACKAGES_prepend = "whiptail ${PN}-python "

do_configure_prepend() {
    ( cd ${S}; sh autogen.sh )
}

FILES_whiptail = "${bindir}/whiptail"
FILES_${PN}-dbg += "${PYTHON_SITEPACKAGES_DIR}/.debug/"
FILES_${PN}-python = "${PYTHON_SITEPACKAGES_DIR}/*"
FILES_${PN}-staticdev = "${libdir}/*.a"
