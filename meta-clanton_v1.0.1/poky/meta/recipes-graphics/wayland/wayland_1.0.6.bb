SUMMARY = "Wayland, a protocol between a compositor and clients"
DESCRIPTION = "Wayland is a protocol for a compositor to talk to its clients \
as well as a C library implementation of that protocol. The compositor can be \
a standalone display server running on Linux kernel modesetting and evdev \
input devices, an X application, or a wayland client itself. The clients can \
be traditional applications, X servers (rootless or fullscreen) or other \
display servers."
HOMEPAGE = "http://wayland.freedesktop.org"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://COPYING;md5=1d4476a7d98dd5691c53d4d43a510c72 \
                    file://src/wayland-server.c;endline=21;md5=079ae21dbf98ada52ec23744851b0a5c"

PR = "r1"

SRC_URI = "http://wayland.freedesktop.org/releases/${BPN}-${PV}.tar.xz"
SRC_URI[md5sum] = "936a2590aea69fa3c0cf234d54b9137c"
SRC_URI[sha256sum] = "f52a012df699eff434b0f49e56000d6978b5f781048402ca8e0232242970fc49"

SRC_URI_append_class-native = " file://just-scanner.patch"

inherit autotools pkgconfig

# We need wayland-native for the wayland-scanner utility
BBCLASSEXTEND = "native"

DEPENDS_virtclass-native = "expat-native libffi-native"
DEPENDS = "expat libffi wayland-native"

EXTRA_OECONF_virtclass-native = "--disable-documentation"
EXTRA_OECONF = "--disable-documentation --disable-scanner"

# Wayland installs a M4 macro for other projects to use. This M4 macro includes
# a path to a Makefile fragment to get the rules to generate stubs from protocol
# description files.  The paths to the sysroot end up incorrect, so fix them.
do_configure_append_class-native() {
  sed -e 's,@prefix@,${STAGING_DIR_NATIVE},g' \
      -e 's,@exec_prefix@,${STAGING_DIR_NATIVE},g' \
      -e 's,@bindir@,${STAGING_BINDIR_NATIVE},g' \
      -e 's,@datarootdir@,${STAGING_DATADIR_NATIVE},g' \
  ${S}/wayland-scanner.m4.in > ${B}/wayland-scanner.m4
}
