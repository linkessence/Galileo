#
# Copyright (C) 2007-2008 OpenedHand Ltd.
#

SUMMARY = "Sato desktop"
LICENSE = "MIT"
PR = "r33"

inherit packagegroup

PACKAGE_ARCH = "${MACHINE_ARCH}"

PACKAGES = "${PN} ${PN}-base ${PN}-apps ${PN}-games"

# For backwards compatibility after rename
RPROVIDES_${PN} = "task-core-x11-sato"
RREPLACES_${PN} = "task-core-x11-sato"
RCONFLICTS_${PN} = "task-core-x11-sato"

RDEPENDS_${PN} = "\
    ${PN}-base \
    ${PN}-apps \
    ${PN}-games \
    "

NETWORK_MANAGER ?= "connman-gnome"
NETWORK_MANAGER_libc-uclibc = ""

SUMMARY_${PN}-base = "Sato desktop - base packages"
RDEPENDS_${PN}-base = "\
    matchbox-desktop \
    matchbox-session-sato \
    matchbox-keyboard \
    matchbox-keyboard-applet \
    matchbox-keyboard-im \
    matchbox-config-gtk \
    xcursor-transparent-theme \
    sato-icon-theme \
    settings-daemon \
    gtk-sato-engine \
    shutdown-desktop \
    libsdl \
    ${NETWORK_MANAGER} \
    "

# pcmanfm doesn't work on mips
FILEMANAGER ?= "pcmanfm"
FILEMANAGER_mips ?= ""

WEB ?= ""
#WEB = "web-webkit"

SUMMARY_${PN}-apps = "Sato desktop - applications"
RDEPENDS_${PN}-apps = "\
    leafpad \
    gaku \
    x11vnc \
    matchbox-terminal \
    sato-screenshot \
    ${FILEMANAGER} \
    ${WEB} \
    "

SUMMARY_${PN}-games = "Sato desktop - games"
RDEPENDS_${PN}-games = "\
    oh-puzzles \
    "
