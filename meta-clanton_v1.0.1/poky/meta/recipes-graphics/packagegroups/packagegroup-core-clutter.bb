#
# Copyright (C) 2007 OpenedHand Ltd.
#

SUMMARY = "Clutter package groups"
LICENSE = "MIT"

PR = "r5"

inherit packagegroup

PACKAGES = "\
    ${PN}-core \
    "

# For backwards compatibility after rename
RPROVIDES_${PN}-core = "task-core-clutter-core"
RREPLACES_${PN}-core = "task-core-clutter-core"
RCONFLICTS_${PN}-core = "task-core-clutter-core"

SUMMARY_${PN}-core = "Clutter graphics library"
RDEPENDS_${PN}-core = "\
    clutter-1.8 \
    clutter-gst-1.8 \
    clutter-gtk-1.8 \
    "
