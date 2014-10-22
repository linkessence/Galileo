DESCRIPTION = "A simple GTK+ based panel for handheld devices."
HOMEPAGE = "http://matchbox-project.org"
BUGTRACKER = "http://bugzilla.openedhand.com/"

LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://COPYING;md5=94d55d512a9ba36caa9b7df079bae19f \
                    file://matchbox-panel/mb-panel.h;endline=10;md5=0b7db28f4b6863fb853d0467e590019a \
                    file://applets/startup/startup.c;endline=22;md5=b0a64fbef3097d79f8264e6907a98f03"

DEPENDS = "gtk+ startup-notification dbus dbus-glib"
DEPENDS += " ${@base_contains("MACHINE_FEATURES", "acpi", "libacpi", "",d)}"
DEPENDS += " ${@base_contains("MACHINE_FEATURES", "apm", "apmd", "",d)}"

SRCREV = "cdf7a22716b87468f10573f622d5c7a58a684e35"
PV = "2.0+git${SRCPV}"
PR = "r0"

RPROVIDES_${PN} = "matchbox-panel"
RREPLACES_${PN} = "matchbox-panel"
RCONFLICTS_${PN} = "matchbox-panel"

SRC_URI = "git://git.yoctoproject.org/${BPN};protocol=git \	
	   file://gcc-4.6.0-compile.patch"

EXTRA_OECONF = "--enable-startup-notification --enable-dbus"
EXTRA_OECONF += " ${@base_contains("MACHINE_FEATURES", "acpi", "--with-battery=acpi", "",d)}"
EXTRA_OECONF += " ${@base_contains("MACHINE_FEATURES", "apm", "--with-battery=apm", "",d)}"

S = "${WORKDIR}/git"

FILES_${PN} += "${libdir}/matchbox-panel/*.so \
                ${datadir}/matchbox-panel/brightness/*.png \
                ${datadir}/matchbox-panel/startup/*.png "
FILES_${PN}-dbg += "${libdir}/matchbox-panel/.debug"
FILES_${PN}-dev += "${libdir}/matchbox-panel/*.la"

inherit autotools pkgconfig
