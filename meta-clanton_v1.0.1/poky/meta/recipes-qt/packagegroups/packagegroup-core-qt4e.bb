SUMMARY = "Qt for Embedded Linux (Qt without X11)"
PR = "r2"
LICENSE = "MIT"

inherit packagegroup

# For backwards compatibility after rename
RPROVIDES_${PN} = "task-qt4e-base"
RREPLACES_${PN} = "task-qt4e-base"
RCONFLICTS_${PN} = "task-qt4e-base"

TOUCH = ' ${@base_contains("MACHINE_FEATURES", "touchscreen", "tslib tslib-calibrate tslib-tests", "",d)}'

RDEPENDS_${PN} = " \
	qt4-embedded \
	libqt-embedded3support4 \
	libqt-embeddedclucene4 \
	libqt-embeddedcore4 \
	libqt-embeddeddbus4 \
	libqt-embeddedgui4 \
	libqt-embeddedhelp4 \
	libqt-embeddedmultimedia4 \
	libqt-embeddednetwork4 \
	libqt-embeddedscript4 \
	libqt-embeddedscripttools4 \
	libqt-embeddedsql4 \
	libqt-embeddedsvg4 \
	libqt-embeddedtest4 \
	libqt-embeddedwebkit4 \
	libqt-embeddedxml4 \
	qt4-embedded-fonts-ttf-dejavu \
	qt4-embedded-fonts-ttf-vera \
	qt4-embedded-plugin-iconengine-svgicon \
	qt4-embedded-plugin-imageformat-gif \
	qt4-embedded-plugin-imageformat-ico \
	qt4-embedded-plugin-imageformat-jpeg \
	qt4-embedded-plugin-imageformat-mng \
	qt4-embedded-plugin-imageformat-svg \
	qt4-embedded-plugin-imageformat-tiff \
	qt4-embedded-plugin-mousedriver-tslib \
	qt4-embedded-plugin-phonon-backend-gstreamer \
	qt4-embedded-plugin-script-dbus \
	qt4-embedded-plugin-sqldriver-sqlite \
	${TOUCH} \
        qt4-embedded-demos \
        qt4-embedded-examples \
        qt-demo-init \
        qt4-embedded-assistant \
"

RRECOMMENDS_${PN} = " \
	libqt-embeddedxmlpatterns4 \
"

