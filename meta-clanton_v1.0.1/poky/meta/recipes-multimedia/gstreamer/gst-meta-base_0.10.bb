DESCRIPTION = "Gstreamer package groups"
LICENSE = "MIT"
DEPENDS = "gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad"
LIC_FILES_CHKSUM = "file://${COREBASE}/LICENSE;md5=3f40d7994397109285ec7b81fdeb3b58 \
                    file://${COREBASE}/meta/COPYING.MIT;md5=3da9cfbcb788c80a0384361b4de20420"


PR = "r13"

PACKAGES = "\
    gst-meta-base \
    gst-meta-x11-base \
    gst-meta-audio \
    gst-meta-debug \
    gst-meta-video"

ALLOW_EMPTY_gst-meta-base = "1"
ALLOW_EMPTY_gst-meta-x11-base = "1"
ALLOW_EMPTY_gst-meta-audio = "1"
ALLOW_EMPTY_gst-meta-debug = "1"
ALLOW_EMPTY_gst-meta-video = "1"

RDEPENDS_gst-meta-base = "\
    ${@base_contains('DISTRO_FEATURES', 'x11', 'gst-meta-x11-base', '', d)} \
    gstreamer \
    gst-plugins-base-playbin \
    gst-plugins-base-decodebin \
    gst-plugins-base-decodebin2 \
    gst-plugins-base-gio \
    gst-plugins-base-alsa \
    gst-plugins-base-volume \
    gst-plugins-base-audioconvert \
    gst-plugins-base-audioresample \
    gst-plugins-base-typefindfunctions \
    gst-plugins-base-videoscale \
    gst-plugins-base-ffmpegcolorspace \
    gst-plugins-good-autodetect \
    gst-plugins-good-souphttpsrc"

RRECOMMENDS_gst-meta-x11-base = "\
    gst-plugins-base-ximagesink \
    gst-plugins-base-xvimagesink"

RDEPENDS_gst-meta-audio = "\
    gst-meta-base \
    gst-plugins-base-vorbis \
    gst-plugins-base-ogg \
    gst-plugins-good-wavparse \
    gst-plugins-good-flac \
    ${COMMERCIAL_AUDIO_PLUGINS}"


RDEPENDS_gst-meta-debug = "\
    gst-meta-base \
    gst-plugins-good-debug \
    gst-plugins-base-audiotestsrc \
    gst-plugins-base-videotestsrc"


RDEPENDS_gst-meta-video = "\
    gst-meta-base \
    gst-plugins-good-avi \
    gst-plugins-good-matroska \
    gst-plugins-base-theora \
    ${COMMERCIAL_VIDEO_PLUGINS}"

RRECOMMENDS_gst-meta-video = "\
    gst-meta-audio"
