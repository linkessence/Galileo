require ofono.inc

S	 = "${WORKDIR}/git"
SRCREV = "14544d5996836f628613c2ce544380ee6fc8f514"
PV	 = "0.12-git${SRCPV}"
PR = "r5"

SRC_URI  = "git://git.kernel.org/pub/scm/network/ofono/ofono.git;protocol=git \
	    file://ofono"

EXTRA_OECONF += "\
    ${@base_contains('DISTRO_FEATURES', 'bluetooth','--enable-bluetooth', '--disable-bluetooth', d)} \
"

do_configure_prepend () {
  ${S}/bootstrap
}

