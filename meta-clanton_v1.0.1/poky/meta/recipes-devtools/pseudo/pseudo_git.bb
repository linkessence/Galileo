require pseudo.inc

SRCREV = "b9eb2b5633b5a23efe72c950494728d93c2b5823"
PV = "1.5.1+git${SRCPV}"
PR = "r0"

DEFAULT_PREFERENCE = "-1"

SRC_URI = "git://git.yoctoproject.org/pseudo;protocol=git"

S = "${WORKDIR}/git"

