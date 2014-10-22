SUMMARY = "Provides an icon to shut down the system cleanly"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COREBASE}/LICENSE;md5=3f40d7994397109285ec7b81fdeb3b58 \
                    file://${COREBASE}/meta/COPYING.MIT;md5=3da9cfbcb788c80a0384361b4de20420"

SRC_URI = "file://shutdown.desktop"

PR = "r1"

S = "${WORKDIR}"

do_install() {
	install -d ${D}${datadir}/applications
	install -m 0644 shutdown.desktop ${D}${datadir}/applications/
}

pkg_postinst_${PN} () {
    grep -q qemuarm $D${sysconfdir}/hostname && \
        sed -i $D${datadir}/applications/shutdown.desktop -e 's/^Exec=halt/Exec=reboot/' \
        || true
}

inherit allarch
