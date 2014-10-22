SUMMARY="Basic networking tools"
DESCRIPTION="A collection of programs that form the base set of the NET-3 networking distribution for the Linux operating system"
HOMEPAGE = "http://net-tools.berlios.de/"
BUGTRACKER = "http://bugs.debian.org/net-tools"
LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://COPYING;md5=8ca43cbc842c2336e835926c2166c28b \
                    file://ifconfig.c;beginline=11;endline=15;md5=d1ca372080ad5401e23ca0afc35cf9ba"
PR = "r0"

SRC_URI = "${DEBIAN_MIRROR}/main/n/net-tools/net-tools_1.60.orig.tar.gz;name=tarball \
           ${DEBIAN_MIRROR}/main/n/net-tools/${BPN}_${PV}.diff.gz;apply=no;name=patch \
           file://net-tools-config.h \
           file://net-tools-config.make" 

S = "${WORKDIR}/net-tools-1.60"

SRC_URI[tarball.md5sum] = "ecaf37acb5b5daff4bdda77785fd916d"
SRC_URI[tarball.sha256sum] = "ec67967cf7b1a3a3828a84762fbc013ac50ee5dc9aa3095d5c591f302c2de0f5"

SRC_URI[patch.md5sum] = "524658bb8df5ff92c4a991f5edcaf240"
SRC_URI[patch.sha256sum] = "170cc024fcb34329f4c25fd88b5f160a06be5d6d3eaf0bc976650fd1b1a6235d"

inherit gettext

# The Makefile is lame, no parallel build
PARALLEL_MAKE = ""

# Unlike other Debian packages, net-tools *.diff.gz contains another series of
# patches maintained by quilt. So manually apply them before applying other local
# patches. Also remove all temp files before leaving, because do_patch() will pop 
# up all previously applied patches in the start
nettools_do_patch() {
	cd ${S}
	quilt pop -a || true
	if [ -d ${S}/.pc-nettools ]; then
		mv ${S}/.pc-nettools ${S}/.pc
		QUILT_PATCHES=${S}/debian/patches quilt pop -a
		rm -rf ${S}/.pc ${S}/debian
	fi
	patch -p1 < ${WORKDIR}/${BPN}_${PV}.diff	
	QUILT_PATCHES=${S}/debian/patches quilt push -a
	mv ${S}/.pc ${S}/.pc-nettools
}

do_unpack[cleandirs] += "${S}"

# We invoke base do_patch at end, to incorporate any local patch
python do_patch() {
    bb.build.exec_func('nettools_do_patch', d)
    bb.build.exec_func('patch_do_patch', d)
}

do_configure() {
	# net-tools has its own config mechanism requiring "make config"
	# we pre-generate desired options and copy to source directory instead
	cp ${WORKDIR}/net-tools-config.h    ${S}/config.h
	cp ${WORKDIR}/net-tools-config.make ${S}/config.make
}

do_compile() {
	# net-tools use COPTS/LOPTS to allow adding custom options
	export COPTS="$CFLAGS"
	export LOPTS="$LDFLAGS"
	unset CFLAGS
	unset LDFLAGS

	oe_runmake
}

do_install() {
	oe_runmake 'BASEDIR=${D}' install
}

inherit update-alternatives

base_sbindir_progs = "arp ifconfig ipmaddr iptunnel mii-tool nameif plipconfig rarp route slattach"
base_bindir_progs  = "dnsdomainname domainname hostname netstat nisdomainname ypdomainname"

ALTERNATIVE_${PN} = "${base_sbindir_progs} ${base_bindir_progs}"
python __anonymous() {
	for prog in d.getVar('base_sbindir_progs', True).split():
		d.setVarFlag('ALTERNATIVE_LINK_NAME', prog, '%s/%s' % (d.getVar('base_sbindir', True), prog))
	for prog in d.getVar('base_bindir_progs', True).split():
		d.setVarFlag('ALTERNATIVE_LINK_NAME', prog, '%s/%s' % (d.getVar('base_bindir', True), prog))
}
ALTERNATIVE_PRIORITY = "100"

