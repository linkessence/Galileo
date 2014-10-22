SUMMARY = "Tools to change and administer password and group data"
DESCRIPTION = "Tools to change and administer password and group data"
HOMEPAGE = "http://pkg-shadow.alioth.debian.org"
BUGTRACKER = "https://alioth.debian.org/tracker/?group_id=30580"
SECTION = "base utils"
LICENSE = "BSD | Artistic-1.0"
LIC_FILES_CHKSUM = "file://COPYING;md5=08c553a87d4e51bbed50b20e0adcaede \
                    file://src/passwd.c;beginline=8;endline=30;md5=d83888ea14ae61951982d77125947661"

PR = "r8"

SRC_URI = "http://pkg-shadow.alioth.debian.org/releases/${BPN}-${PV}.tar.bz2 \
           file://shadow.automake-1.11.patch \
           file://shadow-4.1.3-dots-in-usernames.patch \
           file://shadow-4.1.4.2-env-reset-keep-locale.patch \
           file://add_root_cmd_options.patch \
           file://disable-syslog.patch \
           file://useradd.patch \
           file://shadow_fix_for_automake-1.12.patch \
           "

SRC_URI[md5sum] = "b8608d8294ac88974f27b20f991c0e79"
SRC_URI[sha256sum] = "633f5bb4ea0c88c55f3642c97f9d25cbef74f82e0b4cf8d54e7ad6f9f9caa778" 

inherit autotools gettext native

EXTRA_OECONF += "--without-audit \
                 --without-libcrack \
                 --without-libpam \
                 --without-selinux \
                 --without-nscd"

do_install() {
	oe_runmake DESTDIR="${D}" sbindir="${base_sbindir}" usbindir="${sbindir}" install

	# Info dir listing isn't interesting at this point so remove it if it exists.
	if [ -e "${D}${infodir}/dir" ]; then
		rm -f ${D}${infodir}/dir
	fi

	# Enable CREATE_HOME by default.
	sed -i 's/#CREATE_HOME/CREATE_HOME/g' ${D}${sysconfdir}/login.defs

	# As we are on an embedded system, ensure the users mailbox is in
	# ~/ not /var/spool/mail by default, as who knows where or how big
	# /var is. The system MDA will set this later anyway.
	sed -i 's/MAIL_DIR/#MAIL_DIR/g' ${D}${sysconfdir}/login.defs
	sed -i 's/#MAIL_FILE/MAIL_FILE/g' ${D}${sysconfdir}/login.defs

	# Disable checking emails.
	sed -i 's/MAIL_CHECK_ENAB/#MAIL_CHECK_ENAB/g' ${D}${sysconfdir}/login.defs

	# Now we don't have a mail system. Disable mail creation for now.
	sed -i 's:/bin/bash:/bin/sh:g' ${D}${sysconfdir}/default/useradd
	sed -i '/^CREATE_MAIL_SPOOL/ s:^:#:' ${D}${sysconfdir}/default/useradd

	# Use users group by default
	sed -i 's,^GROUP=1000,GROUP=100,g' ${D}${sysconfdir}/default/useradd
}
