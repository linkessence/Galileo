SUMMARY = "Cron daemon for executing programs at set times"
DESCRIPTION = "Cronie contains the standard UNIX daemon crond that runs \
specified programs at scheduled times and related tools. It is based on the \
original cron and has security and configuration enhancements like the \
ability to use pam and SELinux."
HOMEPAGE = "https://fedorahosted.org/cronie/"
BUGTRACKER = "mmaslano@redhat.com"

# Internet Systems Consortium License
LICENSE = "ISC & BSD"
LIC_FILES_CHKSUM = "file://COPYING;md5=963ea0772a2adbdcd607a9b2ec320c11 \
                    file://src/cron.h;endline=20;md5=b425c334265026177128353a142633b4 \
                    file://src/popen.c;beginline=3;endline=31;md5=edd50742d8def712e9472dba353668a9"

SECTION = "utils"

DEPENDS += "${@base_contains('DISTRO_FEATURES', 'pam', 'libpam', '', d)}"
RDEPENDS_${PN} = "${@base_contains('DISTRO_FEATURES', 'pam', '${PAM_DEPS}', '', d)}"
PAM_DEPS = "libpam libpam-runtime pam-plugin-access pam-plugin-loginuid"

PR = "r0"

SRC_URI = "https://fedorahosted.org/releases/c/r/cronie/cronie-${PV}.tar.gz \
           file://crond.init \
           file://crontab \
           ${@base_contains('DISTRO_FEATURES', 'pam', '${PAM_SRC_URI}', '', d)}"

PAM_SRC_URI = "file://crond_pam_config.patch"


SRC_URI[md5sum] = "9133195e5e6f824ef460f5ccc533f1b7"
SRC_URI[sha256sum] = "bd7f6f118460c452bd1217a24b80fd3c000425d3de28731b98354a81a2133e92"

inherit autotools update-rc.d useradd

EXTRA_OECONF += "\
                ${@base_contains('DISTRO_FEATURES', 'pam', '--with-pam', '--without-pam', d)}"

INITSCRIPT_NAME = "crond"
INITSCRIPT_PARAMS = "start 90 2 3 4 5 . stop 60 0 1 6 ."

USERADD_PACKAGES = "${PN}"
GROUPADD_PARAM_${PN} = "crontab"

do_install_append () {
	install -d ${D}${sysconfdir}/sysconfig/
	install -d ${D}${sysconfdir}/init.d/
	install -m 0644 ${S}/crond.sysconfig ${D}${sysconfdir}/sysconfig/crond
	install -m 0755 ${WORKDIR}/crond.init ${D}${sysconfdir}/init.d/crond

	# below are necessary for a complete cron environment
	install -d ${D}${localstatedir}/spool/cron
	install -m 0755 ${WORKDIR}/crontab ${D}${sysconfdir}/
	mkdir -p ${D}${sysconfdir}/cron.d
	mkdir -p ${D}${sysconfdir}/cron.hourly
	mkdir -p ${D}${sysconfdir}/cron.daily
	mkdir -p ${D}${sysconfdir}/cron.weekly
	mkdir -p ${D}${sysconfdir}/cron.monthly
	touch ${D}${sysconfdir}/cron.deny
	
	# below setting is necessary to allow normal user using crontab

	# setgid for crontab binary
	chown root:crontab ${D}${bindir}/crontab
	chmod 2755 ${D}${bindir}/crontab

	# allow 'crontab' group write to /var/spool/cron
	chown root:crontab ${D}${localstatedir}/spool/cron
	chmod 770 ${D}${localstatedir}/spool/cron

	chmod 600 ${D}${sysconfdir}/crontab
}

FILES_${PN} += "${sysconfdir}/cron*"

