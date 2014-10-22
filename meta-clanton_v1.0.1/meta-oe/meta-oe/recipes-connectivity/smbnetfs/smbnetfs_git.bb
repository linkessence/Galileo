DESCRIPTION = "smbnetfs"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=eb723b61539feef013de476e68b5c50a"
HOMEPAGE ="http://sourceforge.net/projects/smbnetfs"

DEPENDS = "fuse samba"

inherit autotools gitpkgv

PKGV = "${GITPKGVTAG}"

SRCREV = "ace1c519d45fe488b9b7e6cc77a2bcadb6c83464"

SRC_URI = "git://smbnetfs.git.sourceforge.net/gitroot/smbnetfs/smbnetfs;protocol=git;branch=master"

S = "${WORKDIR}/git"
