#
# Copyright (C) 2010 Intel Corporation
#

SUMMARY = "Linux Standard Base (LSB)"
DESCRIPTION = "Packages required to satisfy the Linux Standard Base (LSB) specification"
PR = "r10"
LICENSE = "MIT"

inherit packagegroup

PACKAGES = "\
    packagegroup-core-lsb \
    packagegroup-core-sys-extended \
    packagegroup-core-db \
    packagegroup-core-perl \
    packagegroup-core-python \
    packagegroup-core-tcl \
    packagegroup-core-lsb-misc \
    packagegroup-core-lsb-core \
    packagegroup-core-lsb-perl \
    packagegroup-core-lsb-python \
    packagegroup-core-lsb-desktop \
    packagegroup-core-lsb-runtime-add \
    "


RPROVIDES_packagegroup-core-lsb = "task-core-lsb"
RDEPENDS_packagegroup-core-lsb = "\
    packagegroup-core-sys-extended \
    packagegroup-core-db \
    packagegroup-core-perl \
    packagegroup-core-python \
    packagegroup-core-tcl \
    packagegroup-core-lsb-misc \
    packagegroup-core-lsb-core \
    packagegroup-core-lsb-perl \
    packagegroup-core-lsb-python \
    packagegroup-core-lsb-desktop \
    packagegroup-core-lsb-runtime-add \
    "


RDEPENDS_packagegroup-core-sys-extended = "\
    curl \
    dhcp-client \
    gamin \
    hdparm \
    libaio \
    lrzsz \
    lzo \
    mc \
    mdadm \
    minicom \
    neon \
    parted \
    ${PTH} \
    quota \
    screen \
    setserial \
    sysstat \
    unzip \
    watchdog \
    wget \
    which \
    xinetd \
    zip \
    "

RDEPENDS_packagegroup-core-db = "\
    db \
    sqlite3 \
    "

RDEPENDS_packagegroup-core-perl = "\
    gdbm \
    perl \
    zlib \
    "


RDEPENDS_packagegroup-core-python = "\
    expat \
    gdbm \
    gmp \
    ncurses \
    openssl \
    python \
    readline \
    zip \
    "

RDEPENDS_packagegroup-core-tcl = "\
    tcl \
    "

# Miscellaneous packages required by LSB (or LSB tests)
RDEPENDS_packagegroup-core-lsb-misc = "\
    chkconfig \
    gettext \
    gettext-runtime \
    groff \
    lsbinitscripts \
    lsbtest \
    lsof \
    strace \
    libusb1 \
    usbutils \
    rpm \
    "

SUMMARY_packagegroup-core-lsb-core = "LSB Core"
DESCRIPTION_packagegroup-core-lsb-core = "Packages required to support commands/libraries \
    specified in the LSB Core specification"
RDEPENDS_packagegroup-core-lsb-core = "\
    at \
    bash \
    bc \
    binutils \
    binutils-symlinks \
    coreutils \
    cpio \
    cronie \
    cups \
    diffutils \
    ed \
    eglibc-utils \
    elfutils \
    file \
    findutils \
    fontconfig-utils \
    foomatic-filters \
    gawk \
    ghostscript \
    grep \
    gzip \
    localedef \
    lsb \
    m4 \
    mailx \
    make \
    man \
    man-pages \
    mktemp \
    msmtp \
    patch \
    pax \
    procps \
    psmisc \
    sed \
    shadow \
    tar \
    time \
    util-linux \
    xdg-utils \
    \
    eglibc \
    libgcc \
    libpam \
    libxml2 \
    ncurses \
    zlib \
    nspr \
    libpng12 \
"

SUMMARY_packagegroup-core-lsb-perl = "LSB Runtime Languages (Perl)"
DESCRIPTION_packagegroup-core-lsb-perl = "Packages required to support libraries \
    specified in the LSB Runtime languages specification (Perl parts)"
RDEPENDS_packagegroup-core-lsb-perl = "\
    perl \
    perl-modules \
    perl-misc \
    perl-pod \
    perl-dev \
    perl-doc \
"

SUMMARY_packagegroup-core-lsb-python = "LSB Runtime Languages (Python)"
DESCRIPTION_packagegroup-core-lsb-python = "Packages required to support libraries \
    specified in the LSB Runtime languages specification (Python parts)"
RDEPENDS_packagegroup-core-lsb-python = "\
    python \
    python-modules \
    python-misc \
"

SUMMARY_packagegroup-core-lsb-desktop = "LSB Desktop"
DESCRIPTION_packagegroup-core-lsb-desktop = "Packages required to support libraries \
    specified in the LSB Desktop specification"
RDEPENDS_packagegroup-core-lsb-desktop = "\
    libqtcore4 \
    libqtgui4 \
    libqtsql4 \
    libqtsvg4 \
    libqtxml4 \
    libqtnetwork4 \
    libxt \
    libxxf86vm \
    libdrm \
    libglu \
    libxi \
    libxtst \
    qt4-plugin-sqldriver-sqlite \
    libx11-locale \
    xorg-minimal-fonts \
    gdk-pixbuf-loader-ico \
    gdk-pixbuf-loader-bmp \
    gdk-pixbuf-loader-ani \
    gdk-pixbuf-xlib \
    liberation-fonts \
    gtk+ \
    atk \
    libasound \
    ${@base_contains("DISTRO_FEATURES", "opengl", "libqtopengl4", "", d)} \
"

RDEPENDS_packagegroup-core-lsb-runtime-add = "\
    ldd \
    pam-plugin-wheel \
    e2fsprogs-mke2fs \
    mkfontdir \
    liburi-perl \
    libxml-parser-perl \
    libxml-perl \
    libxml-sax-perl \
    eglibc-localedatas \
    eglibc-gconvs \
    eglibc-charmaps \
    eglibc-binaries \
    eglibc-localedata-posix \
    eglibc-extra-nss \
    eglibc-pcprofile \
    libclass-isa-perl \
    libenv-perl \
    libdumpvalue-perl \
    libfile-checktree-perl \
    libi18n-collate-perl \
    libpod-plainer-perl \
"

PTH = "pth"
PTH_libc-uclibc = ""

