DESCRIPTION = "Opencv : The Open Computer Vision Library"
HOMEPAGE = "http://opencv.willowgarage.com/wiki/"
SECTION = "libs"
LICENSE = "BSD"

DEPENDS = "opencv"

LIC_FILES_CHKSUM = "file://include/opencv2/opencv.hpp;endline=41;md5=6d690d8488a6fca7a2c192932466bb14 \
"

SRC_URI = "${SOURCEFORGE_MIRROR}/opencvlibrary/opencv-unix/${PV}/opencv-${PV}.zip \
"

SRC_URI[md5sum] = "7f958389e71c77abdf5efe1da988b80c"
SRC_URI[sha256sum] = "803010848154988e9cbda8b3fa857fcbb27382c2946ed729e1a7e40600bb4c71"

S = "${WORKDIR}/opencv-${PV}"

do_install() {
    cd samples/c
    install -d ${D}/${bindir}
    install -d ${D}/${datadir}/opencv/samples

    cp * ${D}/${datadir}/opencv/samples || true

    for i in *.c; do
        echo "compiling $i"
        ${CXX} ${CFLAGS} ${LDFLAGS} -ggdb `pkg-config --cflags opencv` -o `basename $i .c` $i `pkg-config --libs opencv` || true
        install -m 0755 `basename $i .c` ${D}/${bindir} || true
        rm ${D}/${datadir}/opencv/samples/`basename $i .c` || true
    done
    for i in *.cpp; do
        echo "compiling $i"
        ${CXX} ${CFLAGS} ${LDFLAGS} -ggdb `pkg-config --cflags opencv` -o `basename $i .cpp` $i `pkg-config --libs opencv` || true
        install -m 0755 `basename $i .cpp` ${D}/${bindir} || true
        rm ${D}/${datadir}/opencv/samples/`basename $i .cpp` || true
    done
}

FILES_${PN}-dev += "${datadir}/opencv/samples/*.c* ${datadir}/opencv/samples/*.vcp* ${datadir}/opencv/samples/build*" 
FILES_${PN} += "${bindir} ${datadir}/opencv"
