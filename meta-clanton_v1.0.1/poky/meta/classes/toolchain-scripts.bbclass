inherit siteinfo kernel-arch

# We want to be able to change the value of MULTIMACH_TARGET_SYS, because it
# doesn't always match our expectations... but we default to the stock value
REAL_MULTIMACH_TARGET_SYS ?= "${MULTIMACH_TARGET_SYS}"

# This function creates an environment-setup-script for use in a deployable SDK
toolchain_create_sdk_env_script () {
	# Create environment setup script
	script=${1:-${SDK_OUTPUT}/${SDKPATH}/environment-setup-${REAL_MULTIMACH_TARGET_SYS}}
	rm -f $script
	touch $script
	echo 'export PATH=${SDKPATHNATIVE}${bindir_nativesdk}:${SDKPATHNATIVE}${bindir_nativesdk}/${REAL_MULTIMACH_TARGET_SYS}:$PATH' >> $script
	echo 'export PKG_CONFIG_SYSROOT_DIR=${SDKTARGETSYSROOT}' >> $script
	echo 'export PKG_CONFIG_PATH=${SDKTARGETSYSROOT}${libdir}/pkgconfig' >> $script
	echo 'export CONFIG_SITE=${SDKPATH}/site-config-${REAL_MULTIMACH_TARGET_SYS}' >> $script
	echo 'export CC="${TARGET_PREFIX}gcc ${TARGET_CC_ARCH} --sysroot=${SDKTARGETSYSROOT}"' >> $script
	echo 'export CXX="${TARGET_PREFIX}g++ ${TARGET_CC_ARCH} --sysroot=${SDKTARGETSYSROOT}"' >> $script
	echo 'export CPP="${TARGET_PREFIX}gcc -E ${TARGET_CC_ARCH} --sysroot=${SDKTARGETSYSROOT}"' >> $script
	echo 'export AS="${TARGET_PREFIX}as ${TARGET_AS_ARCH}"' >> $script
	echo 'export LD="${TARGET_PREFIX}ld ${TARGET_LD_ARCH} --sysroot=${SDKTARGETSYSROOT}"' >> $script
	echo 'export GDB=${TARGET_PREFIX}gdb' >> $script
	echo 'export STRIP=${TARGET_PREFIX}strip' >> $script
	echo 'export RANLIB=${TARGET_PREFIX}ranlib' >> $script
	echo 'export OBJCOPY=${TARGET_PREFIX}objcopy' >> $script
	echo 'export OBJDUMP=${TARGET_PREFIX}objdump' >> $script
	echo 'export AR=${TARGET_PREFIX}ar' >> $script
	echo 'export NM=${TARGET_PREFIX}nm' >> $script
	echo 'export M4=m4' >> $script
	echo 'export TARGET_PREFIX=${TARGET_PREFIX}' >> $script
	echo 'export CONFIGURE_FLAGS="--target=${TARGET_SYS} --host=${TARGET_SYS} --build=${SDK_ARCH}-linux --with-libtool-sysroot=${SDKTARGETSYSROOT}"' >> $script
	if [ "${TARGET_OS}" = "darwin8" ]; then
		echo 'export TARGET_CFLAGS="-I${SDKTARGETSYSROOT}${includedir}"' >> $script
		echo 'export TARGET_LDFLAGS="-L${SDKTARGETSYSROOT}${libdir}"' >> $script
		# Workaround darwin toolchain sysroot path problems
		cd ${SDK_OUTPUT}${SDKTARGETSYSROOT}/usr
		ln -s /usr/local local
	fi
	echo 'export CFLAGS="${TARGET_CFLAGS}"' >> $script
	echo 'export CXXFLAGS="${TARGET_CXXFLAGS}"' >> $script
	echo 'export LDFLAGS="${TARGET_LDFLAGS}"' >> $script
	echo 'export CPPFLAGS="${TARGET_CPPFLAGS}"' >> $script
	echo 'export OECORE_NATIVE_SYSROOT="${SDKPATHNATIVE}"' >> $script
	echo 'export OECORE_TARGET_SYSROOT="${SDKTARGETSYSROOT}"' >> $script
	echo 'export OECORE_ACLOCAL_OPTS="-I ${SDKPATHNATIVE}/usr/share/aclocal"' >> $script
	echo 'export OECORE_DISTRO_VERSION="${DISTRO_VERSION}"' >> $script
	echo 'export OECORE_SDK_VERSION="${SDK_VERSION}"' >> $script
	echo 'export PYTHONHOME=${SDKPATHNATIVE}${prefix_nativesdk}' >> $script
	echo 'export ARCH=${ARCH}' >> $script
}

# This function creates an environment-setup-script in the TMPDIR which enables
# a OE-core IDE to integrate with the build tree
toolchain_create_tree_env_script () {
	script=${TMPDIR}/environment-setup-${REAL_MULTIMACH_TARGET_SYS}
	rm -f $script
	touch $script
	echo 'export PATH=${STAGING_DIR_NATIVE}/usr/bin:${PATH}' >> $script
	echo 'export PKG_CONFIG_SYSROOT_DIR=${PKG_CONFIG_SYSROOT_DIR}' >> $script
	echo 'export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}' >> $script

	echo 'export CONFIG_SITE="${@siteinfo_get_files(d)}"' >> $script

	echo 'export CC="${TARGET_PREFIX}gcc ${TARGET_CC_ARCH} --sysroot=${STAGING_DIR_TARGET}"' >> $script
	echo 'export CXX="${TARGET_PREFIX}g++ ${TARGET_CC_ARCH} --sysroot=${STAGING_DIR_TARGET}"' >> $script
	echo 'export CPP="${TARGET_PREFIX}gcc -E ${TARGET_CC_ARCH} --sysroot=${STAGING_DIR_TARGET}"' >> $script
	echo 'export AS="${TARGET_PREFIX}as ${TARGET_AS_ARCH}"' >> $script
	echo 'export LD="${TARGET_PREFIX}ld ${TARGET_LD_ARCH} --sysroot=${STAGING_DIR_TARGET}"' >> $script
	echo 'export GDB=${TARGET_PREFIX}gdb' >> $script
	echo 'export STRIP=${TARGET_PREFIX}strip' >> $script
	echo 'export RANLIB=${TARGET_PREFIX}ranlib' >> $script
	echo 'export OBJCOPY=${TARGET_PREFIX}objcopy' >> $script
	echo 'export OBJDUMP=${TARGET_PREFIX}objdump' >> $script
	echo 'export AR=${TARGET_PREFIX}ar' >> $script
	echo 'export NM=${TARGET_PREFIX}nm' >> $script
	echo 'export TARGET_PREFIX=${TARGET_PREFIX}' >> $script
	echo 'export CONFIGURE_FLAGS="--target=${TARGET_SYS} --host=${TARGET_SYS} --build=${BUILD_SYS} --with-libtool-sysroot=${STAGING_DIR_TARGET}"' >> $script
	if [ "${TARGET_OS}" = "darwin8" ]; then
		echo 'export TARGET_CFLAGS="-I${STAGING_DIR}${MACHINE}${includedir}"' >> $script
		echo 'export TARGET_LDFLAGS="-L${STAGING_DIR}${MACHINE}${libdir}"' >> $script
		# Workaround darwin toolchain sysroot path problems
		cd ${SDK_OUTPUT}${SDKTARGETSYSROOT}/usr
		ln -s /usr/local local
	fi
	echo 'export CFLAGS="${TARGET_CFLAGS}"' >> $script
	echo 'export CXXFLAGS="${TARGET_CXXFLAGS}"' >> $script
	echo 'export LDFLAGS="${TARGET_LDFLAGS}"' >> $script
	echo 'export CPPFLAGS="${TARGET_CPPFLAGS}"' >> $script
	echo 'export OECORE_NATIVE_SYSROOT="${STAGING_DIR_NATIVE}"' >> $script
	echo 'export OECORE_TARGET_SYSROOT="${STAGING_DIR_TARGET}"' >> $script
	echo 'export OECORE_ACLOCAL_OPTS="-I ${STAGING_DIR_NATIVE}/usr/share/aclocal"' >> $script
	echo 'export OECORE_DISTRO_VERSION="${DISTRO_VERSION}"' >> $script
	echo 'export OECORE_SDK_VERSION="${SDK_VERSION}"' >> $script
	echo 'export ARCH=${ARCH}' >> $script
}

# This function creates an environment-setup-script for use by the ADT installer
toolchain_create_sdk_env_script_for_installer () {
	# Create environment setup script
	local multimach_target_sys=$1
	script=${SDK_OUTPUT}/${SDKPATH}/environment-setup-${multimach_target_sys}
	rm -f $script
	touch $script
	echo 'export PATH=${SDKPATHNATIVE}${bindir_nativesdk}:${SDKPATHNATIVE}${bindir_nativesdk}/'"${multimach_target_sys}"':$PATH' >> $script
	echo 'export PKG_CONFIG_SYSROOT_DIR=##SDKTARGETSYSROOT##' >> $script
	echo 'export PKG_CONFIG_PATH=##SDKTARGETSYSROOT##${target_libdir}/pkgconfig' >> $script
	echo 'export CONFIG_SITE=${SDKPATH}/site-config-'"${multimach_target_sys}" >> $script
	echo 'export CC="${TARGET_PREFIX}gcc ${TARGET_CC_ARCH} --sysroot=##SDKTARGETSYSROOT##"' >> $script
	echo 'export CXX="${TARGET_PREFIX}g++ ${TARGET_CC_ARCH} --sysroot=##SDKTARGETSYSROOT##"' >> $script
	echo 'export CPP="${TARGET_PREFIX}gcc -E ${TARGET_CC_ARCH} --sysroot=##SDKTARGETSYSROOT##"' >> $script
	echo 'export AS="${TARGET_PREFIX}as ${TARGET_AS_ARCH}"' >> $script
	echo 'export LD="${TARGET_PREFIX}ld ${TARGET_LD_ARCH} --sysroot=##SDKTARGETSYSROOT##"' >> $script
	echo 'export GDB=${TARGET_PREFIX}gdb' >> $script
	echo 'export STRIP=${TARGET_PREFIX}strip' >> $script
	echo 'export RANLIB=${TARGET_PREFIX}ranlib' >> $script
	echo 'export OBJCOPY=${TARGET_PREFIX}objcopy' >> $script
	echo 'export OBJDUMP=${TARGET_PREFIX}objdump' >> $script
	echo 'export AR=${TARGET_PREFIX}ar' >> $script
	echo 'export NM=${TARGET_PREFIX}nm' >> $script
	echo 'export TARGET_PREFIX=${TARGET_PREFIX}' >> $script
	echo 'export CONFIGURE_FLAGS="--target=${TARGET_SYS} --host=${TARGET_SYS} --build=${SDK_ARCH}-linux --with-libtool-sysroot=##SDKTARGETSYSROOT##"' >> $script
	if [ "${TARGET_OS}" = "darwin8" ]; then
		echo 'export TARGET_CFLAGS="-I##SDKTARGETSYSROOT##${target_includedir}"' >> $script
		echo 'export TARGET_LDFLAGS="-L##SDKTARGETSYSROOT##{target_libdir}"' >> $script
		# Workaround darwin toolchain sysroot path problems
		cd ${SDK_OUTPUT}${SDKTARGETSYSROOT}/usr
		ln -s /usr/local local
	fi
	echo 'export CFLAGS="${TARGET_CFLAGS}"' >> $script
	echo 'export CXXFLAGS="${TARGET_CXXFLAGS}"' >> $script
	echo 'export LDFLAGS="${TARGET_LDFLAGS}"' >> $script
	echo 'export CPPFLAGS="${TARGET_CPPFLAGS}"' >> $script
	echo 'export OECORE_NATIVE_SYSROOT="${SDKPATHNATIVE}"' >> $script
	echo 'export OECORE_TARGET_SYSROOT="##SDKTARGETSYSROOT##"' >> $script
        echo 'export OECORE_ACLOCAL_OPTS="-I ${SDKPATHNATIVE}/usr/share/aclocal"' >> $script
	echo 'export OECORE_DISTRO_VERSION="${DISTRO_VERSION}"' >> $script
	echo 'export OECORE_SDK_VERSION="${SDK_VERSION}"' >> $script
	echo 'export PYTHONHOME=${SDKPATHNATIVE}${prefix_nativesdk}' >> $script
	echo 'export ARCH=${ARCH}' >> $script
}

#we get the cached site config in the runtime
TOOLCHAIN_CONFIGSITE_NOCACHE = "${@siteinfo_get_files(d, True)}"
TOOLCHAIN_CONFIGSITE_SYSROOTCACHE = "${STAGING_DATADIR}/${TARGET_SYS}_config_site.d"
TOOLCHAIN_NEED_CONFIGSITE_CACHE = "${TCLIBC} ncurses"

#This function create a site config file
toolchain_create_sdk_siteconfig () {
	local siteconfig=$1

	rm -f $siteconfig
	touch $siteconfig

	for sitefile in ${TOOLCHAIN_CONFIGSITE_NOCACHE} ; do
		cat $sitefile >> $siteconfig
	done

	#get cached site config
	for sitefile in ${TOOLCHAIN_NEED_CONFIGSITE_CACHE}; do
		if [ -r ${TOOLCHAIN_CONFIGSITE_SYSROOTCACHE}/${sitefile}_config ]; then
			cat ${TOOLCHAIN_CONFIGSITE_SYSROOTCACHE}/${sitefile}_config >> $siteconfig
		fi
	done
}
# The immediate expansion above can result in unwanted path dependencies here
toolchain_create_sdk_siteconfig[vardepsexclude] = "TOOLCHAIN_CONFIGSITE_SYSROOTCACHE"

#This function create a version information file
toolchain_create_sdk_version () {
	local versionfile=$1
	rm -f $versionfile
	touch $versionfile
	echo 'Distro: ${DISTRO}' >> $versionfile
	echo 'Distro Version: ${DISTRO_VERSION}' >> $versionfile
	echo 'Metadata Revision: ${METADATA_REVISION}' >> $versionfile
	echo 'Timestamp: ${DATETIME}' >> $versionfile
}
toolchain_create_sdk_version[vardepsexclude] = "DATETIME"

python __anonymous () {
    deps = ""
    for dep in (d.getVar('TOOLCHAIN_NEED_CONFIGSITE_CACHE', True) or "").split():
        deps += " %s:do_populate_sysroot" % dep
    d.appendVarFlag('do_configure', 'depends', deps)
}
