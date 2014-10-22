# BB Class inspired by ebuild.sh
#
# This class will test files after installation for certain
# security issues and other kind of issues.
#
# Checks we do:
#  -Check the ownership and permissions
#  -Check the RUNTIME path for the $TMPDIR
#  -Check if .la files wrongly point to workdir
#  -Check if .pc files wrongly point to workdir
#  -Check if packages contains .debug directories or .so files
#   where they should be in -dev or -dbg
#  -Check if config.log contains traces to broken autoconf tests
#  -Ensure that binaries in base_[bindir|sbindir|libdir] do not link
#   into exec_prefix
#  -Check that scripts in base_[bindir|sbindir|libdir] do not reference
#   files under exec_prefix


inherit package
PACKAGE_DEPENDS += "${QADEPENDS}"
PACKAGEFUNCS += " do_package_qa "

# unsafe-references-in-binaries requires prelink-rtld from
# prelink-native, but we don't want this DEPENDS for -native builds
QADEPENDS = "prelink-native"
QADEPENDS_class-native = ""
QADEPENDS_class-nativesdk = ""

#
# dictionary for elf headers
#
# feel free to add and correct.
#
#           TARGET_OS  TARGET_ARCH   MACHINE, OSABI, ABIVERSION, Little Endian, 32bit?
def package_qa_get_machine_dict():
    return {
            "darwin9" : { 
                        "arm" :       (40,     0,    0,          True,          32),
                      },
            "linux" : { 
                        "aarch64" :   (183,    0,    0,          True,          64),
                        "arm" :       (40,    97,    0,          True,          32),
                        "armeb":      (40,    97,    0,          False,         32),
                        "powerpc":    (20,     0,    0,          False,         32),
                        "powerpc64":  (21,     0,    0,          False,         64),
                        "i386":       ( 3,     0,    0,          True,          32),
                        "i486":       ( 3,     0,    0,          True,          32),
                        "i586":       ( 3,     0,    0,          True,          32),
                        "i686":       ( 3,     0,    0,          True,          32),
                        "x86_64":     (62,     0,    0,          True,          64),
                        "ia64":       (50,     0,    0,          True,          64),
                        "alpha":      (36902,  0,    0,          True,          64),
                        "hppa":       (15,     3,    0,          False,         32),
                        "m68k":       ( 4,     0,    0,          False,         32),
                        "mips":       ( 8,     0,    0,          False,         32),
                        "mipsel":     ( 8,     0,    0,          True,          32),
                        "mips64":     ( 8,     0,    0,          False,         64),
                        "mips64el":   ( 8,     0,    0,          True,          64),
                        "s390":       (22,     0,    0,          False,         32),
                        "sh4":        (42,     0,    0,          True,          32),
                        "sparc":      ( 2,     0,    0,          False,         32),
                        "microblaze":  (189,   0,    0,          False,         32),
                        "microblazeel":(189,   0,    0,          True,          32),
                      },
            "linux-uclibc" : { 
                        "arm" :       (  40,    97,    0,          True,          32),
                        "armeb":      (  40,    97,    0,          False,         32),
                        "powerpc":    (  20,     0,    0,          False,         32),
                        "i386":       (   3,     0,    0,          True,          32),
                        "i486":       (   3,     0,    0,          True,          32),
                        "i586":       (   3,     0,    0,          True,          32),
                        "i686":       (   3,     0,    0,          True,          32),
                        "x86_64":     (  62,     0,    0,          True,          64),
                        "mips":       (   8,     0,    0,          False,         32),
                        "mipsel":     (   8,     0,    0,          True,          32),
                        "mips64":     (   8,     0,    0,          False,         64),
                        "mips64el":   (   8,     0,    0,          True,          64),
                        "avr32":      (6317,     0,    0,          False,         32),
			"sh4":        (42,	 0,    0,          True,          32),

                      },
            "uclinux-uclibc" : {
                        "bfin":       ( 106,     0,    0,          True,         32),
                      }, 
            "linux-gnueabi" : {
                        "arm" :       (40,     0,    0,          True,          32),
                        "armeb" :     (40,     0,    0,          False,         32),
                      },
            "linux-uclibceabi" : {
                        "arm" :       (40,     0,    0,          True,          32),
                        "armeb" :     (40,     0,    0,          False,         32),
                      },
            "linux-gnuspe" : {
                        "powerpc":    (20,     0,    0,          False,         32),
                      },
            "linux-uclibcspe" : {
                        "powerpc":    (20,     0,    0,          False,         32),
                      },
            "linux-gnu" :       {
                        "powerpc":    (20,     0,    0,          False,         32),
                        "sh4":        (42,     0,    0,          True,          32),
                      },
            "linux-gnux32" :       {
                        "x86_64":     (62,     0,    0,          True,          32),
                      },
            "linux-gnun32" :       {
                        "mips64":       ( 8,     0,    0,          False,         32),
                        "mips64el":     ( 8,     0,    0,          True,          32),
                      },
        }


# Currently not being used by default "desktop"
WARN_QA ?= "ldflags useless-rpaths rpaths staticdev libdir xorg-driver-abi textrel"
ERROR_QA ?= "dev-so debug-deps dev-deps debug-files arch la2 pkgconfig la perms dep-cmp pkgvarcheck"

ALL_QA = "${WARN_QA} ${ERROR_QA}"

def package_qa_clean_path(path,d):
    """ Remove the common prefix from the path. In this case it is the TMPDIR"""
    return path.replace(d.getVar('TMPDIR',True),"")

def package_qa_write_error(error, d):
    logfile = d.getVar('QA_LOGFILE', True)
    if logfile:
        p = d.getVar('P', True)
        f = file( logfile, "a+")
        print >> f, "%s: %s" % (p, error)
        f.close()

def package_qa_handle_error(error_class, error_msg, d):
    package_qa_write_error(error_msg, d)
    if error_class in (d.getVar("ERROR_QA", True) or "").split():
        bb.error("QA Issue: %s" % error_msg)
        return False
    else:
        bb.warn("QA Issue: %s" % error_msg)
        return True

QAPATHTEST[libexec] = "package_qa_check_libexec"
def package_qa_check_libexec(path,name, d, elf, messages):

    # Skip the case where the default is explicitly /usr/libexec
    libexec = d.getVar('libexecdir', True)
    if libexec == "/usr/libexec":
        return True

    if 'libexec' in path.split(os.path.sep):
        messages.append("%s: %s is using libexec please relocate to %s" % (name, package_qa_clean_path(path, d), libexec))
        return False

    return True

QAPATHTEST[rpaths] = "package_qa_check_rpath"
def package_qa_check_rpath(file,name, d, elf, messages):
    """
    Check for dangerous RPATHs
    """
    if not elf:
        return

    if os.path.islink(file):
        return

    bad_dirs = [d.getVar('TMPDIR', True) + "/work", d.getVar('STAGING_DIR_TARGET', True)]

    if not bad_dirs[0] in d.getVar('WORKDIR', True):
        bb.fatal("This class assumed that WORKDIR is ${TMPDIR}/work... Not doing any check")

    phdrs = elf.run_objdump("-p", d)

    import re
    rpath_re = re.compile("\s+RPATH\s+(.*)")
    for line in phdrs.split("\n"):
    	m = rpath_re.match(line)
	if m:
	    rpath = m.group(1)
	    for dir in bad_dirs:
                if dir in rpath:
                    messages.append("package %s contains bad RPATH %s in file %s" % (name, rpath, file))

QAPATHTEST[useless-rpaths] = "package_qa_check_useless_rpaths"
def package_qa_check_useless_rpaths(file, name, d, elf, messages):
    """
    Check for RPATHs that are useless but not dangerous
    """
    def rpath_eq(a, b):
        return os.path.normpath(a) == os.path.normpath(b)

    if not elf:
        return

    if os.path.islink(file):
        return

    libdir = d.getVar("libdir", True)
    base_libdir = d.getVar("base_libdir", True)

    phdrs = elf.run_objdump("-p", d)

    import re
    rpath_re = re.compile("\s+RPATH\s+(.*)")
    for line in phdrs.split("\n"):
    	m = rpath_re.match(line)
	if m:
	   rpath = m.group(1)
	   if rpath_eq(rpath, libdir) or rpath_eq(rpath, base_libdir):
	      # The dynamic linker searches both these places anyway.  There is no point in
	      # looking there again.
	      messages.append("%s: %s contains probably-redundant RPATH %s" % (name, package_qa_clean_path(file, d), rpath))

QAPATHTEST[dev-so] = "package_qa_check_dev"
def package_qa_check_dev(path, name, d, elf, messages):
    """
    Check for ".so" library symlinks in non-dev packages
    """

    if not name.endswith("-dev") and not name.endswith("-dbg") and not name.endswith("-ptest") and not name.startswith("nativesdk-") and path.endswith(".so") and os.path.islink(path):
        messages.append("non -dev/-dbg/-nativesdk package contains symlink .so: %s path '%s'" % \
                 (name, package_qa_clean_path(path,d)))

QAPATHTEST[staticdev] = "package_qa_check_staticdev"
def package_qa_check_staticdev(path, name, d, elf, messages):
    """
    Check for ".a" library in non-staticdev packages
    There are a number of exceptions to this rule, -pic packages can contain
    static libraries, the _nonshared.a belong with their -dev packages and
    libgcc.a, libgcov.a will be skipped in their packages
    """

    if not name.endswith("-pic") and not name.endswith("-staticdev") and not name.endswith("-ptest") and path.endswith(".a") and not path.endswith("_nonshared.a"):
        messages.append("non -staticdev package contains static .a library: %s path '%s'" % \
                 (name, package_qa_clean_path(path,d)))

def package_qa_check_libdir(d):
    """
    Check for wrong library installation paths. For instance, catch
    recipes installing /lib/bar.so when ${base_libdir}="lib32" or
    installing in /usr/lib64 when ${libdir}="/usr/lib"
    """
    import re

    pkgd = d.getVar('PKGD', True)
    base_libdir = d.getVar("base_libdir",True) + os.sep
    libdir = d.getVar("libdir", True) + os.sep
    exec_prefix = d.getVar("exec_prefix", True) + os.sep

    messages = []
    my_files = []

    for root, dirs, files in os.walk(pkgd):
        for file in files:
            full_path = os.path.join(root,file)
            my_files.append(full_path[len(pkgd):])

    lib_re = re.compile("^/lib.*\.so")
    exec_re = re.compile("^%s.*/lib*.\.so" % exec_prefix)

    for file in my_files:
        if lib_re.match(file):
            if base_libdir not in file:
                messages.append("Found library in wrong location: %s" % file)
        if exec_re.match(file):
            if libdir not in file:
                messages.append("Found library in wrong location: %s" % file)
    if messages:
        package_qa_handle_error("libdir", "\n".join(messages), d)

QAPATHTEST[debug-files] = "package_qa_check_dbg"
def package_qa_check_dbg(path, name, d, elf, messages):
    """
    Check for ".debug" files or directories outside of the dbg package
    """

    if not "-dbg" in name and not "-ptest" in name:
        if '.debug' in path.split(os.path.sep):
            messages.append("non debug package contains .debug directory: %s path %s" % \
                     (name, package_qa_clean_path(path,d)))

QAPATHTEST[perms] = "package_qa_check_perm"
def package_qa_check_perm(path,name,d, elf, messages):
    """
    Check the permission of files
    """
    return

QAPATHTEST[unsafe-references-in-binaries] = "package_qa_check_unsafe_references_in_binaries"
def package_qa_check_unsafe_references_in_binaries(path, name, d, elf, messages):
	"""
	Ensure binaries in base_[bindir|sbindir|libdir] do not link to files under exec_prefix
	"""
	if unsafe_references_skippable(path, name, d):
		return

	if elf:
		import subprocess as sub
		pn = d.getVar('PN', True)

		exec_prefix = d.getVar('exec_prefix', True)
		sysroot_path = d.getVar('STAGING_DIR_TARGET', True)
		sysroot_path_usr = sysroot_path + exec_prefix

		try:
			ldd_output = bb.process.Popen(["prelink-rtld", "--root", sysroot_path, path], stdout=sub.PIPE).stdout.read()
		except bb.process.CmdError:
			error_msg = pn + ": prelink-rtld aborted when processing %s" % path
			package_qa_handle_error("unsafe-references-in-binaries", error_msg, d)
			return False

		if sysroot_path_usr in ldd_output:
			ldd_output = ldd_output.replace(sysroot_path, "")

			pkgdest = d.getVar('PKGDEST', True)
			packages = d.getVar('PACKAGES', True)

			for package in packages.split():
				short_path = path.replace('%s/%s' % (pkgdest, package), "", 1)
				if (short_path != path):
					break

			base_err = pn + ": %s, installed in the base_prefix, requires a shared library under exec_prefix (%s)" % (short_path, exec_prefix)
			for line in ldd_output.split('\n'):
				if exec_prefix in line:
					error_msg = "%s: %s" % (base_err, line.strip())
					package_qa_handle_error("unsafe-references-in-binaries", error_msg, d)

			return False

QAPATHTEST[unsafe-references-in-scripts] = "package_qa_check_unsafe_references_in_scripts"
def package_qa_check_unsafe_references_in_scripts(path, name, d, elf, messages):
	"""
	Warn if scripts in base_[bindir|sbindir|libdir] reference files under exec_prefix
	"""
	if unsafe_references_skippable(path, name, d):
		return

	if not elf:
		import stat
		import subprocess
		pn = d.getVar('PN', True)

		# Ensure we're checking an executable script
		statinfo = os.stat(path)
		if bool(statinfo.st_mode & stat.S_IXUSR):
			# grep shell scripts for possible references to /exec_prefix/
			exec_prefix = d.getVar('exec_prefix', True)
			statement = "grep -e '%s/' %s > /dev/null" % (exec_prefix, path)
			if subprocess.call(statement, shell=True) == 0:
				error_msg = pn + ": Found a reference to %s/ in %s" % (exec_prefix, path)
				package_qa_handle_error("unsafe-references-in-scripts", error_msg, d)
				error_msg = "Shell scripts in base_bindir and base_sbindir should not reference anything in exec_prefix"
				package_qa_handle_error("unsafe-references-in-scripts", error_msg, d)

def unsafe_references_skippable(path, name, d):
	if bb.data.inherits_class('native', d) or bb.data.inherits_class('nativesdk', d):
		return True

	if "-dbg" in name or "-dev" in name:
		return True

	# Other package names to skip:
	if name.startswith("kernel-module-"):
		return True

	# Skip symlinks
	if os.path.islink(path):
		return True

	# Skip unusual rootfs layouts which make these tests irrelevant
	exec_prefix = d.getVar('exec_prefix', True)
	if exec_prefix == "":
		return True

	pkgdest = d.getVar('PKGDEST', True)
	pkgdest = pkgdest + "/" + name
	pkgdest = os.path.abspath(pkgdest)
	base_bindir = pkgdest + d.getVar('base_bindir', True)
	base_sbindir = pkgdest + d.getVar('base_sbindir', True)
	base_libdir = pkgdest + d.getVar('base_libdir', True)
	bindir = pkgdest + d.getVar('bindir', True)
	sbindir = pkgdest + d.getVar('sbindir', True)
	libdir = pkgdest + d.getVar('libdir', True)

	if base_bindir == bindir and base_sbindir == sbindir and base_libdir == libdir:
		return True

	# Skip files not in base_[bindir|sbindir|libdir]
	path = os.path.abspath(path)
	if not (base_bindir in path or base_sbindir in path or base_libdir in path):
		return True

	return False

QAPATHTEST[arch] = "package_qa_check_arch"
def package_qa_check_arch(path,name,d, elf, messages):
    """
    Check if archs are compatible
    """
    if not elf:
        return

    target_os   = d.getVar('TARGET_OS', True)
    target_arch = d.getVar('TARGET_ARCH', True)
    provides = d.getVar('PROVIDES', True)
    bpn = d.getVar('BPN', True)

    # FIXME: Cross package confuse this check, so just skip them
    for s in ['cross', 'nativesdk', 'cross-canadian']:
        if bb.data.inherits_class(s, d):
            return

    # avoid following links to /usr/bin (e.g. on udev builds)
    # we will check the files pointed to anyway...
    if os.path.islink(path):
        return

    #if this will throw an exception, then fix the dict above
    (machine, osabi, abiversion, littleendian, bits) \
        = package_qa_get_machine_dict()[target_os][target_arch]

    # Check the architecture and endiannes of the binary
    if not ((machine == elf.machine()) or \
	("virtual/kernel" in provides) and (target_os == "linux-gnux32")):
        messages.append("Architecture did not match (%d to %d) on %s" % \
                 (machine, elf.machine(), package_qa_clean_path(path,d)))
    elif not ((bits == elf.abiSize()) or  \
	("virtual/kernel" in provides) and (target_os == "linux-gnux32")):
        messages.append("Bit size did not match (%d to %d) %s on %s" % \
                 (bits, elf.abiSize(), bpn, package_qa_clean_path(path,d)))
    elif not littleendian == elf.isLittleEndian():
        messages.append("Endiannes did not match (%d to %d) on %s" % \
                 (littleendian, elf.isLittleEndian(), package_qa_clean_path(path,d)))

QAPATHTEST[desktop] = "package_qa_check_desktop"
def package_qa_check_desktop(path, name, d, elf, messages):
    """
    Run all desktop files through desktop-file-validate.
    """
    if path.endswith(".desktop"):
        desktop_file_validate = os.path.join(d.getVar('STAGING_BINDIR_NATIVE',True),'desktop-file-validate')
        output = os.popen("%s %s" % (desktop_file_validate, path))
        # This only produces output on errors
        for l in output:
            messages.append("Desktop file issue: " + l.strip())

QAPATHTEST[textrel] = "package_qa_textrel"
def package_qa_textrel(path, name, d, elf, messages):
    """
    Check if the binary contains relocations in .text
    """

    if not elf:
        return

    if os.path.islink(path):
        return

    phdrs = elf.run_objdump("-p", d)
    sane = True

    import re
    textrel_re = re.compile("\s+TEXTREL\s+")
    for line in phdrs.split("\n"):
        if textrel_re.match(line):
	   sane = False

    if not sane:
        messages.append("ELF binary '%s' has relocations in .text" % path)

QAPATHTEST[ldflags] = "package_qa_hash_style"
def package_qa_hash_style(path, name, d, elf, messages):
    """
    Check if the binary has the right hash style...
    """

    if not elf:
        return

    if os.path.islink(path):
        return

    gnu_hash = "--hash-style=gnu" in d.getVar('LDFLAGS', True)
    if not gnu_hash:
        gnu_hash = "--hash-style=both" in d.getVar('LDFLAGS', True)
    if not gnu_hash:
        return

    sane = False
    has_syms = False

    phdrs = elf.run_objdump("-p", d)

    # If this binary has symbols, we expect it to have GNU_HASH too.
    for line in phdrs.split("\n"):
        if "SYMTAB" in line:
            has_syms = True
        if "GNU_HASH" in line:
            sane = True
        if "[mips32]" in line or "[mips64]" in line:
	    sane = True

    if has_syms and not sane:
        messages.append("No GNU_HASH in the elf binary: '%s'" % path)


QAPATHTEST[buildpaths] = "package_qa_check_buildpaths"
def package_qa_check_buildpaths(path, name, d, elf, messages):
    """
    Check for build paths inside target files and error if not found in the whitelist
    """
    # Ignore .debug files, not interesting
    if path.find(".debug") != -1:
        return

    # Ignore symlinks
    if os.path.islink(path):
        return

    tmpdir = d.getVar('TMPDIR', True)
    file_content = open(path).read()
    if tmpdir in file_content:
        messages.append("File %s in package contained reference to tmpdir" % package_qa_clean_path(path,d))


QAPATHTEST[xorg-driver-abi] = "package_qa_check_xorg_driver_abi"
def package_qa_check_xorg_driver_abi(path, name, d, elf, messages):
    """
    Check that all packages containing Xorg drivers have ABI dependencies
    """

    # Skip dev, dbg or nativesdk packages
    if name.endswith("-dev") or name.endswith("-dbg") or name.startswith("nativesdk-"):
        return

    driverdir = d.expand("${libdir}/xorg/modules/drivers/")
    if driverdir in path and path.endswith(".so"):
        for rdep in bb.utils.explode_deps(d.getVar('RDEPENDS_' + name, True) or ""):
            if rdep.startswith("xorg-abi-"):
                return
        messages.append("Package %s contains Xorg driver (%s) but no xorg-abi- dependencies" % (name, os.path.basename(path)))

def package_qa_check_license(workdir, d):
    """
    Check for changes in the license files 
    """
    import tempfile
    sane = True

    lic_files = d.getVar('LIC_FILES_CHKSUM', True)
    lic = d.getVar('LICENSE', True)
    pn = d.getVar('PN', True)

    if lic == "CLOSED":
        return True

    if not lic_files:
        bb.error(pn + ": Recipe file does not have license file information (LIC_FILES_CHKSUM)")
        return False

    srcdir = d.getVar('S', True)

    for url in lic_files.split():
        (type, host, path, user, pswd, parm) = bb.decodeurl(url)
        srclicfile = os.path.join(srcdir, path)
        if not os.path.isfile(srclicfile):
            raise bb.build.FuncFailed( pn + ": LIC_FILES_CHKSUM points to an invalid file: " + srclicfile)

        if 'md5' not in parm:
            bb.error(pn + ": md5 checksum is not specified for ", url)
            return False
        beginline, endline = 0, 0
        if 'beginline' in parm:
            beginline = int(parm['beginline'])
        if 'endline' in parm:
            endline = int(parm['endline'])

        if (not beginline) and (not endline):
            md5chksum = bb.utils.md5_file(srclicfile)
        else:
            fi = open(srclicfile, 'r')
            fo = tempfile.NamedTemporaryFile(mode='wb', prefix='poky.', suffix='.tmp', delete=False)
            tmplicfile = fo.name;
            lineno = 0
            linesout = 0
            for line in fi:
                lineno += 1
                if (lineno >= beginline): 
                    if ((lineno <= endline) or not endline):
                        fo.write(line)
                        linesout += 1
                    else:
                        break
            fo.flush()
            fo.close()
            fi.close()
            md5chksum = bb.utils.md5_file(tmplicfile)
            os.unlink(tmplicfile)

        if parm['md5'] == md5chksum:
            bb.note (pn + ": md5 checksum matched for ", url)
        else:
            bb.error (pn + ": md5 data is not matching for ", url)
            bb.error (pn + ": The new md5 checksum is ", md5chksum)
            bb.error (pn + ": Check if the license information has changed in")
            sane = False

    return sane

def package_qa_check_staged(path,d):
    """
    Check staged la and pc files for sanity
      -e.g. installed being false

        As this is run after every stage we should be able
        to find the one responsible for the errors easily even
        if we look at every .pc and .la file
    """

    sane = True
    tmpdir = d.getVar('TMPDIR', True)
    workdir = os.path.join(tmpdir, "work")

    installed = "installed=yes"
    if bb.data.inherits_class("native", d) or bb.data.inherits_class("cross", d):
        pkgconfigcheck = workdir
    else:
        pkgconfigcheck = tmpdir

    # find all .la and .pc files
    # read the content
    # and check for stuff that looks wrong
    for root, dirs, files in os.walk(path):
        for file in files:
            path = os.path.join(root,file)
            if file.endswith(".la"):
                file_content = open(path).read()
                if workdir in file_content:
                    error_msg = "%s failed sanity test (workdir) in path %s" % (file,root)
                    sane = package_qa_handle_error("la", error_msg, d)
            elif file.endswith(".pc"):
                file_content = open(path).read()
                if pkgconfigcheck in file_content:
                    error_msg = "%s failed sanity test (tmpdir) in path %s" % (file,root)
                    sane = package_qa_handle_error("pkgconfig", error_msg, d)

    return sane

# Walk over all files in a directory and call func
def package_qa_walk(path, warnfuncs, errorfuncs, skip, package, d):
    import oe.qa

    #if this will throw an exception, then fix the dict above
    target_os   = d.getVar('TARGET_OS', True)
    target_arch = d.getVar('TARGET_ARCH', True)

    warnings = []
    errors = []
    for path in pkgfiles[package]:
            elf = oe.qa.ELFFile(path)
            try:
                elf.open()
            except:
                elf = None
            for func in warnfuncs:
                func(path, package, d, elf, warnings)
            for func in errorfuncs:
                func(path, package, d, elf, errors)

    for w in warnings:
        bb.warn("QA Issue: %s" % w)
        package_qa_write_error(w, d)
    for e in errors:
        bb.error("QA Issue: %s" % e)
        package_qa_write_error(e, d)

    return len(errors) == 0

def package_qa_check_rdepends(pkg, pkgdest, skip, d):
    # Don't do this check for kernel/module recipes, there aren't too many debug/development
    # packages and you can get false positives e.g. on kernel-module-lirc-dev
    if bb.data.inherits_class("kernel", d) or bb.data.inherits_class("module-base", d):
        return True

    sane = True
    if not "-dbg" in pkg and not "packagegroup-" in pkg and not "-image" in pkg:
        localdata = bb.data.createCopy(d)
        localdata.setVar('OVERRIDES', pkg)
        bb.data.update_data(localdata)

        # Now check the RDEPENDS
        rdepends = bb.utils.explode_deps(localdata.getVar('RDEPENDS', True) or "")

        # Now do the sanity check!!!
        for rdepend in rdepends:
            if "-dbg" in rdepend and "debug-deps" not in skip:
                error_msg = "%s rdepends on %s" % (pkg,rdepend)
                sane = package_qa_handle_error("debug-deps", error_msg, d)
            if (not "-dev" in pkg and not "-staticdev" in pkg) and rdepend.endswith("-dev") and "dev-deps" not in skip:
                error_msg = "%s rdepends on %s" % (pkg, rdepend)
                sane = package_qa_handle_error("dev-deps", error_msg, d)

    return sane

def package_qa_check_deps(pkg, pkgdest, skip, d):
    sane = True

    localdata = bb.data.createCopy(d)
    localdata.setVar('OVERRIDES', pkg)
    bb.data.update_data(localdata)

    def check_valid_deps(var):
        sane = True
        try:
            rvar = bb.utils.explode_dep_versions2(localdata.getVar(var, True) or "")
        except ValueError as e:
            bb.fatal("%s_%s: %s" % (var, pkg, e))
            raise e
        for dep in rvar:
            for v in rvar[dep]:
                if v and not v.startswith(('< ', '= ', '> ', '<= ', '>=')):
                    error_msg = "%s_%s is invalid: %s (%s)   only comparisons <, =, >, <=, and >= are allowed" % (var, pkg, dep, v)
                    sane = package_qa_handle_error("dep-cmp", error_msg, d)
        return sane

    sane = True
    if not check_valid_deps('RDEPENDS'):
        sane = False
    if not check_valid_deps('RRECOMMENDS'):
        sane = False
    if not check_valid_deps('RSUGGESTS'):
        sane = False
    if not check_valid_deps('RPROVIDES'):
        sane = False
    if not check_valid_deps('RREPLACES'):
        sane = False
    if not check_valid_deps('RCONFLICTS'):
        sane = False

    return sane

# The PACKAGE FUNC to scan each package
python do_package_qa () {
    import subprocess

    bb.note("DO PACKAGE QA")

    logdir = d.getVar('T', True)
    pkg = d.getVar('PN', True)

    # Check the compile log for host contamination
    compilelog = os.path.join(logdir,"log.do_compile")

    if os.path.exists(compilelog):
        statement = "grep -e 'CROSS COMPILE Badness:' -e 'is unsafe for cross-compilation' %s > /dev/null" % compilelog
        if subprocess.call(statement, shell=True) == 0:
            bb.warn("%s: The compile log indicates that host include and/or library paths were used.\n \
        Please check the log '%s' for more information." % (pkg, compilelog))

    # Check the install log for host contamination
    installlog = os.path.join(logdir,"log.do_install")

    if os.path.exists(installlog):
        statement = "grep -e 'CROSS COMPILE Badness:' -e 'is unsafe for cross-compilation' %s > /dev/null" % installlog
        if subprocess.call(statement, shell=True) == 0:
            bb.warn("%s: The install log indicates that host include and/or library paths were used.\n \
        Please check the log '%s' for more information." % (pkg, installlog))

    # Scan the packages...
    pkgdest = d.getVar('PKGDEST', True)
    packages = d.getVar('PACKAGES', True)

    # no packages should be scanned
    if not packages:
        return

    testmatrix = d.getVarFlags("QAPATHTEST")
    import re
    # The package name matches the [a-z0-9.+-]+ regular expression
    pkgname_pattern = re.compile("^[a-z0-9.+-]+$")

    g = globals()
    walk_sane = True
    rdepends_sane = True
    deps_sane = True
    for package in packages.split():
        skip = (d.getVar('INSANE_SKIP_' + package, True) or "").split()
        if skip:
            bb.note("Package %s skipping QA tests: %s" % (package, str(skip)))
        warnchecks = []
        for w in (d.getVar("WARN_QA", True) or "").split():
            if w in skip:
               continue
            if w in testmatrix and testmatrix[w] in g:
                warnchecks.append(g[testmatrix[w]])
        errorchecks = []
        for e in (d.getVar("ERROR_QA", True) or "").split():
            if e in skip:
               continue
            if e in testmatrix and testmatrix[e] in g:
                errorchecks.append(g[testmatrix[e]])

        bb.note("Checking Package: %s" % package)
        # Check package name
        if not pkgname_pattern.match(package):
            package_qa_handle_error("pkgname",
                    "%s doesn't match the [a-z0-9.+-]+ regex\n" % package, d)

        path = "%s/%s" % (pkgdest, package)
        if not package_qa_walk(path, warnchecks, errorchecks, skip, package, d):
            walk_sane  = False
        if not package_qa_check_rdepends(package, pkgdest, skip, d):
            rdepends_sane = False
        if not package_qa_check_deps(package, pkgdest, skip, d):
            deps_sane = False


    if 'libdir' in d.getVar("ALL_QA", True).split():
        package_qa_check_libdir(d)

    if not walk_sane or not rdepends_sane or not deps_sane:
        bb.fatal("QA run found fatal errors. Please consider fixing them.")
    bb.note("DONE with PACKAGE QA")
}


python do_qa_staging() {
    bb.note("QA checking staging")

    if not package_qa_check_staged(d.expand('${SYSROOT_DESTDIR}/${STAGING_LIBDIR}'), d):
        bb.fatal("QA staging was broken by the package built above")
}

python do_qa_configure() {
    import subprocess

    ###########################################################################
    # Check config.log for cross compile issues
    ###########################################################################

    configs = []
    workdir = d.getVar('WORKDIR', True)
    bb.note("Checking autotools environment for common misconfiguration")
    for root, dirs, files in os.walk(workdir):
        statement = "grep -e 'CROSS COMPILE Badness:' -e 'is unsafe for cross-compilation' %s > /dev/null" % \
                    os.path.join(root,"config.log")
        if "config.log" in files:
            if subprocess.call(statement, shell=True) == 0:
                bb.fatal("""This autoconf log indicates errors, it looked at host include and/or library paths while determining system capabilities.
Rerun configure task after fixing this. The path was '%s'""" % root)

        if "configure.ac" in files:
            configs.append(os.path.join(root,"configure.ac"))
        if "configure.in" in files:
            configs.append(os.path.join(root, "configure.in"))

    ###########################################################################
    # Check gettext configuration and dependencies are correct
    ###########################################################################

    cnf = d.getVar('EXTRA_OECONF', True) or ""
    if "gettext" not in d.getVar('P', True) and "gcc-runtime" not in d.getVar('P', True) and "--disable-nls" not in cnf:
        ml = d.getVar("MLPREFIX", True) or ""
        if bb.data.inherits_class('native', d) or bb.data.inherits_class('cross', d) or bb.data.inherits_class('crosssdk', d) or bb.data.inherits_class('nativesdk', d):
            gt = "gettext-native"
        elif bb.data.inherits_class('cross-canadian', d):
            gt = "nativesdk-gettext"
        else:
            gt = "virtual/" + ml + "gettext"
        deps = bb.utils.explode_deps(d.getVar('DEPENDS', True) or "")
        if gt not in deps:
            for config in configs:
                gnu = "grep \"^[[:space:]]*AM_GNU_GETTEXT\" %s >/dev/null" % config
                if subprocess.call(gnu, shell=True) == 0:
                    bb.fatal("""%s required but not in DEPENDS for file %s.
Missing inherit gettext?""" % (gt, config))

    ###########################################################################
    # Check license variables
    ###########################################################################

    if not package_qa_check_license(workdir, d):
        bb.fatal("Licensing Error: LIC_FILES_CHKSUM does not match, please fix")

}
# The Staging Func, to check all staging
#addtask qa_staging after do_populate_sysroot before do_build
do_populate_sysroot[postfuncs] += "do_qa_staging "

# Check broken config.log files, for packages requiring Gettext which don't
# have it in DEPENDS and for correct LIC_FILES_CHKSUM
#addtask qa_configure after do_configure before do_compile
do_configure[postfuncs] += "do_qa_configure "

python () {
    tests = d.getVar('ALL_QA', True).split()
    if "desktop" in tests:
        d.appendVar("PACKAGE_DEPENDS", "desktop-file-utils-native")

    ###########################################################################
    # Check various variables
    ###########################################################################

    if d.getVar('do_stage', True) is not None:
        bb.fatal("Legacy staging found for %s as it has a do_stage function. This will need conversion to a do_install or often simply removal to work with OE-core" % d.getVar("FILE", True))

    issues = []
    if (d.getVar('PACKAGES', True) or "").split():
        for var in 'RDEPENDS', 'RRECOMMENDS', 'RSUGGESTS', 'RCONFLICTS', 'RPROVIDES', 'RREPLACES', 'FILES', 'pkg_preinst', 'pkg_postinst', 'pkg_prerm', 'pkg_postrm', 'ALLOW_EMPTY':
            if d.getVar(var):
                issues.append(var)
    for i in issues:
        package_qa_handle_error("pkgvarcheck", "%s: Variable %s is set as not being package specific, please fix this." % (d.getVar("FILE", True), i), d)
}
