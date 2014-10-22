# ex:ts=4:sw=4:sts=4:et
# -*- tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil -*-
#
# This file is used for archiving sources, patches, and logs to a
# tarball.  It also output building environment to xxx.dump.data and
# create xxx.diff.gz to record all content in ${S} to a diff file.
#

ARCHIVE_EXCLUDE_FROM ?= ".pc autom4te.cache"
ARCHIVE_TYPE ?= "tar srpm"
PATCHES_ARCHIVE_WITH_SERIES = 'yes'
SOURCE_ARCHIVE_LOG_WITH_SCRIPTS ?= '${@d.getVarFlag('ARCHIVER_MODE', 'log_type') \
    if d.getVarFlag('ARCHIVER_MODE', 'log_type') != 'none' else 'logs_with_scripts'}'
SOURCE_ARCHIVE_PACKAGE_TYPE ?= '${@d.getVarFlag('ARCHIVER_MODE', 'type') \
    if d.getVarFlag('ARCHIVER_MODE', 'log_type') != 'none' else 'tar'}'
FILTER ?= '${@d.getVarFlag('ARCHIVER_MODE', 'filter') \
    if d.getVarFlag('ARCHIVER_MODE', 'filter')!= 'none' else 'no'}'


COPYLEFT_LICENSE_INCLUDE ?= 'GPL* LGPL*'
COPYLEFT_LICENSE_INCLUDE[type] = 'list'
COPYLEFT_LICENSE_INCLUDE[doc] = 'Space separated list of globs which include licenses'

COPYLEFT_LICENSE_EXCLUDE ?= 'CLOSED Proprietary'
COPYLEFT_LICENSE_EXCLUDE[type] = 'list'
COPYLEFT_LICENSE_INCLUDE[doc] = 'Space separated list of globs which exclude licenses'

COPYLEFT_RECIPE_TYPE ?= '${@copyleft_recipe_type(d)}'
COPYLEFT_RECIPE_TYPE[doc] = 'The "type" of the current recipe (e.g. target, native, cross)'

COPYLEFT_RECIPE_TYPES ?= 'target'
COPYLEFT_RECIPE_TYPES[type] = 'list'
COPYLEFT_RECIPE_TYPES[doc] = 'Space separated list of recipe types to include'

COPYLEFT_AVAILABLE_RECIPE_TYPES = 'target native nativesdk cross crosssdk cross-canadian'
COPYLEFT_AVAILABLE_RECIPE_TYPES[type] = 'list'
COPYLEFT_AVAILABLE_RECIPE_TYPES[doc] = 'Space separated list of available recipe types'

def copyleft_recipe_type(d):
    for recipe_type in oe.data.typed_value('COPYLEFT_AVAILABLE_RECIPE_TYPES', d):
        if oe.utils.inherits(d, recipe_type):
            return recipe_type
    return 'target'

def copyleft_should_include(d):
    """
    Determine if this recipe's sources should be deployed for compliance
    """
    import ast
    import oe.license
    from fnmatch import fnmatchcase as fnmatch

    recipe_type = d.getVar('COPYLEFT_RECIPE_TYPE', True)
    if recipe_type not in oe.data.typed_value('COPYLEFT_RECIPE_TYPES', d):
        return False, 'recipe type "%s" is excluded' % recipe_type

    include = oe.data.typed_value('COPYLEFT_LICENSE_INCLUDE', d)
    exclude = oe.data.typed_value('COPYLEFT_LICENSE_EXCLUDE', d)

    try:
        is_included, reason = oe.license.is_included(d.getVar('LICENSE', True), include, exclude)
    except oe.license.LicenseError as exc:
        bb.fatal('%s: %s' % (d.getVar('PF', True), exc))
    else:
        if is_included:
            if reason:
                return True, 'recipe has included licenses: %s' % ', '.join(reason)
            else:
                return False, 'recipe does not include a copyleft license'
        else:
            return False, 'recipe has excluded licenses: %s' % ', '.join(reason)

def tar_filter(d):
    """
    Only archive the package belongs to COPYLEFT_LICENSE_INCLUDE
    and ignore the one in COPYLEFT_LICENSE_EXCLUDE. Don't exclude any
    packages when \"FILTER\" is \"no\"
    """
    if d.getVar('FILTER', True) == "yes":
        included, reason = copyleft_should_include(d)
        return not included
    else:
        return False

def get_bb_inc(d):
    """
    create a directory "script-logs" including .bb and .inc file in ${WORKDIR}
    """
    import re
    import shutil

    bbinc = []
    pat=re.compile('require\s*([^\s]*\.*)(.*)')
    work_dir = d.getVar('WORKDIR', True)
    bbfile = d.getVar('FILE', True)
    bbdir = os.path.dirname(bbfile)
    target_sys = d.getVar('TARGET_SYS', True)
    pf = d.getVar('PF', True)
    licenses = get_licenses(d)
    script_logs = os.path.join(work_dir, 'script-logs/'+ target_sys + '/' + licenses + '/' + pf + '/script-logs')
    bb_inc = os.path.join(script_logs, 'bb_inc')
    bb.mkdirhier(bb_inc)

    def find_file(dir, file):
        for root, dirs, files in os.walk(dir):
            if file in files:
                return os.path.join(root, file)

    def get_inc (file):
        f = open(file, 'r')
        for line in f.readlines():
            if 'require' not  in line:
                bbinc.append(file)
            else:
                try:
                    incfile = pat.match(line).group(1)
                    incfile = bb.data.expand(os.path.basename(incfile), d)
                    abs_incfile = find_file(bbdir, incfile)
                    if abs_incfile:
                        bbinc.append(abs_incfile)
                        get_inc(abs_incfile)
                except AttributeError:
                    pass
    get_inc(bbfile)
    bbinc = list(set(bbinc))
    for bbincfile in bbinc:
        shutil.copy(bbincfile, bb_inc)

    return script_logs

def get_logs(d):
    """
    create a directory "script-logs" in ${WORKDIR}
    """
    work_dir = d.getVar('WORKDIR', True)
    target_sys = d.getVar('TARGET_SYS', True)
    pf = d.getVar('PF', True)
    licenses = get_licenses(d)
    script_logs = os.path.join(work_dir, 'script-logs/'+ target_sys + '/' + licenses + '/' + pf + '/script-logs')

    try:
        bb.mkdirhier(os.path.join(script_logs, 'temp'))
        oe.path.copytree(os.path.join(work_dir, 'temp'), os.path.join(script_logs, 'temp'))
    except (IOError, AttributeError):
        pass
    return script_logs

def get_series(d):
    """
    copy patches and series file to a pointed directory which will be
    archived to tarball in ${WORKDIR}
    """
    import shutil

    src_patches=[]
    pf = d.getVar('PF', True)
    work_dir = d.getVar('WORKDIR', True)
    s = d.getVar('S', True)
    dest = os.path.join(work_dir, pf + '-series')
    shutil.rmtree(dest, ignore_errors=True)
    bb.mkdirhier(dest)

    src_uri = d.getVar('SRC_URI', True).split()
    fetch = bb.fetch2.Fetch(src_uri, d)
    locals = (fetch.localpath(url) for url in fetch.urls)
    for local in locals:
        src_patches.append(local)
    if not cmp(work_dir, s):
        tmp_list = src_patches
    else:
        tmp_list = src_patches[1:]

    for patch in tmp_list:
        try:
            shutil.copy(patch, dest)
        except IOError:
            if os.path.isdir(patch):
                bb.mkdirhier(os.path.join(dest, patch))
                oe.path.copytree(patch, os.path.join(dest, patch))
    return dest

def get_applying_patches(d):
    """
    only copy applying patches to a pointed directory which will be
    archived to tarball
    """
    import shutil

    pf = d.getVar('PF', True)
    work_dir = d.getVar('WORKDIR', True)
    dest = os.path.join(work_dir, pf + '-patches')
    shutil.rmtree(dest, ignore_errors=True)
    bb.mkdirhier(dest)

    patches = src_patches(d)
    for patch in patches:
        _, _, local, _, _, parm = bb.decodeurl(patch)
        if local:
             shutil.copy(local, dest)
    return dest

def not_tarball(d):
    """
    packages including key words 'work-shared', 'native', 'packagegroup-' will be passed
    """
    workdir = d.getVar('WORKDIR', True)
    s = d.getVar('S', True)
    if 'work-shared' in s or 'packagegroup-' in workdir or 'native' in workdir:
        return True
    else:
        return False

def get_source_from_downloads(d, stage_name):
    """
    copy tarball of $P to $WORKDIR when this tarball exists in $DL_DIR
    """
    if stage_name in 'patched' 'configured':
        return
    pf = d.getVar('PF', True)
    dl_dir = d.getVar('DL_DIR', True)
    try:
        source = os.path.join(dl_dir, os.path.basename(d.getVar('SRC_URI', True).split()[0]))
        if os.path.exists(source) and not os.path.isdir(source):
            return source
    except (IndexError, OSError):
        pass
    return ''

def do_tarball(workdir, srcdir, tarname):
    """
    tar "srcdir" under "workdir" to "tarname"
    """
    import tarfile

    sav_dir = os.getcwd()
    os.chdir(workdir)
    if (len(os.listdir(srcdir))) != 0:
        tar = tarfile.open(tarname, "w:gz")
        tar.add(srcdir)
        tar.close()
    else:
        tarname = ''
    os.chdir(sav_dir)
    return tarname

def archive_sources_from_directory(d, stage_name):
    """
    archive sources codes tree to tarball when tarball of $P doesn't
    exist in $DL_DIR
    """

    s = d.getVar('S', True)
    work_dir=d.getVar('WORKDIR', True)
    PF = d.getVar('PF', True)
    tarname = PF + '-' + stage_name + ".tar.gz"

    if os.path.exists(s) and work_dir in s:
        try:
            source_dir = os.path.join(work_dir, [ i for i in s.replace(work_dir, '').split('/') if i][0])
        except IndexError:
            if not cmp(s, work_dir):
                return ''
    else:
        return ''
    source = os.path.basename(source_dir)
    return do_tarball(work_dir, source, tarname)

def archive_sources(d, stage_name):
    """
    copy tarball from $DL_DIR to $WORKDIR if have tarball, archive
    source codes tree in $WORKDIR if $P is directory instead of tarball
    """
    import shutil

    work_dir = d.getVar('WORKDIR', True)
    file = get_source_from_downloads(d, stage_name)
    if file:
        shutil.copy(file, work_dir)
        file = os.path.basename(file)
    else:
        file = archive_sources_from_directory(d, stage_name)
    return file

def archive_patches(d, patchdir, series):
    """
    archive patches to tarball and also include series files if 'series' is True
    """
    import shutil

    s = d.getVar('S', True)
    work_dir = d.getVar('WORKDIR', True)
    patch_dir = os.path.basename(patchdir)
    tarname = patch_dir + ".tar.gz"
    if series  == 'all' and os.path.exists(os.path.join(s, 'patches/series')):
        shutil.copy(os.path.join(s, 'patches/series'), patchdir)
    tarname = do_tarball(work_dir, patch_dir, tarname)
    shutil.rmtree(patchdir, ignore_errors=True)
    return tarname

def select_archive_patches(d, option):
    """
    select to archive all patches including non-applying and series or
    applying patches
    """
    if option == "all":
        patchdir = get_series(d)
    elif option == "applying":
        patchdir = get_applying_patches(d)
    try:
        os.rmdir(patchdir)
    except OSError:
            tarpatch = archive_patches(d, patchdir, option)
            return tarpatch
    return

def archive_logs(d, logdir, bbinc=False):
    """
    archive logs in temp to tarball and .bb and .inc files if bbinc is True
    """
    import shutil

    pf = d.getVar('PF', True)
    work_dir = d.getVar('WORKDIR', True)
    log_dir =  os.path.basename(logdir)
    tarname = pf + '-' + log_dir + ".tar.gz"
    archive_dir = os.path.join( logdir, '..' )
    tarname = do_tarball(archive_dir, log_dir, tarname)
    if bbinc:
        shutil.rmtree(logdir, ignore_errors=True)
    return tarname

def get_licenses(d):
    """get licenses for running .bb file"""
    import oe.license

    licenses_type = d.getVar('LICENSE', True) or ""
    lics = oe.license.is_included(licenses_type)[1:][0]
    lice = ''
    for lic in lics:
        licens = d.getVarFlag('SPDXLICENSEMAP', lic)
        if licens != None:
            lice += licens
        else:
            lice += lic
    return lice


def move_tarball_deploy(d, tarball_list):
    """move tarball in location to ${DEPLOY_DIR}/sources"""
    import shutil

    if tarball_list is []:
        return
    target_sys = d.getVar('TARGET_SYS', True)
    pf = d.getVar('PF', True)
    licenses = get_licenses(d)
    work_dir = d.getVar('WORKDIR', True)
    tar_sources = d.getVar('DEPLOY_DIR', True) + '/sources/' + target_sys + '/' + licenses + '/' + pf
    if not os.path.exists(tar_sources):
        bb.mkdirhier(tar_sources)
    for source in tarball_list:
        if source:
            if os.path.exists(os.path.join(tar_sources, source)):
                os.remove(os.path.join(tar_sources, source))
            shutil.move(os.path.join(work_dir, source), tar_sources)

def check_archiving_type(d):
    """check the type for archiving package('tar' or 'srpm')"""
    if d.getVar('SOURCE_ARCHIVE_PACKAGE_TYPE', True) not in d.getVar('ARCHIVE_TYPE', True).split():
        bb.fatal("\"SOURCE_ARCHIVE_PACKAGE_TYPE\" is \'tar\' or \'srpm\', no other types")

def store_package(d, package_name):
    """
    store tarbablls name to file "tar-package"
    """
    f = open(os.path.join(d.getVar('WORKDIR', True), 'tar-package'), 'a')
    f.write(package_name + ' ')
    f.close()

def get_package(d):
    """
    get tarballs name from "tar-package"
    """
    work_dir = (d.getVar('WORKDIR', True))
    tarlist = os.path.join(work_dir, 'tar-package')
    if os.path.exists(tarlist):
        f = open(tarlist, 'r')
        line = f.readline().rstrip('\n').split()
        f.close()
        return line
    return []


def archive_sources_patches(d, stage_name):
    """
    archive sources and patches to tarball. stage_name will append
    strings ${stage_name} to ${PR} as middle name. for example,
    zlib-1.4.6-prepatch(stage_name).tar.gz
    """
    import shutil

    check_archiving_type(d)

    source_tar_name = archive_sources(d, stage_name)
    if stage_name == "prepatch":
        if d.getVar('PATCHES_ARCHIVE_WITH_SERIES', True) == 'yes':
            patch_tar_name = select_archive_patches(d, "all")
        elif d.getVar('PATCHES_ARCHIVE_WITH_SERIES', True) == 'no':
            patch_tar_name = select_archive_patches(d, "applying")
        else:
            bb.fatal("Please define 'PATCHES_ARCHIVE_WITH_SERIES' to 'yes' or 'no' ")
    else:
        patch_tar_name = ''

    if d.getVar('SOURCE_ARCHIVE_PACKAGE_TYPE', True) != 'srpm':
        move_tarball_deploy(d, [source_tar_name, patch_tar_name])
    else:
        tarlist = os.path.join(d.getVar('WORKDIR', True), 'tar-package')
        if os.path.exists(tarlist):
            os.remove(tarlist)
        for package in os.path.basename(source_tar_name), patch_tar_name:
            if package:
                store_package(d, str(package) + ' ')

def archive_scripts_logs(d):
    """
    archive scripts and logs. scripts include .bb and .inc files and
    logs include stuff in "temp".
    """
    import shutil

    work_dir = d.getVar('WORKDIR', True)
    temp_dir = os.path.join(work_dir, 'temp')
    source_archive_log_with_scripts = d.getVar('SOURCE_ARCHIVE_LOG_WITH_SCRIPTS', True)
    if source_archive_log_with_scripts == 'logs_with_scripts':
        logdir = get_logs(d)
        logdir = get_bb_inc(d)
    elif source_archive_log_with_scripts == 'logs':
        logdir = get_logs(d)
    else:
        return

    tarlog = archive_logs(d, logdir, True)

    if d.getVar('SOURCE_ARCHIVE_PACKAGE_TYPE', True) == 'srpm':
        store_package(d, tarlog)

def dumpdata(d):
    """
    dump environment to "${P}-${PR}.showdata.dump" including all
    kinds of variables and functions when running a task
    """

    workdir = bb.data.getVar('WORKDIR', d, 1)
    distro = bb.data.getVar('DISTRO', d, 1)
    s = d.getVar('S', True)
    pf = d.getVar('PF', True)
    target_sys = d.getVar('TARGET_SYS', True)
    licenses = get_licenses(d)
    dumpdir = os.path.join(workdir, 'diffgz-envdata/'+ target_sys + '/' + licenses + '/' + pf )
    if not os.path.exists(dumpdir):
        bb.mkdirhier(dumpdir)

    dumpfile = os.path.join(dumpdir, bb.data.expand("${P}-${PR}.showdata.dump", d))

    bb.note("Dumping metadata into '%s'" % dumpfile)
    f = open(dumpfile, "w")
    # emit variables and shell functions
    bb.data.emit_env(f, d, True)
    # emit the metadata which isn't valid shell
    for e in d.keys():
        if bb.data.getVarFlag(e, 'python', d):
            f.write("\npython %s () {\n%s}\n" % (e, bb.data.getVar(e, d, 1)))
    f.close()

def create_diff_gz(d):
    """
    creating .diff.gz in ${DEPLOY_DIR_SRC}/${P}-${PR}.diff.g gz for
    mapping all content in 's' including patches to  xxx.diff.gz
    """
    import shutil
    import subprocess

    work_dir = d.getVar('WORKDIR', True)
    exclude_from = d.getVar('ARCHIVE_EXCLUDE_FROM', True).split()
    pf = d.getVar('PF', True)
    licenses = get_licenses(d)
    target_sys = d.getVar('TARGET_SYS', True)
    diff_dir = os.path.join(work_dir, 'diffgz-envdata/'+ target_sys + '/' + licenses + '/' + pf )
    diff_file = os.path.join(diff_dir, bb.data.expand("${P}-${PR}.diff.gz",d))

    f = open(os.path.join(work_dir,'temp/exclude-from-file'), 'a')
    for i in exclude_from:
        f.write(i)
        f.write("\n")
    f.close()

    s=d.getVar('S', True)
    distro = d.getVar('DISTRO',True) or ""
    dest = s + '/' + distro + '/files'
    if not os.path.exists(dest):
        bb.mkdirhier(dest)
    for i in os.listdir(os.getcwd()):
        if os.path.isfile(i):
            try:
                shutil.copy(i, dest)
            except IOError:
                subprocess.call('fakeroot cp -rf ' + i + " " + dest, shell=True)

    bb.note("Creating .diff.gz in ${DEPLOY_DIR_SRC}/${P}-${PR}.diff.gz")
    cmd = "LC_ALL=C TZ=UTC0 diff --exclude-from=" + work_dir + "/temp/exclude-from-file -Naur " + s + '.org' + ' ' +  s + " | gzip -c > " + diff_file
    d.setVar('DIFF', cmd + "\n")
    d.setVarFlag('DIFF', 'func', '1')
    bb.build.exec_func('DIFF', d)
    shutil.rmtree(s + '.org', ignore_errors=True)

# This function will run when user want to get tarball for sources and
# patches after do_unpack
python do_archive_original_sources_patches(){
    archive_sources_patches(d, 'prepatch')
}

# This function will run when user want to get tarball for patched
# sources after do_patch
python do_archive_patched_sources(){
    archive_sources_patches(d, 'patched')
}

# This function will run when user want to get tarball for configured
# sources after do_configure
python do_archive_configured_sources(){
    archive_sources_patches(d, 'configured')
}

# This function will run when user want to get tarball for logs or both
# logs and scripts(.bb and .inc files)
python do_archive_scripts_logs(){
    archive_scripts_logs(d)
}

# This function will run when user want to know what variable and
# functions in a running task are and also can get a diff file including
# all content a package should include.
python do_dumpdata_create_diff_gz(){
    dumpdata(d)
    create_diff_gz(d)
}

# This functions prepare for archiving "linux-yocto" because this
# package create directory 's' before do_patch instead of after
# do_unpack.  This is special control for archiving linux-yocto only.
python do_archive_linux_yocto(){
    s = d.getVar('S', True)
    if 'linux-yocto' in s:
        source_tar_name = archive_sources(d, '')
    if d.getVar('SOURCE_ARCHIVE_PACKAGE_TYPE', True) != 'srpm':
        move_tarball_deploy(d, [source_tar_name, ''])
}
do_kernel_checkout[postfuncs] += "do_archive_linux_yocto "

# remove tarball for sources, patches and logs after creating srpm.
python do_remove_tarlist(){
    work_dir = d.getVar('WORKDIR', True)
    tarlist = os.path.join(work_dir, 'tar-package')
    if os.path.exists(tarlist):
        os.remove(tarlist)
}
do_remove_tarlist[deptask] = "do_archive_scripts_logs"
do_package_write_rpm[postfuncs] += "do_remove_tarlist "
