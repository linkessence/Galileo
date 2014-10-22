# Processes ref-manual and yocto-project-qs manual (<word>-<word>-<word> style)
s/\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/[a-z]*-[a-z]*-[a-z]*\/[a-z]*-[a-z]*-[a-z]*.html#/\"link\" href=\"#/g

# Processes all other manuals (<word>-<word> style)
s/\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/[a-z]*-[a-z]*\/[a-z]*-[a-z]*.html#/\"link\" href=\"#/g

# Process cases where just an external manual is referenced without an id anchor
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/yocto-project-qs\/yocto-project-qs.html\" target=\"_top\">Yocto Project Quick Start<\/a>/Yocto Project Quick Start/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/dev-manual\/dev-manual.html\" target=\"_top\">Yocto Project Development Manual<\/a>/Yocto Project Development Manual/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/adt-manual\/adt-manual.html\" target=\"_top\">Yocto Project Application Developer's Guide<\/a>/Yocto Project Application Developer's Guide/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/bsp-guide\/bsp-guide.html\" target=\"_top\">Yocto Project Board Support Package (BSP) Developer's Guide<\/a>/Yocto Project Board Support Package (BSP) Developer's Guide/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/profile-manual\/profile-manual.html\" target=\"_top\">Yocto Project Profile and Tracing Manual<\/a>/Yocto Project Profile and Tracing Manual/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/kernel-dev\/kernel-dev.html\" target=\"_top\">Yocto Project Linux Kernel Development Manual<\/a>/Yocto Project Linux Kernel Development Manual/g
s/<a class=\"ulink\" href=\"http:\/\/www.yoctoproject.org\/docs\/1.4.2\/ref-manual\/ref-manual.html\" target=\"_top\">Yocto Project Reference Manual<\/a>/Yocto Project Reference Manual/g
