## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28FP linker.cmd package/cfg/lab5_p28FP.o28FP

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/lab5_p28FP.xdl
	$(SED) 's"^\"\(package/cfg/lab5_p28FPcfg.cmd\)\"$""\"C:/dgnava2_maneyko2/SE423Repo/Labs/lab5/lab5Project/Debug/configPkg/\1\""' package/cfg/lab5_p28FP.xdl > $@
	-$(SETDATE) -r:max package/cfg/lab5_p28FP.h compiler.opt compiler.opt.defs
