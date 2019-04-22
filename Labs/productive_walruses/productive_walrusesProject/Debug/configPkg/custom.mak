## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e674 linker.cmd package/cfg/productive_walruses_pe674.oe674

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/productive_walruses_pe674.xdl
	$(SED) 's"^\"\(package/cfg/productive_walruses_pe674cfg.cmd\)\"$""\"C:/kt2_smm2_dg2_ma2/SE423Repo/Labs/productive_walruses/productive_walrusesProject/Debug/configPkg/\1\""' package/cfg/productive_walruses_pe674.xdl > $@
	-$(SETDATE) -r:max package/cfg/productive_walruses_pe674.h compiler.opt compiler.opt.defs
