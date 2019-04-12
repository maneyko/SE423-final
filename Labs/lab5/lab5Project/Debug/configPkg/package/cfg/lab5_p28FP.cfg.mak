# invoke SourceDir generated makefile for lab5.p28FP
lab5.p28FP: .libraries,lab5.p28FP
.libraries,lab5.p28FP: package/cfg/lab5_p28FP.xdl
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab5\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab5\SYSBIOS/src/makefile.libs clean

