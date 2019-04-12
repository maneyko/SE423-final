# invoke SourceDir generated makefile for lab4_1.p28FP
lab4_1.p28FP: .libraries,lab4_1.p28FP
.libraries,lab4_1.p28FP: package/cfg/lab4_1_p28FP.xdl
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab4_1\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab4_1\SYSBIOS/src/makefile.libs clean

