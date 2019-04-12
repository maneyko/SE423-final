# invoke SourceDir generated makefile for lab3_1.p28FP
lab3_1.p28FP: .libraries,lab3_1.p28FP
.libraries,lab3_1.p28FP: package/cfg/lab3_1_p28FP.xdl
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab3_1\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab3_1\SYSBIOS/src/makefile.libs clean

