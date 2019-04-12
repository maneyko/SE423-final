# invoke SourceDir generated makefile for lab6.pe674
lab6.pe674: .libraries,lab6.pe674
.libraries,lab6.pe674: package/cfg/lab6_pe674.xdl
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab6\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\lab6\SYSBIOS/src/makefile.libs clean

