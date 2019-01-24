# invoke SourceDir generated makefile for Lab1A.pe674
Lab1A.pe674: .libraries,Lab1A.pe674
.libraries,Lab1A.pe674: package/cfg/Lab1A_pe674.xdl
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\Lab1A\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\dgnava2_maneyko2\SE423Repo\Labs\Lab1A\SYSBIOS/src/makefile.libs clean

