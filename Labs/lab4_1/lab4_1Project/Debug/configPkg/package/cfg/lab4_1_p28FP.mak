#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = ti.targets.C28_float{1,0,17.6,0
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/lab4_1_p28FP.o28FP.dep
package/cfg/lab4_1_p28FP.o28FP.dep: ;
endif

package/cfg/lab4_1_p28FP.o28FP: | .interfaces
package/cfg/lab4_1_p28FP.o28FP: package/cfg/lab4_1_p28FP.c package/cfg/lab4_1_p28FP.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) cl28FP $< ...
	$(ti.targets.C28_float.rootDir)/bin/cl2000 -c  -g --optimize_with_debug -qq -pdsw225 -Dfar=  -mo -v28 -DLARGE_MODEL=1 -ml --float_support=fpu32 -eo.o28FP -ea.s28FP   -Dxdc_cfg__xheader__='"configPkg/package/cfg/lab4_1_p28FP.h"'  -Dxdc_target_name__=C28_float -Dxdc_target_types__=ti/targets/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_17_6_0 -O2  $(XDCINCS) -I$(ti.targets.C28_float.rootDir)/include -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s o28FP $< -C   -g --optimize_with_debug -qq -pdsw225 -Dfar=  -mo -v28 -DLARGE_MODEL=1 -ml --float_support=fpu32 -eo.o28FP -ea.s28FP   -Dxdc_cfg__xheader__='"configPkg/package/cfg/lab4_1_p28FP.h"'  -Dxdc_target_name__=C28_float -Dxdc_target_types__=ti/targets/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_17_6_0 -O2  $(XDCINCS) -I$(ti.targets.C28_float.rootDir)/include -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/lab4_1_p28FP.o28FP: export C_DIR=
package/cfg/lab4_1_p28FP.o28FP: PATH:=$(ti.targets.C28_float.rootDir)/bin/;$(PATH)
package/cfg/lab4_1_p28FP.o28FP: Path:=$(ti.targets.C28_float.rootDir)/bin/;$(PATH)

package/cfg/lab4_1_p28FP.s28FP: | .interfaces
package/cfg/lab4_1_p28FP.s28FP: package/cfg/lab4_1_p28FP.c package/cfg/lab4_1_p28FP.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) cl28FP -n $< ...
	$(ti.targets.C28_float.rootDir)/bin/cl2000 -c -n -s --symdebug:none -g --optimize_with_debug -qq -pdsw225 -Dfar=  -v28 -DLARGE_MODEL=1 -ml --float_support=fpu32 -eo.o28FP -ea.s28FP   -Dxdc_cfg__xheader__='"configPkg/package/cfg/lab4_1_p28FP.h"'  -Dxdc_target_name__=C28_float -Dxdc_target_types__=ti/targets/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_17_6_0 -O2  $(XDCINCS) -I$(ti.targets.C28_float.rootDir)/include -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s o28FP $< -C  -n -s --symdebug:none -g --optimize_with_debug -qq -pdsw225 -Dfar=  -v28 -DLARGE_MODEL=1 -ml --float_support=fpu32 -eo.o28FP -ea.s28FP   -Dxdc_cfg__xheader__='"configPkg/package/cfg/lab4_1_p28FP.h"'  -Dxdc_target_name__=C28_float -Dxdc_target_types__=ti/targets/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_17_6_0 -O2  $(XDCINCS) -I$(ti.targets.C28_float.rootDir)/include -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/lab4_1_p28FP.s28FP: export C_DIR=
package/cfg/lab4_1_p28FP.s28FP: PATH:=$(ti.targets.C28_float.rootDir)/bin/;$(PATH)
package/cfg/lab4_1_p28FP.s28FP: Path:=$(ti.targets.C28_float.rootDir)/bin/;$(PATH)

clean,28FP ::
	-$(RM) package/cfg/lab4_1_p28FP.o28FP
	-$(RM) package/cfg/lab4_1_p28FP.s28FP

lab4_1.p28FP: package/cfg/lab4_1_p28FP.o28FP package/cfg/lab4_1_p28FP.mak

clean::
	-$(RM) package/cfg/lab4_1_p28FP.mak
