
:: start cmd /k "cd \dgnava2_maneyko2\SE423Repo\Labs\lab8_dgnava2_maneyko2\lab8_dgnava2_maneyko2Project\Debug"
@ECHO OFF

SET fname="\dgnava2_maneyko2\SE423Repo\Labs\lab8_dgnava2_maneyko2\lab8_dgnava2_maneyko2Project\Debug\lab8_dgnava2_maneyko2.hex"
:: start cmd /c "pscp -pw '' \dgnava2_maneyko2\SE423Repo\Labs\lab8_dgnava2_maneyko2\lab8_dgnava2_maneyko2Project\Debug\lab8_dgnava2_maneyko2.hex root@192.168.1.78:"
pscp -pw '' %fname% root@192.168.1.78:
PAUSE
  