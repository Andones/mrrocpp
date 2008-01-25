@echo off

rem ------------------------------------------------------------
REM cleanDox.bat
rem -----------------------------------------------------------

set CLEAN=cleanDox.bat
set TMP=*.tmp
set SWAP=*~
set LOG=*.log

rem ################################################################
echo dox\cleanDox.bat
rem ################################################################
	if not exist %TMP%  (echo No %TMP% found) 	else (for %%i in (%TMP%) do del %%i) 
	if not exist %SWAP% (echo No %SWAP%  found) else (for %%i in (%SWAP%) do del %%i)
	if not exist %LOG%  (echo No %LOG%  found) 	else (for %%i in (%LOG%) do del %%i)

	FOR /D %%i IN (*.*) DO 	(cd %%i 
													CALL %CLEAN% 
													cd ..)
