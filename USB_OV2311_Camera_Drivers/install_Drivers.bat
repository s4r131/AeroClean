@echo off

>nul 2>&1 "%SYSTEMROOT%\system32\cacls.exe" "%SYSTEMROOT%\system32\config\system"

if '%errorlevel%' NEQ '0' (
goto UACPrompt
) else ( goto gotAdmin )

:UACPrompt
echo Set UAC = CreateObject^("Shell.Application"^) > "%temp%\getadmin.vbs"
echo UAC.ShellExecute "%~s0", "", "", "runas", 1 >> "%temp%\getadmin.vbs"
"%temp%\getadmin.vbs"
exit /B

:gotAdmin
if exist "%temp%\getadmin.vbs" ( del "%temp%\getadmin.vbs" )

pushd "%CD%"

cd /d %~dp0
if "%PROCESSOR_ARCHITECTURE%" == "x86" goto x86
if "%PROCESSOR_ARCHITECTURE%" == "AMD64" goto x64

:x64
echo %PROCESSOR_ARCHITECTURE% 
C:\Windows\System32\pnputil.exe /add-driver .\Drivers\x64\cyusb3.inf /install
goto end

:x86
echo %PROCESSOR_ARCHITECTURE% 
C:\Windows\System32\pnputil.exe /add-driver .\Drivers\x86\cyusb3.inf /install
goto end

:end
echo "Driver installed successfully"
pause