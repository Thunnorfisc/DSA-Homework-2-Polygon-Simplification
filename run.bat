@echo off
setlocal enabledelayedexpansion

:: Get the directory where this script lives
set "SCRIPT_DIR=%~dp0"
:: Remove trailing backslash
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

:: Convert Windows path to WSL path
for /f "delims=" %%i in ('wsl wslpath -a "%SCRIPT_DIR%"') do set "WSL_DIR=%%i"

echo ============================================
echo   Polygon Simplification - Build and Run
echo ============================================
echo.

:: Step 1: Build the executable
echo [1/2] Building simplify executable via WSL...
echo.
wsl bash -c "cd '%WSL_DIR%' && make clean && make"
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Build failed!
    pause
    exit /b 1
)
echo.
echo Build successful!
echo.

:: Create output folder if it doesn't exist
if not exist "%SCRIPT_DIR%\output" mkdir "%SCRIPT_DIR%\output"

:: Step 2: Discover test cases
:: Count sample test cases
set "SAMPLE_COUNT=0"
for %%f in ("%SCRIPT_DIR%\sample_test_cases\*.csv") do set /a SAMPLE_COUNT+=1

:: Count my test cases
set "MY_COUNT=0"
for %%f in ("%SCRIPT_DIR%\my_test_cases\*.csv") do set /a MY_COUNT+=1

:MENU
echo ============================================
echo   Select Test Cases to Run
echo ============================================
echo.
echo   --- Sample Test Cases ---
if %SAMPLE_COUNT% equ 0 (
    echo   [Nothing found]
    echo.
) else (
    set "IDX=0"
    for %%f in ("%SCRIPT_DIR%\sample_test_cases\*.csv") do (
        set /a IDX+=1
        set "SAMPLE_!IDX!=%%~nxf"
        echo   !IDX!. %%~nf
    )
    echo.
    set /a SAMPLE_ALL_IDX=IDX+1
    echo   !SAMPLE_ALL_IDX!. Run ALL sample test cases
    echo.
)

echo   --- My Test Cases ---
if %MY_COUNT% equ 0 (
    echo   [Nothing found]
    echo.
) else (
    set /a MY_START=SAMPLE_ALL_IDX+1
    set "MY_IDX=0"
    for %%f in ("%SCRIPT_DIR%\my_test_cases\*.csv") do (
        set /a MY_IDX+=1
        set /a DISPLAY_IDX=MY_START+MY_IDX-1
        set "MYTEST_!MY_IDX!=%%~nxf"
        echo   !DISPLAY_IDX!. %%~nf
    )
    echo.
    set /a MY_ALL_IDX=MY_START+MY_IDX
    echo   !MY_ALL_IDX!. Run ALL my test cases
    echo.
)

:: Calculate "Run ALL test cases" option number
if %SAMPLE_COUNT% gtr 0 if %MY_COUNT% gtr 0 (
    set /a ALL_IDX=MY_ALL_IDX+1
) else if %SAMPLE_COUNT% gtr 0 (
    set /a ALL_IDX=SAMPLE_ALL_IDX+1
) else if %MY_COUNT% gtr 0 (
    set /a ALL_IDX=MY_ALL_IDX+1
) else (
    echo No test cases found in either folder!
    pause
    exit /b 1
)

if %SAMPLE_COUNT% gtr 0 if %MY_COUNT% gtr 0 (
    echo   --- All ---
    echo   !ALL_IDX!. Run ALL test cases ^(samples + my test cases^)
    echo.
)

echo   0. Exit
echo.
echo ============================================
set /p "CHOICE=Enter your choice: "

if "%CHOICE%"=="0" goto :END

:: Validate choice is a number
set "VALID="
for /f "delims=0123456789" %%i in ("%CHOICE%") do set "VALID=%%i"
if defined VALID (
    echo Invalid choice. Please enter a number.
    echo.
    goto :MENU
)

:: Ask for target vertices
set /p "TARGET=Enter target vertex count: "

:: Validate target is a number
set "VALID="
for /f "delims=0123456789" %%i in ("%TARGET%") do set "VALID=%%i"
if defined VALID (
    echo Invalid target. Please enter a number.
    echo.
    goto :MENU
)

echo.

:: Determine what to run based on choice
:: Check if it's "Run ALL test cases"
if %SAMPLE_COUNT% gtr 0 if %MY_COUNT% gtr 0 (
    if %CHOICE% equ %ALL_IDX% goto :RUN_ALL
)

:: Check if it's "Run ALL sample test cases"
if %SAMPLE_COUNT% gtr 0 (
    if %CHOICE% equ %SAMPLE_ALL_IDX% goto :RUN_ALL_SAMPLES
)

:: Check if it's "Run ALL my test cases"
if %MY_COUNT% gtr 0 (
    if %CHOICE% equ %MY_ALL_IDX% goto :RUN_ALL_MY
)

:: Check if it's an individual sample test case
if %SAMPLE_COUNT% gtr 0 (
    if %CHOICE% geq 1 if %CHOICE% leq %SAMPLE_COUNT% (
        set "FILE=!SAMPLE_%CHOICE%!"
        call :RUN_SINGLE "sample_test_cases" "!FILE!"
        goto :DONE
    )
)

:: Check if it's an individual my test case
if %MY_COUNT% gtr 0 (
    set /a MY_OFFSET=CHOICE-MY_START+1
    if !MY_OFFSET! geq 1 if !MY_OFFSET! leq %MY_COUNT% (
        set "FILE=!MYTEST_%MY_OFFSET%!"
        call :RUN_SINGLE "my_test_cases" "!FILE!"
        goto :DONE
    )
)

echo Invalid choice. Please try again.
echo.
goto :MENU

:: ============================================
:: Run all test cases (samples + my)
:: ============================================
:RUN_ALL
echo Running ALL test cases...
echo.
call :RUN_ALL_SAMPLES_INNER
call :RUN_ALL_MY_INNER
goto :DONE

:: ============================================
:: Run all sample test cases
:: ============================================
:RUN_ALL_SAMPLES
echo Running ALL sample test cases...
echo.
call :RUN_ALL_SAMPLES_INNER
goto :DONE

:RUN_ALL_SAMPLES_INNER
for /l %%i in (1,1,%SAMPLE_COUNT%) do (
    set "FILE=!SAMPLE_%%i!"
    call :RUN_SINGLE "sample_test_cases" "!FILE!"
)
goto :eof

:: ============================================
:: Run all my test cases
:: ============================================
:RUN_ALL_MY
echo Running ALL my test cases...
echo.
call :RUN_ALL_MY_INNER
goto :DONE

:RUN_ALL_MY_INNER
for /l %%i in (1,1,%MY_COUNT%) do (
    set "FILE=!MYTEST_%%i!"
    call :RUN_SINGLE "my_test_cases" "!FILE!"
)
goto :eof

:: ============================================
:: Run a single test case
:: Args: %1 = folder (sample_test_cases or my_test_cases), %2 = filename
:: ============================================
:RUN_SINGLE
set "FOLDER=%~1"
set "FILENAME=%~2"
set "BASENAME=%FILENAME:.csv=%"

echo --------------------------------------------
echo Running: %FOLDER%/%FILENAME% (target: %TARGET% vertices)
echo --------------------------------------------

:: Build output filename with target vertex count
set "OUT_FILE=%SCRIPT_DIR%\output\%BASENAME%_t%TARGET%.txt"

wsl bash -c "cd '%WSL_DIR%' && ./simplify '%FOLDER%/%FILENAME%' %TARGET%" > "%OUT_FILE%" 2>&1

if %errorlevel% neq 0 (
    echo   FAILED - check output file for details
) else (
    echo   Done - output saved to: output\%BASENAME%_t%TARGET%.txt
)
echo.
goto :eof

:: ============================================
:DONE
echo ============================================
echo   All done! Results are in the output folder.
echo ============================================
echo.
pause
goto :MENU

:END
echo Goodbye!
endlocal
