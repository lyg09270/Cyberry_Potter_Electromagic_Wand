@echo off
setlocal enabledelayedexpansion

:: Prompt user for folder paths and increment value
set /p source_folder="Enter source folder path: "
set /p target_folder="Enter target folder path: "
set /p increment_value="Enter the value to increment by: "

:: Check if source folder exists
if not exist "%source_folder%" (
    echo Source folder does not exist. Please check the path and try again.
    pause
    exit /b 1
)

:: Check if source folder contains .txt files
dir /b "%source_folder%\*.txt" >nul 2>&1
if %errorlevel% neq 0 (
    echo No .txt files found in the source folder.
    pause
    exit /b 1
)

:: Create target folder if it does not exist
if not exist "%target_folder%" (
    mkdir "%target_folder%"
)

:: Iterate over all .txt files in the source folder
set "file_count=0"
for %%f in ("%source_folder%\*.txt") do (
    set /a file_count+=1
    set "filename=%%~nxf"
    set "newfile=%target_folder%\!filename!"

    echo Processing file: !filename!

    :: Open file for reading and writing
    (for /f "delims=" %%l in ('type "%%f"') do (
        set "line=%%l"
        :: Find and increment numbers
        set "modified_line=!line!"
        for /f "tokens=1,* delims=0123456789" %%a in ("!line!") do (
            for /f "delims=" %%d in ("%%b") do (
                if "%%d" neq "" (
                    set /a new_number=%%d + increment_value
                    set "modified_line=%%a!new_number!"
                )
            )
        )
        echo(!modified_line!
    )) > "!newfile!"
)

if %file_count% equ 0 (
    echo No .txt files processed.
) else (
    echo Process completed. %file_count% files processed.
)

pause
endlocal