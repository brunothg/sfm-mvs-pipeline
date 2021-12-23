@echo off

if "%~1"=="" goto :help
if /I %1 == docs goto :docs

:help
echo Unknown run goal. Available: docs
goto :eof

:docs
mkdir docs
rmdir /S /Q docs
doxygen Doxyfile

cd docs/latex
call make.bat
cd ../..

goto :eof