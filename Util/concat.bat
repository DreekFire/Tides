:: Concatenates all the class files into a single Tides.lua
:: minification must still be done manually by uploading it to a Lua minifier website (offline minifiers exist, but meh)
@echo off
(for /f "delims= eol=" %%i in ('dir Low Medium High /b/s') do (
  type "%%~i"
  echo.
))>Tides.lua