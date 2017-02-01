del *.exe
del *.user
del *.ncb
del *.pdb
del *.lib
del *.ilk
del *.txt
del *.exp
del *.suo

for /f %%i in ('dir /a:d /s /b Debug*') do rd /S /Q %%i

for /f %%i in ('dir /a:d /s /b Release*') do rd /S /Q %%i