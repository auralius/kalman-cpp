del *.exe
del *.user
del *.ncb
del *.pdb
del *.lib
del *.ilk
del *.txt
del *.exp
del *.suo
del *.filters
del *.db

rd /S /Q .vs

rd /S /Q x64

for /f %%i in ('dir /a:d /s /b Debug*') do rd /S /Q %%i

for /f %%i in ('dir /a:d /s /b Release*') do rd /S /Q %%i