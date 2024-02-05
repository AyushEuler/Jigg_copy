:: Deletes All files in the Current Directory With Prompts and Warnings 
::(Hidden, System, and Read-Only Files are Not Affected) 
:: 
@ECHO OFF 

xpack-openocd-0.11.0-4\bin\openocd.exe -f ./openocd.cfg -c "program build/MarvelTestingJig.bin reset exit 0x8000000"
