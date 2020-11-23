if not exist .\outimage\%1 mkdir .\outimage\%1
del outimage\%1\*.png
 ..\x64\debug\ColorFilter_t.exe C:\temp\11,11使用画像\%1mmy.png ColorFilter.txt 0 66 0 12 90 8 C:\temp\11,11使用画像\%1mmno.png 19 false > outimage\%1\log.txt
move /Y *.png outimage\%1
