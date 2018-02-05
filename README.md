# This code runs on PMD to talk to the host python script

because the PMD tcp commands are too slow.

The whole example code and library is provided for convenience of compiling.

## compile

install the PMD SDK (need the windows port of arm-gcc compiler)

```
cd CMECode/Examples/davinci-comm
make clean
make all
```

warning: won't compile on gnu/linux. the makefile require case insensitivity to work. blame the PMD people.
