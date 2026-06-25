#!/bin/bash
if [ -z $1 ]
then
echo "Error: Script requires segfault address."
exit
fi

arm-none-eabi-addr2line -faps -e ./02_V5/ghost_pros/bin/hot.package.elf $1