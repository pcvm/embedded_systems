#! /bin/sh

cat Phillip_Clock_Thing2_13.ino | egrep "^//" | sed -e 's/TODO/divert(-1)/' | m4
