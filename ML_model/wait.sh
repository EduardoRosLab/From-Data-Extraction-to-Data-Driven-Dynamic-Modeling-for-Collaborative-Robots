#! /bin/bash

for i in {0..15};
do
    sleep 1
    if [ -f $1_commanded ];
    then
	break
    fi
done
