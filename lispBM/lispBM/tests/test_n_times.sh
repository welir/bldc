#!/bin/bash

echo "BUILDING"

make clean
make

n=$1
test=$2
result=0
success_count=0
fail_count=0

for ((i=0; i<n; i++));
do
    ./test_lisp_code_cps -s -h 32000 $test

    result=$?

    echo "------------------------------------------------------------"
    echo Test: $i
    if [ $result -eq 1 ]
    then
        success_count=$((success_count+1))
        echo SUCCESS
    else
        fail_count=$((fail_count+1))
        echo FAILED
        break
    fi
    echo "------------------------------------------------------------"
done

echo OK: $success_count
echo FAILED: $fail_count
