#!/bin/bash

echo ""; echo "========== GRSD ========="
./do_svm.sh g 20 $1_grsd
echo ""; echo "========== CHLAC ========="
./do_svm.sh c 981 $1_chlac
echo ""; echo "========== CONCAT ========="
./do_svm.sh d 1001 $1_concat
echo ""; echo "========== VOSCH ========="
./do_svm.sh r 137 $1_vosch

