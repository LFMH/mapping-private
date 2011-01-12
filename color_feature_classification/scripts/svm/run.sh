#!/bin/bash

echo ""; echo "========== GRSD ========="
bash do_svm.sh g 20 $1_grsd
echo ""; echo "========== CHLAC ========="
bash do_svm.sh c 981 $1_chlac
echo ""; echo "========== CONCAT ========="
bash do_svm.sh d 1001 $1_concat
echo ""; echo "========== VOSCH ========="
bash do_svm.sh r 137 $1_vosch

