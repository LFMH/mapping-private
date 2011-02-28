#!/bin/bash

#bash ../scripts/demos_artificial/0.calc_autoThreshold.sh
bash ../scripts/demos_artificial/0.computeFeature.sh
bash ../scripts/demos_artificial/2.computeFeature.sh
bash ../scripts/demos_artificial/0.calcNormalization.sh
bash ../scripts/demos_artificial/0.compute_compress_axis.sh
bash ../scripts/demos_artificial/1.compute_subspace.sh
bash ../scripts/demos_aficial/test_all.sh ~/tmp/noisy_shapes
