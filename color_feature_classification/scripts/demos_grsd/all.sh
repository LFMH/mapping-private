#!/bin/bash

bash `rospack find color_feature_classification`/scripts/demos_grsd/1.computeGRSD.sh 7 2
bash `rospack find color_feature_classification`/scripts/demos_grsd/1.calcNormalization.sh
bash `rospack find color_feature_classification`/scripts/demos_grsd/1.compute_subspace.sh
