#!/bin/bash

bash `rospack find color_feature_classification`/scripts/demos/0.calc_autoThreshold.sh
bash `rospack find color_feature_classification`/scripts/demos/0.computeFeature.sh 4 1
bash `rospack find color_feature_classification`/scripts/demos/0.computeFeature.sh 7 2
bash `rospack find color_feature_classification`/scripts/demos/2.computeFeature.sh
bash `rospack find color_feature_classification`/scripts/demos/0.calcNormalization.sh
bash `rospack find color_feature_classification`/scripts/demos/0.compute_compress_axis.sh
bash `rospack find color_feature_classification`/scripts/demos/1.compute_subspace.sh

