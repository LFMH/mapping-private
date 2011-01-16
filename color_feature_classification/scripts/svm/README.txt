TRAINING DATA

- if you have your training data in:
color_feature_classification/demos/data/obj*

- in the scripts/svm folder execute:
bash mkHistForSVM_real.sh

- it will save training data in:
hist_data_forSVM_real/train_features_r/ and so on


TESTING DATA

- extract novel view folders to
- color_feature_classification/demos/novel_view/

- in the scripts/svm folder execute:
bash mkHistForSVM_novel.sh 63

- it will save training data in:
hist_data_forSVM_real/novel_view/0/ and so on

- if you want you can rename the numbered foldered as "occlusion1" and so on
to make the classification results more inteligable
(the order of the directories is preserved, but I saved them as numbers..
sorry)


CLASSIFICATION:

- in the scripts/svm folder execute the very simple script:
bash todo_novel_test.sh BLA

- this will create the SVM train and test files with the base name "BLA" in
- the current directory

- it will also output the results on the screen as I keep sending them in
- the mails

- the actual script is run_new.sh and you can see in it that I commented out
the training and testing of GRSD and ColorCHLAC to make it faster, but you
- can re-enable them

