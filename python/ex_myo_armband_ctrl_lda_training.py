# Load libraries
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
import numpy as np
import pandas
from sklearn import model_selection
from sklearn import feature_selection
from MelShare import MelShare


ms_fp = MelShare("training_path")
ms_fn = MelShare("training_name")
ms_lda = MelShare("training_flag")
ms_cv = MelShare("cv_results")


# get full file path and file name for EMG training data
filepath = ms_fp.read_message()
filename = ms_fn.read_message()

# extract dof from file name
str_dof, token = filename.split("_",1)

# read data from file
fullfile = filepath + "\\" + filename + ".csv"
dataset = pandas.read_csv(fullfile, header = 1)
array = dataset.values

#determine the number of active electrodes
n_features = array.shape[1]-1

# extract training data and training labels from file data
X = array[:,0:int(n_features)-1]
Y = array[:,int(n_features)]

# determine number of classes from training labels (assuming they begin at 0)
num_class = int(np.amax(Y))

# determine number of training observations per class
N_train = Y.size / num_class

# create an estimator object of type LDA
lda = LinearDiscriminantAnalysis()
lda.set_params(solver='lsqr',shrinkage=None)

# create a cross-validation generator
# number of folds cannot be more than N_train, and we are artificially capping it at 5 
# no shuffling is applied because the classes were presented randomly during training, and any time-dependent effects are being ignored
# the random seed is left as default parameter None, not sure if this has an effect when shuffling turned off
n_splits = min(N_train, 5)
kfold = model_selection.KFold(n_splits)

# Recursive Feature Extraction with Cross-Validation
# using the output of our lda estimator (coefficients) as an indication of feature importance, features are removed step = 1 at a time
# our kfold cross-validation generator is used to evaluate the scoring = accuracy of each feature subset to find the optimum
step = 1
scoring = 'accuracy'
rfe = feature_selection.RFECV(lda,step,kfold,scoring)
rfe.fit(X,Y)
sel_feats = rfe.get_support(True)
n_sel_features = sel_feats.size

# downselected feature matrix and refit the LDA classifier with selected features
X_transform = rfe.transform(X)
lda.fit(X_transform, Y)

# extract the fitted LDA coefficient matrix and intercept vector
coeff = lda.coef_
intercept = lda.intercept_

# perform cross validation on final subset of features and report results
cv_results = model_selection.cross_val_score(lda, X_transform, Y, cv=kfold, scoring=scoring)
ms_cv.write_data(cv_results)

# build full classifier matrix
if num_class == 2:
	classifier = np.zeros((2,n_features + 1))
	for i in range(0,n_sel_features):
		classifier[0,sel_feats[i]] = -coeff[0][i]
		classifier[0,-1] = -intercept[0]
		classifier[1,sel_feats[i]] = coeff[0][i]
		classifier[1,-1] = intercept[0]
else:
	classifier = np.zeros((num_class,n_features + 1))
	for i in range(0,n_sel_features):
		for j in range(0,num_class):
			classifier[j,sel_feats[i]] = coeff[j][i]
		classifier[j,-1] = intercept[j]

# write full classifier matrix to file
my_classifier = pandas.DataFrame(classifier)
classifier_filename = str_dof + "_" + "myo_armband_python_classifier"
my_classifier.to_csv(filepath + "\\" + classifier_filename + ".csv", index=False, header=False)

# send flag over MelShare indicating code successfully executed
flag = [1]*1
ms_lda.write_data(flag)

