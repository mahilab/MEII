# Load libraries
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
import ctypes
import numpy as np
import pandas
from sklearn import model_selection
from sklearn import feature_selection


# manually enter file path and name
filepath = "C:\\Users\\Ted\\GitHub\\MEII\\EmgRTControl\\EMG_S04\\EFE"
filename = "S04_EFE_training_data"

# extract subject number and dof from file name
str_subject_num, token = filename.split("_",1)
str_dof, token = token.split("_",1)


# read data from file
fullfile = filepath + "\\" + filename + ".csv"
dataset = pandas.read_csv(fullfile, header = None)
array = dataset.values

# extract training data and training labels from file data
X = array[:,0:71]
Y = array[:,72]

# determine number of classes from training labels (assuming they begin at 1)
num_class = int(np.amax(Y))
print("Number of classes: %s" % num_class)

# determine number of training observations per class
N_train = Y.size / num_class
print("Number of training observations per class: %s" % N_train)

# create an estimator object of type LDA
lda = LinearDiscriminantAnalysis()
lda.set_params(solver='lsqr',shrinkage=None)

# create a cross-validation generator
# number of folds cannot be more than N_train, and we are artificially capping it at 5 
# no shuffling is applied because the classes were presented randomly during training, and any time-dependent effects are being ignored
# the random seed is left as default parameter None, not sure if this has an effect when shuffling turned off
n_splits = min(N_train, 5)
print(n_splits)
kfold = model_selection.KFold(n_splits)


# Recursive Feature Extraction with Cross-Validation
# using the output of our lda estimator (coefficients) as an indication of feature importance, features are removed step = 1 at a time
# our kfold cross-validation generator is used to evaluate the scoring = accuracy of each feature subset to find the optimum
step = 1
scoring = 'accuracy'
rfe = feature_selection.RFECV(lda,step,kfold,scoring)
rfe.fit(X,Y)
sel_feats = rfe.get_support(True)
n_features = sel_feats.size
print("Accuracy scores using KFold cross-validation on datasets containing 1-72 features:")
print rfe.grid_scores_
print("Number of selected features: %s" % n_features)
print("selected features indicies: %s" % sel_feats)


# downselected feature matrix and refit the LDA classifier with selected features
X_transform = rfe.transform(X)
lda.fit(X_transform, Y)


# extract the fitted LDA coefficient matrix and intercept vector
coeff = lda.coef_
intercept = lda.intercept_

#if(np.shape(intercept)[0]==4):
#	multidof = True
#else:
#	multidof = False



# perform cross-validation on final subset of features and report results
cv_results = model_selection.cross_val_score(lda, X_transform, Y, cv=kfold, scoring=scoring)
print("%s: %f (%f)" % ("Cross-validation mean, std, and array of scores:", cv_results.mean(), cv_results.std()))
print cv_results
cvArray = ctypes.c_double * n_splits


# build full classifier matrix
if num_class == 2:
	classifier = np.zeros((1,73))
	for i in range(0,n_features):
		classifier[0,sel_feats[i]] = coeff[0][i]
		classifier[0,-1] = intercept[0]
else:
	classifier = np.zeros((num_class,73))
	for i in range(0,n_features):
		for j in range(0,num_class):
			classifier[j,sel_feats[i]] = coeff[j][i]
			classifier[j,-1] = intercept[j]


#if (multidof==True):
#	classifier = np.zeros((4,73))
#	for i in range(0,sel_feats.size):
#		for j in range(0,np.shape(intercept)[0]):
#			classifier[j,sel_feats[i]] = coeff[j][i]
#			classifier[j,-1] = intercept[j]
#	print classifier

#else:
#	classifier = np.zeros((1,73))
#	for i in range(0,sel_feats.size):
#		classifier[0,sel_feats[i]] = coeff[0][i]
#	classifier[0,-1] = intercept[0]
#	print classifier


# write full classifier matrix to file
my_classifier = pandas.DataFrame(classifier)
classifier_filename = str_subject_num + "_" + str_dof + "_" + "emg_dir_classifier"
my_classifier.to_csv(filepath + "\\" + classifier_filename + ".csv", index=False, header=False)





