# Load libraries
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
import numpy as np
import pandas
from sklearn import model_selection
from sklearn import feature_selection



# get full file path and file name for EMG training data
filepath = "C:\\Git\\MEII\\EmgRealTimeControlData\\EMG_S00\\WFE"
print filepath

filename = "S00_WFE_training_data"
print filename


# extract subject number and dof from file name
str_subject_num, token = filename.split("_",1)
str_dof, token = token.split("_",1)

# read data from file
fullfile = filepath + "\\" + filename + ".csv"
dataset = pandas.read_csv(fullfile, header = 1)
array = dataset.values

#determine the number of active electrodes
n_features = array.shape[1]-1

# extract training data and training labels from file data
X = array[:,0:n_features]
Y = array[:,n_features]

# determine number of classes from training labels (assuming they begin at 0)
num_class = int(np.amax(Y))
print("Number of classes: %s" % num_class)

# determine number of training observations per class
N_train = Y.size / num_class
print("Number of training observations per class: %s" % N_train)

# create an estimator object of type LDA
lda = LinearDiscriminantAnalysis()
lda.set_params(solver='lsqr',shrinkage=0.1)

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
print("Accuracy scores using KFold cross-validation on datasets containing 1 to %s features:" % n_features)
print rfe.grid_scores_
print("Number of selected features: %s" % n_sel_features)
print("selected features indicies: %s" % sel_feats)


# downselected feature matrix and refit the LDA classifier with selected features
X_transform = rfe.transform(X)
lda.fit(X_transform, Y)


# extract the fitted LDA coefficient matrix and intercept vector
coeff = lda.coef_
intercept = lda.intercept_


# perform cross validation on final subset of features and report results
cv_results = model_selection.cross_val_score(lda, X_transform, Y, cv=kfold, scoring=scoring)
print("%s: %f (%f)" % ("Cross-validation mean, std, and array of scores:", cv_results.mean(), cv_results.std()))
print cv_results

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
	for i in range(0,num_class):
		for j in range(0,n_sel_features):		
			classifier[i,sel_feats[j]] = coeff[i][j]
			classifier[i,-1] = intercept[i]

# write full classifier matrix to file
my_classifier = pandas.DataFrame(classifier)
classifier_filename = str_subject_num + "_" + str_dof + "_" + "emg_rt_ctrl_python_classifier"
my_classifier.to_csv(filepath + "\\" + classifier_filename + ".csv", index=False, header=False)


