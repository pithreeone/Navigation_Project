#!/usr/bin/env python
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.naive_bayes import GaussianNB
from sklearn.metrics import accuracy_score
import csv
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

# Define class number
# cc: 0, co: 1, oc: 2, oo: 3

# # # Global variables
ang_limited = 60                # only take the value along x-axis +- ang_limited
range_max = 10
data_get = np.zeros(360)        # data get from the laserscan topic

def load_data(path, c):
    data = np.genfromtxt(path, delimiter=',')

    # Print the number of samples
    n_samples = len(data)
    # print('Load data with class: ', c)
    # print('Number of features:', len(data[0]))
    # print('Number of samples:', n_samples-1)

    x1 = data[1:, 0:ang_limited]
    x2 = data[1:,359-ang_limited:359]
    x = np.append(x1, x2, axis=1)
    
    y = np.full(n_samples-1,c)

    return data, x, y

def class_distribution(y):
    bin = np.bincount(y)
    n_class0 = bin[0]
    n_class1 = bin[1]
    n_class2 = bin[2]
    print('Number of samples in class_0:', n_class0)
    print('Number of samples in class_1:', n_class1)
    print('Number of samples in class_2:', n_class2)


def split_dataset(x, y, testset_portion):
    # print('Split dataset.')
    x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = testset_portion, random_state = None, shuffle=True)

    return x_train, x_test, y_train, y_test

def fill_nan(x):
#   max = np.nanmax(x, axis=0)
#   min = np.nanmin(x, axis=0)
    mean = np.nanmean(x, axis=0)
    for i in range(len(mean)):
        if(np.isinf(mean[i])==True):
            mean[i]=range_max

  # # Impute missing values as average of the feature
    for i in range(0, len(x[0])):
        for j in range(len(x)):
            if np.isinf(x[j,i]) == True:
                x[j,i] = mean[i]
                # x[j,i] = range_max
    return x

def feature_scaling(x_train, x_test):
    # print('Feature scaling.')
    scaler = StandardScaler()
    x_train_nor = scaler.fit_transform(x_train)
    x_test_nor = scaler.transform(x_test)

    return scaler, x_train_nor, x_test_nor

def train(x_train, y_train):
    # print('Start training.')
    clf=GaussianNB()
    clf.fit(x_train,y_train)
    return clf

def train_model():
    # Some parameters
    path_co = '/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/data_co.csv'
    path_oc = '/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/data_oc.csv'
    path_cc = '/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/data_cc.csv'
    testset_portion = 0.3

    # Load data
    _, x_cc, y_cc = load_data(path_cc, 0)
    _, x_co, y_co = load_data(path_co, 1)
    _, x_oc, y_oc = load_data(path_oc, 2)

    
    data_x = np.append(x_co, x_oc, axis=0)
    data_x = np.append(data_x, x_cc, axis=0)
    data_y = np.append(y_co, y_oc)
    data_y = np.append(data_y, y_cc)
    class_distribution(data_y)
    print(data_y)

    # # Preprocessing
    x_train, x_test, y_train, y_test = split_dataset(data_x, data_y, testset_portion)
    x_train = fill_nan(x_train)
    x_test = fill_nan(x_test)
    # scaler, x_train_nor, x_test_nor = feature_scaling(x_train, x_test)
    scaler = None

    # # Classification: train and test
    clf = train(x_train, y_train)
    y_pred = test(clf, x_test)

    # # Accuracy
    acc = accuracy_score(y_test, y_pred)
    print('\nAccuracy:', round(acc, 3))
    # output_result(x_test, y_pred, y_test)

    return scaler, clf

def test(clf, x_test):
    # print('Start testing.')
    y_pred = clf.predict(x_test)
    return y_pred

def output_result(x_test, y_test, y_ans):
    csv_file = "/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/result.csv"
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write the data to the CSV file
        for row in x_test:
            writer.writerow(row)

    print(f"Data has been written to {csv_file}")

def ScanCB(data):
    print("Received LaserScan data")
    # laser_data = np.array(data.ranges)
    # print(laser_data.shape)
    delta_ang = data.angle_increment
    n_ranges = len(data.ranges)
    for i in range(360):
        id = int(0 + n_ranges / 360 * i)
        temp = data.ranges[id]
        if temp >= range_max:
            temp = range_max
        data_get[i] = temp


if __name__=='__main__':
    # ROS-NODE
    rospy.init_node('elevator_classifier', anonymous=True)
    sub = rospy.Subscriber('scan_filtered', LaserScan, ScanCB)
    pub = rospy.Publisher('elevator_status', Int32, queue_size=10)

    scaler, clf = train_model()
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        x1 = data_get[0:ang_limited]
        x2 = data_get[359-ang_limited:359]
        data_extract = np.append(x1, x2)
        data_extract = data_extract.reshape(1,-1)

        # data_extract = scaler.transform(data_extract)
        ans = clf.predict(data_extract)
        # publish the final predict
        pub_msg = Int32()
        # pub_msg.data = ans
        pub.publish(pub_msg)
        print(data_extract)
        print('elevator_classifier: elevator is', ans)
        rate.sleep()