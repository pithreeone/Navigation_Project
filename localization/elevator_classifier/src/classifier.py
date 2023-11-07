#!/usr/bin/env python
"""
===================== PLEASE WRITE HERE =====================
- The title of this script.
- A brief explanation of this script, e.g. the purpose of this script, what
can this script achieve or solve, what algorithms are used in this script...

- purpose: HW1 of machine-learning
- author: Yan-Ting Chen
- student number: 109033238
- date: 2023/10/7

- The name of author and the created date of this script.
===================== PLEASE WRITE HERE =====================
"""

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

def load_data(path, c):
    data = np.genfromtxt(path, delimiter=',')

    # Print the number of samples
    n_samples = len(data)
    print('Load data with class: ', c)
    print('Number of features:', len(data[0]))
    print('Number of samples:', n_samples-1)

    # Split the data into features and labels.
    # According to the dataset description, the first column is class
    # identifier.
    ang = 45
    x1 = data[1:, 0:ang]
    x2 = data[1:,359-ang:359]
    x = np.append(x1, x2, axis=1)
    # x = data[1:,:]
    y = np.full(n_samples-1,c)

    return data, x, y

def class_distribution(y):

    bin = np.bincount(y)
    n_class1 = bin[1]
    n_class2 = bin[2]

    print('Number of samples in class_1:', n_class1)
    print('Number of samples in class_2:', n_class2)


def split_dataset(x, y, testset_portion):
    print('Split dataset.')
    x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = testset_portion, random_state = None, shuffle=True)

    return x_train, x_test, y_train, y_test

def fill_nan(x):
    mean = np.nanmean(x, axis=0)
    for i in range(len(mean)):
        if(np.isinf(mean[i])==True):
            mean[i]=10
    # print(mean)
#   max = np.nanmax(x, axis=0)
#   min = np.nanmin(x, axis=0)

  # # Method 2: Impute missing values as average of the feature
    for i in range(0, len(x[0])):
        for j in range(len(x)):
            if np.isinf(x[j,i]) == True:
                x[j,i] = mean[i]
    return x

def feature_scaling(x_train, x_test):
    # print('Feature scaling.')

    scaler = StandardScaler()
    x_train_nor = scaler.fit_transform(x_train)
    x_test_nor = scaler.transform(x_test)

    return x_train_nor, x_test_nor

def train(x_train, y_train):
    # print('Start training.')
    clf=GaussianNB()
    clf.fit(x_train,y_train)

    return clf

def test(clf, x_test):
    print('Start testing.')
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

data_get = np.zeros(360)

def ScanCB(data):
    print("Received LaserScan data")
    # laser_data = np.array(data.ranges)
    # print(laser_data.shape)
    delta_ang = data.angle_increment
    n_ranges = len(data.ranges)
    for i in range(360):
        id = int(0 + n_ranges / 360 * i)
        data_get[i] = data.ranges[id]


if __name__=='__main__':
    # Some parameters
    path_co = '/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/data_co.csv'
    path_oc = '/home/pithreeone/amr_robot/src/navigation/localization/elevator_classifier/data/data_oc.csv'
    testset_portion = 0.01

    # Load data
    _, x_co, y_co = load_data(path_co, 1)
    _, x_oc, y_oc = load_data(path_oc, 2)

    data_x = np.append(x_co, x_oc, axis=0)
    data_y = np.append(y_co, y_oc)
    # print(combined_y.shape)
    class_distribution(data_y)

    # # Preprocessing
    x_train, x_test, y_train, y_test = split_dataset(data_x, data_y, testset_portion)
    x_train = fill_nan(x_train)
    x_test = fill_nan(x_test)
    x_train_nor, x_test_nor = feature_scaling(x_train, x_test)

    # # Classification: train and test
    clf = train(x_train_nor, y_train)
    y_pred = test(clf, x_test_nor)

    # # Accuracy
    acc = accuracy_score(y_test, y_pred)
    print('\nAccuracy:', round(acc, 3))
    # output_result(x_test, y_pred, y_test)

    # ROS-NODE
    rospy.init_node('elevator_classifier')
    sub = rospy.Subscriber('scan_filtered', LaserScan, ScanCB)
    pub = rospy.Publisher('elevator_status', Int32, queue_size=10)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        ang = 45
        x1 = data_get[0:ang]
        x2 = data_get[359-ang:359]
        data_extract = np.append(x1, x2)
        data_extract = data_extract.reshape(1,-1)
        ans = clf.predict(data_extract)
        pub_msg = Int32()
        pub_msg.data = ans
        pub.publish(pub_msg)
        print(data_get)
        print('elevator_classifier: elevator is', ans)
        rate.sleep()