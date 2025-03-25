# 4B25 Coursework 4
**Author**: Monami Yoshioka, my399, Pembroke College

## Overview
This project uses accelerometer data to classify different activities performed by a user. It includes data processing, feature extraction, and classification using a **multiclass logistic regression** algorithm.

## File Structure

### **data/**
- **CSV Files**: Contains raw accelerometer data for different activities (hook, jab, rest, curl, t-pose, shoulder_rotation, and rotating_punch)

### **python/**
- **plot_raw_data.py**: this script plots the raw accelerometer data to visualize the signal for each activity.
  
- **feature_classifier.py**: this script uses raw accelerometer data and extracts features, then trains a multiclass logistic regression algorithm to the learn the mapping from the features to class for classification
  
- **logistic_model.txt**: output of the trained logistic model from the feature_classifier.py file, containing the learned weights and biases that map the extracted features to the activity classes

### **warp/**
This folder contains the files and modifications necessary to run the classification system on the embedded system using Warp

#### **ksdk1.1.0 (Modified Files)**
- **boot.c**: removed everything from while(1) loop and added run_4B25_coursework() function found in 4B25_coursework.c
  
- **CMakeLists-FRDMKL03.txt**: The `4B25_coursework.c` file has been added to the makefile
  
- **4B25_coursework.h**: Header file defining the necessary functions and variables used in `4B25_coursework.c`
  
- **4B25_coursework.c**: Contains the code that implements the functionality of the coursework
