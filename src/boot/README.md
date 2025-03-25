# 4B25 Coursework 4
**Author**: Monami Yoshioka, my399, Pembroke College

## Overview
This project uses accelerometer data to classify different activities performed by a user. It includes data processing, feature extraction, and classification using a **multiclass logistic regression** algorithm.

## File Structure

### **data/**
- **CSV Files**: Contains raw accelerometer data for different activities (hook, jab, rest, curl, t-pose, shoulder_rotation, and rotating_punch).

### **python/**
- **plot_raw_data.py**: This script plots the raw accelerometer data to visualize the signal for each activity.
  
- **feature_classifier.py**: This script processes the raw accelerometer data, extracts relevant features (mean, standard deviation, etc.), and trains a **multiclass logistic regression** model to classify the activity based on these features.
  
- **logistic_model.txt**: The output of the trained logistic regression model. This file contains the learned weights and biases that map the extracted features to the activity classes.

### **warp/**
This folder contains the files and modifications necessary to run the classification system on the embedded system using the Warp SDK (ksdk1.1.0).

#### **ksdk1.1.0 (Modified Files)**
- **boot.c**: The `while(1)` loop has been modified to remove unnecessary code and call the `run_4B25_coursework()` function from `4B25_coursework.c` to integrate the coursework functionality.
  
- **CMakeLists-FRDMKL03.txt**: The `4B25_coursework.c` file has been added to the makefile to ensure it's compiled and linked with the project.
  
- **4B25_coursework.h**: Header file defining the necessary functions and variables used in `4B25_coursework.c` for the coursework.
  
- **4B25_coursework.c**: Contains the code that implements the functionality of the coursework, integrating the activity classification with the embedded platform.
