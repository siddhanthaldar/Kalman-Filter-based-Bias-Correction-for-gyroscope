# Kalman Filter based Bias Correction for Gyroscope

This repository contains the code for using **Linear Kalman Filter** for estimating the bias
present in gyroscope data. Here we take the x, y and z axis data from the gyroscope when the bot is at
rest. The data is used in combination with linear kalman filter to estimate the initial bias in the data and then 
this bias is subtracted from the incoming data to obtain the corrected output.

### Note
Here only the data in the interval between 25s to 45s is considered as during this phase the bot was more or less stable.

## Link for the paper
https://www.researchgate.net/publication/299584644_Kalman_Filter_based_estimation_of_constant_angular_rate_bias_for_MEMS_Gyroscope

## Dataset
The bag file MH_01_easy.bag from the [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) has been used for the purpose.

## Results

![kalman_res](https://user-images.githubusercontent.com/25313941/41195574-ab935928-6c4d-11e8-9046-b9253c6411c1.png)


