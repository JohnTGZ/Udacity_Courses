# Write a short recap of the four tracking steps and what you implemented there (EKF, track management, data association, camera-lidar sensor fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
## EKF
In EKF, the update and predict step is implemented. This part is pretty cool as we get to work with actual sensor data (instead of the sample data that we use in the exercises). Although the assumptions used for the state transition matrix and process noise covariance may be simplified compared to production-level systems, this enabled me to grasp the underlying concept behind them.
However, it was a little confusing how as to which measurement function to use: `get_H()` or `get_hx()`? 

## track management
The track management task requires you to update the score and state of multiple tracks. Deleting them when their score falls below the threshold. This prevents us from assigning a track to every measurement and also provide us with a confidence measure of each tracked item.

## data association
The data association part utilises Single Nearest Neighbour association (with Mahalanobis Distance as the metric) to determine the measurement-track pair association, and utilises the inverse cumulative Chi-squared distribution in the gating function to filter out unlikely association pairs.
This part provides us with the ability to associate measurement with the likeliest track.

## camera-lidar sensor fusion
Here lies the part that we been building up to, the ability to incorporate the observation of multiple sensors. There was quite a bit of effort spent to implement the `get_hx()` measurement function and understand the FOV model. Thankfully, Udacity did the God's work and implemented the Jacobian H.

The devil is in the details, often the results were not correct as I made mistakes or missed out on certain parts of the implementation, requiring some tedious debugging. A tricky example is the track management task, where we had to update multiple tracks and change the state of the track and update its score accordingly. My initial failure to implement it properly led to the score of the track staying constant and staying in the 'tentative' state. 

# Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?
In theory, the camera-lidar fusion should provide us with multiple modalities of sensing, where if lidar performance falls short, the camera is able to pick up, and vice versa. With reference to our results, the camera helps to significantly reduce the number of ghost tracks or reduce their lifetime across the frames, therefore helping us to get rid of false positives. However, there is marginal improvement to the RMS Error. 

# Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
False positives and false negatives from one sensor could affect the final results of the filter. In this project, there are false positives from the lidar readings, which become 'tentative' tracks, but implementing information from the camera prevents these tracks from increasing score and changing state to 'confirmed'. To best deal with such challenges, the underlying mechanism that each sensor relies on to capture information is different, so the sensor fusion model needs to represent the modality of each of these sensors well.
Other challenges are hardware based, where sensor fusion systems may have to deal with the bandwidth and computing power required for processing large samples of data, then, there will be a need to downsample the data before feeding it to the sensor fusion system.

# Can you think of ways to improve your tracking results in the future?
- It seems that there are still quite a few false positives, although they stay as 'tentative' tracks, we could reduce such instances by tuning the detection model.  
- To improve the optimality of measurement-track association, we could use Global Nearest Neighbour (GNN) or Probabilistic Data Association (PDA).
- Furthermore sensor performance could degrade over prolonged use and online calibration of both the lidar/camera sensor intrinsic/extrinsic parameters would improve the accuracy of the measurements.