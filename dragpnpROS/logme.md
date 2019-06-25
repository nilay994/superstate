28-05
- [x] use accelerometer for ROS drag model - since it is biasless and only augmented with gaussian noise
- only tapping the ground thruth at PnP locations - for time stamp problems - maybe this can stay? 
- ask YF why there are repeated time stamps with different position with same time header?
- ask gcd if the velocity can be tapped from current linear drag model, 
  or should it be estimated offline by rotating PnP gradient of positions