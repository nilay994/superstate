2019-04-30_18_02_45.csv - the cute one, 4.5 x 3
2019-05-02_21_13_03.csv - agile push, maybe centrifugal excite
2019-05-13_14_36_31.csv - 90 turn, high pitch roll
2019-05-13_15_04_32.csv - maybe height excite and aggresive
2019-05-16_14_29_44.csv - sideways for thrust map - vortex hel dynamics
2019-06-17_17_01_37.csv - right angle with corol
2019-06-17_14_06_08.csv - switching time optimization
2019-06-18_19_26_15.csv - 90 turn with coroilis
2019-06-18_20_51_09.csv - train for altitude dataset
2019-06-20_16_21_41.csv - maybe 90 degree again
2019-06-18_21_01_48.csv - test for altitude dataset
2019-06-24_14_59_20.csv - 2x 90 deg
2019-06-24_14_29_46.csv - agile push, vbz vbh

28-04
- optiTrack information needs to be handled accurately. filter using bessel is fine for accelerometer raw. 
- using smooth, creates jump in the tail of the vector of optiTrack data. 
- suspect for drift now: moment is not catching up with the quadrotor dynamics. Suspecting missing feedforward terms. 
- Gyroscopic moments in case of stiff blades are transmitted to the body frame.
- some shady psuedos are not estimated well. Lumped parameter identification of drag using polyfit is leaving out a couple of pseudo forces.
- The kdx and kdy (using rpmAvg) are around 1.1, giving barely any drifts (maybe pm 2 metres)
- Lesser kdx and kdy clearly lead to drifts in forces. (need to understand source of drifts from position plots). Is it the moment model or the force model?
- thrust and force model currently use differently filtered optiData, since smooth gives jumps. 
- to evaluate fit of thrust model, keep checking the Normalization pdf of the epsilons

29-04
- Yingfu suggests using KF for drag model and accelerometer fuse
- Christophe suggests using gate detections for drag model ID, measurements will reduce covariance of internal model
- lateral body accelerations have an effect on z altitude. 
- Finding a better way to estimate thrust, so to estimate altitude
- velocity estimates start from zero, integrate/converge upto their way up. Need to find wider initial conditions for convergence.

30-04
- '2019-04-30_18_02_45.csv' is a good 4.5m square * 3 log file, somehow accelerations go wrong for you
- it is true that if the yellow doesn't co-incide, then all filters are worthless, so make sure yellow works - 
  while trying to check convergence for filter1, filter2..
- somehow thrust did have an impact on yellow. 
- maybe yingfu's method of compl filtering along with besseled accelerometers can work. Unbiased estimate of the acceleration being the goal.
- who removed the brackets on my rotation matrix. 

01-05
- while compensating for drag, use the previous velocity estimate and not the current estimate. The current velocity estimate is yet to be calculated.
- all those rotation matrices, worth nothing. All I can do now is compare closely. 
- kumar's method allows for z drag, maybe try that to make ax and ay better?
- why was there a 0.8 if the lumped parameter analysis of drag already accounts for blade flapping

03-05
- tarek is better

05-05
- blue (filter1) is doing bad for 2019-05-02_21_13_03.csv (maybe centrifugal excite)
- onto plotting for documentation, finding why tarek's was better
- 2019-05-02_20_54_28.csv weird one, too much difference

06-05
- need to move towards observer if accelerometer is not biased. 

07-05
- move backwards, assuming optiVel - make the compl filter better

08-05
- no success with compl filter, no success is okay, MPC shouldn't use measurements in model

09-05
- move towards porting thrust models to `checkGrav.m`. Clean up, study importance of compensating in body frame, since after rotation, the z altitude suffers severly if compensated for in world frame.
- currently the Vz difference is due to inadequate body lateral accelerations. The thrust(3) is already coming from the world frame (cheating method)
- although, point being, that now when I migrate to thrust models, I suspect the positions to drift A LOT. 

12-05
- height is a problem even after ground thrust thrust. Majority of height component comes from thrust. 
- even in non agile maneuvers, the height is very sensitive. The velocities start to drift after a point, positions suffer a lot.
- this was when using gt. When using my own thrust model, the positions drift upto 20 meters. Impossible to use. 
- also seems like the thrust model should account for battery juice
- TODO: read more thrust model articles

13-05
- read silva's paper, tomorrow seems a big day - for aggressive maneuvers need to be planned already
- what was helpful from today's discussion was that - 0.5m is fine for a drift (which is currently happening for 4 seconds)
- while trying to understand thrust better, I try to: read papers about power required instead of thrust required. 
- power required might be lesser when blade effectiveness due to translational motion is significant
- learn the power analogy from the current set of papers.

25-05
- add prediction one dimensional - seems to be working
- sensor innovation - in progress, doesn't seem the best way - maybe use ransac scheme?
- show Jelle optimal

26-05
- innovation basic bias fails, althought not that bad, should I keep a check on the trend?

14-06 
- innovation bias success, altitude is surprisingly good huh? need to take plots after the drone is hovering!!'
- need to estimate coroilis paparazzi is ready. 

15-06 
- clean up all folders

17-06
- loggers creating problem while dronerace enter, don't log then, startlogger normally
- create new 90 degree turns
- got the short plots as asked. Coroilis force and centrifugal is still blah :( 
- yaw bias huh! this is new :P
- now move towards thrust acc bias
- maybe keep drone on ground in data set for thrust bias calib
- for now use only zthrust from gt, estimate the lateral stuff using Delftse method and then find the innovation with accelerometer
- deadreckon these compensated accelerations in z direction 
- obviously a least squares identification with current dataset will provide the most convergent z, however that is not the point
- currently can see 6 seconds in future with 30 cms difference, not bad? Can I use this in kumar thrust model? Can I use accelerometer innovation in kumar thrust?

21-06
- the notes has the tasks
- altitude will suck if you use g/costheta cos phi
