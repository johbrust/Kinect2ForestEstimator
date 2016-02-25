# Kinect2ForestEstimator
Head pose estimator based on the kinect v2. It is using kinects own algorithms for the estimation of the head position
and orientation (i.e. the "Face" and "HDFace" libraries). Additionally, a head estimator based on a Discriminative Random
Regression Forest (DRRF) is implemented. This is based on the work of Fanelli (et al., ETH Zurich) and a paper they
released in 2011. 

Known issues:
  - the kinect algorithms use a different scale as the DRRF based head estimator (~factor 10e3)
