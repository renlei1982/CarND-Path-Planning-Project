# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

##MEETS SPECIFICATIONS
   
#### The code compiles correctly.

Yes.

#### The car is able to drive at least 4.32 miles without incident

Yes, up to 30 miles and I stopped it manually.


#### The car drives according to the speed limit

The highest speed is set to 49.5


#### Max Acceleration and Jerk are not Exceeded

Yes, more details below


#### Car does not have collisions

No collision on my computer


#### The car stays in its lane, except for the time between changing lanes

Yes it is.


#### The car is able to change lanes

Yes it is.


## CODE MODEL FOR GENERATING PATH

### Trajectory generation

The trajectory generation code in my project is saved in a single file called 'trajecor_gen.cpp'. Basically in the code a 'T_GEN' class uses the previous path points and the some predicted new way points to generate the next trajectory point by the 'spline' model, and return the x points and y points. In the 'main.cpp' the T_GEN is called on **line 244**. It is used to generate the pretended path for checking the potential collision when changing lane, as from **line 332 to 384**, also used to generate the real trajectory points at the end as from **line 429 to 446**.


### Behavior planning

#### 1. Rank lanes 

The 1st step is to rank the 3 lanes. Three factors are considered in ranking the lanes based on their costs. The 1st factor is the distance of the ego car and the car right in front of it in the lane, the 2nd factor is the speed of the slowest car in the lane, the 3rd factor is if the lane is the current lane, in order to keeping the car in a single lane but not moving around when not necessary. Loop over the three lanes as **line 250 to 283** and find out the closest front car as well as the slowest car in each lane, the calculate the cost as **line 288 to 296**.   


#### 2. Determine the target lane

The 2nd step is to determine the target lane. First find out the lane with the minimum cost as **line 302 to 310**. Then check whether the target lane is on the left side or right side of the lane as **line 313 to 325**. **Please be noted in here that we only determine if the ego car need to switch to the left or to the right by 1 lane, even though the target lane might be 2 lanes away from the current lane**. The reason of the doing this is to avoid the jerk which might be caused by the double-lane change, as well as to avoid the complexity of calculating the potential collisions when changing double lanes.


#### 3. Check the feasibility of switching to the target lane

The 3rd step is to check if it is feasible to switch to the target lane from the current lane, in other word, the potential collision of changing lane is checked. This part of code could be found in **line 329 to 384**. Because only single lane change is considered in here as I mentioned in the part of 'Determine the target lane', so the design of the collision check is achieved by just check the distance of the ego car with the front and back cars of the target lane. If their distance is less than 12 m, then it is not feasible to make the lane change, and the car will keep staying in the current lane. 


#### 4. Make the lane change

If the lane change is feasible, the speed of the car is over 30 mph and the count number is larger than 50, then the make the lane change. The reason of setting up a count in here is to avoid the continuously lane change one right after another which may cause jerk. This part of code could be found in **line 390 to 393**.


#### 5. Regulate the ego car speed

The code for regulating the ego car speed could be found in **line 395 to 426**. In order to do so, the sensor fusion data need to be checked to find out the car right in front of the ego car. The speed of the ego car could go as high as 49.5, but if there is some car right in front of it, it will lower the speed to the same as the front car, and follow it 35 m behind. This is a very stable car speed control algorithm which avoids the unstable speed change back and forth when the ego car need to follow some other car.   


#### 6. Generate the next trajectory

For the last, generate the trajectory for the next x and y points as in **line 429 to 446**, and send the data to the simulator, one cycle of the data process is done.   


## REFLECTION

So for the path planning project, I think the most tricky part is about tuning the parameters of the costs for evaluating the lanes. If the parameters for the distance, or for the speed was not tuned well, the ego car might show a totally different behavior when it make the path plan. Another thing is how to avoid the collision and jerk. It seems like there is a lot of details we need to pay attention to if we want to make the code perfect, for example, how to avoid the collision when we make double lane change. It seems much complicated than a single lane change because not only the final trajectory point should be considered but also those points in the intermediate lane. However, in a simple way, I can just set that only a single lane change is allowed every time, which make the computing more fast and efficient, and the coding work much easier. Finally, how to keep the ego car drive in a stable speed is import for changing lanes steadily without collisions. I set up some code to make the ego car follow the front car if it is blocked by the traffic, and drive at most 49.5 mph if there is no car in front of it.


     