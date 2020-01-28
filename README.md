[//]: # (Image References)

[image1]: ./image/front_and_rear_clearance.png "logic explanation" 
[image2]: ./image/highway.png "illustration" 
[image3]: ./image/boolean.png "logic explanation" 
[image4]: ./image/change_lanes_example.png "example" 

I. File list
------------
- main.cc		: main code implementation
- spline.h		: add one file by myself
- README			: this file

Program can be built using default make arguments.

II. Goals 
----------

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

III. Implementation
----------
A. Though Porcess 

0. Know the Data  

First of all, I would like to know what kind of data structure we have. There are three parts: 

(1) The map waypoints:

All the waypoints are given through a csv file, 

 [waypoints_x, waypoints_y, waypoint_s, waypoint_dx, waypoints_dy] 
- 784.6001 1135.571 0 -0.02359831 -0.9997216
- 815.2679 1134.93 30.6744785308838 -0.01099479 -0.9999396
- 844.6398 1134.911 60.0463714599609 -0.002048373 -0.9999979
- 875.0436 1134.808 90.4504146575928 -0.001847863 -0.9999983

```
// Waypoint map to read from
string map_file_ = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
double max_s = 6945.554;

std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

string line;
while (getline(in_map_, line)) {
  std::istringstream iss(line);
  double x;
  double y;
  float s;
  float d_x;
  float d_y;
  iss >> x;
  iss >> y;
  iss >> s;
  iss >> d_x;
  iss >> d_y;
  map_waypoints_x.push_back(x);
  map_waypoints_y.push_back(y);
  map_waypoints_s.push_back(s);
  map_waypoints_dx.push_back(d_x);
  map_waypoints_dy.push_back(d_y);
  }
```
There is a helpful function called getXY in helper.h for converting the map XY coordinates to Frenet coordinate. I found Frenet is a super useful system that significantly reduces complexity. 

(2) The ego car instant infmoation 

the simultaor feeds the data in every 0.02 seccond. The following is an example: 

```
["telemetry",
{
"d":6.164833,
"end_path_d":0,
"end_path_s":0,
"previous_path_x":[],
"previous_path_y":[],
"s":124.8336,
"sensor_fusion":[[0,775.99,1421.6,0,0,6721.839,-277.6729],[1,775.8,1425.2,0,0,6719.219,-280.1494],[2,775.8,1429,0,0,6716.599,-282.9019],[3,775.8,1432.9,0,0,6713.911,-285.7268],[4,775.8,1436.3,0,0,6711.566,-288.1896],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6711.778,-268.0964],[7,762.1,1425.2,0,0,6709.296,-270.7039],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6657.743,-277.6157],[11,762.1,1441.7,0,0,6653.453,-280.8947]],
"speed":0,
"x":909.48,
"y":1128.67,
"yaw":0
}]

```

(3) The senor_fusion: 
The format of the senser_fusion at each instant: 
 [ id, x, y, vx, vy, s, d]

the following is an example: 
```
[[0,1028.527,1157.093,18.73593,7.745808,246.8913,1.986083],[1,848.2643,1124.903,22.25336,-0.04875283,63.70484,9.995473],[2,1060.983,1161.701,16.32937,6.75282,278.5971,10.01214],[3,775.8,1432.9,0,0,6713.911,-285.7268],[4,775.8,1436.3,0,0,6711.566,-288.1896],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6711.778,-268.0964],[7,762.1,1425.2,0,0,6709.296,-270.7039],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6657.743,-277.6157],[11,762.1,1441.7,0,0,6653.453,-280.8947]]
```
Total, we have 12 (id from 0 to 11) cars on the same side of the highway. I calculate the speed in Frenet coordinates, by ```sqrt(vx*vx+vy*vy)```. Moreover, I classify each of them by three lanes and the distance apart from the ego car, which later will be useful for scenario decision making criteria. I carefully choose to have vectors that record how many vehicles are in the range*. We will discuss the range in the later part. 

1. Plan the path by adapting spline.h libaray 

- This part, I pretty much follow  Aaron's instruction in Project Q &A. Thanks for providing this session. It makes this project much fun. I like the part we only need to specify the lane number, and then the ego car will change lane automatically. I was amazed by this simplicity. 

2. The concept of making the lane changing in code 
- As I described earlier, I know the location of each car (check_car_s) and its estimated future location (check_car_s1) based on the previous path planning. From the check_car_s, I know the car is in front/rear of the ego car. From the check_car_s1, I know its future location within 50m front /30m rear apart.  
 
 ![alt text][image1]

 It is important is to explain my classification here, especially I introduce the vectors like "car_laneX_front" and "car_laneX_rear" where X ranges from 0 to 2. Moreover, I introduced the boolean variables: "laneX_clear_front" and "laneX_clear_rear".  At each 0.02 second, the program will classify the vehicles in "sensor_fusion data". If the conditions meet the criteria, then the push_back function will append the ```check_car_s``` into the designated vector, which is   ```car_laneX_front.push_back(check_car_s)```  , and based on the size of the vector, whether 0 or not, to decide the boolean (the front or rear clearance) true or false. 
 
 ![alt text][image3]

After that, the following is an illustration to demonstrate the front clearance and rear clearance in the situation that the ego car is in lane =1 and other vehicles are in lane =2. If the clearance is not enough, then the collision happens. 

 ![alt text][image2]
```car_s``` is defined as the farest distnace ```s``` in the previous path planning. In other words, the tip of the green guildline.   

- If the follwoing condition meets : (check_car_s - car_s) > 0 and abs( check_car_s - car_s ) < 50, then lane2_clear_fornt will be false, which is case (A).   
- If the follwoing condition meets : (car_s - check_car_s) > 0 and abs( check_car_s - car_s ) < 30, than lane2_clear_rear will be false, whcih are case (C), case(D), and case(E).

To avoid collision during lane changing, we need to provide enough clearance for both front condition and rear condition. I use two conditions to have more flexibility for later trials.  In the case of the front condition,  ```(check_car_s - car_s) > 0 ```, I add the second condition that ```abs(check_car_s1 - car_s) < 50```. The ```check_car_s1``` is the furthest ```s``` in the same previous planing fashion, but for another car, an estimated location of the car base on its speed and time span. Similiar concpet for the case of the rear condition. 

3. Avoid accelrateion, Jerk, and change lanes under certian condictions, below is the example of ego car in lane 1. 

![alt text][image4]

At the start of the simulator, the ego car is situated in lane = 1, and to accelerate without much jerk, I set the initial velocity to be zero, and it will speed up gradulally. But this part may be redundant.  
```
if ((lane == 1) && (lane1_clear_rear == false) && (lane1_clear_front == true)) {
    if (ref_vel < 49.5) {
      ref_vel += 0.224;
    }
} 
```
If there is a car in front of the ego car, the first reaction is to slow down.  
```
else if (lane == 1 && lane1_clear_front == false) { 
  slow_down = true; 
```  
Then check both front and rear clearance for changing lanes (to lane 0 or to lane 2, left priority is higher than right)
```
// if the left lane is clear, move to left lane
if (lane0_clear_front == true && lane0_clear_rear == true) {
  lane = 0;
} 
// if left lane is not the option, then check the right lane 
else if (lane2_clear_front == true && lane2_clear_rear ==true ){
  lane = 2; 
}
```
Sometimes, the ego car could be trapped in a certain lane (ex: lane 1) becuase there is no chance to change lane. The ego car will speed up and slow down repeatably, which is very uncomfortable, so I add two more conditions to slow down, let other car pass by : 

```else if (lane0_clear_front == false && lane0_clear_rear == true) {
  ref_vel -= 0.224/2; 
}  
// if right lane is not ready, then slow down 
else if (lane2_clear_front == false && lane2_clear_rear == true) {
  ref_vel -= 0.224/2; 
}
```
However, I don't' find them very useful; the best option is to follow the front car speed. However, I didn't successfully implement it in my code. I will try it next round.
In summary, my example above is when the ego car is in lane 1; similar concepts also apply to lane 0 and lane 2. 


IV. Summary
----------
My code can pass all the criteria as long as there is no ghost car to interfere with. Simulator in MAC version, the ghost is the red one moving in fuzzy condition that sensor_fusion cannot detect it. 
