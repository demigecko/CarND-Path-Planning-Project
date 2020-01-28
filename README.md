[//]: # (Image References)

[image1]: ./image/highway.png "logic explanation" 


I. File list
------------
- main.cc		: main code implementation
- spline.h		: add one file by myself
- README			: this file

Program can be built using default make arguments.

II. Goals 
----------

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

II. Implementation
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
Total, we have 12 (id from 0 to 11) cars on the same side of the highway. I calculate the speed in Frenet coordinates, by ```sqrt(vx*vx+vy*vy)```. Moreover, I classify each of them by three lanes and the distance apart from the ego car, which later will be useful for scenario decision making criteria. I carefully choose to have vectors that record how many vehicles are in the range*. We will discuss the range in the later prt. 

1. Plan the path by adoping spline.h libaray 

- This part, I pretty much follow  Aaron's instruction in Project Q &A. Thanks for providing this session. It makes this project much fun. I like the part we only need to specify the lane = 0, and then the ego car will change lanes automatically. I was amazed by this simplicity. 

2. The concept of making the lane changing in code 
- As I described earlier, I know the location of each car (check_car_s) and its estimated future location (check_car_s1) based on the previous path planning. From the check_car_s, I know the car is in front/rear of the ego car. From the check_car_s1, I know its future location within 50m front /30m rear apart.  
 ```
         // This for loop is to check each car's location in lanes 
          for (int i = 1; i < sensor_fusion.size(); i++) {  
            float d = sensor_fusion[i][6]; 
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            double check_car_s1 = check_car_s + ((double)prev_size * 0.02 * check_speed);

            if ((check_car_s > car_s) && abs(check_car_s1 - car_s) < 50){
              if (d < 4 && d >= 0) {
                car_lane0_front.push_back(check_car_s);
              } else if (d < 8 && d >= 4) {
                car_lane1_front.push_back(check_car_s);
              } else if (d < 12 && d >= 8) {
                car_lane2_front.push_back(check_car_s);
              } else {
                continue;
              }
            }
            else if ((check_car_s < car_s) && abs(car_s - check_car_s1) < 30){
              if (d < 4 && d >= 0) {
                car_lane0_rear.push_back(check_car_s);
              } else if (d < 8 && d >= 4) {
                car_lane1_rear.push_back(check_car_s);
              } else if (d < 12 && d >= 8) {
                car_lane2_rear.push_back(check_car_s);
              } else {
                continue;
              }
            }
            else {
              continue;
            }
          } // end of for loop 
 ```
 ![alt text][image1]
It is important is explain my classification here, especially car_laneX_front and car_laneX_rear. I define the front and rear in the present locatoin that other car is either front and rear of my ego car. In the case of the cars in front of my ego car in the presnet, I add the second condition that  ```abs(check_car_s1 - car_s) < 50```. the check_car_s1 is an estimated location of the car in front in the present, but its velocity could be lower than my ego car, and ego car can passing throught it. so the absolute value is to make sure it was showing up from front. This can help to aviod collision in a situation that a slower car in the ajsent lane and the ego car decideda to pass by and change lane 


3. Avoid accelrateion, Jerk, and change lanes under certian condictions

At the start of the simulator, the ego car is situated in lane = 1, and to accelerate without much jerk, I set the initial velocity to be zero, and it will speed up gradulally. 

```
// if the ego car is in lane = 1 
          // only when the car just start in lane 1   
          if ((lane == 1) && (lane1_clear_rear == false) && (lane1_clear_front == true)) {
              if (ref_vel < 49.5) {
                ref_vel += 0.224;
              }
          } 
          // if there is a car in front of the ego car
          else if (lane == 1 && lane1_clear_front == false) { 
            //slow down first 
            slow_down = true; 
            // check if it is safe of make a lane chane 
            // if the left lane is clear, move to left lane
            if (lane0_clear_front == true && lane0_clear_rear == true) {
              lane = 0;
            } 
            // if left lane is not the option, then check the right lane 
            else if (lane2_clear_front == true && lane2_clear_rear ==true ){
              lane = 2; 
            }
            // if left lane is not ready, then slow down 
            else if (lane0_clear_front == false && lane0_clear_rear == true) {
              ref_vel -= 0.224/2; 
            }  
            // if right lane is not ready, then slow down 
            else if (lane2_clear_front == false && lane2_clear_rear == true) {
              ref_vel -= 0.224/2; 
            }
            else {
              //ref_vel -= 0.224;
              slow_down = true; 
            }
          } 

```



B. AVL Tree

1. Insertion
An interative insert function was written to give better performance because
recursive functions has to allocate/deallocate multiple stack frames.

2. AVLTNode
Because an AVL tree is a self-balancing tree, the node structure needs to be
extended to handle height information.


C. Splay Tree

1. Iteration vs. Recursion
Iteration was chosen specifically for code reuse purposes. The general algorithm
for any splay tree operation was to perform the operation then splay at that
particular node. Using the provided functions from the binary search tree code
allowed finding a particular node easy. Then once found, it was a matter of
simply splaying it to the root (while its not the root, splay it). The specific
algorithm for inserting was taken directly from the BST code, with only small
modifications for splaying and personal preferences on handling returns (try to
have return keyword in only one place).  

2. Small functions
They are easier to test, easier to code, easier to read,and definitely easier to
understand. The general concept is to minimize the syntax baggage for public
interface functions, letting them act as drivers, and have protected functions
do the gruntwork.

3. zig, zigZig, zigZag
Named after the operations that we discussed in class, these are the driver
functions for rotation.  Their names attempt to describe the general pattern
(although each describes two different symmetrical patterns) of a
node-parent-grandparent relationship.  In each function, there are calls to the
appropriate hanging function to simulate the rotation (see 5 for more
information).  Because of the binary ordering properties of a splay tree, each
function could make a set of assumptions on the data as to how to rotate, or
hang parent nodes.

4. hangPLeft, hangPRight 
The heart of rotation, these functions perform the basic operation of "hanging"
a parent (P) off of its child in a function name specified direction.  They
handle all of the pointer manipulation that maintains proper tree structure.
Although synonymous with rotation to some degree, it was felt that "hanging" and
direction gave a more complete and accurate description of what the function was
doing compared to "rotating".  Specifically, a rotation is nothing more than
hanging a parent off of its child, and a double rotation is nothing more than a
strictly ordered coupling of these hang operations.


D. Binary Heap (MaxHeap)

1. Specialization
Specializations were made to the binary heap.  This includes the omission of an
insert() function, no generic node type for the array, and making it a max heap.
Max heap was the most logical choice for this assignment because this assignment
dealt with word frequency.


2. Implementation
The MaxHeap was implemented using parallel arrays, one each for words and
frequency.  buildHeap() was implemented using Floyd's algorithm, which runs in
O(n).


3. heapSort()
This was not implemented as a member function of the MaxHeap class.  The basic
algorithm for this function is to print the maximum value in the heap and then
delete the maximum value.  The deletion will also call percolateDown() on the
new root.  The printMax() method used in heapSort() is the main bottleneck of
this algorithm due to the calls to the system I/O.



III. Analysis
-------------
The text of both authors, Bacon and Shakespeare, show a predominance of the
words "the," "of," and "and."  In Bacon's "The Essays" and "The New Atlantis,"
these common words are occurring approximately 12.4% and 14.3% of the time,
respectively.  In Shakespeare's "Hamlet" and "All's Well That Ends Well," these
words occur approximately 6.86% and 6.56%, respectively.  Based on this
evidence, we conclude that Bacon did not write Shakespeare's works.

Take that, you conspiracy theorists.



IV. Expected Bottlenecks
------------------------
A. Binary Search Tree

1. insert()/findNode()
It was hard to separate these two functions because insert() relies on
findNode() as a part of its algorithm.  This takes the longest in the BST
because it has no balancing properties.  In the case of this sorted list, it is
essentially a doubly-linked list with O(n) running time.


2. getDataAsArray()/recursiveCopy()
Stack frame allocation/deallocation from recursiveCopy() kills the performance
of these functions.


B. AVL Tree


1. insert()/findNode()
This needs to call findNode() from the BinarySearchTree class, which takes
O(log n).


C. Splay Tree


1. insert()
Similar to the AVL Tree, this needs to call findNode() from the BinarySearchTree
class, which takes O(log n).

2. splay()
This has to be called every time.  Although this is a constant time operation
for sorted input, the total running time will be linearly proportional to the
amount of data.


3. hangPRight()
This again is called every time.  It is interesting to note that this was called
twice as many times as insert(), though only one rotation is done per insert.



V. Real Bottlenecks
-------------------
A. Data weirdness

1. findNode()/std::min()
When looking at the data for the BinarySearchTree, findNode() was found to
take approximately 85% of the running time with approximately 45,000 calls.
However, std::min() makes approximately 231,000,000,000 calls but only takes
15.5% of the running time.  We believe the data is skewed/inaccurate because of
these findings.


B. Binary Search Tree

1. insert()/findNode()
As expected, these two functions took most of the processing time.

2. getDataAsArray()/recursiveCopy()
Again, as expected, these algorithms took a long time to run, although only
one call was made to getDataAsArray().


C. AVL Tree

1. insert()
This took the longest of the AVLTree functions at approximately 6.14%.


D. Splay Tree

1. insert()
Total processing time was approximately 2.11%.  


2. splay()
Total processing time was also approximately 2.11%.


3. hangPRight()
This also took approximately 2.11%.  However, it is interesting to note that
twice as many hangPRight() calls were made compared to splay().



VI. Sorting Algorithm Analysis
------------------------------

1. HeapSort - Heapsort normally runs in N log N for best, worst and average case
   It is thus reasonably efficient for binary comparisons. It should on average run
   in parallel with quicksort.

2. Selection Sort- Selection Sort is supposed to run in N^2 time for worst and best
   case scenario, and thus should be the worst of the three algorithms, except for
   extremely aberrant input.

3. QuickSort- Quicksort should run in N log N for best and average case scenarios.
   Again, it is reasonably efficient on binary comparisons and should be running
   equally well as heapsort.


In reality, heapsort is clearly the most efficient on our particular data set. We
speculate that the reason it outperforms quicksort for all the inputs is that
quicksort is not running at average time - our input is a poor represenatation of
average input for quicksort. Specifically, the input is very dense, so that many
of the different items have the same value, and thus a lot of swapping must occur,
slowing a normally fast algorithm down. It still is not obviously enough to force
quicksort to run at *worst* case time, as it still better than selection sort.
Essentially, heapsort is less affected by our semi-aberrant data than quicksort,
and selection sort is pretty bad for large data sets to begin with.

(See plots for reference)
