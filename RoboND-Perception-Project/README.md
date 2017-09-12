# Project: Perception Pick & Place


[gazebo_robot]: ./imgs/gazebo_robot.jpg
[cloud_noise]: ./imgs/cloud_noise.png
[cloud_clean]: ./imgs/cloud_clean.png
[cloud_objects]: ./imgs/cloud_objects.png
[cloud_clusters]: ./imgs/cloud_clusters.png
[cloud_labeled]: ./imgs/cloud_labeled.png
[camera_view]: ./imgs/camera_view.png
[book_vector]: ./imgs/book_vector.png
[confusion_matrix]: ./imgs/confusion_matrix.png

---
| ![camera_view] |
|:--:|
| *In this project I work with raw point clouds to label some everyday objects * |

This project focused on building an image perception pipeline for an RBGD camera. A table with assorted objects was presented in Gazebo simulation space, and after cleaning and segmenting the data I built an SVM perception model to identify each object. My perception pipeline accurately identified all items in each of the 3 provided scenes and packaged the information for a robot to pick up and place each object in a predefined bin.


| ![gazebo_robot] |
|:--:|
| *The simulated robot has no idea what its looking at yet* |

---

## Pipeline Overview
Code found at `/pr2_robot/scripts/pcl_anal.py`

| ![cloud_noise] |
|:--:|
| *In the real world, cameras tend to capture a lot of noise and unwanted points. We need to fix that.* |

| ![cloud_clean] |
|:--:|
| *After removing outliers and scaling down the number of points, it will be much easier for our robot to analyze the scene.* |

| ![cloud_objects] |
|:--:|
| *We only care about what's on the table, so two methods were used to filter out the table and surroundings. This is still perceived by the robot as one point cloud.* |

| ![cloud_clusters] |
|:--:|
| *We want to analyze each object separately, so I've used a clustering algorithm to break apart each object into separate point clouds.* |

| ![cloud_labeled] |
|:--:|
| *After reading the features of each object and applying a SVM model everything is labeled!* |

## Technical Details
### Filtering and RANSAC Plane Fitting
4 different filters are applied to go from the raw point cloud data to a point cloud of only the objects on the table.

1. A statistical outlier filter removes noise by looking for solo points floating in empty space.
2. The total number of points is decreased by using voxel grid down-sampling. This speeds up the calculations.
3. The points below the table and the table's front edge are removed using passthrough filters that eliminate anything beyond a plane defined in space. Two different passthrough filters were used in this project, one in the z-plane and one in the x-plane.
4. The tabletop is removed using a RANSAC plane filter. This recognized the table as a flat plane and removed all the points associated with it.

There are a variety of ways to isolate the relevant objects from the raw data. These steps proved highly effective for this use case.

### Clustering and Segmenting the Remaining Points
With all the objects on the table contained in one point cloud, the next step is to segment the cloud to determine which points are associated with separate objects. Euclidean clustering is used in this project to measure the distance between points. A k-d tree is used to iterate through the points. Effectively the method uses the distance between points to determine which points create a cluster together.


### Object Recognition using SVM
Now that individual point clouds for each object are available for analysis, we must decide what data is available for use in object recognition, and how much of that is useful. A support vector machine (SVM) is going to be used due to our ability to easily create a labeled training dataset.

#### Color Information and HSV
Because the camera used in this case captured RGB color data to go along with each point, we can create histograms of color values as an effective way to differentiate the objects. I switched from RGB color space to HSV to minimize the effect of shadows.

#### Surface normals
Measuring surface normals is an effective way to differentiate shapes. There is some concern with using random orientations to build the model, but the surface normal histograms did appear to be a useful tool.

#### Capturing a data set to train the model
One disadvantage of using SVMs is the requirement of training data with known labels. In this case I captured the previously described features in 25 random orientations for each object and combined them into vectors.

| ![book_vector] |
|:--:|
| *An example of the data vectors available to feed into our model. Color consists of hue, saturation, and value. The surface normal points are broken down into their x, y, and z components* |

#### The SVM model
I fed the known data into the SVM and tested with the known data. This allowed iteration of the parameters controlling the SVM to find peak accuracy. I found using a linear model produced a strong balance of accuracy and computation time. My final model ended up with an accuracy of 93.5% when tested with 5-fold validation.

| ![confusion_matrix] |
|:--:|
| *Using a 5-fold validation method, the model was tested and observed to have a high accuracy. (Note: Final model used had higher accuracy results)* |

While developing the model I tested how important the different features included in the vector were. It turned out in some cases my model could achieve higher than 90% accuracy while using only the hue and saturation histograms. This shows how powerful the SVM method can be with only 2D images. The model I ended up using included the full hsv histogram, but only the x and y components of the surface normals.

### Parsing a Pick List and Passing On What the Camera Has Learned
At this point the perception algorithm understands that recognized objects are located on a table in front of it, but the robot still has no idea what to do with that information. With a provided pick list, the code will parse it and create a .yaml file with an ordered and labeled list along with their midpoints and a placing location. This list will be used by a picking and placing rosservice.

## Extra Challenges: Complete the Pick & Place (TODO)
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!