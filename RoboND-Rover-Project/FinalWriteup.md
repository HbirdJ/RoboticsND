
## Project: Search and Sample Return
### Bradford Johnson

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/sample_id.png
[image2]: ./misc/movie_capture.png
[image3]: ./misc/mapped.png 


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

Identification of samples required adding an upper bound to the color threshold function. Testing revealed appropriate RGB bounds for the golden rock samples and can be seen in the following image:

![alt text][image1]

This image also demonstrates the method of determining obstacles by subtracting the ground and rock mappings from an unmasked binary image.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
The `process_image()` function built in the jupyter notebook offered a base understanding of how image analysis would be used in the program. An screenshot from the movie created using user generated tesd data is seen following: 

![alt text][image2]

The methods used for analysis were further refined for use in development of `perception.py`

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

In building `perception_step()`, several modifications were made from the example code.
* As noted above, the masking function required addition of a max value for the rock recognition. Example: 

`rock_rgb = (0, 50, 0)`

`rock_rgb_max = (255, 255, 35)`
* It was noted that fidelity of the transform is lost at edges of transformed image, so map building was limited to a smaller box region of the transformed image. Only data closer to the rover was used.
* The tranform function is not designed to recognize pitch and roll changes of the rover. Because of this an innacuracy analysis was added. If the combined error was above .2, the percieved data was not added to the global map.

`inaccuracy = (1 - np.cos(pitch))*1000+(1 - np.cos(roll))*1000`

* Also to increase fidelity, each perception data point was not taken as full truth. To resolve conflicts in what the rover percieved as ground and obstacles, it slowly built the map in RGB increases or decreases of 25. This helped account for errors due to the transform.
* The goal angle and average ground distance in front of rover were added to the rovermap displayed on left of screen. This assisted in user iteration.

Some of the fixes and improvements applied to `decision.py` are as follows:
* Added a 'stuck' mode that reverses the rover if stuck
* Outputs from the cartesian to polar space function were adjusted to match the true rover space.
* The rover now checks how much space it has available by averaging the polar ground distance values
* Added checks for NaN array conditions.
* To exit 'stop' mode, rover uses same conditions as entering it.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

*Note: The simulator was run at the lowest resolution and graphics available (640x480 and 'Fastest')*

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

The rover left to run in autonomous mode will detect all rocks and map the entire ground map. Depending on the run, fidelity seems to fluctuate around 72-80%. The below image is from a completed run:

![alt text][image3]

Mapping percentage would be larger if the entire perception image were used. The current method makes more sense in real life because the rover can't assume there is no ground beyond the canyon walls.

Points for improvement:
* Fully black obstacles [(0,0,0) in RGB] are not recognized as anything. The challenge here was due to the transformed image using black for the rover space outside of the camera's view.
* The Rover tends to drive straight into smaller obstacles if they're faced head on. This is because the open ground on either side of the obstacle averages out and the rover wants to maintain its heading.
* Rover has not yet been programmed to pick up rocks. This can be achieved by adding steering priority to their positions once they've been found. The rover would then stop once the 'near objective' flag is triggered and signal the arm to pick up the rock.
* Both the 'stop' and 'stuck' modes could be improved to make smarter decisions. If the rover was programmed to remember where it recently saw available ground, it could prioratize turning to those locations.

