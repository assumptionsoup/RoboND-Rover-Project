## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

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

[image1]: ./misc/screenshot_01.png
[image2]: ./misc/screenshot_02.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Notebook Analysis
#### 1. I ran all the functions provided in the notebook on the provided images, then on my own data set.
I had a lot of difficulty at first outputting my own data-set, until I realized that I couldn't just
"select" the folder I wanted to save data to, I had to navigate inside it.  I saved a lot of rover
images to my home folder on accident :O

**Functions Changed**
I changed the color_thresh function signature to take an upper-bound:

    def (img, lower_thresh=(160, 160, 160), upper_thresh=(256, 256, 256))

This was helpful for finding obstacle data, since all of the rocky things are dark.  It also helped with the rocks I wanted to pick up.

    Obstacle Rocks: (-1, -1, -1), (120, 120, 90)
    Gold Rocks: (100, 100, -1), (220, 190, 80)

In a different attempt (not in the notebook), I found all the pixels under the obstacle data,
allowing me to use a much looser terrain filter. Merging the two gave me a really clear
image of the terrain - but it didn't improve my fidelity score in the simulation, so I took it out.


#### 1. I populated the `process_image()` function.

![alt text][image1]

I actually didn't notice this function until I read the submission guidelines,
so I used a lot of the code I had in my rover project. In my rover project
I created the class `RoverImageData` to perform a lot of the transforms
to the images.

I added a "culling" step to my image-saving process that discarded data
past 7.5 meters. I was finding the data unreliable past that point, reducing
my fidelity score.  Also, I only record data to the worldmap when
the rover is relatively level (thanks to the hints for this one).

It's worth nothing that I still had the **ffmpeg error** despite the
`imageio.plugins.ffmpeg.download()` line.  I switched the codec to
`libvpx` and set the extension to webm, and everything worked alright
after that.


### Autonomous Navigation and Mapping

#### 1. I filled in the `perception_step()` and `decision_step()` functions.

My perception step is nearly identical to `process_image()` in the notebook,
as I ended up writing them nearly simultaneously. It finds the terrain, the
obstacles, the sky, and the rocks I want to gather. It culls out any data
past 7.5 meters before saving the data to the world map.

It also records the angles to the rock data if they exist, which I use
in `decision_step()`


My decision step tries a few small changes.

- It tries to back the rover up if it appears stuck.
- It tries to keep to the right wall
- It tries to move towards rock samples and pick them up.


#### 2. Autonomous navigation
![alt text][image2]

I ran my simulator on linux at the 1280x768 Fantastic settings and my FPS ranged from 23 to 26.

**Backing up**

I wanted my rover to try backing up, because I saw it get stuck a LOT
with the default settings. I track if the rover hasn't been moving as
much as I think it should have. If it hasn't, I start backing up the
rover. I keep track of the rover's previous position and use this to
guage how far to back the rover up.

None of these approaches are particularly robust. Which in some cases
is actually a good thing. I don't actually keep track if the rover
has moved backward, just that the "distance traveled" has increased.
Which means if the rover backs up into something, the jitter of
it struggling will eventually make it stop. It's almost like
two wrongs make a right...

I think this was a good idea, having the rover occasionally back up
if it's stuck on terrain, but it could be done better, by using
the worldmap to determine if the rover CAN back up, and what the
best route from there is.

**Sample pickup**

I wanted my rover to pick up samples that it could see.  To this end,
if the rover can see a sample, it uses the same technique used for
terrain navigation to move toward that sample. This leads to really
abrupt navigational changes as the rover lurches towards its prey.

This approach has problems if an obstacle is in the way between the
rover and its prize. I've also seen the rover overshoot the target,
and merrily keep going, since it only moves towards a sample while
it can see it.  I've even seen it fail to pick up a sample because it
was too close and couldn't turn toward it. It will also drive between
samples if two are in view.

On the other hand, this rough approach has some benefits: if rover
hits an obstacle, it may obscure the sample, and the rover will
stop trying to get it. Or if the rover hits an obstacle for too
long, it will try to back up. And either try again for the sample,
or forget it exists.

A better approach would be to use the world map data of obstacles /
samples and plot a path to the sample. I ran out of time to try
this. It would have helped with just about every aspect of the
problem.


**Wall Hugger**

This was a fun trick. I really wanted my rover to hug walls. But I
didn't have time to do A* path finding, let alone figure out how to
detect / prefer the wall.

I tried a few initial approaches of finding the edges of the wall, and
using that as part of the polar coordinate selection.  None of it worked
very well.

Then I had a stupid idea :D I detect if the rover is close to a wall. If
yes, do nothing. But if it's not near a wall, and it **sees** a wall
somewhere, then it turns to the right slightly. It'll stop turning when
it gets to close because of the original condition. As a quick solution,
I'm pretty happy with this!

This approach has a few problems. If the rover is in a large enough open
space - and I don't think one exists on this map - then it'll go in
circles.  So this is only good for tight ravines. The rover might also
end up following a large rock in circles. If the rover hits something
and gets turned around, it may circum-navigate the entire map before
again returning to that spot.

A better solution would be to use path detection. As an alternative,
weighting transversed areas lower when choosing where to turn would also
help.


**Breaks**

I tried to detect if something was directly in front of the rover and stop,
but it's finicky, and has a lot of edge conditions that don't work well.
Is the rover trying to pick something up? Is that the thing in front of it?
The rover shouldn't stop just short of its goal!  Again, navigating
with real pathing would solve this.




