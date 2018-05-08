# Capstone Project
Self-Driving Car Engineer Nanodegree Program

---

### Introduction

The goal of this project is to write Waypoint Updater, Twist Controller and Traffic Light Detector ROS nodes in self-driving car control project. The vehivle should follow the filtered waypoints smoothly and stop on traffic lights on the test track.

### Setup

You can see detailed information on dependencies installation and build instructions in parent repository [here](https://github.com/udacity/CarND-Capstone).

### Algorithm description

Here I will describe overview of the logic of each node. During implementation I followed the same order as in project walkthrough: basic Waypoint Updater, then Twist Controller, then Traffic Light Detector, then back to incorporate traffic light stops into Waypoint Updater. Hovewer here I list these nodes in logical order.

#### Traffic Light Detector

Traffic Light Detector node looks through the list of pre-defined traffic lights, finds the closest one in front of the car and checks its status. If the light is red then the node posts index of the waypoint closest to the traffic light stop line as the stop waypoint.

#### Waypoint Updater

The role of the Waypoint Updater is to filter the waypoints of the full vehicle path and report only the part of it to the Waypoint Follower. The goal is to find the next N waypoints (200 by default, but due to the bad performance on my test system I reduced it to 50) and then adjust them using additional stop waypoint information reported by Traffic Light Detector (if it is reported then instead of the posting original waypoint target speed we adjust it so the vehicle stops smoothly).

#### Twist Controller

The goal of this node is to convert target vehicle position and velocity into commands to the steering, throttle and breaks so that vehicle follows the target waypoints smoothly.

Internally this controller uses the following pre-defined classes: Yaw Controller for steering information, PID controller for throttle and LowPass Filter for filtering current velocity before using it in calculation.

### Final Project

As you can from the video below there were quite a lot of performance problem on my configuration. There were sudden drops in performance, controllers were not able to process waypoints in realtime and the vehicle was lost on the track as the result. Only after some optimization and reducing the number of waypoints to 50 the vehicle was able to finish the track, but you can still see some instances of performance drop (it is when there are some leftower waypoints drawn behind the car).

[![](https://img.youtube.com/vi/RLh2INBoFxE/0.jpg)](https://www.youtube.com/watch?v=RLh2INBoFxE)