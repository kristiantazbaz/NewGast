# Gastronomous

# Requirements
Requires git-lfs as images are saved using it.
`https://github.com/git-lfs/git-lfs/wiki/Installation`
If running on Raspberry Pi, requires compiling from source.

Requires Rosbridge-server
`sudo apt-get install ros-kinetic-rosbridge-server`

# Usage
Clone into catkin workspace. Build with `catkin_make`, then run the launch file `roslaunch gastronomous gastro.launch`. This will start the 5 nodes.
- Rosbridge server
- sauce1 (stepper server)
- pasta1 (stepper server)
- Web server
- Action server

## Rosbridge Server
Required for using `roslibjs`. Allows communication from web to ROS.

## Stepper Server
Used to control the stepper motors. Multiple instances are started with different parameters. This required the `pigpiod` library to be running in the background on the Raspberry Pi.

You can either run this on the Raspberry Pi or run everything on another computer and have the `pigpiod` service connect to the Raspberry Pi. Look at line 17 of the Stepper.py file to change the IP of the Raspberry Pi.

## Web Server
This runs the user facing web interface. It can be accessed on 127.0.0.1:8080. The index page allows you to send Meal requests to the server. There is also a `/test.html` page where you can move/sleep individual motors.

Sometimes the web server does not exit properly on killing the ros lauch terminal. If on startup you have errors with port binding, trying again a few times and waiting 10-15 seconds seems to work.

## Action Server
Script that responds to meal requests, queues them, and actions them.

# ROS
## Services
- /meal

  This listens for meal requests and queues them as needed.
  Calls and uses other services/topics

- /xxx/move

  Allows you to move the move a given distance. Currently `xxx` can be `pasta1` or `sauce1`

- /xxx/sleep, /xxx/sleep

  Allows to sleep or wake a motor. Seen above for `xxx`.

## Topics
- /meal/update

  Subscribe to this topic to receive updates from your meal request.
  The Meal service will respond with a unique id. Use that to filter the messages on this topic.
