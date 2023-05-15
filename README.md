# RBE2002 Team 16

The goal of this project is to program a set of three robots to work together to escape from a randomized grid-based maze.

The first robot searches for a ramp which leads to a platform raised above the rest of the arena. The robot can detect the ramp with an internal IMU sensor and will wait at the top of the platform for the second robot.

The second robot seeks out a button located on one of the walls of the maze. The button has an IR LED that can be detected by the robot using an IR sensor. When the button is pressed, it reveals a special visual tag to the first robot sitting on top of the platform. The first robot will see the tag with its camera, decode the message, and send a signal over Wi-Fi to the third robot telling it where the exit door is located.

Finally, the third robot will drive to the exit door and flash the secret code with an IR emitter. The secret code is received over Wi-Fi from the second robot depending on the location of the button. Once the door receives the secret code, it opens and lets the third robot escape the maze.
