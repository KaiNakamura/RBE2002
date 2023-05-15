# Final Project

Here is where the code for your final project goes.

You have options for how to manage the three different robots:

- You could create one universal code and use build flags to build programs for each robot.
- You could create three different branches, one for each robot. But because the libraries are in this same repo, you risk not having updates make it to all three robots.
- You could create three directories here, one for each robot.

You will need at a minimum,

- a `src` directory
- a `platformio.ini` file

If you create multiple projects, then you'll need subdirectories here, like `robot1`, `robotB`, etc.

Platformio may also want the `include`, `lib`, and `test` directories, but I don't think they are strictly necessary.
