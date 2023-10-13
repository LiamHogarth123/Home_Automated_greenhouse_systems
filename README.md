# Industrial Robotics Autonomous Greenhouse systems.

Group memebers
Daniel Chen
Jonah Mann
Liam Hogarth


# Project Description
Our Chosen project is an Autonomous greenhouse system that will manage and a greenhouse provide fresh produce to a community. The two robots will interact taking a plant from a greenhouse shelving unit and then delivering the plant to the second robot. The Second Robot Waters and inspect the quality of the plant in case it needs fertiliser.


Robot 1 - Linear Ur5
![183264-13204725](https://github.com/LiamHogarth123/Home_Automated_greenhouse_systems/assets/126121211/f1ca663e-6fd4-4861-b8ab-58282a861561)

Robot 2 - A0509
![Product-Image-2020-07-16T144008 659](https://github.com/LiamHogarth123/Home_Automated_greenhouse_systems/assets/126121211/8425b175-6905-46a5-b915-02b24bc415ad)


DUE DATES
1) Proposal                        21:00 Sunday 24 September
2) Initial Peer Feedback Stage     21:00 Friday 13 October
3) Short Promotion Video           21:00 Friday 27 October
4) Final System Demonstration      17:00 Friday 3 November
5) Final Video                     21:00 Sunday 5 November
6) Final Peer Feedback Optional task: A week after the demo


SYSTEM REQUIREMENTS
1) Include 2 robot arms that interact. 1 should be an existing robot arm from the Robotics Toolbox and 1
needs to be a new one that you create. Thus, all groups need to model, in simulation, a commercial robot
arm other than the ones in the Toolbox. There are many robots with different capabilities, so please
propose one that is suitable for your application and discuss it with a tutor.

2) Include a MATLAB/Python graphical user interface (GUI) to interact with the system. The GUI should have
advanced “teach” functionality that allows jogging the robot. It should include both individual joint
movements (like the Toolbox’s “teach”) plus enable [x,y,z] Cartesian movements. A valid addition is to use
a joystick or gamepad to augment and replace mouse GUI button presses.

3) Incorporate a functional estop (both simulated and real) that immediately stops operations. Disengaging
the e-stop must not immediately resume the robot system but only permit resuming (meaning two actions
are necessary to resume). For full marks, your system must be able to recover/resume after an e-stop
event. Also, using uiwait, or similar busy “while” loop functionality, to pause everything will be penalised.

4) Place the system in a simulated environment that includes safety hardware (e.g., barriers, warning
signs/lights/sirens), and if relevant this may be implemented on the real robot as well and augmented with
active sensors (BONUS) where data or signals (other than the e-stop) are passed back to the main
program.

5) Incorporate safety functionality:
    a) To react to an asynchronous stop signal by a user. The system will stop based upon an action from the
    user (e.g. simulated (or real) sensing of something/someone entering an unsafe zone).
    b) To prevent collisions. When a simulated object (that you make and control) is placed in the path of the
    robot, it will stop until there is no predicted collision or move to avoid the collision. Active collision
    avoidance will be marked higher than simply detecting a collision and stopping.

6) Ensure the team’s MATLAB/Python code is available on a code repository for tutors to access and
download. The code and your understanding will be closely scrutinised during the individual code viva. You
are expected to still adhere to a particular code standard (either the one provided, or one chosen by your
group).
