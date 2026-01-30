from robots.robotv2 import Robot
from pose import Pose
from purepursuit.path import Path
from purepursuit.bezier_stuff import BezierCurve, BezierLine

import time

# All necessary variables go here (e.g. poses to go to, motors, servos etc.)
robot = Robot(Pose(0, 0, 90))
state = 0

# path1 = Path(BezierCurve([Pose(0, 0, 90), Pose(0, 28, 90), Pose(24, 24, 90)]))
# path2 = Path(BezierCurve([Pose(0,0,90), Pose(0,12,90), Pose(0, 48, 90)]))

path1LeftAuto = Path(BezierCurve([Pose(0,0,90), Pose(0,50,90), Pose(0, 109, 90)]))
path2LeftAuto = Path(BezierCurve([Pose(0, 109, 0), Pose(27.5, 108, 0), Pose(55, 107, 0)]))

path3LAutoRed = Path(BezierCurve([Pose(55, 107, 0), Pose((55+20)/2, (107+85)/2, 0), Pose(20, 85, 0)]))

path3LAutoGreen = Path(BezierCurve([Pose(55, 107, 0), Pose((55+41)/2, (107+32)/2, 0), Pose(41, 32, 0)]))

#----------------------------------------------------------------------

path1RightAuto = Path(BezierCurve([Pose(0,0,90), Pose(15/2, 114/2, 90), Pose(15, 114, 90)]))
path2RightAuto = Path(BezierCurve([Pose(15, 114, 175), Pose((15-40)/2, 1), Pose(-40, 116, 175)]))

path3RAutoRed = Path(BezierCurve([Pose(-40, 116, 175), Pose((-40+30)/2, (116+40)/2, 175), Pose(30, 40, 175)]))
path3RAutoGreen = Path(BezierCurve([Pose(-40, 116, 175), Pose((-40-20)/2, (116+80)/2, 175), Pose(-27, 68, 175)]))

def autonomousPathUpdate(autoVersion):
    """
    State machine for auton goes here
    """

    global state
    if autoVersion[0] == "L":
        match (state):
            case 0:
                robot.servo_to_start()
                robot.intake(100)
                robot.follow_path(path1LeftAuto, False)
                # robot.turnToDegrees(180)
                state = 1
            case 1:
                if (robot.current_pose.distanceFrom(Pose(0, 109, 90)) < 0.5):
                    robot.servo_off()
                    robot.turnToDegrees(0)
                    state = 2
            case 2:
                if not robot.isBusy:
                    robot.follow_path(path2LeftAuto, False)
                    state = 3
            case 3:
                if (robot.current_pose.distanceFrom(Pose(55, 107, 0)) < 0.5):
                    if autoVersion == "LRed":
                        robot.follow_path(path3LAutoRed, False)
                    else:
                        robot.follow_path(path3LAutoGreen, False)
                state = 4
            case 4:
                if autoVersion == "LRed" and robot.current_pose.distanceFrom(Pose(20, 85, 0)) < 0.5:
                    robot.stack_cans()
                    state = -1
                elif autoVersion == "LGreen" and robot.current_pose.distanceFrom(Pose(41, 32, 0)) < 0.5:
                    robot.stack_cans()
                    state = -1

    else:
        match (state):
            case 0:
                robot.servo_to_start()
                robot.intake(100)
                robot.follow_path(path1RightAuto, False)
                # robot.turnToDegrees(180)
                state = 1
            case 1:
                if (robot.current_pose.distanceFrom(Pose(15, 114, 0)) < 0.5):
                    robot.servo_off()
                    robot.turnToDegrees(175)
                    state = 2
            case 2:
                if not robot.isBusy:
                    robot.follow_path(path2RightAuto, False)
                    state = 3
            case 3:
                if (robot.current_pose.distanceFrom(Pose(-40, 116, 175)) < 0.5):
                    if autoVersion == "RRed":
                        robot.follow_path(path3RAutoRed, False)
                    else:
                        robot.follow_path(path3RAutoGreen, False)
                state = 4
            case 4:
                if autoVersion == "RGreen" and robot.current_pose.distanceFrom(Pose(-27, 68, 175)) < 0.5:
                    robot.stack_cans()
                    state = -1
                elif autoVersion == "RRed" and robot.current_pose.distanceFrom(Pose(30, 40, 175)) < 0.5:
                    robot.stack_cans()
                    state = -1

def init():
    """
    All stuff that only runs ONCE right after pressing start goes here
    """

def loop():
    """
    Main loop for auton. Executes after initialize and will run for the rest of the round.
    
    state machine should go in here
    """
    robot.update()
    autonomousPathUpdate("LRed")
    robot.print_coordinates()

def main():
    init()

    while True:
        loop()
        time.sleep(0.02)

if __name__ == "__main__":
    main()