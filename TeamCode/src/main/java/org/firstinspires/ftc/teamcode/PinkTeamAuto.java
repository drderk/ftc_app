package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 Code Modified by Derek Perdomo
 */
@Autonomous (name= "AutoBasic")
public class PinkTeamAuto extends LinearOpMode {
    PinkTeamHardware robot   = new PinkTeamHardware();   // Use a Pushbot's hardware
    static double leftWheelPosPrevious = 0;     // Used to calculate velocity for PID control
    static double rightWheelPosPrevious = 0;    // Used to calculate velocity for PID control
    static double previousHeading = 0;          // Used to calculate velocity for PID control
    static int autoStep = 0;
    static boolean delayButtonPrevious = false;
    private ElapsedTime     time = new ElapsedTime();
    double autonomousStartTime;

    double targetBeaconArmPos;

    // Other preset positions are defined in PinkTeleop.java
    static final double BUTTON_PUSH_POS = 1;
    static final double BUTTON_PUSH_NEUTRAL = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware components to software variables

        double leftWheelPos;
        double rightWheelPos;
        double leftWheelVel;
        double rightWheelVel;
        double avgWheelVel;
        double angularVel;
        boolean robotAutoConfigured = false;
        boolean blueAlliance = true;
        double delaySeconds = 0;
        int targetAngle = 0;
        int targetDistance = 0;
        int currentHeading = 0;
        int blueBrightness = 1;
        double whiteLine  = 0.2;
        boolean blue      = false;
        double blueColor  = 8;

        double flywheel = 0;
        double release  = 0;
        // Initialize
        autoStep = 0;

        while (!PinkNavigate.resetBasePosition())
        {
            telemetry.addData("INIT", "Resetting BASE Position");
        }

        // Calibrate the gyro - beware of infinite loops here if it fails cal
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating())
        {
            telemetry.addData("INIT", "Calibrating GYRO Angle");
            Thread.sleep(100);
        }

//        BeaconColor.enableLed(false);   // Turn off the on-board LED

        // Select both the alliance color and the starting delay
        while (!robotAutoConfigured)
        {
            if ((gamepad1.y)&&(!delayButtonPrevious)) // Top (yellow) button
            {
                delaySeconds += 1;
            } else if ((gamepad1.a)&&(!delayButtonPrevious))  // Bottom (green) button
            {
                delaySeconds -= 1;
            }
            delaySeconds = Range.clip(delaySeconds, 0, 20);
            delayButtonPrevious = ((gamepad1.y)||(gamepad1.a));

            if (gamepad1.x)     // Blue button
            {
                blueAlliance = true;
            } else if (gamepad1.b)     // Red button
            {
                blueAlliance = false;
            }

            if (blueAlliance)
            {
                telemetry.addData("Alliance Color", "Blue");
            } else
            {
                telemetry.addData("Alliance Color", "Red");
            }

            if (gamepad1.start)
            {
                robotAutoConfigured = true;
            }
            telemetry.addData("Auto Delay ", delaySeconds);
            telemetry.addData("INIT", "Press start button to configure");
        }
        telemetry.addData("INIT", "PRESS PLAY TO START");
        waitForStart();
        //Clear Autonomous start settings from screen
        telemetry.clearAll();
        autonomousStartTime = time.time();  // Seconds as a double

        while (opModeIsActive())
        {
            // Gyro reads from 0 to 360.  For PID, we must use +/- so convert
            currentHeading = robot.gyro.getHeading();
            angularVel = currentHeading - previousHeading;
            previousHeading = currentHeading;

            // Calculate velocities (delta positions) for PID control
            leftWheelPos = robot.front_left.getCurrentPosition();
            rightWheelPos = robot.front_right.getCurrentPosition();
            leftWheelVel = leftWheelPos - leftWheelPosPrevious;
            rightWheelVel = rightWheelPos - rightWheelPosPrevious;
            avgWheelVel = (leftWheelVel + rightWheelVel) / 2.0;
            leftWheelPosPrevious = leftWheelPos;
            rightWheelPosPrevious = rightWheelPos;

            switch (autoStep) {
                case 0:     // Init
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (PinkNavigate.resetBasePosition())
                    {
                        PinkNavigate.runWithoutEncoders();
                        time.reset();  // Mark time for delay reference
                        autoStep = 1;
                    }
                    break;
                }
                case 1:     // Delay if need be
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if ((time.seconds()) >= delaySeconds)
                    {
                        robot.gyro.resetZAxisIntegrator();
                        autoStep = 2;
                    }
                    break;
                }
                case 2:     // Drive straight out to clear the ramp
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    targetAngle = 0;
                    targetDistance = 34;    //34
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        time.reset();
                        autoStep = 3;
                    }
                    break;
                }
                case 3:     // Shoot (waiting for now)
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    targetAngle = 0;
                    targetDistance = 34;    //34
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    flywheel = 1;
                    if ((time.seconds()) >= 1)
                    {
                        release = 1;
                    }
                    if ((time.seconds()) >= 2)
                    {
                        autoStep = 4;
                    }
                    break;
                }
                case 4:     // Turn towards the wall
                {
                    if (blueAlliance)
                    {
                        targetAngle = 90;
                    }
                    else
                    {
                        targetAngle = -90;
                    }
                    targetDistance = 34;        //34
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        autoStep = 5;
                    }
                    break;
                }
                case 5:     // drive towards the wall
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    targetDistance = 85;
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                        if(robot.ultraSound.getDistance(DistanceUnit.INCH) < 5){
                            autoStep = 6;
                            time.reset();  // Mark time to let the bucket move to pos
                        }
                    break;
                    }
                case 6:     // Turn towards the beacon
                {
                    if (blueAlliance)
                    {
                        targetAngle = 0;
                    }
                    else
                    {
                        targetAngle = 180;
                    }
                    targetDistance = 80;
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        autoStep = 7;
                    }
                    break;
                }
                case 7:     // Drive towards the beacon until white line is seen
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (blueAlliance)
                    {
                        targetAngle = 0;
                    }
                    else
                    {
                        targetAngle = 180;
                    }
                    if (blueAlliance)
                    {
                        targetDistance = 115;
                    }
                    else
                    {
                        targetDistance = 45;
                    }
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    if (robot.bwSensor.getLightDetected() > whiteLine)
                    {
                        time.reset();
                        autoStep = 8;
                    }
                    break;
                }
                case 8:     // check color
                {
                    if (robot.colorSensor.blue() > blueColor) {
                        blue = true;
                    } else {
                        blue = false;
                    }
                    if (time.seconds() > 0.1) {
                        autoStep = 9;
                    }
                    break;
                }
                case 9:     // Fix color
                {
 //                   blueBrightness = BeaconColor.red();
                    if (blueAlliance && blue)
                    {
                            targetDistance = 112;
                            targetAngle    = 0;
                        }
                    else if (blueAlliance && (blue ==false))
                        {
                            targetDistance = 118;
                            targetAngle    = 0;
                        }
                    else if ((blueAlliance == false) && blue )
                    {
                            targetDistance = 42;
                            targetAngle    = 180;
                        }
                    else
                        {
                            targetDistance = 48;
                            targetAngle    = 180;
                        }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))   // Seconds
                    {
                        autoStep = 10;
                    }
                    break;
                }
                case 10:                //Press button
                {
                    targetBeaconArmPos = BUTTON_PUSH_POS;
                    if(time.seconds() > 1){
                        targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                        autoStep = 11;
                    }
                }
                case 11:     // Drive towards the beacon until white line is seen
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (blueAlliance)
                    {
                        targetAngle = 0;
                    }
                    else
                    {
                        targetAngle = 180;
                    }
                    if (blueAlliance)
                    {
                        targetDistance = 139; //115 + 24
                    }
                    else
                    {
                        targetDistance = 21;
                    }
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    if (robot.bwSensor.getLightDetected() > whiteLine)
                    {
                        time.reset();
                        autoStep = 12;
                    }
                    break;
                }
                case 12:     // check color
                {
                    if (robot.colorSensor.blue() > blueColor) {
                        blue = true;
                    } else {
                        blue = false;
                    }
                    if (time.seconds() > 0.1) {
                        autoStep = 13;
                    }
                    break;
                }
                case 13:     // Fix color
                {
                    //                   blueBrightness = BeaconColor.red();
                    if (blueAlliance && blue)
                    {
                        targetDistance = 112;
                        targetAngle    = 0;
                    }
                    else if (blueAlliance && (blue ==false))
                    {
                        targetDistance = 118;
                        targetAngle    = 0;
                    }
                    else if ((blueAlliance == false) && blue )
                    {
                        targetDistance = 42;
                        targetAngle    = 180;
                    }
                    else
                    {
                        targetDistance = 48;
                        targetAngle    = 180;
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))   // Seconds
                    {
                        time.reset();
                        autoStep = 14;
                    }
                    break;
                }
                case 14:
                {
                    targetBeaconArmPos = 1;
                    if(time.seconds() > 1){
                        targetBeaconArmPos = 0;
                        autoStep = 15;
                    }
                }
                case 15:     // turn towards opposing beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (blueAlliance)
                    {
                        targetAngle = -45;
                    }
                    else
                    {
                        targetAngle = 225;
                    }
                    if (blueAlliance && blue)
                    {
                        targetDistance = 112;
                    }
                    else if (blueAlliance && (blue ==false))
                    {
                        targetDistance = 118;
                    }
                    else if ((blueAlliance == false) && blue )
                    {
                        targetDistance = 42;
                    }
                    else
                    {
                        targetDistance = 48;
                    }
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    if (robot.bwSensor.getLightDetected() > whiteLine)
                    {
                        time.reset();
                        autoStep = 16;
                    }
                    break;
                }
                case 16:     // Drive towards opposing beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (blueAlliance)
                    {
                        targetAngle = -45;
                    }
                    else
                    {
                        targetAngle = 225;
                    }
                    if (blueAlliance)
                    {
                        targetDistance = 152; //115 + 24
                    }
                    else
                    {
                        targetDistance = 8;
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        time.reset();
                        autoStep = 17;
                    }
                    break;
                }
                case 18:     // Wait for teleop to start
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    PinkNavigate.stopBase();
                    break;
                }
            }
              robot.buttonPusher.setPosition(targetBeaconArmPos);
              robot.flywheel.setPower(flywheel);
              robot.release.setPosition(release);
//            telemetry.addData("Target Position", targetDistance);
//            telemetry.addData("Current Position", (front_left.getCurrentPosition() + front_left.getCurrentPosition())/206.0);
//            telemetry.addData("Current Position", (front_left.getCurrentPosition() + front_left.getCurrentPosition())/206.0);
//            telemetry.addData("Left Pos ", (front_left.getCurrentPosition()/103.0));
//            telemetry.addData("Right Pos ", (MotorRightWheels.getCurrentPosition()/103.0));
//            telemetry.addData("Max Vel ", maxVelocity);
//            telemetry.addData("Left Motor Cmd", front_left.getPower());
//            telemetry.addData("targetAngle", targetAngle);
//            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("autoStep", autoStep);
//            telemetry.addData("resetIterations", resetIterations);
//            telemetry.addData("maxAngularVelocity", maxAngularVelocity);
//            telemetry.addData("currentTime", currentTime);
        }
    }
}
