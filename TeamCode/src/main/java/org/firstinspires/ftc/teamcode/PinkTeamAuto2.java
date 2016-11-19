package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 Code Modified by Derek Perdomo
 */
@Autonomous (name= "Just Shoot It V.1.2")
public class PinkTeamAuto2 extends LinearOpMode {

    PinkTeamHardware robot   = new PinkTeamHardware();   // Use a Pushbot's hardware
    PinkNavigate nav = new PinkNavigate();

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
    static final double BUTTON_PUSH_NEUTRAL = 0;

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
        boolean redAlliance = true;
        double delaySeconds = 0;
        int targetAngle = 0;
        int targetDistance = 0;
        int currentHeading = 0;

        double flywheel = 0;
        double release  = 0.8;
        double Collector = 0;
        // Initialize
        autoStep = 0;

        robot.init(hardwareMap);

        nav.robot = robot;

        while (!nav.resetBasePosition())
        {
            telemetry.addData("INIT", "Resetting BASE Position");
            telemetry.update();
        }

        // Calibrate the gyro - beware of infinite loops here if it fails cal
        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating())
        {
            telemetry.addData("INIT", "Calibrating GYRO Angle");
            telemetry.update();
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
                redAlliance = false;
            } else if (gamepad1.b)     // Red button
            {
                redAlliance = true;
            }

            if (redAlliance)
            {
                telemetry.addData("Alliance Color", "Red");
            } else
            {
                telemetry.addData("Alliance Color", "Blue");
            }

            if (gamepad1.start)
            {
                robotAutoConfigured = true;
            }
            telemetry.addData("Auto Delay ", delaySeconds);
            telemetry.addData("INIT", "Press start button to configure");
            telemetry.update();
        }
        telemetry.addData("INIT", "PRESS PLAY TO START");
        telemetry.update();
        waitForStart();
        //Clear Autonomous start settings from screen
        telemetry.clearAll();
        autonomousStartTime = time.time();  // Seconds as a double

        while (opModeIsActive())
        {
            // Gyro reads from 0 to 360.  For PID, we must use +/- so convert
            currentHeading = -robot.gyro.getIntegratedZValue();
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
                    if (nav.resetBasePosition())
                    {
                        nav.runWithoutEncoders();
                        time.reset();  // Mark time for delay reference
                        autoStep = 10;
                    }
                    break;
                }
                case 10:     // Delay if need be
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    flywheel = 1;
                    if ((time.seconds()) >= delaySeconds)
                    {
                        autoStep = 20;
                    }
                    break;
                }
                case 20:     // Drive straight out to clear the ramp
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    Collector = 1;
                    flywheel =1;
                    targetAngle = 0;
                    targetDistance = 34;    //34
                    if (nav.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.5))
                    {
                        Collector = 1;
                        time.reset();
                        autoStep = 30;
                    }
                    break;
                }
                case 30:     // Shoot (waiting for now)
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    Collector = 1;
                    targetAngle = 0;
                    targetDistance = 34;    //34
                    nav.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.5);
                    flywheel = 1;
                    if ((time.seconds()) >= 1)
                    {
                        release = 0.1;
                    }
                    if ((time.seconds()) >= 6)
                    {
                        autoStep = 35;
                    }
                    break;
                }
                case 35: //turn to opposing beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    Collector = 0;
                    if(redAlliance){
                        targetAngle = -12;
                    }
                    else
                    {
                        targetAngle = 12;
                    }
                    targetDistance = 34;
                    if (nav.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.5)){
                        autoStep = 40;
                    }
                }

                case 40:     // Drive towards opposing beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    targetDistance = 144;
                    if (nav.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.5))
                    {
                        time.reset();
                        autoStep = 50;
                    }
                    break;
                }
                case 50:     // Wait for teleop to start
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    nav.stopBase();
                    break;
                }
            }
              robot.buttonPusher.setPosition(targetBeaconArmPos);
              robot.flywheel.setPower(flywheel);
              robot.release.setPosition(release);
              robot.Collector.setPower(Collector);
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
