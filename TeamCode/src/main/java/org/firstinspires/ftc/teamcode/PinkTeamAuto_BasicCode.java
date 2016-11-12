package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinkNavigate;
import org.firstinspires.ftc.teamcode.PinkTeamHardware;


/**
 * Created by Andy on 11/13/2015.
 *
 * Auto IN - Score N Park
 *
 * Starts near the ramp
 * Drives across the ramp
 * Backs up to the beacon
 * Scores the climbers
 * Parks
 */
@Autonomous (name= "AutoBasic")
public class PinkTeamAuto_BasicCode extends LinearOpMode {
    PinkTeamHardware robot   = new PinkTeamHardware();   // Use a Pushbot's hardware
    static double leftWheelPosPrevious = 0;     // Used to calculate velocity for PID control
    static double rightWheelPosPrevious = 0;    // Used to calculate velocity for PID control
    static double previousHeading = 0;          // Used to calculate velocity for PID control
    static int autoStep = 0;
    static boolean delayButtonPrevious = false;
    private ElapsedTime     time = new ElapsedTime();
    double currentTime;
    double sequenceStartTime;
    double autonomousStartTime;

//    Servo ServoBeacon;
//    GyroSensor gyro;
//    ColorSensor BeaconColor;


    double leftWheelCmd;    // Not used, base is controlled in PinkNavigate.java
    double rightWheelCmd;   // Not used, base is controlled in PinkNavigate.java
    double RotateCmd;
    double targetBeaconArmPos;

    // Other preset positions are defined in PinkTeleop.java
    static final double BEACON_SERVO_RIGHT_POS = 180;
    static final double BEACON_SERVO_LEFT_POS = 0;
    static final double BEACON_SERVO_NEUTRAL_POS = 90;

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
        int redBrightness = 1;
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
                redAlliance = false;
            } else if (gamepad1.b)     // Red button
            {
                redAlliance = true;
            }

            if (redAlliance)
            {
                telemetry.addData("Alliance Color", "RED");
            } else
            {
                telemetry.addData("Alliance Color", "BLUE");
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
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
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
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    if ((time.seconds()) >= delaySeconds)
                    {
                        robot.gyro.resetZAxisIntegrator();
                        autoStep = 2;
                    }
                    break;
                }
                case 2:     // Drive straight out to clear the ramp
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    targetAngle = 0;
                    targetDistance = 34;    //22
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        time.reset();
                        autoStep = 3;
                    }
                    break;
                }
                case 3:     // Shoot (waiting for now)
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    targetAngle = 0;
                    targetDistance = 34;    //34
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    if ((time.seconds()) >= 2)
                    {
                        autoStep = 4;
                    }
                    break;
                }
                case 4:     // Turn and drive
                {
                    if (redAlliance)
                    {
                        targetAngle = -45;
                    }
                    else
                    {
                        targetAngle = 45;
                    }
                    targetDistance = 58;        //34+24
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        targetAngle = 0;
                        autoStep = 5;
                    }
                    break;
                }
                case 5:     // Turn and drive in front of the beacon
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;

                    if (redAlliance)
                    {
                        if(robot.ultraSound.getDistance(DistanceUnit.INCH) > 1){
                            targetAngle++;
                        }
                        targetDistance = 70;  //34+24+12
                    }
                    else
                    {
                        targetDistance = 70;  //34+24+12
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        autoStep = 6;
                        time.reset();  // Mark time to let the bucket move to pos
                    }
                    break;
                }
                case 6:     // Back up near the wall to let the sensor see
                {
 //                   redBrightness = BeaconColor.red();
                    if (redAlliance)
                    {
                        if (redBrightness > 8)      // Sensor is on the right
                        {
                            targetBeaconArmPos = BEACON_SERVO_RIGHT_POS;
                        } else
                        {
                            targetBeaconArmPos = BEACON_SERVO_LEFT_POS;
                        }
                        targetAngle = 90;
                    }
                    else
                    {
                        if (redBrightness > 8)      // Sensor is on the right
                        {
                            targetBeaconArmPos = BEACON_SERVO_LEFT_POS;
                        } else
                        {
                            targetBeaconArmPos = BEACON_SERVO_RIGHT_POS;
                        }
                        targetAngle = -90;
                    }
                    targetDistance = 85;   //60+22+15-12
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    if ((time.seconds()) > 0.5)   // Seconds
                    {
                        autoStep = 7;
                    }
                    break;
                }
                case 7:     // Back up to the wall
                {
                    // targetBeaconArmPos stays the same for the dump operation
                    if (redAlliance)
                    {
                        targetAngle = 90;
                    }
                    else
                    {
                        targetAngle = -90;
                    }
                    targetDistance = 79;   //60+22+12-18
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7))
                    {
                        time.reset();  // Mark time to let the bucket move to pos
                        autoStep = 11;
                    }
                    break;
                }
                case 11:     // Dump the bucket
                {
                    // targetBeaconArmPos stays the same for the dump operation
                    if (redAlliance)
                    {
                        targetAngle = 90;
                    }
                    else
                    {
                        targetAngle = -90;
                    }
                    targetDistance = 79;   //60+22+12-18
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 1.0);
                    if ((time.seconds()) > 1.0)   // Seconds
                    {
                        autoStep = 12;
                    }
                    break;
                }
                case 12:     // Prepare the arm to stow it
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    if (redAlliance)
                    {
                        targetAngle = 90;
                    }
                    else
                    {
                        targetAngle = -90;
                    }
                    targetDistance = 79;   // 60+22+12-18
                    PinkNavigate.driveToPos(
                            targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 1.0);
                    if (true)
                    {
                        autoStep = 13;
                    }
                    break;
                }
                case 13:     // Stow the arm and prepare for teleop
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    if (redAlliance)
                    {
                        targetAngle = 90;
                    }
                    else
                    {
                        targetAngle = -90;
                    }
                    targetDistance = 79;   //60+22+12-18
                    PinkNavigate.driveToPos(
                            targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 1.0);
                    if (true)   // Zero is completely down
                    {
                        autoStep = 14;
                    }
                    break;
                }
                case 14:     // Move the collector to horizontal
                {
                    if (true)  // Paddle detected -> fingers are up, so stop
                    {
                        autoStep = 15;
                    } else
                    {
                    }
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    PinkNavigate.stopBase();
                    break;
                }
                case 15:     // Wait for teleop to start
                {
                    targetBeaconArmPos = BEACON_SERVO_NEUTRAL_POS;
                    PinkNavigate.stopBase();
                    break;
                }
            }
              //robot.buttonPusher.setPosition(targetBeaconArmPos);
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
