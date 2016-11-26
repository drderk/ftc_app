package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 Code Modified by Derek Perdomo
 */
@Autonomous (name= "Auto and Hit Buttons")
public class Auto_HitButtons extends LinearOpMode {
    PinkTeamHardware robot   = new PinkTeamHardware();   // Use a Pushbot's hardware

    static double leftWheelPosPrevious = 0;     // Used to calculate velocity for PID control
    static double rightWheelPosPrevious = 0;    // Used to calculate velocity for PID control
    static double previousHeading = 0;          // Used to calculate velocity for PID control
    static int autoStep = 0;
    static boolean delayButtonPrevious = false;
    private ElapsedTime     time = new ElapsedTime();
    private ElapsedTime     time2 = new ElapsedTime();
    double autonomousStartTime;
    int timer = 1;
    double CPS = 0;
    long lastPosition = 0;
    int lastMaxSpeed = 0;
    int maxSpeed = 1180;
    boolean shoot = false;
    boolean blue = true;
    double turnSpeed = 0.6;
    double driveSpeed = 0.6;
    int lastPos = 0;
    String stage = "start";


    // Other preset positions are defined in PinkTeleop.java
    static final double BUTTON_PUSH_POS = 1;
    static final double BUTTON_PUSH_NEUTRAL = 0.5;
    static final double BUTTON_PUSH_RETRACT = 0.2;

    double targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;




    @Override
    public void runOpMode() throws InterruptedException {

        // Local Variables
        double leftWheelPos;
        double rightWheelPos;

        // Velocities
        double leftWheelVel;
        double rightWheelVel;
        double avgWheelVel;
        double angularVel;

        // States
        boolean robotAutoConfigured = false;
        boolean blueAlliance = true;


        double delaySeconds = 0;
        int targetAngle = 0;
        int targetDistance = 0;
        int currentHeading = 0;

        double flywheelPower = 0;
        double ballFeedPosition  = 0.8;
        double collectorPower = 0;

        // Initialize
        autoStep = 0;

        // Init Robot Hardware
        robot.init(hardwareMap);

        PinkNavigate.robot = robot; // Pass the existing hardware config to Navigate

        robot.colorSensor.enableLed(false);   // Turn off the on-board LED

        while (!PinkNavigate.resetBasePosition())
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
            Thread.sleep(50);
            idle();
        }

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
            double redColor= robot.colorSensor.red();
            double blueColor= robot.colorSensor.blue();

            if((Math.abs(CPS - maxSpeed))<120)
            {
                shoot = true;
            } else
            {
                shoot = false;
            }

            switch (autoStep) {
                case 0:     // Init
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if (PinkNavigate.resetBasePosition())
                    {
                        PinkNavigate.runWithoutEncoders();
                        time.reset();  // Mark time for delay reference
                        autoStep = 10;
                    }
                    break;
                }
                case 10:     // Delay if need be
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    if ((time.seconds()) >= delaySeconds)
                    {
                        robot.gyro.resetZAxisIntegrator();
                        autoStep = 20;
                    }
                    break;
                }
                case 20:     // Drive straight out to clear the ramp
                {
                    flywheelPower = 0.9;
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = 1;
                    targetAngle = 0;
                    targetDistance = 32;    //34
                    stage ="Drive straight out to clear the ramp";
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed))
                    {
                        ballFeedPosition = 0.2;
                        time.reset();
                        time2.reset();
                        autoStep = 30;
                    }
                    break;
                }
                case 30:     // Shoot
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = 1;
                    targetAngle = 0;
                    targetDistance = 32;    //34
                    //ballFeedPosition = 0.2;
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed);
                    flywheelPower = 0.9;
                    stage = "shoot";
                    switch (timer){
                        case 1:
                            time.reset();
                            timer = 2;
                            break;
                        case 2:
                            CPS = ((robot.flywheel.getCurrentPosition() - lastPosition) / time.seconds()); //delete in time for tournament
                            if(time.seconds() > 2){
                                time.reset();
                                lastPosition = robot.flywheel.getCurrentPosition();
                                timer = 1;
                            }
                            break;
                    }
                    if (shoot){
                        ballFeedPosition = 0.2;
                    }
                    else {
                        ballFeedPosition = 0.8;
                    }
                    if ((time2.seconds()) >= 10)
                    {
                        ballFeedPosition = 0.8;
                        time2.reset();
                        autoStep = 35;
                    }
                    /*
                    else if(time.seconds() > 8){
                        ballFeedPosition = 0.2;
                    }
                    else if(time.seconds() > 6){
                        ballFeedPosition = 0.8;
                    }
                    else if (time.seconds() > 4){
                        ballFeedPosition = 0.2;
                    }
                    else if (time.seconds() > 2){
                        ballFeedPosition = 0.8;
                    }
                    else {
                        ballFeedPosition = 0.2;
                    }
                    */
                    break;
                }

                 case 35: //turn to our beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    stage = "turn to our beacon";
                    if(blueAlliance){
                        targetAngle = 58;
                    }
                    else
                    {
                        targetAngle = -58;
                    }
                    targetDistance = 32;    // 32
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, turnSpeed)){
                        autoStep = 36;
                    }
                    break;
                }
                case 36: //Drive towards our beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    stage = "Drive towards our beacon";
                    if(blueAlliance){
                        targetAngle = 58;
                    }
                    else
                    {
                        targetAngle = -58;
                    }
                    targetDistance = 79;    // 32 + 47
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed)){
                        autoStep = 37;
                    }
                    break;
                }

                case 37: //turn to press the beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    stage = "turn to press beacon";
                    if(blueAlliance){
                        targetAngle =172;
                    }
                    else
                    {
                        targetAngle = 0;
                    }
                    targetDistance = 79;    // 32+47
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, turnSpeed)){
                        time2.reset();
                        autoStep = 38;
                    }
                    break;
                }
                case 38:     // check to see if its reading a color
                {
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.7);
                    stage = "check to see if its reading a color";
                    if ((blueColor > 2) || (redColor > 2)){
                        autoStep = 39;
                        time2.reset();
                        lastPos = (((robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition()) /2) / 54);
                    }
                    else {
                        targetDistance = 85;
                    }
                    break;
                }
                case 39:     // check color
                {
                    stage = "check color";
                    if (blueColor > 2) {
                        blue = true;
                    } else {
                        blue = false;
                    }
                    if (time2.seconds() > 2) {
                        autoStep = 40;
                    }

                    break;
                }
                case 40:     // Fix color
                {
                    stage = "fix color";
                    if (blueAlliance && blue)
                    {
                        targetDistance = lastPos;
                        targetAngle    = 172;
                    }
                    else if (blueAlliance && (blue ==false))
                    {
                        targetDistance = (lastPos + 6);
                        targetAngle    = 172;
                    }
                    else if ((blueAlliance == false) && blue )
                    {
                        targetDistance = (lastPos - 6);
                        targetAngle    = 0;
                    }
                    else
                    {
                        targetDistance = lastPos;
                        targetAngle    = 0;
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed))   // Seconds
                    {
                        time2.reset();
                        autoStep = 41;
                    }
                    break;
                }

                case 41:     // Press 1st Button
                {
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed);
                    stage = "Press 1st button";
                    if (time2.seconds() < 1) {
                        targetBeaconArmPos = BUTTON_PUSH_POS;
                    }
                    else if (time2.seconds() < 3){
                        targetBeaconArmPos = BUTTON_PUSH_RETRACT;
                    }
                    if (time2.seconds() > 3){
                        autoStep = 42;
                        targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    }
                    break;
                }
                case 42: //Drive towards our beacon
                {
                    targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    if(blueAlliance){
                        targetAngle = 172;
                        targetDistance = 28;// 32 + 44 - 48
                    }
                    else
                    {
                        targetAngle = 0;
                        targetDistance = 124; // 32 + 44 - 48
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, 0.45)){
                        autoStep = 43;
                        time2.reset();
                    }
                    break;
                }
                case 43:     // check color
                {
                    if (robot.colorSensor.blue() >= blueColor) {
                        blue = true;
                    } else {
                        blue = false;
                    }
                    if (time2.seconds() > 2) {
                        autoStep = 43;
                    }

                    break;
                }
                case 44:     // Fix color
                {
                    if (blueAlliance && blue)
                    {
                        targetDistance = 31;
                        targetAngle    = 172;
                    }
                    else if (blueAlliance && (blue ==false))
                    {
                        targetDistance = 25;
                        targetAngle    = 172;
                    }
                    else if ((blueAlliance == false) && blue )
                    {
                        targetDistance = 121;
                        targetAngle    = 0;
                    }
                    else
                    {
                        targetDistance = 127;
                        targetAngle    = 0;
                    }
                    if (PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed))   // Seconds
                    {
                        time2.reset();
                        autoStep = 45;
                    }
                    break;
                }

                case 45:     // Press 2nd Button
                {
                    collectorPower = -0.2;
                    flywheelPower = 0;
                    PinkNavigate.driveToPos(targetDistance, targetAngle, currentHeading, avgWheelVel, angularVel, driveSpeed);
                    if (time2.seconds() < 1) {
                        targetBeaconArmPos = BUTTON_PUSH_POS;
                    }
                    else if (time2.seconds() < 2){
                        targetBeaconArmPos = BUTTON_PUSH_RETRACT;
                    }
                    if (time2.seconds() > 2){
                        autoStep = 50;
                        targetBeaconArmPos = BUTTON_PUSH_NEUTRAL;
                    }
                    break;
                }

                case 50:     // Wait for teleop to start
                {
                    flywheelPower = 0;
                    collectorPower = 0;
                    ballFeedPosition = 0.8;
                    PinkNavigate.stopBase();
                    break;
                }

            }

            robot.beaconPusher.setPosition(targetBeaconArmPos);

            robot.flywheel.setPower(flywheelPower);
            robot.ballFeeder.setPosition(ballFeedPosition);
            robot.Collector.setPower(collectorPower);
            robot.flywheel.setMaxSpeed(maxSpeed);
//            telemetry.addData("Target Position", targetDistance);
//            telemetry.addData("Current Position", (front_left.getCurrentPosition() + front_left.getCurrentPosition())/206.0);
//            telemetry.addData("Current Position", (front_left.getCurrentPosition() + front_left.getCurrentPosition())/206.0);
//            telemetry.addData("Left Pos ", (front_left.getCurrentPosition()/103.0));
//            telemetry.addData("Right Pos ", (MotorRightWheels.getCurrentPosition()/103.0));
//            telemetry.addData("Max Vel ", maxVelocity);
//            telemetry.addData("Left Motor Cmd", front_left.getPower());
//            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("Flywheel CPS", CPS); //delete in time for tournament
            telemetry.addData("Flywheel getMaxSpeed", robot.flywheel.getMaxSpeed()); //delete in time for tournament
            telemetry.addData("AutoStep", autoStep); //delete in time for tournament
            telemetry.addData("Color", robot.colorSensor.blue()); //delete in time for tournament
            telemetry.addData("Stage:", stage); //delete in time for tournament
            telemetry.update();
            // telemetry.addData("autoStep", autoStep);
//            telemetry.addData("resetIterations", resetIterations);
//            telemetry.addData("maxAngularVelocity", maxAngularVelocity);
//            telemetry.addData("currentTime", currentTime);
        }
    }
}
