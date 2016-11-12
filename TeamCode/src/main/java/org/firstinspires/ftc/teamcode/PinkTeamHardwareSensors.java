package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PinkTeamHardwareSensors
{
    //Motors
    public DcMotor          front_left         =       null;
    public DcMotor          front_right        =       null;
    public DcMotor          back_left          =       null;
    public DcMotor          back_right         =       null;
    public DcMotor          flywheel           =       null;
    public DcMotor          Collector          =       null;
    public GyroSensor       gyro               =       null;

    //Servos
    //public Servo            buttonPusher       =       null;
    //public Servo            release            =       null;

    //Sensors
    //public ColorSensor      colorSensor        =       null;
    public ModernRoboticsI2cRangeSensor ultraSound         =       null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public PinkTeamHardwareSensors(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        front_left   = hwMap.dcMotor.get("front_left");
        front_right  = hwMap.dcMotor.get("front_right");
        back_left   = hwMap.dcMotor.get("back_left");
        back_right  = hwMap.dcMotor.get("back_right");

        gyro        = hwMap.gyroSensor.get("gyro");
        //buttonPusher = hwMap.servo.get("buttonPusher");
        //colorSensor = hwMap.colorSensor.get("colorSensor");

        front_left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        back_left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        back_right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //colorSensor = hwMap.colorSensor.get("colorSensor");
        //IR_sensor1 = hwMap.irSeekerSensor.get("IR_sensor1");
        //IR_sensor2 = hwMap.irSeekerSensor.get("IR_sensor2");
        //flywheel = hwMap.dcMotor.get("flywheel");
        //Collector = hwMap.dcMotor.get("Collector");
        //buttonPusher = hwMap.servo.get("buttonPusher");
        //release = hwMap.servo.get("release");
        ultraSound = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultraSound");
        // Set all motors to zero power
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        // Set all motors to run without encode
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

