package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andy on 11/13/2015.
 */

public class PinkNavigate
{
    static PinkTeamHardware robot       = new PinkTeamHardware();


    static final double COUNTS_PER_INCH = 103;  // Base travel
    static final double POSITION_THRESHOLD = 30.0;   // Counts
    static final double ANGLE_THRESHOLD = 2.0;     // Degrees

    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public static boolean driveToPos(double targetPos, double targetAngle, int currentAngle,
    double linearVelocity, double angularVelocity, double maxPower)
    {
        double motorCmd;
        double targetPosCounts = targetPos * COUNTS_PER_INCH;
        double leftMotorCmd, rightMotorCmd;
        double leftWheelPos = robot.front_right.getCurrentPosition();
        double rightWheelPos = robot.front_left.getCurrentPosition();
        double angleErrorDegrees = targetAngle - currentAngle;
        double currentPosCounts = (leftWheelPos + rightWheelPos)/2.0;
        double angleOffset;
        double linearError = targetPosCounts - currentPosCounts;
        double angularError = targetAngle - currentAngle;

        // Determine the baseline motor speed command
        motorCmd = PinkPD.getMotorCmd(0.004, 0.002, linearError, linearVelocity);
        motorCmd = Range.clip(motorCmd, -0.95, 0.95);

        // Determine and add the angle offset
        angleOffset = PinkPD.getMotorCmd(0.04, 0.04, angularError, angularVelocity);
        leftMotorCmd = motorCmd + angleOffset;
        rightMotorCmd = motorCmd - angleOffset;
        leftMotorCmd = Range.clip(leftMotorCmd, -1.0, 1.0);
        rightMotorCmd = Range.clip(rightMotorCmd, -1.0, 1.0);

        // Scale the motor commands back to account for the MC windup problem
        // (if the motor cant keep up with the command, error builds up)
        leftMotorCmd *= maxPower;
        rightMotorCmd *= maxPower;

        robot.front_left.setPower(leftMotorCmd);
        robot.front_right.setPower(rightMotorCmd);
        robot.back_left.setPower(leftMotorCmd);
        robot.back_right.setPower(rightMotorCmd);
        if((Math.abs(targetPosCounts - currentPosCounts)<POSITION_THRESHOLD)&&(Math.abs(angleErrorDegrees)<ANGLE_THRESHOLD))
        {
            return true;
        } else
        {
            return false;
        }
    }

    public static void stopBase()
    {
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.front_right.setPower(0);
    }

    public static void runUsingEncoders()
    {
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void runWithoutEncoders()
    {
        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static boolean resetBasePosition()
    {
        boolean resetStatus = false;

        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if ((robot.front_left.getCurrentPosition()==0)&&(robot.front_right.getCurrentPosition()==0))
        {
            resetStatus = true;
        }
        return resetStatus;
    }

    public static double getBasePosition()
    {
        return ((robot.front_left.getCurrentPosition() + robot.front_right.getCurrentPosition())/2.0);
    }

    /**
     * Created by Andy on 11/12/2015.
     *
     * Both motors and continuous servos are covered.
     */
}
