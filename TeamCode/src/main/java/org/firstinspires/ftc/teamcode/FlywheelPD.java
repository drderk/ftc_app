package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Pureawesomeness on 11/8/2016.
*/
public class FlywheelPD {
    static PinkTeamHardware robot       = new PinkTeamHardware();
    private FlywheelPD() {
    }

    // Use a PD to determine a motor command, which has a range of -1 to 1 (-1=rev; 0=stop; 1=fwd)
    public static double getMotorCmd(double RPM) {
        double Kp = 0.4;
        double Kd = 0.004;
        double error = RPM - (robot.flywheel.getCurrentPosition());
        double currentVel = 0;
        double motorCmd;

        motorCmd = (Kp * error) - (Kd * currentVel) + RPM;
        //        motorCmd = Range.clip(motorCmd, -1.0, 1.0);

        return motorCmd;
    }

    // Use a PD to determine a continuous servo command, which has a range of 0 to 1 (0=rev; .5=stop; 1=fwd)
    public static double getServoCmd(double Kp, double Kd, double error, double currentVel) {

        double servoCmd;

        servoCmd = (Kp * error) - (Kd * currentVel);
        servoCmd = Range.clip(servoCmd, -1.0, 1.0);

        servoCmd = Range.scale(servoCmd, -1, 1, 0, 1);
        servoCmd = Range.clip(servoCmd, 0, 1.0);

        return servoCmd;
    }

}