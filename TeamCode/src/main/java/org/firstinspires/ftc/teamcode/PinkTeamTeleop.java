
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

// *************************************************************************************************
// TELE-OP - 2016-2017 - Velocity Vortex - Pink Alliance
// *************************************************************************************************

@TeleOp(name="Teleop v.1.6")
public class PinkTeamTeleop extends OpMode{
    PinkTeamHardware robot = new PinkTeamHardware(); // This is our robots electronics. Go to HardwarePushbot to add hardware.

    // Local Variables
    double leftJoystick = 0;
    double rightJoystick = 0;
    double collectorPower = 0;
    double pusherPower = 0;
    double flywheelPower = 0;
    double ballFeedPosition = 0;
    double CPS = 0;
    private ElapsedTime time = new ElapsedTime(); //delete in time for tournament
    int timer = 1;
    long lastPosition = 0;
    int lastMaxSpeed = 0;
    int maxSpeed = 1180;
    boolean shoot = false;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Init Robot Hardware
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Good Luck and Have Fun Drivers");
        telemetry.update();
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if((Math.abs(CPS - maxSpeed))<110)
        {
            shoot = true;
        } else
        {
            shoot = false;
        }
        // *****************************************************
        // ****************** Driver Code **********************

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftJoystick = -gamepad1.left_stick_y;      // Left Joystick
        rightJoystick = -gamepad1.right_stick_y;    // Right Joystick

        // Ball Collector
        if(gamepad1.left_trigger > 0.1){
            collectorPower = 1;     // Load Ball
        }
        else if(gamepad1.right_trigger > 0.1) {
            collectorPower = -1;    // Eject Ball
        }
        else {
            collectorPower = 0;     // Restore to 0 power
        }


        // *****************************************************
        // ****************** Gunner Code **********************
        // Flywheel
        if (gamepad2.right_trigger > 0.1) {
            flywheelPower = 0.9;
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
                    }
                    break;
            }
        }
        else {
            timer = 1;
            flywheelPower = 0;
            time.reset();
        }

        // Push Beacon Button
        if(gamepad2.x)
        {
             pusherPower = 1;       // Push Beacon Button (Extend Arm)
        }
        else if(gamepad2.b){
             pusherPower = 0.2;    // Retract Arm
        } else {
             pusherPower = 0.5;       // Stop
         }

        // Ball Feeder
        if (gamepad2.right_bumper && shoot)
        {
            ballFeedPosition = 0.2; // Feed Ball (Action)
        }
        else {
            ballFeedPosition = 0.8;   // Block ball
        }
        if (gamepad2.dpad_up){
            maxSpeed = (lastMaxSpeed + 10);
        }
        else if (gamepad2.dpad_down) {
            maxSpeed = (lastMaxSpeed - 10);
        }
        else {
            lastMaxSpeed = maxSpeed;
        }

        // Motor Power (Drive Train)
        robot.front_left.setPower(leftJoystick);
        robot.front_right.setPower(rightJoystick);
        robot.back_left.setPower(leftJoystick);
        robot.back_right.setPower(rightJoystick);

        // Ball Collector Speed/Direction
        robot.Collector.setPower(collectorPower);

        // Flywheel
        robot.flywheel.setPower(flywheelPower);

        // Feed the ball in the collector using the ball feed servo
        robot.ballFeeder.setPosition(ballFeedPosition);

        //Set Values Below
        robot.beaconPusher.setPosition(pusherPower);

        robot.flywheel.setMaxSpeed(maxSpeed);


        // **** Telemetry (Phone messages)
        // Send telemetry message to signify robot running;
        telemetry.addData("Left Power",  "%.2f", leftJoystick);
        telemetry.addData("Right Power", "%.2f", rightJoystick);
        telemetry.addData("Ball Feeder Position", robot.ballFeeder.getPosition());
        telemetry.addData("Beacon Arm Power", robot.beaconPusher.getPosition());
        telemetry.addData("Flywheel CPS", CPS); //delete in time for tournament
        telemetry.addData("Flywheel getMaxSpeed", robot.flywheel.getMaxSpeed()); //delete in time for tournament
        telemetry.addData("Can Shoot?", shoot); //delete in time for tournament

        telemetry.update(); // Refresh the phone messages
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
