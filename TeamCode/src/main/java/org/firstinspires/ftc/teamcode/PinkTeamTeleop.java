/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop v.1.7")
public class PinkTeamTeleop extends OpMode{
    PinkTeamHardwareTeleopOnly robot       = new PinkTeamHardwareTeleopOnly(); // This is our robots electronics. Go to HardwarePushbot to add hardware.
    double left;
    double right;
    double collector;
    double buttonPos;
    double flywheel;
    double release;
    //public boolean getFlywheelSpeed;
    //enum shooting {prepare, shoot, reload}
    //int shooting = 1;
    /* Declare OpMode members. */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        left = 0;
        right = 0;
        collector = 0;
        buttonPos = 0;
        release = 0;
        flywheel = 0;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Good Luck and Have Fun Drivers");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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
        //Base Driver Code
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        //Collect
        if(gamepad1.left_trigger > 0.1){
            collector = 1;
        }
        //Eject
        else if(gamepad1.right_trigger > 0.1) {
            collector = -1;
        }
        //Restore to 0 power
        else if(gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1){
            collector = 0;
        }

        //Gunner Code

        //Push Button
         if(gamepad2.x){
            buttonPos = 1;
        }
        //Retract Button
        else if(gamepad2.b){
            buttonPos = -1;
        }
        else {
             buttonPos = 0;
         }
        //Spin up the Flywheel
        if (gamepad2.right_trigger > 0.1) {
            flywheel = 0.85;
        }
        else {
            flywheel = 0;
        }
        if (gamepad2.right_bumper){
            release = 0.1;
        }
        else {
            release = 0.8;
        }

        //Set Values Below
        robot.release.setPosition(release);
        robot.buttonPusher.setPosition(buttonPos);
        robot.Collector.setPower(collector);
        robot.flywheel.setPower(flywheel);
        robot.front_left.setPower(left);
        robot.front_right.setPower(right);
        robot.back_left.setPower(left);
        robot.back_right.setPower(right);
        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("release position",robot.release.getPosition());
        telemetry.addData("button position",robot.buttonPusher.getPosition());

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
