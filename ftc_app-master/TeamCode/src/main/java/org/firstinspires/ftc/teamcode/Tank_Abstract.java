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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/*
    Cod echipa: RO018
 */

@TeleOp(name="JustTank", group="Custom Opmode")  // @Autonomous(...) is the other common choice
public class Tank_Abstract extends OpMode
{
    Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.runtime.reset();

    }

    @Override
    public void loop() {
        //Player 1:

        //Control miscare
        robot.leftMotor1.setPower(-gamepad1.left_stick_y * robot.power);
        robot.leftMotor2.setPower(-gamepad1.left_stick_y * robot.power);
        robot.rightMotor1.setPower(-gamepad1.right_stick_y * robot.power);
        robot.rightMotor2.setPower(-gamepad1.right_stick_y * robot.power);

        //Control viteza cu bila
        if (gamepad1.a){
            if (!robot.SWviteza) {
                robot.SWviteza = true;
                if (robot.power == 1.0)
                    robot.power = 0.5;
                else
                    robot.power = 0.7;
            }
        }
        else
            robot.SWviteza = false;

        //Brate Beacon
        if(gamepad1.left_bumper){
            robot.leftarm.setPosition(robot.arm_up);
        }
        else
        {
            robot.leftarm.setPosition(robot.arm_down);
        }
        if(gamepad1.right_bumper){
            robot.rightarm.setPosition(robot.arm_up + robot.right_offset);
        }
        else{
            robot.rightarm.setPosition(robot.arm_down + robot.right_offset);
        }
        //Player 2:

        //Control Servo; X fata, B spate
        if(gamepad2.x){
            robot.bila1.setPosition(0.55);
        }
        else{
            robot.bila1.setPosition(0.35);
        }
        if(gamepad2.b){
            robot.bila2.setPosition(0.3);
        }
        else{
            robot.bila2.setPosition(0.9);
        }

        //Maturici
        if (gamepad2.y){
            robot.maturici.setPower(-1.0);
        }
        else {
            robot.maturici.setPower(gamepad2.right_trigger);
        }

        //Control scuipici A trigger, aprox 2-3 secunde pana la viteza maxima
        if (gamepad2.a && !robot.last_a){
            robot.last_a = true;
            if(robot.motorScuipici1.getPower() == 0.00) {
                robot.motorScuipici1.setPower(0.25);
                robot.motorScuipici2.setPower(0.25);
            }
            else
            {
                robot.motorScuipici1.setPower(0.0);
                robot.motorScuipici2.setPower(0.0);
            }
        }
        else if(!gamepad2.a){
            robot.last_a = false;
        }

        //Macara DPad Up - Down
        if(gamepad2.right_bumper){
            robot.stopper.setPosition(0.0);
        }
        else{
            robot.stopper.setPosition(0.5);
        }

        if(gamepad2.left_bumper){
            robot.macara.setPower(-1.0);
        }
        else {
            robot.macara.setPower(gamepad2.left_trigger);
        }
        /*
        if (gamepad2.dpad_up){
            macara.setPower(0.6);
        }
        else if (gamepad2.dpad_down){
            macara.setPower(-0.6);
        }
        else {
            macara.setPower(0.0);
        }
        */
        //Log:
        telemetry.addData("Status", "Running: " + robot.runtime.toString());
        telemetry.addData("Motoare " , null);
        telemetry.addData("Stanga ", robot.leftMotor1.getPowerFloat());
        telemetry.addData("Dreapta ", robot.rightMotor1.getPowerFloat());
        telemetry.addData("Maturica ", robot.maturici.getPowerFloat());
        telemetry.addData("Scuiparici ", robot.motorScuipici1.getPowerFloat());
        telemetry.addData("Macara ", robot.macara.getPowerFloat());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

}
