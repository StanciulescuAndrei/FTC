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
public class JustTank extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private double arm_up = 0.3;
    private double arm_down = 0.8;
    private double right_offset = 0.03;

    private DcMotor leftMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor rightMotor2 = null;

    private DcMotor maturici = null;

    private DcMotor motorScuipici1 = null;
    private DcMotor motorScuipici2 = null;

    private DcMotor macara = null;

    private Servo bila1 = null;
    private Servo bila2 = null;
    private Servo leftarm = null;
    private Servo rightarm = null;
    private Servo stopper = null;

    private double power = 0.7;
    private boolean SWviteza = false;
    private boolean last_a = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor1  = hardwareMap.dcMotor.get("leftmotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftmotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightmotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightmotor2");

        maturici = hardwareMap.dcMotor.get("maturici");

        motorScuipici1  = hardwareMap.dcMotor.get("scuipat1");
        motorScuipici2 = hardwareMap.dcMotor.get("scuipat2");
        motorScuipici1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorScuipici2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        macara = hardwareMap.dcMotor.get("macara");

        bila1 = hardwareMap.servo.get("bila1");
        bila2 = hardwareMap.servo.get("bila2");
        leftarm = hardwareMap.servo.get("leftarm");
        rightarm = hardwareMap.servo.get("rightarm");
        stopper = hardwareMap.servo.get("stopper");
        leftarm.setDirection(Servo.Direction.REVERSE);

        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        maturici.setDirection(DcMotor.Direction.REVERSE);

        motorScuipici1.setDirection(DcMotor.Direction.REVERSE);
        motorScuipici2.setDirection(DcMotor.Direction.FORWARD);

        macara.setDirection(DcMotor.Direction.REVERSE);
        bila1.setPosition(0.4);
        bila2.setPosition(0.9);
        stopper.setPosition(0.5);
        rightarm.setPosition(arm_down+right_offset);
        leftarm.setPosition(arm_down);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {
        //Player 1:

        //Control miscare
        leftMotor1.setPower(-gamepad1.left_stick_y * power);
        leftMotor2.setPower(-gamepad1.left_stick_y * power);
        rightMotor1.setPower(-gamepad1.right_stick_y * power);
        rightMotor2.setPower(-gamepad1.right_stick_y * power);

        //Control viteza cu bila
        if (gamepad1.a){
            if (!SWviteza) {
                SWviteza = true;
                if (power == 1.0)
                    power = 0.5;
                else
                    power = 1.0;
            }
        }
        else
            SWviteza = false;

        //Brate Beacon
        if(gamepad1.left_bumper){
            leftarm.setPosition(arm_up);
        }
        else
        {
            leftarm.setPosition(arm_down);
        }
        if(gamepad1.right_bumper){
            rightarm.setPosition(arm_up+right_offset);
        }
        else{
            rightarm.setPosition(arm_down+right_offset);
        }
        //Player 2:

        //Control Servo; X fata, B spate
        if(gamepad2.x){
            bila1.setPosition(0.55);
        }
        else{
            bila1.setPosition(0.35);
        }
        if(gamepad2.b){
            bila2.setPosition(0.3);
        }
        else{
            bila2.setPosition(0.9);
        }

        //Maturici
        if (gamepad2.y){
            maturici.setPower(-1.0);
        }
        else {
            maturici.setPower(gamepad2.right_trigger);
        }

        //Control scuipici A trigger, aprox 2-3 secunde pana la viteza maxima
        if (gamepad2.a && !last_a){
            last_a = true;
            if(motorScuipici1.getPower() == 0.00) {
                motorScuipici1.setPower(0.25);
                motorScuipici2.setPower(0.25);
            }
            else
            {
                motorScuipici1.setPower(0.0);
                motorScuipici2.setPower(0.0);
            }
        }
        else if(!gamepad2.a){
            last_a = false;
        }

        //Macara DPad Up - Down
        if(gamepad2.right_bumper){
            stopper.setPosition(0.0);
        }
        else{
            stopper.setPosition(0.5);
        }

        if(gamepad2.left_bumper){
            macara.setPower(-1.0);
        }
        else {
            macara.setPower(gamepad2.left_trigger);
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
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Motoare " , null);
        telemetry.addData("Stanga ", leftMotor1.getPowerFloat());
        telemetry.addData("Dreapta ", rightMotor1.getPowerFloat());
        telemetry.addData("Maturica ", maturici.getPowerFloat());
        telemetry.addData("Scuiparici ", motorScuipici1.getPowerFloat());
        telemetry.addData("Macara ", macara.getPowerFloat());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

}
