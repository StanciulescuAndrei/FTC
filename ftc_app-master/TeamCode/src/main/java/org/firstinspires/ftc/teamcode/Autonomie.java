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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomie", group="Autonomie")  // @Autonomous(...) is the other common choice
public class Autonomie extends LinearOpMode {

    private int steps_per_rotation = 1220;
    private double cruise_speed = 0.2;
    private double wheelc = 10*3.14;
    private ElapsedTime runtime = new ElapsedTime();
    private int current_heading = 0;
    double rotation_speed = 0.6;
    double arm_up = 0.3;
    double arm_down = 0.8;
    double right_offset = 0.03;

    private DcMotor leftMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor rightMotor2 = null;
    private DcMotorController leftdrive = null;
    private DcMotorController rightdrive = null;

    private DcMotor motorScuipici1 = null;
    private DcMotor motorScuipici2 = null;

    private Servo bila1 = null;
    private Servo bila2 = null;

    private Servo leftarm = null;
    private Servo rightarm = null;

    private ModernRoboticsI2cColorSensor beacon = null;

    private ModernRoboticsI2cColorSensor line1 = null;
    private ModernRoboticsI2cColorSensor line2 = null;

    private ModernRoboticsI2cGyro gyro;   // Hardware Device Object

    private ModernRoboticsI2cRangeSensor range = null;

    private boolean aprox(int a, int b, int e){
        if(a >= b-e && a<= b+e)
            return true;
        return false;
    }

    public void drive(double power1, double power2){
        leftMotor1.setPower(power1);
        leftMotor2.setPower(power1);
        rightMotor1.setPower(power2);
        rightMotor2.setPower(power2);
    }

    public boolean running(){
        return runtime.seconds()<=30 && opModeIsActive();
    }

    public void drive(double power, int distance){
        distance = (int)((float)(distance/wheelc)*steps_per_rotation/2);
        if(power < 0) {
            power = power*-1;
        }
        leftMotor1.setMaxSpeed(4000);
        rightMotor2.setMaxSpeed(4000);
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor1.setTargetPosition(distance);
        rightMotor2.setTargetPosition(distance);
        if(distance < 0){
            rightMotor1.setPower(-power);
            leftMotor2.setPower(-power);
        }
        else{
            rightMotor1.setPower(power);
            leftMotor2.setPower(power);
        }
        rightMotor2.setPower(power);
        leftMotor1.setPower(power);
        while((leftMotor1.isBusy() || rightMotor2.isBusy()) && running()){
            telemetry.addData("Distanta", leftMotor1.getTargetPosition());
            telemetry.addData("Enc R", rightMotor2.getCurrentPosition());
            telemetry.addData("Enc L", leftMotor1.getCurrentPosition());
            telemetry.update();
        }
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(int degrees){
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int new_heading = current_heading + degrees;
        if(current_heading < new_heading){
            while(current_heading<new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(rotation_speed, -rotation_speed);
            }
        }
        else{
            while(current_heading > new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(-rotation_speed, rotation_speed);
            }
        }
        drive(0.0, 0.0);
    }

    public void HW_init(){
        leftMotor1  = hardwareMap.dcMotor.get("leftmotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftmotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightmotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightmotor2");

        motorScuipici1  = hardwareMap.dcMotor.get("scuipat1");
        motorScuipici2 = hardwareMap.dcMotor.get("scuipat2");

        bila1 = hardwareMap.servo.get("bila1");
        bila2 = hardwareMap.servo.get("bila2");
        leftarm = hardwareMap.servo.get("leftarm");
        rightarm = hardwareMap.servo.get("rightarm");
        leftarm.setDirection(Servo.Direction.REVERSE);

        beacon = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "beacon");

        line1 = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "lineleft");
        line2 = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "lineright");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        motorScuipici1.setDirection(DcMotor.Direction.REVERSE);
        motorScuipici2.setDirection(DcMotor.Direction.FORWARD);

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        HW_init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gyro.calibrate();
        while(gyro.isCalibrating());
        telemetry.addData("Gyro", "Calibrated");
        telemetry.update();
        waitForStart();
        runtime.reset();
        beacon.enableLed(false);
        line1.enableLed(false);
        line2.enableLed(false);
        leftarm.setPosition(arm_down);
        rightarm.setPosition(arm_down + right_offset);
        drive(cruise_speed, cruise_speed);
        while(range.cmUltrasonic() > 10){
            idle();
        }
        drive(0.0, 0.0);
        while (running()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Beacon Red", (double)beacon.red());
            telemetry.addData("Beacon Blue", (double)beacon.blue());
            telemetry.addData("Line Left", (double)line1.alpha());
            telemetry.addData("Line Right", (double)line2.alpha());
            telemetry.addData("DistanceU", range.cmUltrasonic());
            telemetry.update();

            current_heading = gyro.getIntegratedZValue();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
