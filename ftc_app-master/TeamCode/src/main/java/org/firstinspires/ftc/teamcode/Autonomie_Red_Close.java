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

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomie_Red", group="Autonomie")  // @Autonomous(...) is the other common choice
public class Autonomie_Red_Close extends LinearOpMode {

    private int steps_per_rotation = 1220;
    private int error_red, error_blue;
    private double cruise_speed = 0.2;
    private double wheelc = 10*3.14;
    private double throwSpeed = 2.5;

    private ElapsedTime runtime = new ElapsedTime();
    private int current_heading = 0;
    double rotation_speed = 0.35;

    private int left_black, right_black;
    private double propGain = 0.005;
    private double white_threshhold = 40;
    private double drive_error;

    private double arm_up = 0.3;
    private double arm_down = 0.8;
    private double right_offset = 0.03;

    private DcMotor leftMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor rightMotor2 = null;

    private DcMotor motorScuipici1 = null;
    private DcMotor motorScuipici2 = null;

    private Servo bila1 = null;
    private Servo bila2 = null;

    private Servo leftarm = null;
    private Servo rightarm = null;
    private Servo stopper = null;

    private ColorSensor beacon = null;

    private ColorSensor line1 = null;
    private ColorSensor line2 = null;
    private DeviceInterfaceModule Sens = null;
    private VoltageSensor voltage = null;
    private ModernRoboticsI2cGyro gyro = null;   // Hardware Device Object

    private ModernRoboticsI2cRangeSensor range = null;

    private boolean aprox(int a, int b, int e){
        return a >= b - e && a <= b + e;
    }

    private void drive(double power1, double power2){
        leftMotor1.setPower(power1);
        leftMotor2.setPower(power1);
        rightMotor1.setPower(power2);
        rightMotor2.setPower(power2);
    }

    private boolean running(){
        return runtime.seconds()<=30 && opModeIsActive();
    }

    private void drive(double power, int distance) throws InterruptedException {
        distance = (int)((float)(distance/wheelc)*steps_per_rotation/2);
        if(power < 0) {
            power = power*-1;
        }
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
            idle();
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

    private void drive(int degrees) throws InterruptedException{
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int new_heading = current_heading + degrees;
        if(current_heading < new_heading){
            while(current_heading<new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(-rotation_speed, rotation_speed);
                idle();
                telemetry.addData("Heading", current_heading);
                telemetry.update();
            }
        }
        else{
            while(current_heading > new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(rotation_speed, -rotation_speed);
                idle();
                telemetry.addData("Heading", current_heading);
                telemetry.update();
            }
        }
        drive(0.0, 0.0);
    }

    private void HW_init() throws InterruptedException{
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
        stopper = hardwareMap.servo.get("stopper");
        stopper.setPosition(0.5);
        leftarm.setDirection(Servo.Direction.REVERSE);

        beacon = hardwareMap.colorSensor.get("beacon");

        line1 = hardwareMap.colorSensor.get("lineleft");
        line1.setI2cAddress(I2cAddr.create8bit(0x30));
        line2 = hardwareMap.colorSensor.get("lineright");
        line2.setI2cAddress(I2cAddr.create8bit(0x3a));
        Sens = hardwareMap.deviceInterfaceModule.get("Sens");
        voltage = hardwareMap.voltageSensor.get("leftdrive");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        hardwareMap.logDevices();

        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        motorScuipici1.setDirection(DcMotor.Direction.REVERSE);
        motorScuipici2.setDirection(DcMotor.Direction.FORWARD);
        motorScuipici1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorScuipici2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bila2.setPosition(0.9);
        bila1.setPosition(0.35);
        leftarm.setPosition(arm_down);
        rightarm.setPosition(arm_down + right_offset);

        telemetry.addData("Gyro", "Calibrating");
        telemetry.update();
        gyro.calibrate();
        while(gyro.isCalibrating()){
            idle();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.update();
        telemetry.addData("CSens", "Calibrating");
        telemetry.update();
        error_blue = error_red = 0;
        left_black = right_black = 0;
        for(int i=0;i<100;i++){
            error_red += beacon.red();
            error_blue += beacon.blue();
            left_black += line1.alpha();
            right_black += line2.alpha();
            idle();
        }
        error_red /= 100;
        error_blue /= 100;
        left_black /= 100;
        right_black /= 100;
    }

    private int getBeaconColor() throws InterruptedException{
        int beacon_red, beacon_blue;
        beacon_red = -10* error_red;
        beacon_blue = -10*error_blue;
        for(int i=0;i<10;i++){
            beacon_red += beacon.red();
            beacon_blue += beacon.blue();
            idle();
        }
        beacon_red /= 10;
        beacon_blue /= 10;
        if(aprox(beacon_blue, beacon_red, 30))
            return Color.BLACK;
        else if(beacon_blue>beacon_red)
            return Color.BLUE;
        else
            return Color.RED;
    }

    private void setThrow(boolean state){
        if(state){
            motorScuipici1.setPower(throwSpeed);
            motorScuipici2.setPower(throwSpeed);
        }
        else{
            motorScuipici1.setPower(0.0);
            motorScuipici2.setPower(0.0);
        }
    }

    private void pushBall() throws InterruptedException{
        bila1.setPosition(0.55);
        sleep(500);
        bila1.setPosition(0.35);
        sleep(500);
    }

    private void letBall() throws InterruptedException{
        bila2.setPosition(0.3);
        sleep(1000);
        bila2.setPosition(0.9);
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        HW_init();

        telemetry.addData("Status", voltage.getVoltage());
        telemetry.update();

        waitForStart();
        runtime.reset();
        beacon.enableLed(false);
        line1.enableLed(true);
        line2.enableLed(true);
        throwSpeed = 0.2 + (-voltage.getVoltage() + 14.7)*0.025;
        cruise_speed = 0.2 + (-voltage.getVoltage() + 14.7)*0.03;
        drive(cruise_speed, 20);
        //Arunca bilele
        setThrow(true);
        leftarm.setPosition(arm_up);
        rightarm.setPosition(arm_up + right_offset);
        sleep(4000);
        pushBall();
        letBall();
        pushBall();
        setThrow(false);
        leftarm.setPosition(arm_down);
        rightarm.setPosition(arm_down + right_offset);
        drive(cruise_speed, 20);
        drive(-25);
        drive(cruise_speed, 140);
        drive(-35);
        sleep(1000);
        telemetry.addData("Ready","Going to beacon");
        telemetry.update();
        double distance = range.cmUltrasonic();
        while(distance > 20 && running()) {
            drive_error = 0;
            if (line1.alpha() >= left_black + white_threshhold || line2.alpha() >= right_black + white_threshhold) {
                drive_error = (line2.alpha() - line1.alpha()) * propGain;
                drive(cruise_speed/2 + drive_error + distance*0.002, cruise_speed/2 - drive_error + distance*0.002);
            } else
                drive(cruise_speed/2  + distance*0.002, cruise_speed/2 + distance*0.002);
            idle();
            distance = range.cmUltrasonic();
        }
        drive(0.0, 0.0);
        telemetry.addData("Ready","Reading beacon");
        telemetry.update();
        sleep(1000);
        int c = getBeaconColor();
        if(c == Color.RED){
            rightarm.setPosition(arm_up+right_offset);
            drive(cruise_speed, 10);
            drive(cruise_speed, -40);
        }
        else if(c == Color.BLUE){
            leftarm.setPosition(arm_up);
            drive(cruise_speed, 10);
            drive(cruise_speed, -40);
        }
        else{
            drive(cruise_speed, -40);
        }

        drive(-85);
        drive(cruise_speed, 70);
        drive(10);
        drive(cruise_speed, 30);
        while (running()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Line Left", (double)line1.alpha());
            telemetry.addData("Line Right", (double)line2.alpha());
            telemetry.addData("DistanceU", range.cmUltrasonic());
            switch (getBeaconColor()){
                case Color.BLACK:
                    telemetry.addData("Beacon", "Black");
                    break;
                case Color.RED:
                    telemetry.addData("Beacon", "Red");
                    Sens.setLED(0, true);
                    Sens.setLED(0, false);
                    break;
                case Color.BLUE:
                    telemetry.addData("Beacon", "Blue");
                    Sens.setLED(0, false);
                    Sens.setLED(0, true);
                    break;
            }
            telemetry.update();

            current_heading = gyro.getIntegratedZValue();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
