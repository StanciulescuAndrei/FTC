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


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@Autonomous(name="AutonomousCNITV", group="Custom Opmode")  // @Autonomous(...) is the other common choice
public class AutonomousCNITV extends OpMode
{
    /* Declare OpMode members. */
    private int steps_per_rotation = 1440;
    private int wheeld = 12;
    private ElapsedTime runtime = new ElapsedTime();
    private int current_heading = 0;

    private DcMotor leftMotor1 = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor1 = null;
    private DcMotor rightMotor2 = null;

    private DcMotor motorScuipici1 = null;
    private DcMotor motorScuipici2 = null;

    private Servo bila1 = null;
    private Servo bila2 = null;

    private Servo brat1 = null;
    private Servo brat2 = null;

    private ColorSensor beacon = null;

    private ColorSensor line1 = null;
    private ColorSensor line2 = null;

    private ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    private int xVal, yVal, zVal = 0;     // Gyro rate Values
    private int heading = 0;              // Gyro integrated heading
    private int angleZ = 0;

    private ModernRoboticsI2cRangeSensor distanceSensor = null;

    public void drive(double power1, double power2){
        leftMotor1.setPower(power1);
        leftMotor2.setPower(power1);
        rightMotor1.setPower(power2);
        rightMotor2.setPower(power2);
    }

    public void drive(double power, int distance){
        distance = (int)((float)(distance/wheeld)*steps_per_rotation);

        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor1.setTargetPosition(distance);
        leftMotor2.setTargetPosition(distance);
        rightMotor1.setTargetPosition(distance);
        rightMotor2.setTargetPosition(distance);

        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor1.setPower(power);
        leftMotor2.setPower(power);
        rightMotor1.setPower(power);
        rightMotor2.setPower(power);

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(int degrees){
        double rotation_speed = 0.3;
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int new_heading = current_heading + degrees;
        if(current_heading < new_heading){
            while(current_heading<new_heading){
                current_heading = gyro.getIntegratedZValue();
                drive(rotation_speed, -rotation_speed);
            }
        }
        else{
            while(current_heading > new_heading){
                current_heading = gyro.getIntegratedZValue();
                drive(-rotation_speed, rotation_speed);
            }
        }
        drive(0.0, 0.0);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor1  = hardwareMap.dcMotor.get("leftmotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftmotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightmotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightmotor1");

        motorScuipici1  = hardwareMap.dcMotor.get("scuipat1");
        motorScuipici2 = hardwareMap.dcMotor.get("scuipat2");

        bila1 = hardwareMap.servo.get("servo1");
        bila2 = hardwareMap.servo.get("servo2");

        beacon = hardwareMap.colorSensor.get("beacon");

        line1 = hardwareMap.colorSensor.get("lineleft");
        line2 = hardwareMap.colorSensor.get("lineright");

        brat1 = hardwareMap.servo.get("brat1");
        brat2 = hardwareMap.servo.get("brat2");

        distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);

        motorScuipici1.setDirection(DcMotor.Direction.REVERSE);
        motorScuipici2.setDirection(DcMotor.Direction.FORWARD);

        bila1.setPosition(180);
        bila2.setPosition(180);

        brat1.setPosition(0);
        brat2.setPosition(0);

        telemetry.addData("Status", "Initialized");
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
        gyro.calibrate();
        while(gyro.isCalibrating());
        bila2.setPosition(0.9);
        bila1.setPosition(0.35);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        /* Aruncare bile */
        if (runtime.seconds() < 3){
            motorScuipici1.setPower(0.25);
            motorScuipici2.setPower(0.25);
        }
        else if (runtime.seconds() < 5 && runtime.seconds() > 3) {
            bila1.setPosition(0.55);
        }
        else if (runtime.seconds() < 7 && runtime.seconds() > 5) {
            bila1.setPosition(0.35);
        }
        else if (runtime.seconds() < 9 && runtime.seconds() > 7) {
            bila1.setPosition(0.55);
        }
        else {
            motorScuipici1.setPower(0);
            motorScuipici2.setPower(0);
            bila1.setPosition(0.35);
        }


/*
        if (runtime.seconds() > 6){
            while (!(beacon1.argb() <= 11 && beacon1.argb() >= 10) && !(beacon2.argb() <= 11 && beacon2.argb() >= 10)){
                drive(1.0,1.0);
            }
            drive(0.25,distanceSensor.cmOptical());
            drive(-45);
            drive(0.75,0.75);
            while (beacon1.argb() == 0 && beacon2.argb() == 0){
                if (line1.argb() == 16 && line2.argb() != 16){
                    drive(0.5,0.75);
                }
                else if (line1.argb() != 16 && line2.argb() == 16)
                    drive(0.75,0.5);
                else
                    drive(0.75,0.75);
            }
            drive(0.0, 0.0);
            if (beacon1.argb() == 2 || beacon1.argb() == 3){
                brat1.setPosition(180);
            }
            else{
                brat2.setPosition(180);
            }
            drive(-0.75,100);
            drive(-45);
            while (line1.argb() == 0 && line2.argb() == 0){
                drive(0.5,0.5);
            }
            drive(0,0);
        }
*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
