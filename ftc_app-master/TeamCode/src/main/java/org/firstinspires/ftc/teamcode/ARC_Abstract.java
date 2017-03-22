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
public class ARC_Abstract extends LinearOpMode {
    Robot robot = new Robot();

    private void pushBall() throws InterruptedException{
        robot.bila1.setPosition(0.55);
        sleep(500);
        robot.bila1.setPosition(0.35);
        sleep(500);
    }

    private void letBall() throws InterruptedException{
        robot.bila2.setPosition(0.3);
        sleep(1000);
        robot.bila2.setPosition(0.9);
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", robot.voltage.getVoltage());
        telemetry.update();

        waitForStart();
        robot.runtime.reset();
        robot.beacon.enableLed(false);
        robot.line1.enableLed(true);
        robot.line2.enableLed(true);
        robot.throwSpeed = 0.2 + (-robot.voltage.getVoltage() + 14.7)*0.025;
        robot.cruise_speed = 0.2 + (-robot.voltage.getVoltage() + 14.7)*0.03;
        robot.drive(robot.cruise_speed, 20);
        //Arunca bilele
        robot.setThrow(true);
        robot.leftarm.setPosition(robot.arm_up);
        robot.rightarm.setPosition(robot.arm_up + robot.right_offset);
        sleep(4000);
        pushBall();
        letBall();
        pushBall();
        robot.setThrow(false);
        robot.leftarm.setPosition(robot.arm_down);
        robot.rightarm.setPosition(robot.arm_down + robot.right_offset);
        robot.drive(robot.cruise_speed, 20);
        robot.drive(-25);
        robot.drive(robot.cruise_speed, 140);
        robot.drive(-35);
        sleep(1000);
        telemetry.addData("Ready","Going to beacon");
        telemetry.update();
        double distance = robot.range.cmUltrasonic();
        while(distance > 20 && robot.running()) {
            robot.drive_error = 0;
            if (robot.line1.alpha() >= robot.left_black + robot.white_threshhold || robot.line2.alpha() >= robot.right_black + robot.white_threshhold) {
                robot.drive_error = (robot.line2.alpha() - robot.line1.alpha()) * robot.propGain;
                robot.drive(robot.cruise_speed/2 + robot.drive_error + distance*0.002, robot.cruise_speed/2 - robot.drive_error + distance*0.002);
            } else
                robot.drive(robot.cruise_speed/2  + distance*0.002, robot.cruise_speed/2 + distance*0.002);
            idle();
            distance = robot.range.cmUltrasonic();
        }
        robot.drive(0.0, 0.0);
        telemetry.addData("Ready","Reading beacon");
        telemetry.update();
        sleep(1000);
        int c = robot.getBeaconColor();
        if(c == Color.RED){
            robot.rightarm.setPosition(robot.arm_up + robot.right_offset);
            robot.drive(robot.cruise_speed, 10);
            robot.drive(robot.cruise_speed, -40);
        }
        else if(c == Color.BLUE){
            robot.leftarm.setPosition(robot.arm_up);
            robot.drive(robot.cruise_speed, 10);
            robot.drive(robot.cruise_speed, -40);
        }
        else{
            robot.drive(robot.cruise_speed, -40);
        }

        robot.drive(-85);
        robot.drive(robot.cruise_speed, 70);
        robot.drive(10);
        robot.drive(robot.cruise_speed, 30);
        while (robot.running()) {
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
            telemetry.addData("Line Left", (double)robot.line1.alpha());
            telemetry.addData("Line Right", (double)robot.line2.alpha());
            telemetry.addData("DistanceU", robot.range.cmUltrasonic());
            switch (robot.getBeaconColor()){
                case Color.BLACK:
                    telemetry.addData("Beacon", "Black");
                    break;
                case Color.RED:
                    telemetry.addData("Beacon", "Red");
                    robot.Sens.setLED(0, true);
                    robot.Sens.setLED(0, false);
                    break;
                case Color.BLUE:
                    telemetry.addData("Beacon", "Blue");
                    robot.Sens.setLED(0, false);
                    robot.Sens.setLED(0, true);
                    break;
            }
            telemetry.update();

            robot.current_heading = robot.gyro.getIntegratedZValue();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
