package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Silviu on 3/22/2017.
 */

public class Robot {
    public int steps_per_rotation = 1220;
    public int error_red, error_blue;
    public double cruise_speed = 0.2;
    public double wheelc = 10 * 3.14;
    public double throwSpeed = 0.0;

    public ElapsedTime runtime = new ElapsedTime();
    public int current_heading = 0;
    public double rotation_speed = 0.4;

    public double power = 0.7;
    public boolean SWviteza = false;
    public boolean last_a = false;

    public int left_black, right_black;
    public double propGain = 0.005;
    public double white_threshhold = 40;
    public double drive_error;

    public double arm_up = 0.3;
    public double arm_down = 0.8;
    public double right_offset = 0.03;

    private HardwareMap hwmap = null;

    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;
    public VoltageSensor voltage = null;

    public DcMotor motorScuipici1 = null;
    public DcMotor motorScuipici2 = null;
    public DcMotor maturici = null;
    public DcMotor macara = null;

    public Servo bila1 = null;
    public Servo bila2 = null;

    public Servo leftarm = null;
    public Servo rightarm = null;
    public Servo stopper = null;

    public ColorSensor beacon = null;

    public ColorSensor line1 = null;
    public ColorSensor line2 = null;
    public DeviceInterfaceModule Sens = null;

    public ModernRoboticsI2cGyro gyro = null;

    public ModernRoboticsI2cRangeSensor range = null;

    private void HW_init() throws InterruptedException {
        leftMotor1 = hwmap.dcMotor.get("leftmotor1");
        leftMotor2 = hwmap.dcMotor.get("leftmotor2");
        rightMotor1 = hwmap.dcMotor.get("rightmotor1");
        rightMotor2 = hwmap.dcMotor.get("rightmotor2");

        motorScuipici1 = hwmap.dcMotor.get("scuipat1");
        motorScuipici2 = hwmap.dcMotor.get("scuipat2");

        bila1 = hwmap.servo.get("bila1");
        bila2 = hwmap.servo.get("bila2");
        leftarm = hwmap.servo.get("leftarm");
        rightarm = hwmap.servo.get("rightarm");
        stopper = hwmap.servo.get("stopper");
        stopper.setPosition(0.5);
        leftarm.setDirection(Servo.Direction.REVERSE);

        beacon = hwmap.colorSensor.get("beacon");

        line1 = hwmap.colorSensor.get("lineleft");
        line1.setI2cAddress(I2cAddr.create8bit(0x30));
        line2 = hwmap.colorSensor.get("lineright");
        line2.setI2cAddress(I2cAddr.create8bit(0x3a));
        Sens = hwmap.deviceInterfaceModule.get("Sens");
        voltage = hwmap.voltageSensor.get("leftdrive");

        range = hwmap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyro = (ModernRoboticsI2cGyro) hwmap.gyroSensor.get("gyro");

        hwmap.logDevices();

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

        error_blue = error_red = 0;
        left_black = right_black = 0;

    }

    public void calibrateColor() {
        gyro.calibrate();
        while (gyro.isCalibrating()) {

        }
    }

    public void calibrateGyro() {
        for (int i = 0; i < 100; i++) {
            error_red += beacon.red();
            error_blue += beacon.blue();
            left_black += line1.alpha();
            right_black += line2.alpha();
        }
        error_red /= 100;
        error_blue /= 100;
        left_black /= 100;
        right_black /= 100;
    }

    public void init(HardwareMap hm) {
        hwmap = hm;
        try {
            HW_init();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void drive(double power1, double power2) {
        leftMotor1.setPower(power1);
        leftMotor2.setPower(power1);
        rightMotor1.setPower(power2);
        rightMotor2.setPower(power2);
    }

    public boolean running() {
        return runtime.seconds() <= 30;
    }

    public void drive(double power, int distance) throws InterruptedException {
        distance = (int) ((float) (distance / wheelc) * steps_per_rotation / 1.5);
        if (power < 0) {
            power = power * -1;
        }
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor1.setTargetPosition(distance);
        rightMotor2.setTargetPosition(distance);
        if (distance < 0) {
            rightMotor1.setPower(-power);
            leftMotor2.setPower(-power);
        } else {
            rightMotor1.setPower(power);
            leftMotor2.setPower(power);
        }
        rightMotor2.setPower(power);
        leftMotor1.setPower(power);
        while ((leftMotor1.isBusy() || rightMotor2.isBusy()) && running()) {

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

    private boolean aprox(int a, int b, int e) {
        return a >= b - e && a <= b + e;
    }

    public int getBeaconColor() throws InterruptedException {
        int beacon_red, beacon_blue;
        beacon_red = -10 * error_red;
        beacon_blue = -10 * error_blue;
        for (int i = 0; i < 10; i++) {
            beacon_red += beacon.red();
            beacon_blue += beacon.blue();
        }
        beacon_red /= 10;
        beacon_blue /= 10;
        if (aprox(beacon_blue, beacon_red, 30))
            return Color.BLACK;
        else if (beacon_blue > beacon_red)
            return Color.BLUE;
        else
            return Color.RED;
    }
    public void setThrow(boolean state){
        if(state){
            motorScuipici1.setPower(throwSpeed);
            motorScuipici2.setPower(throwSpeed);
        }
        else{
            motorScuipici1.setPower(0.0);
            motorScuipici2.setPower(0.0);
        }
    }
    public void drive(int degrees) throws InterruptedException{
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int new_heading = current_heading + degrees;
        if(current_heading < new_heading){
            while(current_heading<new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(-rotation_speed, rotation_speed);
            }
        }
        else{
            while(current_heading > new_heading && running()){
                current_heading = gyro.getIntegratedZValue();
                drive(rotation_speed, -rotation_speed);
            }
        }
        drive(0.0, 0.0);
    }
}