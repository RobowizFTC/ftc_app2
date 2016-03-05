package com.qualcomm.ftcrobotcontroller.opmodes;


//import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

//import android.content.Context;
//import android.hardware.Sensor;
//import android.hardware.SensorEvent;
//import android.hardware.SensorEventListener;
//import android.hardware.SensorManager;


/**
 * Created by ayylmao on 11/22/15.
 */
public class TankDriveAutonomous extends LinearOpMode {

    // current autonomous plan:
    // use encoders to get to rescue zone - navx to ensure stays on course
    // use navx to turn properly
    // optical distance sensor + ultrasonic sensor to detect distance
    // light sensor to follow line
    // touch sensor to detect when beacon is hit
    // dump climbers

    // declare variables

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor tape;
    Servo adjust;
    Servo deposit;
    LegacyModule legacy;
    UltrasonicSensor ultra;
    LightSensor light;
    OpticalDistanceSensor ods;
    TouchSensor touch;

    // variables for converting inches to ticks

    int ANDYMARK_TICKS_PER_REV = 1120;
    double Inches_per_Rotation = 13.125;

    // basic driving commands

    public void forward(double val) {
        frontLeftDrive.setPower(val);
        frontRightDrive.setPower(val);
        backLeftDrive.setPower(val);
        backRightDrive.setPower(val);
    }
    public void backwards(double val) {
        frontLeftDrive.setPower(-val);
        frontRightDrive.setPower(-val);
        backLeftDrive.setPower(-val);
        backRightDrive.setPower(-val);
    }
    public void turnLeft(double val) {
        frontLeftDrive.setPower(-val);
        frontRightDrive.setPower(val);
        backLeftDrive.setPower(-val);
        backRightDrive.setPower(val);
    }
    public void turnRight(double val) {
        frontLeftDrive.setPower(val);
        frontRightDrive.setPower(-val);
        backLeftDrive.setPower(val);
        backRightDrive.setPower(-val);
    }

    // encoder methods

    public void driveForwardDistance(double power, int distance)
    {
        frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);

        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        forward(power);

        while(backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy()){

        }

        stopDrive();
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void turnLeftDistance(double power, int distance)
    {
        frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        frontLeftDrive.setTargetPosition(-distance);
        frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(-distance);
        backRightDrive.setTargetPosition(distance);

        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        turnLeft(power);

        while(backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy()){

        }

        stopDrive();
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void turnRightDistance(double power, int distance)
    {
        frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(-distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(-distance);

        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        turnRight(power);

        while(backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy()){

        }

        stopDrive();
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    // convert inches to ticks to use in encoder methods

    public int findTicks(double distance){
        return (int) (ANDYMARK_TICKS_PER_REV * distance / Inches_per_Rotation);
    }

    public void stopDrive(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void fullStop(){
        stopDrive();
        tape.setPower(0);
        //adjust.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // define variables
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        tape = hardwareMap.dcMotor.get("tape");
        adjust = hardwareMap.servo.get("adjust");
        deposit = hardwareMap.servo.get("deposit");
        legacy = hardwareMap.legacyModule.get("legacy");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        light = hardwareMap.lightSensor.get("light");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        touch = hardwareMap.touchSensor.get("touch");
        //enable ultrasonic (port 5) and light sensor (port 6)
        legacy.enable9v(5, true);
        legacy.enable9v(6, true);
        // activate encoders
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();


    }
}