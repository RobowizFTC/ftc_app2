package com.qualcomm.ftcrobotcontroller.opmodes;


//import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

//import android.content.Context;
//import android.hardware.Sensor;
//import android.hardware.SensorEvent;
//import android.hardware.SensorEventListener;
//import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.text.SimpleDateFormat;
import java.util.Date;


import java.util.UnknownFormatConversionException;

/**
 * Start backwards and align with climber thing
 */
public class EncoderTest extends LinearOpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor tape;
    Servo adjust;
    Servo deposit;

    int ANDYMARK_TICKS_PER_REV = 1680;
    double Inches_per_Rotation = 13.125;

    @Override public void runOpMode() throws InterruptedException{
        //Initialize hardware
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        //tape = hardwareMap.dcMotor.get("tape");

        //adjust = hardwareMap.servo.get("adjust");
        //deposit = hardwareMap.servo.get("deposit");

        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //tape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //Wait for the game to start
        waitForStart();

        //deposit.setPosition(Servo.MAX_POSITION);

//        backwards(.5);
//        sleep(2500);
//        stopDrive();
//        sleep(250);
//
//        forward(.5);
//        sleep(700);
//        stopDrive();
//        sleep(250);
//
//        turnLeft(.5);
//        sleep(800);
//        stopDrive();
//        sleep(250);
//        forward(.75);
//        sleep(1500);
//        stopDrive();

        telemetry.addData("ticks", "" + findTicks(24));
        driveForwardDistance(1,findTicks(24));
        stop();

    }

    public int findTicks(double distance){
        return (int) (2 * ANDYMARK_TICKS_PER_REV * distance / Inches_per_Rotation);
    }

    public void forward(double val) {

        frontLeftDrive.setPower(val);
        frontRightDrive.setPower(val);
        backLeftDrive.setPower(val);
        backRightDrive.setPower(val);
    }

    public void driveForwardDistance(double power, int distance)
    {
        //frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //frontLeftDrive.setTargetPosition(distance);
        //frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);

        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        forward(power);

        while(backLeftDrive.isBusy() && backRightDrive.isBusy()){

        }

        stopDrive();
        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
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

    public void stopDrive(){
        forward(0);
    }

}