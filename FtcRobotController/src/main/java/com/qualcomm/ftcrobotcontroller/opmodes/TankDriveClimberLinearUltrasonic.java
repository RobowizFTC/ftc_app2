package com.qualcomm.ftcrobotcontroller.opmodes;


//import android.hardware.Sensor;

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
 * Created by ayylmao on 11/22/15.
 */
public class TankDriveClimberLinearUltrasonic extends LinearOpMode {

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor tape;
    Servo adjust;
    Servo deposit;
    LegacyModule legacyModule;
    UltrasonicSensor ultraLeft;
    UltrasonicSensor ultraRight;
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


    public boolean seeRamp(UltrasonicSensor ultraLeft, UltrasonicSensor ultraRight){
        telemetry.addData("ultraL", "Left: " + ultraLeft.getUltrasonicLevel());
        telemetry.addData("ultraR", "Right: " + ultraRight.getUltrasonicLevel());

        double diff = Math.abs(ultraLeft.getUltrasonicLevel() - ultraRight.getUltrasonicLevel());
        if( diff <= 5)
            return true;
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        tape = hardwareMap.dcMotor.get("tape");
        adjust = hardwareMap.servo.get("adjust");
        deposit = hardwareMap.servo.get("deposit");
        //legacyModule = hardwareMap.legacyModule.get("legacy");
        //ultraLeft = hardwareMap.ultrasonicSensor.get("ultraLeft");
        //ultraRight = hardwareMap.ultrasonicSensor.get("ultraRight");
        waitForStart();

        deposit.setPosition(Servo.MAX_POSITION);

//        boolean seen = false;
//        forward(.60);
//        sleep(1000);
//
//        turnLeft(.60);
//        while(!seen) {
//            seen = seeRamp(ultraLeft, ultraRight);
//        }
//        stopDrive();

        backwards(.5);
        sleep(2500);
        stopDrive();
        sleep(250);

        forward(.5);
        sleep(700);
        stopDrive();
        sleep(250);


        turnLeft(.5);
        sleep(900);
        stopDrive();
        sleep(250);

        forward(.5);
        sleep(1000);
        stopDrive();
        sleep(250);

        tape.setPower(1);
        sleep(3500);
        tape.setPower(0);
        sleep(250);

        adjust.setPosition(Servo.MIN_POSITION);
        sleep(500);
        tape.setPower(-.5);
        forward(1);
        sleep(700);
        tape.setPower(0);
        stopDrive();
        sleep(250);

    }
}