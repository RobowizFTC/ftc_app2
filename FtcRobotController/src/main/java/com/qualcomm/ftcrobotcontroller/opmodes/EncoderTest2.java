package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class EncoderTest2 extends LinearOpMode
{
    DcMotor RightF;
    DcMotor RightR;
    DcMotor LeftF;
    DcMotor LeftR;
    DcMotor Sweeper;

    double pulses = 1680;
    double circumference = 13.5;

    public int getTicks(double inches){
        return (int) (2 * inches / circumference * pulses);

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        RightR = hardwareMap.dcMotor.get("backRightDrive");
        RightR.setDirection(DcMotor.Direction.REVERSE);
        RightF = hardwareMap.dcMotor.get("frontRightDrive");
        RightF.setDirection(DcMotor.Direction.REVERSE);
        LeftR = hardwareMap.dcMotor.get("backLeftDrive");
        LeftF = hardwareMap.dcMotor.get("frontLeftDrive");

        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());

        LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitForStart();


        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());

        //waitForStart();

        int ticks = getTicks(40);                                                   //Out 2.5ish tiles

        while(LeftR.getCurrentPosition() < ticks)
        {
            LeftF.setPower(0.95);
            LeftR.setPower(0.95);
            RightF.setPower(0.95);
            RightR.setPower(0.95);
            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
        }

        LeftF.setPower(0);
        LeftR.setPower(0);
        RightF.setPower(0);
        RightR.setPower(0);
        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());


        while(LeftF.getCurrentPosition() > 5)
        {
            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
        }

//
        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//
        ticks = getTicks(25.132);                                                   //Turn about 70 degrees
        while(RightR.getCurrentPosition() < ticks)
        {

            LeftF.setPower(0);
            LeftR.setPower(0);
            RightF.setPower(0.95);
            RightR.setPower(0.95);
            telemetry.addData("RightF Position", RightF.getCurrentPosition());
        }

        LeftF.setPower(0);
        LeftR.setPower(0);
        RightF.setPower(0);
        RightR.setPower(0);
        while(RightR.getCurrentPosition() > 5)
        {
            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
        }

        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());


        ticks = getTicks(50);                                                   //Get in the vicinity of the zone
        while(RightF.getCurrentPosition() < ticks)
        {
            LeftF.setPower(0.95);
            LeftR.setPower(0.95);
            RightF.setPower(0.95);
            RightR.setPower(0.95);
        }

        LeftF.setPower(0);
        LeftR.setPower(0);
        RightF.setPower(0);
        RightR.setPower(0);

        while(RightF.getCurrentPosition() > 5)
        {
            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
        }



//        while(LeftF.getCurrentPosition() < 1000);
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0);
//            RightR.setPower(0);
//        }
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightF.setPower(0);
//        RightR.setPower(0);
//        Sweeper.setPower(0);
//        while(LeftF.getCurrentPosition() > 5)
//        {
//            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        }
//
//        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//
//        while(RightF.getCurrentPosition() < 1000)
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0.5);
//            RightR.setPower(0.5);
//        }
//        while(LeftF.getCurrentPosition() < 1000);
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0);
//            RightR.setPower(0);
//        }
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightF.setPower(0);
//        RightR.setPower(0);
//        while(LeftF.getCurrentPosition() > 5)
//        {
//            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        }
//
//        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//
//        while(RightF.getCurrentPosition() < 1000)
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0.5);
//            RightR.setPower(0.5);
//        }
//        while(LeftF.getCurrentPosition() < 1000);
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0);
//            RightR.setPower(0);
//        }
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightF.setPower(0);
//        RightR.setPower(0);
//        while(LeftF.getCurrentPosition() > 5)
//        {
//            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        }
//
//        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        while(RightF.getCurrentPosition() < 1000)
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0.5);
//            RightR.setPower(0.5);
//        }
//        while(LeftF.getCurrentPosition() < 1000);
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0);
//            RightR.setPower(0);
//        }
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightF.setPower(0);
//        RightR.setPower(0);
//        while(LeftF.getCurrentPosition() > 5)
//        {
//            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        }
//
//        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        while(RightF.getCurrentPosition() < 1000)
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0.5);
//            RightR.setPower(0.5);
//        }
//        while(LeftF.getCurrentPosition() < 1000);
//        {
//            LeftF.setPower(0.5);
//            LeftR.setPower(0.5);
//            RightF.setPower(0);
//            RightR.setPower(0);
//        }
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightF.setPower(0);
//        RightR.setPower(0);
//        while(LeftF.getCurrentPosition() > 5)
//        {
//            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
//        }
//
//        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
    }
}




//package com.qualcomm.ftcrobotcontroller.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import java.text.SimpleDateFormat;
//import java.util.Date;
//
///**
// * Created by bk on 9/21/2015.
// */
//public class EncoderTest2 extends OpMode {
//    private String            startDate;
//    private ElapsedTime runTime = new ElapsedTime();
//
//
//    DcMotor frontLeftDrive;
//    DcMotor frontRightDrive;
//    DcMotor backLeftDrive;
//    DcMotor backRightDrive;
//
//    public EncoderTest2() {
//
//    }
//
//    @Override public void init() {
//        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
//
//        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
//        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
//        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        // The strings must match names given in Settings->Configure Robot
//
//
//        // Do not do RESET_ENCODERS and RUN_WITHOUT_ENCODERS
//        // If you do - it will not run.
////        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
//        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        setDrivePower(0.0, 0.0);
//
//        runTime.reset();
//    }
//
//    @Override public void loop() {
//        // Negative is up on the joystick,
//        // Positive is down on the joystick,
//        // Send the "negative" of the joystick so up is now positive
//        setDrivePower(-gamepad1.left_stick_y,-gamepad1.right_stick_y);
//
//        telemetry.addData("1 Motor 1", backRightDrive.getCurrentPosition());
//        telemetry.addData("2 Motor 2", backLeftDrive.getCurrentPosition());
//    }
//
//    @Override public void stop() {
//        setDrivePower(0.0, 0.0);
//    }
//
//    /**
//     * Set the power to left and right motors, the values must range
//     * between -1 and 1.
//     * @param left
//     * @param right
//     */
//    public void setDrivePower(double left, double right) {
//        // This assumes power is given as -1 to 1
//        // The range clip will make sure it is between -1 and 1
//        // An incorrect value can cause the program to exception
//        backLeftDrive.setPower(Range.clip(left, -1.0, 1.0));
//        backRightDrive.setPower(Range.clip(right, -1.0, 1.0));
//    }
//
//
//    /**
//     * Sets the drive mode for each motor.
//     * The types of Run Modes are
//     *   DcMotorController.RunMode.RESET_ENCODERS
//     *      Resets the Encoder Values to 0
//     *   DcMotorController.RunMode.RUN_TO_POSITION
//     *      Runs until the encoders are equal to the target position
//     *   DcMotorController.RunMode.RUN_USING_ENCODERS
//     *      Attempts to keep the robot running straight utilizing
//     *      the PID the reduces the maximum power by about 15%
//     *   DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
//     *      Applies the power directly
//     * @param mode
//     */
//    public void setDriveMode(DcMotorController.RunMode mode) {
//        if (backLeftDrive.getChannelMode() != mode) {
//            backLeftDrive.setChannelMode(mode);
//        }
//
//        if (backRightDrive.getChannelMode() != mode) {
//            backRightDrive.setChannelMode(mode);
//        }
//    }
//}
//
//
////
////
//////import android.hardware.Sensor;
////
////import com.qualcomm.robotcore.hardware.DcMotorController;
////import com.qualcomm.robotcore.hardware.LegacyModule;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.LegacyModule;
////import com.qualcomm.robotcore.hardware.Servo;
////import com.qualcomm.robotcore.hardware.UltrasonicSensor;
////
////import android.content.Context;
////import android.hardware.Sensor;
////import android.hardware.SensorEvent;
////import android.hardware.SensorEventListener;
////import android.hardware.SensorManager;
////
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////
////import java.text.SimpleDateFormat;
////import java.util.Date;
////
////
////import java.util.UnknownFormatConversionException;
////
/////**
//// * Start backwards and align with climber thing
//// */
////public class EncoderTest extends LinearOpMode {
////
////    DcMotor backLeftDrive;
////    DcMotor backRightDrive;
////    DcMotor frontLeftDrive;
////    DcMotor frontRightDrive;
////    DcMotor tape;
////    Servo adjust;
////    Servo deposit;
////
////    int ANDYMARK_TICKS_PER_REV = 1680;
////    double Inches_per_Rotation = 13.125;
////
////    @Override public void runOpMode() throws InterruptedException{
////        //Initialize hardware
////        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
////        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
////        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
////        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
////        //tape = hardwareMap.dcMotor.get("tape");
////
////        //adjust = hardwareMap.servo.get("adjust");
////        //deposit = hardwareMap.servo.get("deposit");
////
//////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        //tape.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////
//////        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//////        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
////
////        //Wait for the game to start
////        waitForStart();
////
////        //deposit.setPosition(Servo.MAX_POSITION);
////
//////        backwards(.5);
//////        sleep(2500);
//////        stopDrive();
//////        sleep(250);
//////
//////        forward(.5);
//////        sleep(700);
//////        stopDrive();
//////        sleep(250);
//////
//////        turnLeft(.5);
//////        sleep(800);
//////        stopDrive();
//////        sleep(250);
//////        forward(.75);
//////        sleep(1500);
//////        stopDrive();
////
////        //telemetry.addData("ticks", "" + findTicks(24));
////        //driveForwardDistance(0.95,findTicks(24));
////        stop();
////
////    }
////
////    public int findTicks(double distance){
////        return (int) (2 * ANDYMARK_TICKS_PER_REV * distance / Inches_per_Rotation);
////    }
////
////    public void forward(double val) {
////
////        frontLeftDrive.setPower(val);
////        frontRightDrive.setPower(val);
////        backLeftDrive.setPower(val);
////        backRightDrive.setPower(val);
////    }
////
////    public void driveForwardDistance(double power, int distance)
////    {
////        //frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        //frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
////        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//////
//////        //frontLeftDrive.setTargetPosition(distance);
//////        //frontRightDrive.setTargetPosition(distance);
//////        backLeftDrive.setTargetPosition(distance);
//////        backRightDrive.setTargetPosition(distance);
//////
//////        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//////        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//////        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//////
//////        forward(power);
//////
//////        while(backLeftDrive.isBusy() && backRightDrive.isBusy()){
//////
//////        }
//////
//////        stopDrive();
//////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        //frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//////        //frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////
////
////    }
////
////    public void turnLeftDistance(double power, int distance)
////    {
////        frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
////        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////
////        frontLeftDrive.setTargetPosition(-distance);
////        frontRightDrive.setTargetPosition(distance);
////        backLeftDrive.setTargetPosition(-distance);
////        backRightDrive.setTargetPosition(distance);
////
////        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////
////        turnLeft(power);
////
////        while(backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy()){
////
////        }
////
////        stopDrive();
////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////    }
////
////    public void turnRightDistance(double power, int distance)
////    {
////        frontRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        frontLeftDrive.setMode(DcMotorController. RunMode.RESET_ENCODERS);
////        backLeftDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        backRightDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////
////        frontLeftDrive.setTargetPosition(distance);
////        frontRightDrive.setTargetPosition(-distance);
////        backLeftDrive.setTargetPosition(distance);
////        backRightDrive.setTargetPosition(-distance);
////
////        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        frontRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        backRightDrive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////
////        turnRight(power);
////
////        while(backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy()){
////
////        }
////
////        stopDrive();
////        backLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        backRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        frontLeftDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        frontRightDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////    }
////
////    public void backwards(double val) {
////        frontLeftDrive.setPower(-val);
////        frontRightDrive.setPower(-val);
////        backLeftDrive.setPower(-val);
////        backRightDrive.setPower(-val);
////    }
////
////    public void turnLeft(double val) {
////        frontLeftDrive.setPower(-val);
////        frontRightDrive.setPower(val);
////        backLeftDrive.setPower(-val);
////        backRightDrive.setPower(val);
////    }
////
////    public void turnRight(double val) {
////        frontLeftDrive.setPower(val);
////        frontRightDrive.setPower(-val);
////        backLeftDrive.setPower(val);
////        backRightDrive.setPower(-val);
////    }
////
////    public void stopDrive(){
////        forward(0);
////    }
////
////}