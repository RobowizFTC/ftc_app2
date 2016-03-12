package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TankDriveTapeTest extends OpMode{

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor tapeRight;
    DcMotor tapeLeft;

    Servo leftAdjust;
    Servo rightAdjust;

    Servo climberLeft;
    Servo climberRight;

    Servo middle;

    Servo deposit;
    Servo depositTilt;

    Servo allClearLeft;
    Servo allClearRight;

    int i = 0;
    double pos;

    /**
     * Constructor
     */
    public TankDriveTapeTest() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        tapeRight = hardwareMap.dcMotor.get("tapeRight");
        tapeLeft = hardwareMap.dcMotor.get("tapeLeft");
        tapeLeft.setDirection(DcMotor.Direction.REVERSE);

        climberLeft = hardwareMap.servo.get("climberLeft");
        climberRight = hardwareMap.servo.get("climberRight");

        leftAdjust = hardwareMap.servo.get("leftAdjust");
        rightAdjust = hardwareMap.servo.get("rightAdjust");
        rightAdjust.setPosition(Servo.MAX_POSITION);
        leftAdjust.setPosition(Servo.MIN_POSITION);

        middle = hardwareMap.servo.get("middle");

        deposit = hardwareMap.servo.get("deposit");
        depositTilt = hardwareMap.servo.get("depositTilt");

        allClearRight = hardwareMap.servo.get("allClearRight");
        allClearLeft = hardwareMap.servo.get("allClearLeft");
        allClearRight.setPosition(0.5);
        allClearLeft.setPosition(0.5);
    }

    @Override
    public void loop() {

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // write the values to the motors

        if (right >= .8)                                //Controls for Center Helper Wheel
            middle.setPosition(Servo.MIN_POSITION);
        else if (right <= -.8)
            middle.setPosition(Servo.MAX_POSITION);
        else
            middle.setPosition(0.5);

        frontRightDrive.setPower(right);            //Drive Controls
        frontLeftDrive.setPower(left);
        backRightDrive.setPower(right);
        backLeftDrive.setPower(left);


        // update the position of the climberServo.
        if (gamepad1.y) {                             //Controls for Climber Servos
            // if the Y button is pushed on gamepad1, increment the position of
            // the climberServo servo.
            climberRight.setPosition(Servo.MIN_POSITION * .75);
            climberLeft.setPosition(Servo.MIN_POSITION * .75);
        }

        if (gamepad1.a) {
            // if the A button is pushed on gamepad1, decrease the position of
            // the climberServo servo.
            climberRight.setPosition(Servo.MAX_POSITION * .95);
            climberLeft.setPosition(Servo.MAX_POSITION * .95);
        }


        if (gamepad1.dpad_up) {                     //All Clear Controls
            allClearLeft.setPosition(Servo.MIN_POSITION);
            allClearRight.setPosition(Servo.MAX_POSITION);
        }

        else if(gamepad1.dpad_down) {
            allClearLeft.setPosition(Servo.MAX_POSITION);
            allClearRight.setPosition(Servo.MIN_POSITION);
        }

        if (gamepad2.x) {
            if (deposit.getPosition() > 0.05){
                deposit.setPosition(deposit.getPosition() - 0.05);
            }
        }

        if (gamepad2.b) {
            if (deposit.getPosition() < 0.95){
                deposit.setPosition(deposit.getPosition() + 0.05);
            }
        }

        if (gamepad2.y) {
            depositTilt.setPosition(Servo.MIN_POSITION);
        }

        if (gamepad2.a) {
            depositTilt.setPosition(Servo.MAX_POSITION);
        }


        if (gamepad1.right_trigger > 0.3) {            //Controls for Right Tape Measure
            tapeRight.setPower(.5);
        }

        else if (gamepad1.left_trigger > 0.3) {
            tapeRight.setPower(-.95);
        }

        else {
            tapeRight.setPower(0);
        }


        if(gamepad1.right_bumper){
            if (rightAdjust.getPosition() < 0.98) {
                rightAdjust.setPosition(rightAdjust.getPosition() + 0.015);
            }
            if (leftAdjust.getPosition() > 0.01) {
                leftAdjust.setPosition(leftAdjust.getPosition() - 0.01);
            }

        }

        if ((gamepad1.left_bumper)){
            if (rightAdjust.getPosition() > 0.01) {
                rightAdjust.setPosition(rightAdjust.getPosition() - 0.01);
            }
            if (leftAdjust.getPosition() < 0.98) {
                leftAdjust.setPosition(leftAdjust.getPosition() + 0.015);
            }

        }


        if (gamepad2.right_trigger > 0.3) {         //Controls for Left Tape Measure
            tapeLeft.setPower(0.5);
        }

        else if (gamepad2.left_trigger > 0.3) {
            tapeLeft.setPower(-0.95);
        }
        else {
            tapeLeft.setPower(0);
        }

    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {
        allClearLeft.close();
        allClearRight.close();
        climberLeft.close();
        climberRight.close();
        leftAdjust.close();
        rightAdjust.close();
        middle.close();
        depositTilt.close();
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 0.85, 0.85 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}