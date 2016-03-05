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
    Servo climberLeft;
    Servo climberRight;
    Servo middle;
    DcMotor tapeRight;
    DcMotor tapeLeft;
    Servo adjust;
    //Servo deposit;
    // Servo safe;
    Servo allClearLeft;
    Servo allClearRight;
    boolean reversed = false;
    boolean rightExtended = false;
    boolean leftExtended = false;
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
        climberLeft = hardwareMap.servo.get("climberLeft");
        climberRight = hardwareMap.servo.get("climberRight");
        tapeRight = hardwareMap.dcMotor.get("tapeRight");
        tapeLeft = hardwareMap.dcMotor.get("tapeLeft");
        tapeLeft.setDirection(DcMotor.Direction.REVERSE);
        adjust = hardwareMap.servo.get("adjust");
        middle = hardwareMap.servo.get("middle");
        // deposit = hardwareMap.servo.get("deposit");
        // safe = hardwareMap.servo.get("safe");
        allClearLeft = hardwareMap.servo.get("allClearLeft");
        allClearRight = hardwareMap.servo.get("allClearRight");
        allClearLeft.setPosition(Servo.MIN_POSITION);
        allClearRight.setPosition(Servo.MIN_POSITION);
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

        if (right >= .8)
            middle.setPosition(Servo.MIN_POSITION);
        else if (right <= -.8)
            middle.setPosition(Servo.MAX_POSITION);
        else
            middle.setPosition(0.5);

        frontRightDrive.setPower(right);
        frontLeftDrive.setPower(left);
        backRightDrive.setPower(right);
        backLeftDrive.setPower(left);


        // update the position of the climberServo.
//        if (gamepad1.y) {
//            // if the Y button is pushed on gamepad1, increment the position of
//            // the climberServo servo.
//            climber.setPosition(Servo.MIN_POSITION * .75);
//        }
//
//        if (gamepad1.a) {
//            // if the A button is pushed on gamepad1, decrease the position of
//            // the climberServo servo.
//            climber.setPosition(Servo.MAX_POSITION * .95);
//        }
//        if (gamepad1.b) {
//            pos = deposit.getPosition() + .1;
//            if (pos > 1)
//                pos = .99;
//            deposit.setPosition(pos);
//        }
//
        if (gamepad1.dpad_up) {
            allClearLeft.setPosition(Servo.MAX_POSITION);
            allClearRight.setPosition(Servo.MAX_POSITION);
        }

        if (gamepad1.b){
            if (rightExtended) {
                climberRight.setPosition(Servo.MIN_POSITION);
                rightExtended = false;
            }
            else {
                climberRight.setPosition(0.5);
                rightExtended = true;
            }
        }

        if (gamepad1.x){
            if (leftExtended) {
                climberLeft.setPosition(Servo.MIN_POSITION);
                leftExtended = false;
            }
            else {
                climberLeft.setPosition(0.5);
                leftExtended = true;
            }
        }

        else if(gamepad1.dpad_down) {
            allClearLeft.setPosition(Servo.MIN_POSITION);
            allClearRight.setPosition(Servo.MIN_POSITION);
        }
//
//        if (gamepad1.x) {
//            pos = deposit.getPosition() - .1;
//            if (pos < 0)
//                pos = 0.01;
//            deposit.setPosition(pos);
//        }
        if (gamepad1.right_trigger >= .3) {
            tapeRight.setPower(.5);
        }

        else if (gamepad1.left_trigger >= .3) {
            tapeRight.setPower(-.95);
        }

        else {
            tapeRight.setPower(0);
        }

        if (gamepad1.right_bumper) {
//            if (adjust.getPosition() < 0.99) {
//                adjust.setPosition(adjust.getPosition() + 0.01);
//            }
            tapeLeft.setPower(0.5);
        }

        else if (gamepad1.left_bumper) {
//            if (adjust.getPosition() > 0.01) {
//                adjust.setPosition(adjust.getPosition() - 0.01);
//            }
            tapeLeft.setPower(-0.95);
        }
        else {
            tapeLeft.setPower(0);
        }

        if(gamepad1.a){
            if (adjust.getPosition() < 0.99) {
                adjust.setPosition(adjust.getPosition() + 0.01);
            }
        }

        if ((gamepad1.y)){
            if (adjust.getPosition() > 0.01) {
                adjust.setPosition(adjust.getPosition() - 0.01);
            }
        }


//        if (gamepad1.guide) {
//
//            if (reversed) {
//                reversed = false;
//                adjust.setDirection(Servo.Direction.FORWARD);
//                tape.setDirection(DcMotor.Direction.FORWARD);
//            }
//
//            else {
//                reversed = true;
//                adjust.setDirection(Servo.Direction.REVERSE);
//                tape.setDirection(DcMotor.Direction.REVERSE);
//            }
//        }

//        if (gamepad2.a) {
//            allClear.setPosition(Servo.MAX_POSITION);
//        }
//
//        if (gamepad2.y) {
//            allClear.setPosition(Servo.MIN_POSITION);
//        }


        // update the position of the claw
        // clip the position values so that they never exceed their allowed range.
        //armPosition = Range.clip(armPosition, ARM_MIN_RANGE,›› ARM_MAX_RANGE);
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

        // write position values to the wrist and claw servo
        //climberServo.setPosition(armPosition);
        //claw.setPosition(clawPosition);
    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {
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