package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


public class EncoderTest2 extends LinearOpMode {
    DcMotor RightF;
    DcMotor RightR;
    DcMotor LeftF;
    DcMotor LeftR;

    Servo deposit;
    Servo depositTilt;

    Servo middle;

    Servo allClearLeft;
    Servo allClearRight;

    LightSensor lsL;
    LightSensor lsR;
    LightSensor ls;
    TouchSensor touch;
    UltrasonicSensor ultraL;
    UltrasonicSensor ultraR;
    LegacyModule legacy;

    double pulses = 1680;
    double circumference = 13.5;
    boolean hit = false;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private double TARGET_ANGLE_DEGREES = 65.0;
    private final double TOLERANCE_DEGREES = 5.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;

    public int getTicks(double inches) {
        return (int) (2 * inches / circumference * pulses);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        RightR = hardwareMap.dcMotor.get("backRightDrive");
        RightR.setDirection(DcMotor.Direction.REVERSE);
        RightF = hardwareMap.dcMotor.get("frontRightDrive");
        RightF.setDirection(DcMotor.Direction.REVERSE);
        LeftR = hardwareMap.dcMotor.get("backLeftDrive");
        LeftF = hardwareMap.dcMotor.get("frontLeftDrive");
        deposit = hardwareMap.servo.get("deposit");
        deposit.setPosition(Servo.MIN_POSITION);
        depositTilt = hardwareMap.servo.get("depositTilt");
        middle = hardwareMap.servo.get("middle");
        middle.setPosition(0.5);

        allClearRight = hardwareMap.servo.get("allClearRight");
        allClearLeft = hardwareMap.servo.get("allClearLeft");
        allClearRight.setPosition(0.5);
        allClearLeft.setPosition(0.5);

        legacy = hardwareMap.legacyModule.get("legacy");

        // spooky sensors

        ultraL = hardwareMap.ultrasonicSensor.get("ultraL"); // legacy
        ultraR = hardwareMap.ultrasonicSensor.get("ultraR"); // legacy
        legacy.enable9v(4, true);
        legacy.enable9v(5, true); // enable the ports for the ULTRASONICS
        //lsL = hardwareMap.lightSensor.get("lsL"); // left side Light sensor
        //lsR = hardwareMap.lightSensor.get("lsR"); // right side Light sensor
        ls = hardwareMap.lightSensor.get("ls");
        ls.enableLed(true);
        touch = hardwareMap.touchSensor.get("touch");



        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());

        yawPIDController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        df = new DecimalFormat("#.##");

        LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

        deposit.setPosition(Servo.MAX_POSITION);
        depositTilt.setPosition(Servo.MIN_POSITION);

        waitForStart();


        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        telemetry.addData("LeftF Position", LeftF.getCurrentPosition());

        int ticks = getTicks(50);                                                   //Out 2.5ish tiles

        while (LeftR.getCurrentPosition() < ticks) {
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

        while (LeftF.getCurrentPosition() > 5) {
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

//        /////////////////Turn 70 Degrees///////////////////
        LeftF.setDirection(DcMotor.Direction.REVERSE);
        LeftR.setDirection(DcMotor.Direction.REVERSE);

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if ( calibration_complete ) {
                navx_device.zeroYaw();
                telemetry.addData("navX-Micro", "worked");
            } else {

                sleep(200);
            }
        }
        while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }

        navx_device.zeroYaw();
        yawPIDController.enable(true);
        boolean turning = true;
        telemetry.addData("Initial Yaw", navx_device.getYaw());
        while (turning) {
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    LeftR.setPowerFloat();
                    RightR.setPowerFloat();
                    LeftF.setPowerFloat();
                    RightF.setPowerFloat();
                    telemetry.addData("Motor Output", "made it!!!!");
                    turning = false;
                } else {
                    double output = yawPIDResult.getOutput();
                    LeftR.setPower(output);
                    RightR.setPower(output);
                    LeftF.setPower(output);
                    RightF.setPower(output);
                    telemetry.addData("Yaw", navx_device.getYaw());
                }
            }
        }


        while (LeftF.getCurrentPosition() > 5) {
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


        telemetry.addData("While Loop", "Over");
////         use straight line driving to move towards the zone

        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftR.setDirection(DcMotor.Direction.FORWARD);

        double leftDrive;
        double rightDrive;

        sleep(200);
        double drive_speed = 0.95;
        ticks = getTicks(40);

        while (RightF.getCurrentPosition() < ticks) {
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    LeftF.setPower(drive_speed);
                    RightF.setPower(drive_speed);
                    LeftR.setPower(drive_speed);
                    RightR.setPower(drive_speed);
                } else {
                    double output = yawPIDResult.getOutput();
                    leftDrive = drive_speed + output;
                    rightDrive = drive_speed - output;


                    if (leftDrive > .95)
                        leftDrive = .95;

                    if (leftDrive < -.95)
                        leftDrive = -.95;

                    if (rightDrive > .95)
                        rightDrive = .95;

                    if (rightDrive < -.95)
                        rightDrive = -.95;

                    telemetry.addData("LMotor: ", leftDrive);
                    telemetry.addData("RMotor: ", rightDrive);

                    LeftF.setPower(leftDrive);
                    LeftR.setPower(leftDrive);
                    RightF.setPower(rightDrive);
                    RightR.setPower(rightDrive);
                }

            }
        }

        LeftF.setPower(0);
        LeftR.setPower(0);
        RightF.setPower(0);
        RightR.setPower(0);

        while (LeftF.getCurrentPosition() > 5) {
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


        // define variables for line following

        double reflection = ls.getLightDetected();
        double left, right;
        double LIGHT_THRESHOLD = 0.3;
        double MOTOR_POWER = 0.4;
        ElapsedTime runtime = new ElapsedTime();
        double TIMEOUT = 5.0;

        // forward until hit line

        while (reflection > LIGHT_THRESHOLD) {
            LeftF.setPower(.7);
            LeftR.setPower(.7);
            RightR.setPower(.7);
            RightF.setPower(.7);
            reflection = ls.getLightDetected();
            if (runtime.time() > TIMEOUT)
                break;
        }

        sleep(200);
        telemetry.addData("Detected the line", "aylmao");
        LeftF.setPower(0);
        LeftR.setPower(0);
        RightR.setPower(0);
        RightF.setPower(0);
        sleep(200);

        while (LeftF.getCurrentPosition() > 5) {
            LeftF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            LeftR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightF.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            RightR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            telemetry.addData("LeftF Position", LeftF.getCurrentPosition());
        }

        LeftF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        LeftR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightF.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        RightR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);       //Reset Encoders



        sleep(200);

        LeftF.setDirection(DcMotor.Direction.FORWARD);
        LeftR.setDirection(DcMotor.Direction.FORWARD);

//        sleep(200);
//        telemetry.addData("Detected the line", "aylmao");
//        LeftF.setPower(0);
//        LeftR.setPower(0);
//        RightR.setPower(0);
//        RightF.setPower(0);
//        sleep(200);
//
//
        // actual line following
        sleep(600);
        LeftF.setPower(-1);
        LeftR.setPower(-1);
        sleep(400);
        LeftF.setPower(0);
        LeftR.setPower(0);
        sleep(250);
        double ultraLVal = ultraL.getUltrasonicLevel();
        double ultraRVal = ultraR.getUltrasonicLevel();

        telemetry.addData("L", ultraLVal);
        telemetry.addData("R",ultraRVal);
        boolean abort = false;

        while (ultraLVal > 16 && ultraRVal > 16 && !abort) {

            telemetry.addData("TOUCH", hit);
            telemetry.addData("L", ultraLVal);
            telemetry.addData("R", ultraRVal);
            telemetry.addData("reflection", reflection);
            reflection = ls.getLightDetected();
            if (reflection > LIGHT_THRESHOLD) {
                /*
                 * if reflection is less than the threshold value, then assume we are above dark spot.
                 * turn to the right.
                 */
                left = MOTOR_POWER;
                right = 0.0;
            } else {
                /*
                 * assume we are over a light spot.
                 * turn to the left.
                 */
                left = 0.0;
                right = MOTOR_POWER;
            }

            /*
             * set the motor power
             */

            RightR.setPower(right);
            LeftR.setPower(left);
            RightF.setPower(right);
            LeftF.setPower(left);

            hit = touch.isPressed();

            if (hit) {
                if(ultraLVal > 18 && ultraRVal > 18){
                    abort = true;
                    //ABORT MISSION WERE FUCKED
                }
            }

            ultraLVal = ultraL.getUltrasonicLevel();
            ultraRVal = ultraR.getUltrasonicLevel();
            while (ultraLVal == 0 || ultraRVal == 0) {
                ultraLVal = ultraL.getUltrasonicLevel();
                ultraRVal = ultraR.getUltrasonicLevel();
            }
        }


        LeftR.setPower(0);
        LeftF.setPower(0);
        RightR.setPower(0);
        RightF.setPower(0);

        telemetry.addData("stopped", "stopped");

        // deposit the climbers
        sleep(1000);
        deposit.setPosition(Servo.MIN_POSITION);
        sleep(1000);
        depositTilt.setPosition(Servo.MAX_POSITION);
        sleep(1000);
        depositTilt.setPosition(Servo.MIN_POSITION);
        sleep(1000);
        deposit.setPosition(Servo.MAX_POSITION);
        sleep(1000);

    }


}
