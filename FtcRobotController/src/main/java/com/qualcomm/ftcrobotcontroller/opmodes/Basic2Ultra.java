package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class Basic2Ultra extends LinearOpMode {

    LegacyModule legacy;
    UltrasonicSensor ultraL;
    UltrasonicSensor ultraR;

    DcMotor LeftF;
    DcMotor LeftR;
    DcMotor RightF;
    DcMotor RightR;

    @Override
    public void runOpMode() throws InterruptedException {
        legacy = hardwareMap.legacyModule.get("legacy");
        ultraL = hardwareMap.ultrasonicSensor.get("ultraL");
        ultraR = hardwareMap.ultrasonicSensor.get("ultraR");
        LeftF = hardwareMap.dcMotor.get("frontLeftDrive");
        LeftR = hardwareMap.dcMotor.get("backLeftDrive");
        RightF = hardwareMap.dcMotor.get("frontRightDrive");
        RightR = hardwareMap.dcMotor.get("backRightDrive");
        double ULTRA_THRESHOLD = 5.0;
        double distanceL;
        double distanceR;
        double distanceFromWall = 30.0;
        double motor_power = .75;
        legacy.enable9v(4, true);
        legacy.enable9v(5, true);

        waitForStart();

        distanceL = ultraL.getUltrasonicLevel();
        distanceR = ultraR.getUltrasonicLevel();

        boolean both = true;
        while (both) {
            distanceL = ultraL.getUltrasonicLevel();
            distanceR = ultraR.getUltrasonicLevel();

            while (distanceL == 0 || distanceR == 0){
                distanceL = ultraL.getUltrasonicLevel();
                distanceR = ultraR.getUltrasonicLevel();
            }
            telemetry.addData(""+distanceL,""+distanceR);
            double power = scale(motor_power, distanceL, distanceR);
            if (distanceR > distanceL){

                //LeftF.setPower(-power);
                //LeftR.setPower(-power);
            }


            if (distanceL > distanceR) {
                //RightF.setPower(-power);
                //RightR.setPower(-power);
            }

            telemetry.addData("Output", power);

            if(distanceL > 0 || distanceR > 0)
                both = Math.abs(distanceL-distanceR) >= ULTRA_THRESHOLD;
            else
                both = true;



            telemetry.addData("data", "" + distanceL + ", " + distanceR);
        }

        telemetry.addData("done", "done");
        telemetry.addData("data", "" + distanceL + ", " + distanceR);
        LeftF.setPower(0);
        LeftR.setPower(0);
        RightF.setPower(0);
        RightR.setPower(0);

    }

    public double scale (double base, double d1, double d2) {
        return (1 - (base / (Math.abs(d1-d2)))) * 0.25;
    }
}
