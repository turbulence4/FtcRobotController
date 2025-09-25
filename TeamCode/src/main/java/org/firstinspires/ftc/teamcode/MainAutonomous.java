package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class MainAutonomous extends LinearOpMode
{
    DcMotor frontLeft, frontRight, backLeft, backRight;
    ElapsedTime timer = new ElapsedTime();

    public void drive(double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches)
    {
        if(opModeIsActive())
        {
            int frontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * Constants.MecanumConstants.ticksToInch);
            int frontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * Constants.MecanumConstants.ticksToInch);
            int backLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * Constants.MecanumConstants.ticksToInch);
            int backRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * Constants.MecanumConstants.ticksToInch);


            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-0.7);
            frontRight.setPower(-0.7);
            backLeft.setPower(0.7);
            backRight.setPower(0.7);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {}

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

    }


    //@Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(Constants.MecanumConstants.invertLeft);
        frontRight.setDirection(Constants.MecanumConstants.invertRight);
        backLeft.setDirection(Constants.MecanumConstants.invertLeft);
        backRight.setDirection(Constants.MecanumConstants.invertRight);

        waitForStart();

        if(opModeIsActive())
        {
            drive(-44, 44, -44, 44);
            //add numbers once we actually do stuff with autonomous
            //negative numbers are because motors are upside down
        }
    }
}
