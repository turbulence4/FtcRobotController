package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BrokenAutonomous extends LinearOpMode
{
    DcMotor frontLeft, frontRight, backLeft, backRight;
    ElapsedTime timer = new ElapsedTime();
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final int TICKS_PER_MM = (int)(ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));


    public void drive(double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches)
    {


        if(opModeIsActive())
        {
            int frontLeftTarget = frontLeft.getCurrentPosition() + ((int)(frontLeftInches) * TICKS_PER_MM);
            int frontRightTarget = frontRight.getCurrentPosition() + ((int)(frontRightInches) * TICKS_PER_MM);
            int backLeftTarget = backLeft.getCurrentPosition() + ((int)(backLeftInches) * TICKS_PER_MM);
            int backRightTarget = backRight.getCurrentPosition() + ((int)(backRightInches) * TICKS_PER_MM);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.7);
            frontRight.setPower(0.7);
            backLeft.setPower(0.7);
            backRight.setPower(0.7);

            while(opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {}

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
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(Constants.MecanumConstants.invertLeft);
        backRight.setDirection(Constants.MecanumConstants.invertRight);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if(opModeIsActive())
        {
            //forward 3 in
            drive(-100, -100, -100, -100);

            //backward 3ft
            //drive(-3, -3, -3, -3);

            //attempt to rotate 90 deg
            //drive(-20, 20, -20, 20);

            //attempt to rotate -90 deg
            //drive(20, -20, 20, -20);
        }
    }
}