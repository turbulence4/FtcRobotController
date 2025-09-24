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

    public void drive()
    {
        if(opModeIsActive())
        {
            System.out.println("op mode is active");
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
            drive();
            //add numbers once we actually do stuff with autonomous
        }
    }
}
