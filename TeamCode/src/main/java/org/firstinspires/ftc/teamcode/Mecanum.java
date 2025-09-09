package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Mecanum
{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    public Mecanum(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backRightMotor);
    }
}
