//password for robot wifi is bttitans

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mecanum
{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private final double ticksPerRev = 8192;
    private final double wheelD = 38; //38mm in inches
    private final double gearRatio = 1;
    private final double ticksToInch = (8192 / (wheelD * Math.PI)) * 0.75;
    public static boolean alt;

    public Mecanum(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot
                (RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
    }

    public void teleop(Gamepad gamepad1, boolean mode)
    {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotX = gamepad1.right_stick_x; //turning in place

        //counteract for imperfect strafing:
        //make x slightly stronger so that you don't have to push the stick as hard
        x *= 1.1;

        if(gamepad1.options)
        {
            imu.resetYaw();
        }

        if(mode)
        {
            x /= 10;
            y /= 10;
            rotX /= 10;
        }

        drive(x, y, rotX, true );
    }

    public void setPower(double frontLeftVal, double frontRightVal, double backLeftVal, double backRightVal)
    {
        //dont set frontLeftVal as negative if copying this code for another robot
        //prev: -- -
        frontLeft.setPower(-frontLeftVal * 2);
        frontRight.setPower(-frontRightVal * 2);
        backLeft.setPower(backLeftVal * 2);
        backRight.setPower(-backRightVal * 2);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOrientated)
    {
        double rotY = ySpeed;
        double rotX = xSpeed;

        if(fieldOrientated)
        {
            double botDir = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //rotate the movement direction counter to the bot's rotation
            rotX = xSpeed * Math.cos(-botDir) - ySpeed * Math.sin(-botDir);
            rotY = xSpeed * Math.sin(-botDir) + ySpeed * Math.cos(-botDir);

            rotX *= 1.1; //counteract imperfect strafing
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public double[] getPower()
    {
        double[] power = {frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower()};

        return power;
    }

    public double getLeftDist()
    {
        return frontLeft.getCurrentPosition() * Constants.MecanumConstants.ticksToInch;
    }

    public double getRightDist()
    {
        return frontRight.getCurrentPosition() * Constants.MecanumConstants.ticksToInch;
    }

    public double getAvgDist()
    {
        return (getLeftDist() + getRightDist()) / 2;
    }

    public double getYaw() //yaw is rotation on the x axis (the only one we can do)
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Drive:");
        telemetry.addData("Front Left:", getPower()[0]);
        telemetry.addData("Back Left:", getPower()[1]);
        telemetry.addData("Front Right:", getPower()[2]);
        telemetry.addData("Back Right:", getPower()[3]);
        telemetry.addLine("Position");
        telemetry.addData("Left:", frontLeft.getCurrentPosition());
        telemetry.addData("Right:", frontRight.getCurrentPosition());
        telemetry.addData("Distance:", getAvgDist());
        telemetry.addData("Yaw:", getYaw());
    }

    private DcMotor motorConfig(DcMotor motor)
    {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return motor;
    }

    public void resetMotors()
    {
        motorConfig(frontLeft);
        motorConfig(frontRight);
        motorConfig(backLeft);
        motorConfig(backRight);
    }
}