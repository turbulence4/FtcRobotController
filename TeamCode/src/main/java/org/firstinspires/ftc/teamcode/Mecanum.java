package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.Math;
import java.util.LinkedList;

public final class Mecanum
{
    public static class params
    {
        public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public static double tickPerRev = 8192, wheelID = 38, gearRatio = 1;
        public static double ticksToInch = (tickPerRev/(wheelID * Math.PI) * 0.041);
        public static double lateralInPerTick = ticksToInch;
        public static double trackWidthTicks = 0;

        public static double speed = 0, velocity = 0, acceleration = 0;
        public static double maxWheelVel = 50, minProfileAccel = -30, maxProfileAccel = 50;
        public double maxAngVel = Math.PI, maxAngAccel = Math.PI;

    }

    public Mecanum(HardwareMap hardwareMap, Localizer localizer)
    {
        frontLeft = hardwareMap.get(DcMotorEx.class, Constants.MecanumConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.MecanumConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotorEx.class, Constants.MecanumConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotorEx.class, Constants.MecanumConstants.backRightMotor);
    }

    //actual mecanum code starts here
    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private LazyImu lazyIMU;
    private VoltageSensor voltageSensor;
    public Localizer localizer;
    public LazyImu lazyImu;

    public Pose2d pose;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<Pose2d>();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            params.ticksToInch * params.trackWidthTicks, params.ticksToInch / params.lateralInPerTick);


    public class DriveLocalize implements Localizer {
        public final Encoder frontLeft, frontRight, backLeft, backRight;
        public final IMU imu;

        private int lastFrontLeftPos, lastBackLeftPos, lastBackRightPos; public int lastFrontRightPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalize(Pose2d pose) {
            frontLeft = new OverflowEncoder(new RawEncoder(Mecanum.this.frontLeft));
            frontRight = new OverflowEncoder(new RawEncoder(Mecanum.this.frontRight));
            backLeft = new OverflowEncoder(new RawEncoder(Mecanum.this.backLeft));
            backRight = new OverflowEncoder(new RawEncoder(Mecanum.this.backRight));

            imu = lazyIMU.get();
            this.pose = pose;
        }

        public void setPose(Pose2d pose)
        {
            this.pose = pose;
        }

        public Pose2d getPose(Pose2d pose) {
            return pose;
        }

        public PoseVelocity2d update()
        {
            PositionVelocityPair frontLeftPosVel = frontLeft.getPositionAndVelocity();
            PositionVelocityPair frontRightPosVel = frontRight.getPositionAndVelocity();
            PositionVelocityPair backLeftPosVel = backLeft.getPositionAndVelocity();
            PositionVelocityPair backRightPosVel = backRight.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized)
            {
                initialized = true;

                lastFrontLeftPos = frontLeftPosVel.position;
                lastFrontRightPos = frontRightPosVel.position;
                lastBackLeftPos = backLeftPosVel.position;
                lastBackRightPos = backRightPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>
            (
                new DualNum<Time>(new double[]{(frontLeftPosVel.position - lastFrontLeftPos), frontLeftPosVel.velocity}).times(params.ticksToInch),
                new DualNum<Time>(new double[]{(frontRightPosVel.position - lastBackRightPos), frontRightPosVel.velocity}).times(params.ticksToInch),
                new DualNum<Time>(new double[]{(backLeftPosVel.position - lastBackLeftPos), backLeftPosVel.velocity}).times(params.ticksToInch),
                new DualNum<Time>(new double[]{(backRightPosVel.position - lastBackRightPos), backRightPosVel.velocity}).times(params.ticksToInch)
            ));

            lastFrontLeftPos = frontLeftPosVel.position;
            lastFrontRightPos = frontRightPosVel.position;
            lastBackLeftPos = backLeftPosVel.position;
            lastBackRightPos = backRightPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));
            return twist.velocity().value();
        }
    }

    public Mecanum(HardwareMap hardwareMap, Pose2d pose)
    {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for(LynxModule module : hardwareMap.getAll(LynxModule.class))
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                params.logoFacingDirection, params.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new Mecanum.DriveLocalize(pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }



}

