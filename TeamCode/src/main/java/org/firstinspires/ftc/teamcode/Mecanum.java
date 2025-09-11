package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.Math;
import java.util.ArrayList;
import java.util.LinkedList;

public final class Mecanum
{
    public static class params
    {
        public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public static double tickPerRev = 8192, wheelID = 38, gearRatio = 1;
        public static double ticksToInch = (tickPerRev/(wheelID * Math.PI) * 0.041);

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
    public final Localizer localizer;

    public final Pose2d pose;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<Pose2d>();

    public class DriveLocalize implements Localizer {
        public final Encoder frontLeft, frontRight, backLeft, backRight;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
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

                lastLeftFrontPos = frontLeftPosVel.position;
                lastLeftBackPos = frontRightPosVel.position;
                lastRightBackPos = backLeftPosVel.position;
                lastRightFrontPos = backRightPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }
        }
    }



}

