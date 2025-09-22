
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
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
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class Mecanum
{
    public static class params
    {
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public double kS, kV, kA = 0;

        public double tickPerRev = 8192, wheelID = 38; //gearRatio = 1;
        public double ticksToInch = (tickPerRev/(wheelID * Math.PI) * 0.041);
        public double lateralInPerTick = ticksToInch;
        public double trackWidthTicks = 0;

        public double maxWheelVel = 50, minProfileAccel = -30, maxProfileAccel = 50;
        public double maxAngVel = Math.PI, maxAngAccel = Math.PI;

        public double axialGain , lateralGain, headingGain = 0.0;
        public double axialVelGain, lateralVelGain, headingVelGain = 0.0;

    }

    public static Mecanum.params params = new Mecanum.params();

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

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public Pose2d pose;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<Pose2d>();

    public final MecanumKinematics kinematics = new MecanumKinematics
    (
        params.ticksToInch * params.trackWidthTicks,
    params.ticksToInch / params.lateralInPerTick
    );

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            params.maxAngVel, -params.maxAngAccel, params.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(params.maxWheelVel),
                    new AngularVelConstraint(params.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(params.minProfileAccel, params.maxProfileAccel);

    public class DriveLocalize implements Localizer
    {
        public final Encoder frontLeft, frontRight, backLeft, backRight;
        public final IMU imu;

        private int lastFrontLeftPos, lastBackLeftPos, lastBackRightPos; public int lastFrontRightPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalize(Pose2d pose)
        {
            frontLeft = new OverflowEncoder(new RawEncoder(Mecanum.this.frontLeft));
            frontRight = new OverflowEncoder(new RawEncoder(Mecanum.this.frontRight));
            backLeft = new OverflowEncoder(new RawEncoder(Mecanum.this.backLeft));
            backRight = new OverflowEncoder(new RawEncoder(Mecanum.this.backRight));

            imu = lazyIMU.get();
            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {this.pose = pose;}

        @Override
        public Pose2d getPose() {return pose;}

        @Override
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

        FlightRecorder.write("MECANUM_PARAMS", params);
    }

    public void setDrivePowers(PoseVelocity2d powers)
    {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeft.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        frontRight.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
        backLeft.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRight.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range
                    (
                            0, t.path.length(),
                            Math.max(2, (int) Math.ceil(t.path.length() / 2))
                    );

            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];

            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;

            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController
                    (
                            params.axialGain, params.lateralGain, params.headingGain,
                            params.axialVelGain, params.lateralVelGain, params.headingVelGain
                    ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(params.kS, params.kV / params.ticksToInch, params.kA / params.ticksToInch);
            double frontLeftPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double frontRightPower = feedforward.compute(wheelVels.rightFront) / voltage;
            double backLeftPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double backRightPower = feedforward.compute(wheelVels.rightBack) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(voltage, frontLeftPower, frontRightPower, backLeftPower, backRightPower));

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }
    }

    public final class TurnAction implements Action
    {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    params.axialGain, params.lateralGain, params.headingGain,
                    params.axialVelGain, params.lateralVelGain, params.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(params.kS,
                    params.kV / params.ticksToInch, params.kA / params.ticksToInch);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            frontRight.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
            backLeft.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRight.setPower(feedforward.compute(wheelVels.rightBack) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c)
        {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

    }

    public PoseVelocity2d updatePoseEstimate()
    {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while(poseHistory.size() > 100)
        {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }


    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}