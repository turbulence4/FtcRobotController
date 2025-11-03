package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants
{
    public static final class MecanumConstants
    {
        public static final String frontLeftMotor = "frontLeft";
        public static final String frontRightMotor = "frontRight";
        public static final String backLeftMotor = "backLeft";
        public static final String backRightMotor = "backRight";

        public static final DcMotor.Direction invertLeft = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction invertRight = DcMotor.Direction.REVERSE;

        public static final double ticksPerRev = 8192;
        public static final double wheelD = 38; //38mm in inches
        public static final double gearRatio = 1;
        public static final double ticksToInch = (ticksPerRev / (wheelD * Math.PI)) * 0.75;
        public static final double inchesToTick = ((wheelD * Math.PI) / ticksPerRev) * 0.75;

    }
}
