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
        public static final double millimetersToInch = 0.0393701;
        public static final double inchesToMillimeters = 25.4;
        public static final double ticksToInch = millimetersToInch * 100;
        public static final double inchesToTick = inchesToMillimeters * 100;
        //100 ticks per millimeter
    }
}
