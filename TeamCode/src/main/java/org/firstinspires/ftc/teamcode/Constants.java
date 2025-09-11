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
    }
}
