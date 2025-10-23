package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mecanum;

public class Intake
{
    private DcMotor intakeMotor;
    private CRServo beltLeft, beltRight, beltTop;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltTop = hardwareMap.get(CRServo.class, "beltTop");
    }

    public void teleop(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(!Mecanum.alt) {
            if(gamepad1.left_bumper)
            {
                intakeMotor.setPower(0.85);
                beltLeft.setPower(-1);
                beltRight.setPower(1);

            } else {
                intakeMotor.setPower(0);
                beltLeft.setPower(0);
                beltRight.setPower(0);
            }

            if(gamepad1.left_trigger != 0)
            {
                beltTop.setPower(1);
            } else if(gamepad1.right_bumper) {
                beltTop.setPower(-1);
            } else {
                beltTop.setPower(0);
            }
        } else {
            if(gamepad2.left_bumper)
            {
                intakeMotor.setPower(0.85);
                beltLeft.setPower(-1);
                beltRight.setPower(1);

            } else {
                intakeMotor.setPower(0);
                beltLeft.setPower(0);
                beltRight.setPower(0);
            }

            if(gamepad2.left_trigger != 0)
            {
                beltTop.setPower(1);
            } else if(gamepad1.right_bumper) {
                beltTop.setPower(-1);
            } else {
                beltTop.setPower(0);
            }
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Intake:");
    }
}
