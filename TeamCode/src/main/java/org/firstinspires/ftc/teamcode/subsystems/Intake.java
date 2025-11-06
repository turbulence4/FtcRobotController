package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake
{
    private DcMotor intakeMotor;
    private CRServo beltLeft, beltRight, beltTopLeft, beltTopRight;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltTopLeft = hardwareMap.get(CRServo.class, "beltTop");
        beltTopRight = hardwareMap.get(CRServo.class, "beltDown");
    }

    public void teleop(Gamepad gamepad)
    {
        if(gamepad.left_bumper)
        {
            intakeMotor.setPower(0.85);
            beltLeft.setPower(-1);
            beltRight.setPower(1);
        }
        if(!gamepad.left_bumper && !gamepad.right_bumper)
        {
            intakeMotor.setPower(0);
            beltLeft.setPower(0);
            beltRight.setPower(0);
        }
        if(gamepad.left_trigger > 0)
        {
            beltTopLeft.setPower(-1);
            beltTopRight.setPower(-1);
        }
        if(gamepad.right_bumper)
        {
            beltTopLeft.setPower(1);
            beltTopRight.setPower(1);
            beltRight.setPower(-1);
            beltLeft.setPower(1);
        }
        if(gamepad.square)
        {
            intakeMotor.setPower(-0.85);
        }
        if (!gamepad.square && !gamepad.right_bumper && gamepad.left_trigger == 0)
        {
            beltTopLeft.setPower(0);
            beltTopRight.setPower(0);
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Intake: " + intakeMotor.getPower());
        telemetry.addLine("Belt Bottom: " + beltLeft.getPower());
        telemetry.addLine("Belt Top: " + beltTopLeft.getPower());
    }
}
