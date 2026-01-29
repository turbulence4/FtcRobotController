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
    private DcMotor transitionMotor;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltTopLeft = hardwareMap.get(CRServo.class, "beltTop");
        beltTopRight = hardwareMap.get(CRServo.class, "beltDown");
        transitionMotor = hardwareMap.get(DcMotor.class, "transitionMotor");
    }

    public void teleop(Gamepad gamepad)
    {
        if(gamepad.left_bumper)
        {
            intakeMotor.setPower(0.85);
            transitionMotor.setPower(-0.35);
        }
        if(!gamepad.left_bumper && !gamepad.right_bumper) {
            intakeMotor.setPower(0);
            transitionMotor.setPower(0);
        }
        if(gamepad.square) {
            intakeMotor.setPower(-0.85);
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Intake: " + intakeMotor.getPower());
        telemetry.addLine("Transition: " + transitionMotor.getPower());
    }
}
