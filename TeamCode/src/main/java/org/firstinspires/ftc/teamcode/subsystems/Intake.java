package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake
{
    private DcMotor intakeMotor;
    private boolean isIntakeActive;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void teleop(Gamepad gamepad1)
    {
        if(gamepad1.x)
        {
            isIntakeActive = !isIntakeActive;

            if (isIntakeActive)
            {
                intakeMotor.setPower(0.7);
            } else {
                intakeMotor.setPower(0);
            }
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Intake:");
    }
}
