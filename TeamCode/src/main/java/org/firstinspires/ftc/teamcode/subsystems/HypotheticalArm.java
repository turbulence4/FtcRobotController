package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HypotheticalArm
{
    private Servo rotXmotor;
    private CRServo rotYmotor;
    private DcMotor wheelMotor;
    private boolean isWheelActive, isYActive = false;
    private int rotXmotorIndex = 0;

    public HypotheticalArm(HardwareMap hardwareMap)
    {
        rotXmotor = hardwareMap.get(Servo.class, "rotXmotor");
        rotYmotor = hardwareMap.get(CRServo.class, "rotYmotor");
        wheelMotor = hardwareMap.get(DcMotor.class, "wheelMotor");
    }

    public void teleop(Gamepad gamepad1)
    {
        rotXmotor.setPosition(0);

        if(gamepad1.a)
        {
            rotXmotorIndex++;
            if(rotXmotorIndex > 2) {rotXmotorIndex = 0;}

            switch(rotXmotorIndex)
            {
                case 0:
                    rotXmotor.setPosition(0);
                    break;
                case 1:
                    rotXmotor.setPosition(0.25);
                    break;
                case 2:
                    rotXmotor.setPosition(0.5);
                    break;
                default:
                    break;
            }
        }

        if(gamepad1.left_stick_x != 0)
        {
            rotYmotor.setPower(gamepad1.left_stick_x);
        } else {
            rotYmotor.setPower(0);
        }

        if(gamepad1.right_stick_x != 0)
        {
            wheelMotor.setPower(gamepad1.right_stick_x);
        } else {
            wheelMotor.setPower(0);
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("rotXmotor:");
        telemetry.addLine("rotYmotor: ");
        telemetry.addLine("wheelMotor: ");
    }
}
