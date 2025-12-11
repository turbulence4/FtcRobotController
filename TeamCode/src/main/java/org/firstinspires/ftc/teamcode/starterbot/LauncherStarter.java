package org.firstinspires.ftc.teamcode.starterbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherStarter {
    private DcMotor launcherMotor;
    private CRServo leftServo, rightServo;

    public LauncherStarter(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");

        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void teleop(Gamepad gamepad){
        if (gamepad.right_trigger > 0.0){
            launcherMotor.setPower(.63);
        } else {
            launcherMotor.setPower(0);
        }

        if(gamepad.a){
            leftServo.setPower(0.5);
            rightServo.setPower(-0.5);
        } else {
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
    }
}
