package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
@Disabled

public class SIMPLE extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightRear, rightFront;
    public static double power = 0.3;
    public static boolean a = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                a = !a;
            }

            if (gamepad1.dpad_up) {
                if (!a)
                    leftFront.setPower(power);
                else
                    leftFront.setPower(-power);
            } else {
                leftFront.setPower(0);
            }


            if (gamepad1.dpad_down) {
                if (!a)
                    rightFront.setPower(power);
                else
                    rightFront.setPower(-power);
            } else {
                rightFront.setPower(0);
            }

            if (gamepad1.dpad_left) {
                if (!a)
                    leftRear.setPower(power);
                else
                    leftRear.setPower(-power);
            } else {
                leftRear.setPower(0);
            }

            if (gamepad1.dpad_right) {
                if (!a)
                    rightRear.setPower(power);
                else
                    rightRear.setPower(-power);
            } else {
                rightRear.setPower(0);
            }


        }
    }
}
