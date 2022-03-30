/*
 *
 * ©Thobor 2021-2022
 *
 *           _
 *       .__(.)< (MEOW)
 *        \___)
 * ~~~~~~~~~~~~~~~~~~
 *
 *
 */
package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.MEDIUM_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_ABS;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT_MARKER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT_TELEOP;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_GHEARA_MARKER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_RATA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
public class TeleOpulAlaBlana extends LinearOpMode {

    private double v1, v2, v3, v4;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // joysticks
            
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = (gamepad1.right_stick_x);
            v1 = r * Math.cos(robotAngle) + rightX;
            v2 = r * Math.sin(robotAngle) - rightX;
            v3 = r * Math.sin(robotAngle) + rightX;
            v4 = r * Math.cos(robotAngle) - rightX;

            robot.bagaViteza(v1, v2, v3, v4);

            // miscari din dpad uri
            while (gamepad1.dpad_down) {
                robot.bagaViteza(-LOW_POWER, -LOW_POWER, -LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.dpad_right) {
                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
            }
            while (gamepad1.dpad_up) {
                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
            }
            while (gamepad1.dpad_left) {
                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
            }

            // rotiri fine din triggere
            while (gamepad1.right_trigger != 0) {
                robot.bagaViteza(LOW_POWER, -LOW_POWER, LOW_POWER, -LOW_POWER);
            }
            while (gamepad1.left_trigger != 0) {
                robot.bagaViteza(-LOW_POWER, LOW_POWER, -LOW_POWER, LOW_POWER);
            }

            // schimbare putere
//            if (gamepad2.a) {
//                NuSeMaiUmbla.FULL_POWER = 1;
//            } else if (gamepad2.b) {
//                NuSeMaiUmbla.FULL_POWER = 0.7;
//            }

            // absorbtia
            if (gamepad1.right_bumper) {
                robot.setAbsortiePower(POWER_ABS);
            } else if (gamepad1.left_bumper) {
                robot.setAbsortiePower(-POWER_ABS);
            } else {
                robot.setAbsortiePower(0);
            }

            // cuva
            if (gamepad2.x) {
                robot.setCuvaPosition(POZITIE_NORMAL_CUVA);
            } else if (gamepad2.y) {
                robot.setCuvaPosition(POZITIE_ARUNCA_CUVA);
            }

            // brat
            if (gamepad2.left_bumper) {
                robot.setPowerBrat(POWER_BRAT_TELEOP);
            } else if (gamepad2.right_bumper) {
                robot.setPowerBrat(-POWER_BRAT_TELEOP);
            } else {
                robot.setPowerBrat(0);
            }

            // rata
            if (gamepad2.left_trigger != 0) {
                robot.setRataPower(POWER_RATA);
            } else if (gamepad2.right_trigger != 0) {
                robot.setRataPower(-POWER_RATA);
            } else {
                robot.setRataPower(0);
            }

            // marker


//            if (gamepad2.dpad_up) {
//                robot.setBratMarkerPower(POWER_BRAT_MARKER);
//            } else {
//                robot.setBratMarkerPower(0);
//            }
//
//            if (gamepad2.dpad_down) {
//                robot.setBratMarkerPower(-POWER_BRAT_MARKER);
//            } else {
//                robot.setBratMarkerPower(0);
//            }

            if(gamepad2.dpad_up){
                robot.setBratMarkerPower(POWER_BRAT_MARKER);
            }
            else robot.setBratMarkerPower(0);

            if(gamepad2.dpad_down){
                robot.setBratMarkerPower(-POWER_BRAT_MARKER);
            }
            else robot.setBratMarkerPower(0);

            if(gamepad1.a){
                robot.setGhearaPower(POWER_GHEARA_MARKER);
            }
            else robot.setGhearaPower(0);

            if(gamepad1.b){
                robot.setGhearaPower(-POWER_GHEARA_MARKER);
            }
            else robot.setGhearaPower(0);

            robot.update();
        }
    }


}


