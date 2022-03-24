package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.LocalizareFrt.pe_unde_e_oare;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.LOW_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.MEDIUM_POWER;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_ABS;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_RATA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_MARKER_IA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_MARKER_LUAT;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initial;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.inauntru1;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_pose;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_pose2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
@Disabled
public class TeleOpCuAuto extends LinearOpMode {
    private double v1, v2, v3, v4;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            robot.setPoseEstimate(pe_unde_e_oare);

            switch (currentMode) {
                case DRIVER_CONTROL:
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
                        robot.setPowerBrat(POWER_BRAT);
                    } else if (gamepad2.right_bumper) {
                        robot.setPowerBrat(-POWER_BRAT);
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
                    Pose2d poseEstimate = robot.getPoseEstimate();

                    if (gamepad1.a) {
                        telemetry.addData("x: ", poseEstimate.getX());
                        telemetry.addData("y: ", poseEstimate.getY());
                        telemetry.addData("heading: ", poseEstimate.getHeading());
                        telemetry.update();
                    }

                    if (gamepad1.x) {
                        Trajectory in_wearhouse = robot.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(back_to_initial)
                                .build();
                        robot.followTrajectory(in_wearhouse);
                        Trajectory du_te_frt = robot.trajectoryBuilder(in_wearhouse.end())
                                .lineToLinearHeading(inauntru1)
                                .build();
                        robot.followTrajectory(du_te_frt);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    if (gamepad1.b) {
                        Trajectory in_teren = robot.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(inauntru1)
                                .build();
                        robot.followTrajectory(in_teren);
                        Trajectory du_te_frt = robot.trajectoryBuilder(in_teren.end())
                                .lineToLinearHeading(back_to_initial)
                                .build();
                        robot.followTrajectory(du_te_frt);
                        Trajectory right = robot.trajectoryBuilder(du_te_frt.end())
                                .lineToLinearHeading(la_hub_pose2)
                                .build();
                        robot.followTrajectory(right);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    robot.update();

                    break;
                case AUTOMATIC_CONTROL:
                    // daca apasam pe y din gmp2 se opreste autonomul
                    if (gamepad1.y) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    //ne intoarcem inapoi in driving mode dupa ce e gata autonom
                    if (!robot.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            robot.update();

        }

    }
}
