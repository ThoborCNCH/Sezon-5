package org.firstinspires.ftc.teamcode.Amin.AutonoameNatio.Blue;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_ABS;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.RED;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.SEC;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initial2BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initialBLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.inauntru1BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.initialBLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hubBLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_pose2BLUE;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.park_poseBLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Amin.LocalizareFrt;
import org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.Detectie;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "blue")
public class BlueSt2Cub extends LinearOpMode {
    SampleMecanumDrive robot;
    OpenCvCamera camera;
    private ElapsedTime timp = new ElapsedTime();
    private final long sleep = 650;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot = new SampleMecanumDrive(hardwareMap);

        int cameraViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraViewId);

        Detectie detectie = new Detectie();

        camera.setPipeline(detectie);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        robot.setPoseEstimate(initialBLUE);

        waitForStart();
        while (opModeIsActive()) {
            switch (detectie.getA()) {
                case 1:
                    mijloc();
                    break;
                case -1:
                    jos();
                    break;
                default:
                    sus();
                    break;
            }
        }
        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();

    }

    private void sus() throws InterruptedException {
//        ColorSensor colorSensor = robot.color;
        Trajectory mergi_la_hubBLUE = robot.trajectoryBuilder(initialBLUE)
                .lineToConstantHeading(la_hubBLUE)
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.brat.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();

        robot.followTrajectory(mergi_la_hubBLUE);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare = robot.trajectoryBuilder(mergi_la_hubBLUE.end(), true)
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);


        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1BLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
//                .addTemporalMarker(time -> time * 0.9, () -> {
//                    robot.setAbsortiePower(0);
//                })
                .build();
        robot.followTrajectory(ia_l_frt);


        timp.reset();

        double lowPower = -0.4;
        while (
                (robot.color.red() <= RED)
                        && timp.seconds() <= SEC && opModeIsActive()) {
            robot.setMotorPowers(lowPower, lowPower, lowPower, lowPower);
            robot.setAbsortiePower(POWER_ABS);
            telemetry.addData("Red  ", robot.color.red());
            telemetry.update();
        }
        if (robot.color.red() > RED) {
            telemetry.addLine("l-am luat in pula");
            telemetry.update();
        }
        robot.setAbsortiePower(-POWER_ABS);
        robot.setMotorPowers(0, 0, 0, 0);

        Trajectory du_l_frt = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(-POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.6, () -> {
                    robot.setAbsortiePower(0);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
//                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2BLUE
//                        , SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
                )
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare2 = robot.trajectoryBuilder(right.end(), true)
                .lineToLinearHeading(back_to_initial2BLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.75, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare2);

//        robot.setMotorPowers(1,1,1,1);
//        sleep(500);
//        robot.setMotorPowers(0,0,0,0);


        TrajectorySequence park = robot.trajectorySequenceBuilder(bare2.end())
                .lineToLinearHeading(park_poseBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
//        )
                .strafeLeft(25)
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectorySequence(park);

        stop();

    }

    private void mijloc()  throws InterruptedException {
//        ColorSensor colorSensor = robot.color;
        Trajectory mergi_la_hubBLUE = robot.trajectoryBuilder(initialBLUE)
                .lineToConstantHeading(la_hubBLUE)
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(1.370, () -> {
                    robot.brat.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();

        robot.followTrajectory(mergi_la_hubBLUE);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare = robot.trajectoryBuilder(mergi_la_hubBLUE.end(), true)
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);


        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1BLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
//                .addTemporalMarker(time -> time * 0.9, () -> {
//                    robot.setAbsortiePower(0);
//                })
                .build();
        robot.followTrajectory(ia_l_frt);


        timp.reset();

        double lowPower = -0.4;
        while (
                (robot.color.red() <= RED)
                        && timp.seconds() <= SEC && opModeIsActive()) {
            robot.setMotorPowers(lowPower, lowPower, lowPower, lowPower);
            robot.setAbsortiePower(POWER_ABS);
            telemetry.addData("Red  ", robot.color.red());
            telemetry.update();
        }
        if (robot.color.red() > RED) {
            telemetry.addLine("l-am luat in pula");
            telemetry.update();
        }
        robot.setAbsortiePower(-POWER_ABS);
        robot.setMotorPowers(0, 0, 0, 0);

        Trajectory du_l_frt = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(-POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.6, () -> {
                    robot.setAbsortiePower(0);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
//                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2BLUE
//                        , SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
                )
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare2 = robot.trajectoryBuilder(right.end(), true)
                .lineToLinearHeading(back_to_initial2BLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.75, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare2);


        TrajectorySequence park = robot.trajectorySequenceBuilder(bare2.end())
                .lineToLinearHeading(park_poseBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
//        )
                .strafeLeft(25)
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectorySequence(park);


        stop();

    }

    private void jos()  throws InterruptedException {
//        ColorSensor colorSensor = robot.color;
        Trajectory mergi_la_hubBLUE = robot.trajectoryBuilder(initialBLUE)
                .lineToConstantHeading(la_hubBLUE)
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(1.243, () -> {
                    robot.brat.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();

        robot.followTrajectory(mergi_la_hubBLUE);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare = robot.trajectoryBuilder(mergi_la_hubBLUE.end(), true)
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.443, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);


        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1BLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
//                .addTemporalMarker(time -> time * 0.9, () -> {
//                    robot.setAbsortiePower(0);
//                })
                .build();
        robot.followTrajectory(ia_l_frt);


        timp.reset();

        double lowPower = -0.4;
        while (
                (robot.color.red() <= RED)
                        && timp.seconds() <= SEC && opModeIsActive()) {
            robot.setMotorPowers(lowPower, lowPower, lowPower, lowPower);
            robot.setAbsortiePower(POWER_ABS);
            telemetry.addData("Red  ", robot.color.red());
            telemetry.update();
        }
        if (robot.color.red() > RED) {
            telemetry.addLine("l-am luat in pula");
            telemetry.update();
        }
        robot.setAbsortiePower(-POWER_ABS);
        robot.setMotorPowers(0, 0, 0, 0);

        Trajectory du_l_frt = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(back_to_initialBLUE
                        , SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
//        )
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(-POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.6, () -> {
                    robot.setAbsortiePower(0);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
//                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2BLUE
//                        , SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
                )
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory bare2 = robot.trajectoryBuilder(right.end(), true)
                .lineToLinearHeading(back_to_initial2BLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.75, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare2);

        TrajectorySequence park = robot.trajectorySequenceBuilder(bare2.end())
                .lineToLinearHeading(park_poseBLUE
                        , SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
//        )
                .strafeLeft(25)
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectorySequence(park);



        stop();

    }
}
