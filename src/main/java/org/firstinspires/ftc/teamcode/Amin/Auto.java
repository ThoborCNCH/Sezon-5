//package org.firstinspires.ftc.teamcode.Amin;
//
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_ABS;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initial;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.initial;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_pose;
//import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_rata;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.Detectie;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@Autonomous()
//public class Putin extends LinearOpMode {
//    SampleMecanumDrive robot;
//
//
//    private final double diameter_hex = 0.787402;
//    private final double ticks = 288;
//    private final double counts_per_inch = ticks / (diameter_hex * Math.PI);
//    OpenCvCamera camera;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot = new SampleMecanumDrive(hardwareMap);
//
//        int cameraViewId = hardwareMap.appContext
//                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraViewId);
//
//        Detectie detectie = new Detectie();
//
//        camera.setPipeline(detectie);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        robot.setPoseEstimate(initial);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            telemetry.addData("a: ", detectie.getA());
//            telemetry.update();
//
//            switch (detectie.getA()) {
//                case 1:
//                    mijloc(4.8);
//                    break;
//                case -1:
//                    jos(2.4);
//                    break;
//                default:
//                    sus(8.7);
//                    break;
//            }
//
////            camera.stopStreaming();
//            stop();
//        }
//    }
//
//    private void sus(double dist) throws InterruptedException {
//        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
//                .splineToConstantHeading(la_hub, Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//
//        robot.followTrajectory(mergi_la_hub);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//
//        robot.puneCub();
//
//        se_ridica_din_mormant(-dist);
//        telemetry.addData("asd", "gata a doua");
//
//        telemetry.update();
//
//        Trajectory mergi_la_rate = robot.trajectoryBuilder(mergi_la_hub.end())
//                .lineToLinearHeading(la_rata)
//                .build();
//        robot.followTrajectory(mergi_la_rate);
//
//        robot.invarteRata();
//
//        Trajectory park_ready = robot.trajectoryBuilder(mergi_la_rate.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//
//        robot.followTrajectory(park_ready);
//
//        Trajectory back_sa_ia = robot.trajectoryBuilder(park_ready.end())
//                .back(37)
//                .addTemporalMarker(0, () -> {
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(back_sa_ia);
//        robot.setAbsortiePower(0);
//        Trajectory back_sa_duca = robot.trajectoryBuilder(back_sa_ia.end())
//                .forward(50)
//                .build();
//        robot.followTrajectory(back_sa_duca);
//
//        Trajectory right = robot.trajectoryBuilder(back_sa_duca.end())
//                .lineToLinearHeading(la_hub_pose)
//                .build();
//        robot.followTrajectory(right);
//
//        se_ridica_din_mormant(4.6);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//        se_ridica_din_mormant(-4.6);
//
//        telemetry.addData("asd", "gata a doua");
//        telemetry.update();
//
//        Trajectory again = robot.trajectoryBuilder(right.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//        robot.followTrajectory(again);
//
//        Trajectory fin_park = robot.trajectoryBuilder(again.end())
//                .back(30)
//                .build();
//        robot.followTrajectory(fin_park);
//    }
//
//    private void mijloc(double dist) throws InterruptedException {
//        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
//                .splineToConstantHeading(la_hub, Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//
//        robot.followTrajectory(mergi_la_hub);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//        se_ridica_din_mormant(-dist);
//        telemetry.addData("asd", "gata a doua");
//
//        telemetry.update();
//
//        Trajectory mergi_la_rate = robot.trajectoryBuilder(mergi_la_hub.end())
//                .lineToLinearHeading(la_rata)
//                .build();
//        robot.followTrajectory(mergi_la_rate);
//
//        robot.invarteRata();
//
//        Trajectory park_ready = robot.trajectoryBuilder(mergi_la_rate.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//
//        robot.followTrajectory(park_ready);
//
//        Trajectory back_sa_ia = robot.trajectoryBuilder(park_ready.end())
//                .back(37)
//                .addTemporalMarker(0, () -> {
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(back_sa_ia);
//        robot.setAbsortiePower(0);
//        Trajectory back_sa_duca = robot.trajectoryBuilder(back_sa_ia.end())
//                .forward(50)
//                .build();
//        robot.followTrajectory(back_sa_duca);
//
//        Trajectory right = robot.trajectoryBuilder(back_sa_duca.end())
//                .lineToLinearHeading(la_hub_pose)
//                .build();
//        robot.followTrajectory(right);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//        se_ridica_din_mormant(-dist);
//
//        telemetry.addData("asd", "gata a doua");
//        telemetry.update();
//
//        Trajectory again = robot.trajectoryBuilder(right.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//        robot.followTrajectory(again);
//
//        Trajectory fin_park = robot.trajectoryBuilder(again.end())
//                .back(30)
//                .build();
//        robot.followTrajectory(fin_park);
//    }
//
//    private void jos(double dist) throws InterruptedException {
//        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
//                .splineToConstantHeading(la_hub, Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//
//        robot.followTrajectory(mergi_la_hub);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//        se_ridica_din_mormant(-dist);
//        telemetry.addData("asd", "gata a doua");
//
//        telemetry.update();
//
//        Trajectory mergi_la_rate = robot.trajectoryBuilder(mergi_la_hub.end())
//                .lineToLinearHeading(la_rata)
//                .build();
//        robot.followTrajectory(mergi_la_rate);
//
//        robot.invarteRata();
//
//        Trajectory park_ready = robot.trajectoryBuilder(mergi_la_rate.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//
//        robot.followTrajectory(park_ready);
//
//        Trajectory back_sa_ia = robot.trajectoryBuilder(park_ready.end())
//                .back(37)
//                .addTemporalMarker(0, () -> {
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(back_sa_ia);
//        robot.setAbsortiePower(0);
//        Trajectory back_sa_duca = robot.trajectoryBuilder(back_sa_ia.end())
//                .forward(50)
//                .build();
//        robot.followTrajectory(back_sa_duca);
//
//        Trajectory right = robot.trajectoryBuilder(back_sa_duca.end())
//                .lineToLinearHeading(la_hub_pose)
//                .build();
//        robot.followTrajectory(right);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//        se_ridica_din_mormant(-dist);
//
//        telemetry.addData("asd", "gata a doua");
//        telemetry.update();
//
//        Trajectory again = robot.trajectoryBuilder(right.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//        robot.followTrajectory(again);
//
//        Trajectory fin_park = robot.trajectoryBuilder(again.end())
//                .back(30)
//                .build();
//        robot.followTrajectory(fin_park);
//    }
//
//    private void se_ridica_din_mormant(double dist) {
//        robot.brat.setPower(0);
//        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        int target = robot.brat.getCurrentPosition() - (int) (dist * counts_per_inch);
//        robot.brat.setTargetPosition(target);
//        robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.setPowerBrat(POWER_BRAT);
//
//
//        while (robot.brat.isBusy() && opModeIsActive()) {
//            telemetry.addData("1) unde e: ", robot.brat.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.setPowerBrat(0);
//
//        robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//}
package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_ABS;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initial;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.inauntru1;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.initial;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_pose2;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_rata_vector;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.Detectie;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = ".Autonom regionala")
@Disabled
public class Auto extends LinearOpMode {
    SampleMecanumDrive robot;
    private double back1 = 39;
    private double back2 = 43;
    private double forward = 50;
    private final long sleep = 650;
    private final double diameter_hex = 2.637795;
    private final double ticks = 288;
    private final double counts_per_inch = ticks / (diameter_hex * Math.PI);
    OpenCvCamera camera;
    ElapsedTime timer = new ElapsedTime();

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

        robot.setPoseEstimate(initial);

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            telemetry.addData("a: ", detectie.getA());
            telemetry.update();

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
//            se_ridica_din_mormant(-7.5);

//            camera.stopStreaming();
            telemetry.addData("cat timp mai e? ", timer.seconds());
            telemetry.update();

            LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();

            stop();
        }
    }

    private void sus() throws InterruptedException {
        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
                .lineToConstantHeading(la_hub
//                        ,
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
                )
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.brat.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
//                    .addDisplacementMarker(time -> time, () -> {
//                        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
//                    })
                .build();

        robot.followTrajectory(mergi_la_hub);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
//            se_ridica_din_mormant(10.5);


        Trajectory bare = robot.trajectoryBuilder(mergi_la_hub.end(), true)
                .lineToLinearHeading(back_to_initial,
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);

        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1)
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(ia_l_frt);

        Trajectory du_l_frt = robot.trajectoryBuilder(ia_l_frt.end())
                .lineToLinearHeading(back_to_initial)
                .addTemporalMarker(0.2, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2)
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

//        a zis mereuta sa mergi ca daca nu te fute
//        si tatal lui ciocanel


        Trajectory rata = robot.trajectoryBuilder(right.end(), true)
                .lineTo(la_rata_vector,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(rata);
        robot.invarteRata();

        TrajectorySequence bare2 = robot.trajectorySequenceBuilder(rata.end())
                .forward(10)
                .lineToLinearHeading(new Pose2d(3.004788048944423, -66.5000000, Math.toRadians(175)))
                .lineToLinearHeading(inauntru1)
                .build();

        robot.followTrajectorySequence(bare2);


    }

    private void mijloc() throws InterruptedException {
        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
                .lineToConstantHeading(la_hub,
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
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

        robot.followTrajectory(mergi_la_hub);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

        Trajectory bare = robot.trajectoryBuilder(mergi_la_hub.end(), true)
                .lineToLinearHeading(back_to_initial,
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(1.370, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);

        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1)
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(ia_l_frt);

        Trajectory du_l_frt = robot.trajectoryBuilder(ia_l_frt.end())
                .lineToLinearHeading(back_to_initial)
                .addTemporalMarker(0.2, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2)
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
//                    sleep(sleep);
//                    robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);


        Trajectory rata = robot.trajectoryBuilder(right.end(), true)
                .lineTo(la_rata_vector
                        , SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })

                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(rata);
        robot.invarteRata();

//        TrajectorySequence park = robot.trajectorySequenceBuilder(rata.end())
//                .lineToLinearHeading(park_ready,
//                        SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(70))
//                .lineToLinearHeading(inauntru1)
//                .build();
//
//        robot.followTrajectorySequence(park);

        telemetry.addData("cat timp mai e? asta e din timpul traiectoriei: ", timer.seconds());
        telemetry.update();

        TrajectorySequence bare2 = robot.trajectorySequenceBuilder(rata.end())
                .forward(10)
                .lineToLinearHeading(new Pose2d(3.004788048944423, -66.5000000, Math.toRadians(175)))
                .lineToLinearHeading(inauntru1)
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .addTemporalMarker(0, () -> {
//                    robot.brat.setPower(1);
//                })
//                .addTemporalMarker(0.5, () -> {
//                    robot.brat.setPower(0);
//                })
                .build();

        robot.followTrajectorySequence(bare2);
//        Trajectory ia_l_frt2 = robot.trajectoryBuilder(bare2.end())
//        Trajectory ia_l_frt2 = robot.trajectoryBuilder(park.end())
//                .lineToLinearHeading(inauntru1)
//                .addTemporalMarker(0, () -> {
//                 robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(ia_l_frt2);

//        Trajectory du_l_frt2 = robot.trajectoryBuilder(ia_l_frt2.end())
//                .lineToLinearHeading(back_to_initial)
//                .addTemporalMarker(0.5, () -> {
//                    robot.brat.setPower(-1);
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .addTemporalMarker(1.5, () -> {
//                    robot.brat.setPower(0);
//                    robot.setAbsortiePower(0);
//                })
//                .build();
//        robot.followTrajectory(du_l_frt2);
//        robot.setAbsortiePower(0);
//        Trajectory right2 = robot.trajectoryBuilder(du_l_frt2.end())
//                .lineToLinearHeading(la_hub_pose)
//                .addDisplacementMarker(() -> {
//                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
//                })
//                .build();
//        robot.followTrajectory(right2);
//        sleep(sleep);
//        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

//        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        TrajectorySequence park2 = robot.trajectorySequenceBuilder(ia_l_frt2.end())
//                .lineToLinearHeading(park_ready)
//                .addTemporalMarker(0, () -> {
//                    robot.brat.setPower(1);
//                })
//
//                .addTemporalMarker(0.6, () -> {
//                    robot.brat.setPower(0);
//                })
//                .lineToLinearHeading(new Pose2d(45, -70, Math.toRadians(180)))
//                .build();
//
//        robot.followTrajectorySequence(park2);
    }

    private void jos() throws InterruptedException {
        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
                .lineToConstantHeading(new Vector2d(-6.804788048944423, -43.89714739102929),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(1.243, () -> {
                    robot.brat.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                })
//                    .addDisplacementMarker(time -> time, () -> {
//                        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
//                    })
                .build();

        robot.followTrajectory(mergi_la_hub);

        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

//            se_ridica_din_mormant(10.5);

        Trajectory bare = robot.trajectoryBuilder(mergi_la_hub.end(), true)
                .lineToLinearHeading(back_to_initial,
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0.8, () -> {
                    robot.brat.setPower(1);
                })
                .addTemporalMarker(1.243, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(bare);

        Trajectory ia_l_frt = robot.trajectoryBuilder(bare.end())
                .lineToLinearHeading(inauntru1)
                .addTemporalMarker(0, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(time -> time * 0.9, () -> {
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(ia_l_frt);

        Trajectory du_l_frt = robot.trajectoryBuilder(ia_l_frt.end())
                .lineToLinearHeading(back_to_initial)
                .addTemporalMarker(0.2, () -> {
                    robot.setAbsortiePower(POWER_ABS);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.brat.setPower(-1);
                })
                .addTemporalMarker(time -> time * 0.8, () -> {
                    robot.brat.setPower(0);
                    robot.setAbsortiePower(0);
                })
                .build();
        robot.followTrajectory(du_l_frt);

        Trajectory right = robot.trajectoryBuilder(du_l_frt.end())
                .lineToLinearHeading(la_hub_pose2)
                .addDisplacementMarker(() -> {
                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
//                    sleep(sleep);
//                    robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
                })
                .build();
        robot.followTrajectory(right);
        sleep(sleep);
        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

        Trajectory rata = robot.trajectoryBuilder(right.end(), true)
                .lineTo(la_rata_vector
                        , SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
//                )
                .addTemporalMarker(0, () -> {
                    robot.brat.setPower(1);
                })

                .addTemporalMarker(0.6, () -> {
                    robot.brat.setPower(0);
                })
                .build();

        robot.followTrajectory(rata);
        robot.invarteRata();

//        TrajectorySequence park = robot.trajectorySequenceBuilder(rata.end())
//                .lineToLinearHeading(park_ready,
//                        SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(70))
//                .lineToLinearHeading(inauntru1)
//                .build();
//
//        robot.followTrajectorySequence(park);

        telemetry.addData("cat timp mai e? asta e din timpul traiectoriei: ", timer.seconds());
        telemetry.update();

        TrajectorySequence bare2 = robot.trajectorySequenceBuilder(rata.end())
                .forward(10)
                .lineToLinearHeading(new Pose2d(3.004788048944423, -66.5000000, Math.toRadians(175)))
                .lineToLinearHeading(inauntru1)
                .build();

        robot.followTrajectorySequence(bare2);
//        Trajectory ia_l_frt2 = robot.trajectoryBuilder(bare2.end())
//        Trajectory ia_l_frt2 = robot.trajectoryBuilder(park.end())
//                .lineToLinearHeading(inauntru1)
//                .addTemporalMarker(0, () -> {
//                 robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(ia_l_frt2);

//        Trajectory du_l_frt2 = robot.trajectoryBuilder(ia_l_frt2.end())
//                .lineToLinearHeading(back_to_initial)
//                .addTemporalMarker(0.5, () -> {
//                    robot.brat.setPower(-1);
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .addTemporalMarker(1.5, () -> {
//                    robot.brat.setPower(0);
//                    robot.setAbsortiePower(0);
//                })
//                .build();
//        robot.followTrajectory(du_l_frt2);
//        robot.setAbsortiePower(0);
//        Trajectory right2 = robot.trajectoryBuilder(du_l_frt2.end())
//                .lineToLinearHeading(la_hub_pose)
//                .addDisplacementMarker(() -> {
//                    robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
//                })
//                .build();
//        robot.followTrajectory(right2);
//        sleep(sleep);
//        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);

//        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        TrajectorySequence park2 = robot.trajectorySequenceBuilder(ia_l_frt2.end())
//                .lineToLinearHeading(park_ready)
//                .addTemporalMarker(0, () -> {
//                    robot.brat.setPower(1);
//                })
//
//                .addTemporalMarker(0.6, () -> {
//                    robot.brat.setPower(0);
//                })
//                .lineToLinearHeading(new Pose2d(45, -70, Math.toRadians(180)))
//                .build();
//
//        robot.followTrajectorySequence(park2);
    }
//    private void jos(double dist) throws InterruptedException {
//        Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
//                .splineToConstantHeading(new Vector2d(-5.504788048944423, -44.550714739102929), Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//
//        robot.followTrajectory(mergi_la_hub);
//
//        se_ridica_din_mormant(dist);
//        telemetry.addData("asd", "gata prima");
//
//        robot.puneCub();
//
//        se_ridica_din_mormant(-dist);
//        telemetry.addData("asd", "gata a doua");
//
//        telemetry.update();
//
//        Trajectory mergi_la_rate = robot.trajectoryBuilder(mergi_la_hub.end())
//                .lineToLinearHeading(la_rata)
//                .build();
//        robot.followTrajectory(mergi_la_rate);
//
//        robot.invarteRata();
//
//        Trajectory park_ready = robot.trajectoryBuilder(mergi_la_rate.end())
//                .lineToLinearHeading(back_to_initial)
//                .build();
//
//        robot.followTrajectory(park_ready);
//
//        Trajectory back_sa_ia = robot.trajectoryBuilder(park_ready.end())
//                .back(37)
//                .addTemporalMarker(0, () -> {
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(back_sa_ia);
//
//        Trajectory back_sa_duca = robot.trajectoryBuilder(back_sa_ia.end())
//                .forward(50)
//                .addTemporalMarker(0, () -> {
//                    robot.setAbsortiePower(POWER_ABS);
//                })
//                .build();
//        robot.followTrajectory(back_sa_duca);
//        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//
//        robot.setAbsortiePower(0);
//        Trajectory right = robot.trajectoryBuilder(back_sa_duca.end())
////                .lineToLinearHeading(la_hub_pose)
//                .lineToLinearHeading(la_hub_pose_la_fel)
//                .addTemporalMarker(0, () -> {
//                    LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//                })
//                .addDisplacementMarker(() -> {
//                    LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//                })
//                .build();
//        robot.followTrajectory(right);
//        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//
//        se_ridica_din_mormant(10);
//        telemetry.addData("asd", "gata prima");
//        robot.puneCub();
//
//
////        Trajectory park_la_mama = robot.trajectoryBuilder(right.end())
////                .lineToLinearHeading(park_mai_putin_csf)
////                .addTemporalMarker(0,()->{
////                    LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
////                })
////                .addDisplacementMarker(()->{
////                    LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
////                })
////                .build();
////        robot.followTrajectory(park_la_mama);
////
////        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//
////        se_ridica_din_mormant(-4.6);
////
////        telemetry.addData("asd", "gata a doua");
////        telemetry.update();
////
//        Trajectory again = robot.trajectoryBuilder(right.end())
//                .lineToLinearHeading(new Pose2d(9.504788048944423, -64.6000000, Math.toRadians(180)))
////                .addTemporalMarker(0, () -> {
////                    robot.brat.setPower(POWER_BRAT);
////                })
//                .build();
//        robot.followTrajectory(again);
//        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//        robot.brat.setPower(POWER_BRAT);
//
//        Trajectory fin_park = robot.trajectoryBuilder(again.end())
//                .back(37)
//                .addTemporalMarker(0, () -> {
//                    LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//                })
//                .build();
//        robot.followTrajectory(fin_park);
//        LocalizareFrt.pe_unde_e_oare = robot.getPoseEstimate();
//        robot.brat.setPower(0);
//        se_ridica_din_mormant(-10);
//
//    }

    private synchronized void se_ridica_din_mormant(double dist) {
        robot.brat.setPower(0);
        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int target = robot.brat.getCurrentPosition() - (int) (dist * counts_per_inch);
        robot.brat.setTargetPosition(target);
        robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setPowerBrat(POWER_BRAT);


        while (robot.brat.isBusy() && opModeIsActive()) {
            telemetry.addData("1) unde e: ", robot.brat.getCurrentPosition());
            telemetry.update();
        }

        robot.setPowerBrat(0);

        robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
