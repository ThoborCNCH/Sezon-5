package org.firstinspires.ftc.teamcode.Amin;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POWER_BRAT;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_ARUNCA_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.POZITIE_NORMAL_CUVA;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.back_to_initial;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.initial;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.la_hub_middle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class Test extends LinearOpMode {
    SampleMecanumDrive robot;


    private final double diameter_hex = 2.637795;
    private final double ticks = 288;
    private final double counts_per_inch = ticks / (diameter_hex * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new SampleMecanumDrive(hardwareMap);
        robot.setPoseEstimate(initial);

        waitForStart();
        while (opModeIsActive()) {

            Trajectory mergi_la_hub = robot.trajectoryBuilder(initial)
                    .splineToConstantHeading(la_hub, Math.toRadians(90),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .addTemporalMarker(0.5, () -> {
                        robot.brat.setPower(-1);
                    })
                    .addTemporalMarker(0.99, () -> {
                        robot.brat.setPower(0);
                    })
                    .addDisplacementMarker(() -> {
                        robot.cuva.setPosition(POZITIE_ARUNCA_CUVA);
                        sleep(1000);
                        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
                    })
//                    .addDisplacementMarker(time -> time, () -> {
//                        robot.cuva.setPosition(POZITIE_NORMAL_CUVA);
//                    })
                    .build();

            robot.followTrajectory(mergi_la_hub);

//            se_ridica_din_mormant(10.5);

            Trajectory bare = robot.trajectoryBuilder(mergi_la_hub.end(), true)
                    .splineToLinearHeading(back_to_initial, Math.toRadians(180),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .addTemporalMarker(0, () -> {
                        robot.brat.setPower(1);
                    })
                    .addTemporalMarker(0.5, () -> {
                        robot.brat.setPower(0);
                    })
                    .build();

            robot.followTrajectory(bare);
            stop();
        }

    }
//    private synchronized void se_ridica_din_mormant(double dist) {
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
}
