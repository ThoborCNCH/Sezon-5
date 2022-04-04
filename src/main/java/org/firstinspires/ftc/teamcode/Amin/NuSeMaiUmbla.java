package org.firstinspires.ftc.teamcode.Amin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config()
public class NuSeMaiUmbla {
    public static double FULL_POWER = 1;
    public static double MEDIUM_POWER = 0.5;
    public static double LOW_POWER = 0.35;

    public static final double POWER_RATA = 1;
    public static final double POWER_BRAT = 1;
    public static double POWER_BRAT_TELEOP = 0.9;

    public static final double POWER_ABS = 1;
    public static double POWER_BRAT_MARKER = 0.5;
    public static double POWER_GHEARA_MARKER = 1;

    public static double POZITIE_NORMAL_CUVA = 0.53;
    public static double POZITIE_ARUNCA_CUVA = 0.12;

    public static double POZITIE_MARKER_IA = 1;
    public static double POZITIE_MARKER_LUAT = 0.18;

    public static double r_l = 20;
    public static double g_l = 50;
    public static double b_l = 40;

    public static double r_h = 100;
    public static double g_h = 255;
    public static double b_h = 255;

    public static double RED = 60;
    public static double SEC = 2;
    public static double DISTANCE = 11.5;


    public static Pose2d initial = new Pose2d(12.004788048944423, -64.30714739102929, Math.toRadians(90));

//  public static Vector2d la_hub = new Vector2d(-5.504788048944423, -52.50714739102929);
//  public static Vector2d la_hub = new Vector2d(-5.504788048944423, -47.50714739102929);

    //RED DR
    public static Vector2d la_hub = new Vector2d(-7.304788048944423, -45.30714739102929);
    public static Vector2d la_hub_middle = new Vector2d(-5.504788048944423, -47.990714739102929);
    public static Pose2d la_hub_pose_la_fel = new Pose2d(-5.504788048944423, -45.650714739102929, Math.toRadians(90));
    public static Pose2d la_hub_pose = new Pose2d(8.504788048944423, -44.790714739102929, Math.toRadians(150));
    public static Pose2d la_hub_pose2 = new Pose2d(-7.804788048944423, -44.90714739102929, Math.toRadians(90));
    public static Pose2d la_hub_pose3 = new Pose2d(-7.804788048944423, -45.90714739102929, Math.toRadians(90));
    public static Pose2d la_rata = new Pose2d(-52.50714739102929, -48.370714739102929, Math.toRadians(90));
    public static Vector2d la_rata_vector = new Vector2d(-60.20714739102929, -58.570714739102929);
    public static Pose2d back_to_initial = new Pose2d(10.004788048944423, -67.5000000, Math.toRadians(180));
    public static Pose2d back_to_initial2 = new Pose2d(9.004788048944423, -68.500000, Math.toRadians(180));

    public static Pose2d inauntru1 = new Pose2d(49.6, -67.6, Math.toRadians(180));
    public static Pose2d inauntru2 = new Pose2d(51, -68.1, Math.toRadians(180));
    public static Pose2d park_pose = new Pose2d(40, -68.4, Math.toRadians(180));

    public static Pose2d PARK_BLUE_DR_READY = new Pose2d(10.004788048944423, -69.400000, Math.toRadians(180));


    //RED ST
    public static Pose2d initialREDST = new Pose2d(-36.02923842281328, -64.30714739102929, Math.toRadians(90));
    public static Vector2d la_hubREDST = new Vector2d(-10.804788048944423, -45.30714739102929);
    public static Vector2d la_hubREDST_reven = new Vector2d(-10.804788048944423, -54.30714739102929);
    public static Pose2d back_to_initialREDST = new Pose2d(11.004788048944423, -68.0000000, Math.toRadians(180));
    public static Pose2d back_to_initialREDST2 = new Pose2d(11.004788048944423, -68.5000000, Math.toRadians(180));
    public static Pose2d la_hub_pose2REDST2 = new Pose2d(-12.304788048944423, -44.90714739102929, Math.toRadians(90));
    public static Vector2d la_hub_simple_part1 = new Vector2d(-60.20714739102929, -30);
    public static Pose2d la_hub_simple_part2 = new Pose2d(-30.20714739102929, -24, Math.toRadians(0));
    public static Pose2d park_mai_putin_csf = new Pose2d(-59, -35, Math.toRadians(0));

    //BLUE ST
    public static Pose2d initialBLUE = new Pose2d(12.004788048944423, 64.30714739102929, Math.toRadians(-90));

    public static Vector2d la_hubBLUE = new Vector2d(-10.804788048944423, 45.90714739102929);
    public static Pose2d la_hub_pose2BLUE = new Pose2d(-12.804788048944423, 45.90714739102929, Math.toRadians(-90));
    public static Pose2d la_hub_pose3BLUE = new Pose2d(-14.804788048944423, 45.90714739102929, Math.toRadians(-90));
    //    public static Vector2d la_rata_vectorBLUE = new Vector2d(-60.20714739102929, 58.570714739102929);
    public static Pose2d la_rata_vectorBLUE = new Pose2d(-60.20714739102929, 58.570714739102929, Math.toRadians(-30));
    public static Pose2d back_to_initialBLUE = new Pose2d(10.004788048944423, 69.1, Math.toRadians(-180));
    public static Pose2d back_to_initial2BLUE = new Pose2d(9.004788048944423, 69.3, Math.toRadians(-180));

    public static Pose2d inauntru1BLUE = new Pose2d(49.6, 68.7, Math.toRadians(-180));
    public static Pose2d inauntru2BLUE = new Pose2d(51, 68.99999, Math.toRadians(-180));
    public static Pose2d park_poseBLUE = new Pose2d(40, 69.2, Math.toRadians(-180));

    public static Pose2d PARK_BLUE_ST_READY = new Pose2d(10.004788048944423, 69.400000, Math.toRadians(-180));

    //    BLUE DR
    public static Pose2d initialBLUEDR = new Pose2d(-36.02923842281328, 64.30714739102929, Math.toRadians(-90));
    public static Pose2d la_hubBLUEDR = new Pose2d(-15.804788048944423, 45.30714739102929, Math.toRadians(-90));
    public static Pose2d la_hubBLUEDR_reven = new Pose2d(-10.804788048944423, 54.30714739102929, Math.toRadians(-90));
    public static Pose2d back_to_initialBLUEDR = new Pose2d(11.004788048944423, 67.8000000, Math.toRadians(-180));
    public static Pose2d back_to_initialBLUEDR2 = new Pose2d(11.004788048944423, 68.3000000, Math.toRadians(-180));
    public static Pose2d la_hub_pose2BLUEDR2 = new Pose2d(-15.304788048944423, 44.90714739102929, Math.toRadians(-90));
    public static Vector2d la_hub_simple_part1BLUE = new Vector2d(-53.20714739102929, 30);
    public static Pose2d la_hub_simple_part2BLUE = new Pose2d(-31.00714739102929, 24, Math.toRadians(0));
    public static Pose2d park_mai_putin_csfBLUE = new Pose2d(-62, 42, Math.toRadians(0));


}
