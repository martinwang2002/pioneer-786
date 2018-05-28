package org.firstinspires.ftc.Pioneer2018;

/**
 * Created by aosc on 1/25/18.
 */

public class MechanismUtil {
    public static double[] calcv(double ome, double x, double y) {
        double a = 0.17125;
        double b = 0.1705;
        double c = Math.sqrt(2);
        double vw1 = (y-x+ome*(a+b))/c;
        double vw2 = (y+x-ome*(a+b))/c;
        double vw3 = (y-x-ome*(a+b))/c;
        double vw4 = (y+x+ome*(a+b))/c;
        double max = Math.max(
                Math.max(Math.abs(vw1), Math.abs(vw2)),
                Math.max(Math.abs(vw3), Math.abs(vw4))
        );
        if (max > 0.001) {
            vw1 /= max;
            vw2 /= max;
            vw3 /= max;
            vw4 /= max;
        }

        return new double[]{vw1, vw2, vw3, vw4};
    }
}
