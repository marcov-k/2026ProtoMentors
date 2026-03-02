package frc.robot.utilities;

public class ShooterMap
{
    private static final double[] Distance = { 1.0, 2.5, 4.0, 5.5, 7.0 };
    private static final double[] RPM = { 1800, 2300, 2800, 3300, 3800};

    public static double rpmForDistance(double meters)
    {
        if (meters <= Distance[0]) return RPM[0];

        for (int i = 0; i < Distance.length - 1; i++)
        {
            if (meters <= Distance[i + 1])
            {
                double t = (meters - Distance[i]) / (Distance[i + 1] - Distance[i]);
                return RPM[i] + t * (RPM[i + 1] - RPM[i]);
            }
        }
        return RPM[RPM.length - 1];
    }

    private ShooterMap() {}
}