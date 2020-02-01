

public class Stimulus {
    public static final double STIM_DURATION_S = 10;
    /**
     * Get the actual value of the physical quantity being estimated
     * @param t
     * @return
     */
    public static double GetProcessActual(double t) {
        // Right now this is just a hard-coded progression
        if( t < 2) {
            return 10 * t;
        } else if (t < 4) {
            return 20;
        } else if (t < 6) {
            return 20-10*(t-4);
        } else if(t < 8) {
            return 20 * (t-6);
        } else {
            return 40;
        }
    }
}