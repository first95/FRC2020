import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.jfree.data.xy.XYSeries;

public class Main {
    public static final double DT_S = 0.05;

    public static void main(String[] args) {
        XYSeries actual = new XYSeries("Actual");

        for (double t = 0; t < Stimulus.STIM_DURATION_S; t += DT_S) {
            actual.add(t, Stimulus.GetProcessActual(t));
        }

        ScatterPlotter sp = new ScatterPlotter("Retrospective state estimator test");
        sp.AddSeries(actual);

        sp.PopulateChart("State", "Time(s)", "Value");
        SwingUtilities.invokeLater(() -> {
            sp.setSize(1024, 768);
            sp.setLocationRelativeTo(null);
            sp.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            sp.setVisible(true);
        });

    }
}