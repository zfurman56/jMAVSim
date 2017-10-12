package me.zfurman.jmavsim;

import java.io.IOException;
import javax.swing.SwingUtilities;
import me.drton.jmavsim.AbstractSimulator;

public class Main {
   
    public static void main(String[] args)
        throws InterruptedException, IOException {
            AbstractSimulator.parseArguments(args);
            SwingUtilities.invokeLater(new RocketSimulator());
    }

}