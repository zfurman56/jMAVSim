package me.drton.jmavsim.vehicle;

import java.io.*;

/**
 * Loads a thrust curve from a .eng file, and finds thrust at a
 * given time using linear interpolation.
 */
public class RocketEngine {
    private double [][] data = new double[31][2];

    public RocketEngine(String file) {

        try {
            BufferedReader br = new BufferedReader(new FileReader((System.getProperty("user.dir")+file)));  
            String line = null;
            int i = 0;
            while ((line = br.readLine()) != null) {
                // Lines starting with a semicolon are comments
                if (line.charAt(0) != ';') {
                    // Ignore the first line - it contains motor information, not thrust data
                    if (i>0) {
                        data[i] = new double[]{Double.parseDouble(line.split("\\s+")[0]), Double.parseDouble(line.split("\\s+")[1])};
                    }
                    i++;
                }
            }
        } catch (IOException e) {
            // Usually some form of a file not found error
            e.printStackTrace();
            System.exit(-1);
        }
    }

    // Get engine thrust at a given time
    public double thrust(double time) {

        // If the rocket hasn't launched yet, return zero thrust
        if (time<data[0][0]) {
            return 0.0;
        }

        for(int i = 0; i<(data.length-1); i++) {
            if (time == data[i][0]) {
                return data[i][0];
            } else if (time < data[i+1][0]) {
                return interpolate(data[i][0], data[i][1], data[i+1][0], data[i+1][1], time);
            }
        }

        // If the engine has burned out, return zero thrust
        return 0.0;

    }
    
    // Linear interpolation for x between the points (x1, y1) and (x2, y2)
    private static double interpolate(double x1, double y1, double x2, double y2, double x) {
        return y1 + ((x - x1) * ((y2 - y1) / (x2 - x1)));
    }

}