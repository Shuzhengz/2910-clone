package com.team1678.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * The logger class
 * Logs robot status and things
 */
public class Logger {

    private Logger() {
        // The logger method
    }

    /**
     * Writes logs to log.txt
     *
     * @param marks The things to be logged
     * @param print If wants to println the writer
     */
    public static void log(String marks, boolean print) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/log.txt", true))) {

            writer.print(marks);

             if (print) {
                 writer.println();
             }

        } catch (IOException logFail) {
            logFail.printStackTrace();
        }
    }

    /**
     * Clears the existing logs in log.txt
     */
    public static void clearLog() {
        try {
            java.lang.Runtime.getRuntime().exec("/bin/rm -f /home/lvuser/log.txt");
        } catch (IOException logFailClear) {
            logFailClear.printStackTrace();
        }
    }

}
