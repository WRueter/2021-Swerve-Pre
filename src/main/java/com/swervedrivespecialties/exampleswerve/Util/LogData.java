// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.exampleswerve.Util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.time.LocalTime;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;
import java.util.concurrent.TimeUnit;


/** Add your docs here. */
public class LogData {

    private long timeMs;
    private long dt;
    private long initTimeMs;
    private PrintWriter _writer;
    private TimeUnit _timeUnit = TimeUnit.MILLISECONDS;

    public LogData() {
    }



    public void log(boolean init, boolean end, String inputName, double input){
        if (init) {
            Date now = new Date();
            initTimeMs = (long) (now.getTime());
            timeMs = (long) 0.0;
            dt = (long) 0.0;
            Calendar cal = Calendar.getInstance();
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
            String fileName = outputFormatter.format(new Date());
            try {
                _writer = new PrintWriter(new File("/media/sda1/" + inputName + fileName + ".tsv"));
                
            } catch (FileNotFoundException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            _writer.append("StartDeltaMS\t" + "LastScanDeltaMS\t" + inputName + "\n"
            + Double.toString(timeMs) + "\t" + Double.toString(dt) + "\t" + Double.toString(input) + "\n");
        } else if(_writer != null) {
            Date now = new Date();
            dt = _timeUnit.convert((long) (now.getTime() - initTimeMs - timeMs), TimeUnit.MILLISECONDS);
            timeMs = _timeUnit.convert((long) (now.getTime() - initTimeMs), TimeUnit.MILLISECONDS);
            _writer.append(Long.toString(timeMs) + "\t" + Long.toString(dt) + "\t" + Double.toString(input) + "\n");
            if (end & _writer != null){
                _writer.close();
            }
        }



    }



}
