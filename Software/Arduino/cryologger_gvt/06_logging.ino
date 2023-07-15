// Create timestamped log file name
void getLogFileName()
{
  sprintf(logFileName, "%s_20%02d%02d%02d_%02d%02d%02d.ubx",
          UID, rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds);

  DEBUG_PRINT("Info - logFileName: "); DEBUG_PRINTLN(logFileName);
}

// Create debugging log file
void createDebugFile()
{
  // Debug log file name
  sprintf(debugFileName, "%s_debug.csv", UID);

  // Create debug log filewritte
  // O_CREAT - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE - Open the file for writing
  if (!debugFile.open(debugFileName, O_CREAT | O_APPEND | O_WRITE))
  {
    DEBUG_PRINTLN("Warning - Failed to create debug file.");
    return;
  }
  else
  {
    DEBUG_PRINT("Info - Created "); DEBUG_PRINTLN(debugFileName);
  }

  // Write header to file
  debugFile.println("datetime,battery,online_microsd,online_gnss,online_log_gnss,online_log_debug,"
                    "timer_battery,timer_microsd,timer_gnss,timer_sync_rtc,timer_log_gnss,timer_log_debug,"
                    "rtc_sync_flag,rtc_drift,bytes_written,max_buffer_bytes,counter_sfrbx,counter_rawx,counter_wdt_max,"
                    "write_fail_counter,sync_fail_counter,close_fail_counter,debug_counter");

  // Sync the debug file
  if (!debugFile.sync())
  {
    DEBUG_PRINTLN("Warning - Failed to sync debug file.");
  }

  // Update the file create timestamp
  updateFileCreate(&debugFile);

  // Close log file
  if (!debugFile.close())
  {
    DEBUG_PRINTLN("Warning - Failed to close debug file.");
  }
}

// Log debugging information
void logDebug()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Increment debug counter
  counterDebug++;

  // Open debug file for writing
  if (!debugFile.open(debugFileName, O_APPEND | O_WRITE))
  {
    DEBUG_PRINTLN("Warning - Failed to open debug file.");
    online.logDebug = false; // Set flag
    return;
  }
  else
  {
    DEBUG_PRINT("Info - Opened "); DEBUG_PRINTLN(debugFileName);
    online.logDebug = true; // Set flag
  }

  // Create datetime string
  char dateTime[30];
  sprintf(dateTime, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds);

  // Log debugging information
  debugFile.print(dateTime);            debugFile.print(",");
  debugFile.print(readVoltage());       debugFile.print(",");
  debugFile.print(online.microSd);      debugFile.print(",");
  debugFile.print(online.gnss);         debugFile.print(",");
  debugFile.print(online.logGnss);      debugFile.print(",");
  debugFile.print(online.logDebug);     debugFile.print(",");
  debugFile.print(timer.voltage);       debugFile.print(",");
  debugFile.print(timer.microSd);       debugFile.print(",");
  debugFile.print(timer.gnss);          debugFile.print(",");
  debugFile.print(timer.syncRtc);       debugFile.print(",");
  debugFile.print(timer.logGnss);       debugFile.print(",");
  debugFile.print(timer.logDebug);      debugFile.print(",");
  debugFile.print(flagRtcSync);         debugFile.print(",");
  debugFile.print(rtcDrift);            debugFile.print(",");
  debugFile.print(bytesWritten);        debugFile.print(",");
  debugFile.print(maxBufferBytes);      debugFile.print(",");
  debugFile.print(counterSfrbx);        debugFile.print(",");
  debugFile.print(counterRawx);         debugFile.print(",");
  debugFile.print(counterWdtMax);       debugFile.print(",");
  debugFile.print(counterSdWriteFail);    debugFile.print(",");
  debugFile.print(counterSdSyncFail);     debugFile.print(",");
  debugFile.print(counterSdCloseFail);    debugFile.print(",");
  debugFile.println(counterDebug);

  // Sync the debug file
  if (!debugFile.sync())
  {
    DEBUG_PRINTLN("Warning - Failed to sync debug file.");
    counterSdSyncFail++; // Count number of failed file syncs
  }

  // Update file access timestamps
  updateFileAccess(&debugFile);

  // Close the debug file
  if (!debugFile.close())
  {
    DEBUG_PRINTLN("Warning - Failed to close debug file.");
    counterSdCloseFail++; // Count number of failed file closes
  }

  DEBUG_PRINT("Info - Wrote to debug file: ");

  // Print debugging information
  DEBUG_PRINT(dateTime);          DEBUG_PRINT(",");
  DEBUG_PRINT(readVoltage());     DEBUG_PRINT(",");
  DEBUG_PRINT(online.microSd);    DEBUG_PRINT(",");
  DEBUG_PRINT(online.gnss);       DEBUG_PRINT(",");
  DEBUG_PRINT(online.logGnss);    DEBUG_PRINT(",");
  DEBUG_PRINT(online.logDebug);   DEBUG_PRINT(",");
  DEBUG_PRINT(timer.voltage);     DEBUG_PRINT(",");
  DEBUG_PRINT(timer.microSd);     DEBUG_PRINT(",");
  DEBUG_PRINT(timer.gnss);        DEBUG_PRINT(",");
  DEBUG_PRINT(timer.syncRtc);     DEBUG_PRINT(",");
  DEBUG_PRINT(timer.logGnss);     DEBUG_PRINT(",");
  DEBUG_PRINT(timer.logDebug);    DEBUG_PRINT(",");
  DEBUG_PRINT(flagRtcSync);       DEBUG_PRINT(",");
  DEBUG_PRINT(rtcDrift);          DEBUG_PRINT(",");
  DEBUG_PRINT(bytesWritten);      DEBUG_PRINT(",");
  DEBUG_PRINT(maxBufferBytes);    DEBUG_PRINT(",");
  DEBUG_PRINT(counterSfrbx);      DEBUG_PRINT(",");
  DEBUG_PRINT(counterRawx);       DEBUG_PRINT(",");
  DEBUG_PRINT(counterWdtMax);     DEBUG_PRINT(",");
  DEBUG_PRINT(counterSdWriteFail);  DEBUG_PRINT(",");
  DEBUG_PRINT(counterSdSyncFail);   DEBUG_PRINT(",");
  DEBUG_PRINT(counterSdCloseFail);  DEBUG_PRINT(",");
  DEBUG_PRINTLN(counterDebug);

  // Stop the loop timer
  timer.logDebug = millis() - loopStartTime;
}
