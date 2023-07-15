void printLine()
{
  for (byte i = 0; i < 80; i++)
  {
    DEBUG_PRINT("-");
  }
  DEBUG_PRINTLN();
}

void printTab(byte _times)
{
  for (byte i = 0; i < _times; i++)
  {
    DEBUG_PRINT("\t");
  }
}

// Print logging settings
void printLoggingSettings()
{
  printLine();
  DEBUG_PRINTLN("Logging Configuration");
  printLine();

  DEBUG_PRINT("Logging mode: "); printTab(2);
  if (operationMode == 1)
  {
    DEBUG_PRINTLN("Daily");
    char dailyBuffer[100];
    sprintf(dailyBuffer, "Logging start:\t\t%02d:%02d:00 UTC\nLogging stop:\t\t%02d:%02d:00 UTC",
            alarmStartHour, alarmStartMinute, alarmStopHour, alarmStopMinute);
    DEBUG_PRINTLN(dailyBuffer);
  }
  if (operationMode == 2)
  {
    DEBUG_PRINTLN("Rolling");
    char rollingBuffer[100];
    sprintf(rollingBuffer, "Logging duration:\t%d hours %d minutes\nSleep duration:\t\t%d hours %d minutes",
            alarmAwakeHours, alarmAwakeMinutes, alarmSleepHours, alarmSleepMinutes);
    DEBUG_PRINTLN(rollingBuffer);
  }
  if (operationMode == 3)
  {
    DEBUG_PRINTLN("Continuous");
  }
  printLine();
}

// Print values of u-blox registers
void printGnssSettings()
{
  printLine();
  DEBUG_PRINTLN("u-blox ZED-F9P Configuration");
  printLine();

  // Create custom packet
  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes

  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  gnss.newCfgValget(&customCfg, MAX_PAYLOAD_SIZE, VAL_LAYER_RAM); // Create a new VALGET construct
  gnss.addCfgValget(&customCfg, UBLOX_CFG_I2C_ENABLED);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SPI_ENABLED);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART1_ENABLED);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2_ENABLED);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_USB_ENABLED);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART1_BAUDRATE);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2_BAUDRATE);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_I2COUTPROT_UBX);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_I2COUTPROT_RTCM3X);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2OUTPROT_UBX);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2OUTPROT_NMEA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2OUTPROT_RTCM3X);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2INPROT_UBX);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2INPROT_NMEA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_UART2INPROT_RTCM3X);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_RATE_MEAS);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_RATE_NAV);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_GPS_ENA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_GLO_ENA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_GAL_ENA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_BDS_ENA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_QZSS_ENA);
  gnss.addCfgValget(&customCfg, UBLOX_CFG_SIGNAL_SBAS_ENA);

  if (gnss.sendCfgValget(&customCfg)) // Send the VALGET
  {
    // Use a template method to extract the value

    bool i2cEnabled; // Specify the type
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2C_ENABLED, &i2cEnabled, sizeof(i2cEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_I2C_ENABLED: "); printTab(2); DEBUG_PRINTLN(i2cEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool spiEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SPI_ENABLED, &spiEnabled, sizeof(spiEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SPI_ENABLED: "); printTab(2); DEBUG_PRINTLN(spiEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool uart1Enabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART1_ENABLED, &uart1Enabled, sizeof(uart1Enabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART1_ENABLED: "); printTab(1); DEBUG_PRINTLN(uart1Enabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool uart2Enabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2_ENABLED, &uart2Enabled, sizeof(uart2Enabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2_ENABLED: ");  printTab(1); DEBUG_PRINTLN(uart2Enabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool usbEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_USB_ENABLED, &usbEnabled, sizeof(usbEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_USB_ENABLED: "); printTab(2); DEBUG_PRINTLN(usbEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    uint16_t rateMeas;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_RATE_MEAS, &rateMeas, sizeof(rateMeas)))
    {
      DEBUG_PRINT("UBLOX_CFG_RATE_MEAS: "); printTab(2); DEBUG_PRINTLN(rateMeas);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    uint16_t rateNav;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_RATE_NAV, &rateNav, sizeof(rateNav)))
    {
      DEBUG_PRINT("UBLOX_CFG_RATE_NAV: "); printTab(2); DEBUG_PRINTLN(rateNav);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    uint32_t uart1Baud;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART1_BAUDRATE, &uart1Baud, sizeof(uart1Baud)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART1_BAUDRATE: "); printTab(1); DEBUG_PRINTLN(uart1Baud);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    uint32_t uart2Baud;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2_BAUDRATE, &uart1Baud, sizeof(uart1Baud)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2_BAUDRATE: "); printTab(1); DEBUG_PRINTLN(uart1Baud);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool i2cOutProtUbx;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2COUTPROT_UBX, &i2cOutProtUbx, sizeof(i2cOutProtUbx)))
    {
      DEBUG_PRINT("UBLOX_CFG_I2COUTPROT_UBX: "); printTab(1); DEBUG_PRINTLN(i2cOutProtUbx);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool i2cOutProtNmea;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2COUTPROT_NMEA, &i2cOutProtNmea, sizeof(i2cOutProtNmea)))
    {
      DEBUG_PRINT("UBLOX_CFG_I2COUTPROT_NMEA: "); printTab(1); DEBUG_PRINTLN(i2cOutProtNmea);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool i2cOutProtRtcm;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_I2COUTPROT_RTCM3X, &i2cOutProtRtcm, sizeof(i2cOutProtRtcm)))
    {
      DEBUG_PRINT("UBLOX_CFG_I2COUTPROT_RTCM3X: "); printTab(1); DEBUG_PRINTLN(i2cOutProtRtcm);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool uart2OutProtUbx;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2OUTPROT_UBX, &uart2OutProtUbx, sizeof(uart2OutProtUbx)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2OUTPROT_UBX: "); printTab(1); DEBUG_PRINTLN(uart2OutProtUbx);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool uart2OutProtNmea;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2OUTPROT_NMEA, &uart2OutProtNmea, sizeof(uart2OutProtNmea)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2OUTPROT_NMEA: "); printTab(1); DEBUG_PRINTLN(uart2OutProtNmea);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool uart2OutProtRtcm;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2OUTPROT_RTCM3X, &uart2OutProtRtcm, sizeof(uart2OutProtRtcm)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2OUTPROT_RTCM3X: "); printTab(1); DEBUG_PRINTLN(uart2OutProtRtcm);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool uart2InProtUbx;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2INPROT_UBX, &uart2InProtUbx, sizeof(uart2InProtUbx)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2INPROT_UBX: "); printTab(1); DEBUG_PRINTLN(uart2InProtUbx);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool uart2InProtNmea;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2INPROT_NMEA, &uart2InProtNmea, sizeof(uart2InProtNmea)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2INPROT_NMEA: "); printTab(1); DEBUG_PRINTLN(uart2InProtNmea);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool uart2InProtRtcm;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_UART2INPROT_RTCM3X, &uart2InProtRtcm, sizeof(uart2InProtRtcm)))
    {
      DEBUG_PRINT("UBLOX_CFG_UART2INPROT_RTCM3X: "); printTab(1); DEBUG_PRINTLN(uart2InProtRtcm);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }

    bool gpsEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_GPS_ENA, &gpsEnabled, sizeof(gpsEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_GPS_ENA: "); printTab(1); DEBUG_PRINTLN(gpsEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool gloEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_GLO_ENA, &gloEnabled, sizeof(gloEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_GLO_ENA: "); printTab(1); DEBUG_PRINTLN(gloEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool galEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_GAL_ENA, &galEnabled, sizeof(galEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_GAL_ENA: "); printTab(1); DEBUG_PRINTLN(galEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool bdsEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_BDS_ENA, &bdsEnabled, sizeof(bdsEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_BDS_ENA: "); printTab(1); DEBUG_PRINTLN(bdsEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool qzssEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_QZSS_ENA, &qzssEnabled, sizeof(qzssEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_QZSS_ENA: "); printTab(1); DEBUG_PRINTLN(qzssEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
    bool sbasEnabled;
    if (gnss.extractConfigValueByKey(&customCfg, UBLOX_CFG_SIGNAL_SBAS_ENA, &sbasEnabled, sizeof(sbasEnabled)))
    {
      DEBUG_PRINT("UBLOX_CFG_SIGNAL_SBAS_ENA: "); printTab(1); DEBUG_PRINTLN(sbasEnabled);
    }
    else
    {
      DEBUG_PRINTLN("extractConfigValueByKey failed!");
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning - VALGET failed!");
  }

  printLine();
}

void clearTimers()
{
  // Clear timer structure
  memset(&timer, 0x00, sizeof(timer));
}

void printTimers()
{
  printLine();
  DEBUG_PRINTLN("Function Execution Timers");
  printLine();
  DEBUG_PRINT("readWdt: ");     printTab(1);  DEBUG_PRINTLN(timer.wdt);
  DEBUG_PRINT("readRtc: ");     printTab(1);  DEBUG_PRINTLN(timer.rtc);
  DEBUG_PRINT("microSD: ");     printTab(1);  DEBUG_PRINTLN(timer.microSd);
  DEBUG_PRINT("voltage: ");     printTab(1);  DEBUG_PRINTLN(timer.voltage);
  DEBUG_PRINT("gnss: ");        printTab(2);  DEBUG_PRINTLN(timer.gnss);
  DEBUG_PRINT("syncRtc: ");     printTab(1);  DEBUG_PRINTLN(timer.syncRtc);
  DEBUG_PRINT("logDebug: ");    printTab(1);  DEBUG_PRINTLN(timer.logDebug);
  DEBUG_PRINT("logGnss: ");     printTab(1);  DEBUG_PRINTLN(timer.logGnss);
  printLine();
}
