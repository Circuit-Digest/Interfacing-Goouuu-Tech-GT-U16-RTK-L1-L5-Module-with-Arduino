#define GPSSerial   Serial
#define DEBUGSerial Serial

const unsigned long PRINT_INTERVAL = 1000;

// GNSS Data
String utcTime = "--:--:--";
String localTime = "--:--:--";
String latitude = "--";
String longitude = "--";
String altitude = "--";
String fixQuality = "--";
String speedKmh = "--";

// Satellite Signal Info
int gpsCount = 0, beidouCount = 0, galileoCount = 0, qzssCount = 0, glonassCount = 0;
String snrList = "";
String nmeaBuffer = "";

void setup() {
  GPSSerial.begin(115200);
  DEBUGSerial.begin(115200);
  DEBUGSerial.println("Waiting for GNSS data...");
}

void loop() {
  readGNSSData();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    printGNSSInfo();
    resetCounters();
  }
}

// ===================== GNSS Data Parsing =======================

void readGNSSData() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    if (c == '\n') {
      processNMEALine(nmeaBuffer);
      nmeaBuffer = "";
    } else if (c != '\r') {
      nmeaBuffer += c;
    }
  }
}

void processNMEALine(const String &line) {
  if (!line.startsWith("$")) return;

  if (line.startsWith("$GPGSV") || line.startsWith("$GBGSV") ||
      line.startsWith("$GAGSV") || line.startsWith("$GQGSV") ||
      line.startsWith("$GLGSV")) {
    parseGSVLine(line);
  } else if (line.startsWith("$GNRMC")) {
    parseRMC(line);
  } else if (line.startsWith("$GNGGA")) {
    parseGGA(line);
  }
}

void parseRMC(const String &line) {
  int commas[13], idx = -1;
  for (int i = 0; i < 13; i++) {
    idx = line.indexOf(',', idx + 1);
    if (idx == -1) break;
    commas[i] = idx;
  }

  if (commas[1] - commas[0] > 1) {
    String rawTime = line.substring(commas[0] + 1, commas[1]);
    int hour = rawTime.substring(0, 2).toInt();
    int minute = rawTime.substring(2, 4).toInt();
    int second = rawTime.substring(4, 6).toInt();

    utcTime = formatTime(hour, minute, second);

    // Local time adjustment (+5:30)
    minute += 30;
    if (minute >= 60) {
      minute -= 60;
      hour += 1;
    }
    hour = (hour + 5) % 24;
    localTime = formatTime(hour, minute, second);
  }

  if (commas[7] > commas[6]) {
    float knots = line.substring(commas[6] + 1, commas[7]).toFloat();
    speedKmh = String(knots * 1.852, 1);
  }
}

void parseGGA(const String &line) {
  int commas[15], idx = -1;
  for (int i = 0; i < 15; i++) {
    idx = line.indexOf(',', idx + 1);
    if (idx == -1) break;
    commas[i] = idx;
  }

  String latVal = line.substring(commas[1] + 1, commas[2]);
  String latDir = line.substring(commas[2] + 1, commas[3]);
  String lonVal = line.substring(commas[3] + 1, commas[4]);
  String lonDir = line.substring(commas[4] + 1, commas[5]);

  latitude = convertLatLon(latVal, latDir);
  longitude = convertLatLon(lonVal, lonDir);

  fixQuality = line.substring(commas[5] + 1, commas[6]);
  altitude = line.substring(commas[8] + 1, commas[9]) + " m";
}

String convertLatLon(const String &val, const String &dir) {
  if (val.length() < 4) return "--";
  int degLen = (dir == "N" || dir == "S") ? 2 : 3;
  float degrees = val.substring(0, degLen).toFloat();
  float minutes = val.substring(degLen).toFloat();
  float decimal = degrees + (minutes / 60.0);

  if (dir == "S" || dir == "W") decimal *= -1;
  return String(decimal, 6);
}

String formatTime(int h, int m, int s) {
  char buffer[9];
  sprintf(buffer, "%02d:%02d:%02d", h, m, s);
  return String(buffer);
}

String describeFixQuality(const String &code) {
  if (code == "0") return "0 → Not fixed";
  if (code == "1") return "1 → Standalone";
  if (code == "2") return "2 → DGPS Fix";
  if (code == "3") return "3 → PPS Fix";
  if (code == "4") return "4 → RTK Fixed";
  if (code == "5") return "5 → RTK Float";
  return code + " → Unknown";
}

// ====================== Satellite Info ==========================

void parseGSVLine(const String &line) {
  if (line.startsWith("$GPGSV")) gpsCount += countSatellitesInLine(line);
  else if (line.startsWith("$GBGSV")) beidouCount += countSatellitesInLine(line);
  else if (line.startsWith("$GAGSV")) galileoCount += countSatellitesInLine(line);
  else if (line.startsWith("$GQGSV")) qzssCount += countSatellitesInLine(line);
  else if (line.startsWith("$GLGSV")) glonassCount += countSatellitesInLine(line);

  int fieldStart = 0, fieldEnd = 0, fieldCount = 0;
  while ((fieldEnd = line.indexOf(',', fieldStart)) != -1) {
    String field = line.substring(fieldStart, fieldEnd);
    fieldStart = fieldEnd + 1;
    fieldCount++;

    if (fieldCount >= 7 && ((fieldCount - 7) % 4 == 0)) {
      appendSNR(field);
    }
  }

  fieldEnd = line.indexOf('*');
  if (fieldEnd != -1 && fieldStart < fieldEnd) {
    appendSNR(line.substring(fieldStart, fieldEnd));
  }
}

int countSatellitesInLine(const String &line) {
  int count = 0, idx = -1;
  for (int i = 0; i < 20; i++) {
    idx = line.indexOf(',', idx + 1);
    if (idx == -1) break;
    if (i >= 4 && (i - 4) % 4 == 0) count++;
  }
  return count;
}

void appendSNR(const String &field) {
  if (field.length() > 0 && isDigit(field[0])) {
    int snr = field.toInt();
    if (snr > 0 && snr <= 99) {
      snrList += String(snr) + " dB  ";
    }
  }
}

void summarizeSNR() {
  int excellent = 0, good = 0, fair = 0, weak = 0, invalid = 0;
  int start = 0;

  while (start >= 0 && start < snrList.length()) {
    int end = snrList.indexOf(" dB", start);
    if (end == -1) break;
    int val = snrList.substring(start, end).toInt();

    if (val >= 40) excellent++;
    else if (val >= 30) good++;
    else if (val >= 20) fair++;
    else if (val >= 1) weak++;
    else invalid++;

    start = snrList.indexOf("  ", end);
    if (start == -1) break;
    start += 2;
  }

  DEBUGSerial.println("---- Signal Strength Summary ----");
  DEBUGSerial.print("Excellent (≥40 dB) : "); DEBUGSerial.println(excellent);
  DEBUGSerial.print("Good     (30–39 dB): "); DEBUGSerial.println(good);
  DEBUGSerial.print("Fair     (20–29 dB): "); DEBUGSerial.println(fair);
  DEBUGSerial.print("Weak     (1–19 dB) : "); DEBUGSerial.println(weak);
  DEBUGSerial.print("Invalid (0 or N/A) : "); DEBUGSerial.println(invalid);
}

// ======================= Display ===========================

void printGNSSInfo() {
  DEBUGSerial.println("\n====== GNSS INFO ======");
  DEBUGSerial.print("Fix Quality: "); DEBUGSerial.println(describeFixQuality(fixQuality));

  if (fixQuality != "0" && fixQuality != "") {
    DEBUGSerial.print("UTC Time   : "); DEBUGSerial.println(utcTime);
    DEBUGSerial.print("Local Time : "); DEBUGSerial.println(localTime);
    DEBUGSerial.print("Latitude   : "); DEBUGSerial.println(latitude);
    DEBUGSerial.print("Longitude  : "); DEBUGSerial.println(longitude);
    DEBUGSerial.print("Speed      : "); DEBUGSerial.print(speedKmh); DEBUGSerial.println(" km/h");
    DEBUGSerial.print("Altitude   : "); DEBUGSerial.println(altitude);
  } else {
    DEBUGSerial.println("UTC Time   : --");
    DEBUGSerial.println("Local Time : --");
    DEBUGSerial.println("Latitude   : --");
    DEBUGSerial.println("Longitude  : --");
    DEBUGSerial.println("Speed      : --");
    DEBUGSerial.println("Altitude   : --");
  }

  DEBUGSerial.println("---- Satellites Visible ----");
  DEBUGSerial.print("GPS(USA)        : "); DEBUGSerial.println(gpsCount);
  DEBUGSerial.print("BeiDou(China)   : "); DEBUGSerial.println(beidouCount);
  DEBUGSerial.print("Galileo(Europe) : "); DEBUGSerial.println(galileoCount);
  DEBUGSerial.print("QZSS(Japan)     : "); DEBUGSerial.println(qzssCount);
  DEBUGSerial.print("GLONASS(Russia) : "); DEBUGSerial.println(glonassCount);

  summarizeSNR();
  DEBUGSerial.println("===================================");
}

void resetCounters() {
  gpsCount = beidouCount = galileoCount = qzssCount = glonassCount = 0;
  snrList = "";
}
