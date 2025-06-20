#define GPSSerial  Serial
#define DEBUGSerial Serial


void setup()	
{
  GPSSerial.begin(115200);			
  DEBUGSerial.begin(115200);  
  DEBUGSerial.println("Waiting for GNSS data...");
}

void loop()		
{
  while (GPSSerial.available()) {   
     DEBUGSerial.write(GPSSerial.read());
  }
}

