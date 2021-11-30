import processing.serial.*;
// import java.io.FileReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

boolean first_time = true;

INonblockingReadLine connectUSBSerial(int baud) {
  // Return a presumed process run by ./remote-serial
  // return a Serial object that we think is the usb-serial (arduino), 
  // or null

  
  File cmd_script = new File( sketchPath("remote-serial"));
  boolean exists = cmd_script.exists();
  println("remote-serial exists? ", cmd_script.getAbsolutePath()," ",exists);
  if (exists) {
    ProcessBuilder builder = new ProcessBuilder(cmd_script.getAbsolutePath());
    builder.redirectErrorStream(true);
    try {
      Process process = builder.start();                
      OutputStream stdin = process.getOutputStream ();
      InputStream stderr = process.getErrorStream ();
      InputStream stdout = process.getInputStream ();
      BufferedReader br = new BufferedReader (new InputStreamReader(stdout));    
      NonblockingBufferedReadLine nbrl = new NonblockingBufferedReadLine(br, new OutputStreamWriter(stdin));
      return nbrl;
    }
    catch (IOException e) {
      println("Failed to run ./remote-serial");
      println(e);
      return null;
    }
  }


  String[] flipDotPorts = Serial.list();
  if (first_time) println(flipDotPorts);
  String arduinoPortName = null;

  for (int i = 0; i < flipDotPorts.length; i++) {
    if (
      // guess, based on historical names of ports
      // We are taking the first 1
      flipDotPorts[i].contains("/dev/ttyUSB") // "/dev/ACM") // linux pi-serial
      || flipDotPorts[i].contains("/dev/ttyACM") // linux arduino
      || flipDotPorts[i].contains("cu.usbmodem") // mac
      || flipDotPorts[i].contains("tty.usbmodem") // windows?
      ) { 
      arduinoPortName = flipDotPorts[i];
    }
  }

  INonblockingReadLine found = null;

  if (arduinoPortName != null) {
    println("(can take a few seconds...)");
    try {
      found = new NonblockingSerialReadLine(new Serial( this, arduinoPortName, baud));
      print("Connected to ");
      println(arduinoPortName);
    }
    catch (RuntimeException e) {
      if (first_time) {
        print("Failed to open ");
        println(e);
        println("Will keep trying");
      }
    }
  }

  if (first_time && found==null) println("Failed to find an usb serial");
  first_time = false;
  return found;
}
