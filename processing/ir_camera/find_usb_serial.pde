import processing.serial.*;

boolean first_time = true;
Serial connectUSBSerial(int baud) {
  // return a Serial object that we think is the usb-serial (arduino), or null

  /*
  File fifo = new File("/home/awgrover/remote-serial");
   boolean exists = fifo.exists();
   println("remote-serial exists? ", exists);
   if (exists) { 
   return fifo;
   }
   */

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
      || flipDotPorts[i].contains("tty. usbmodem") // windows?
      ) { 
      arduinoPortName = flipDotPorts[i];
    }
  }

  Serial found = null;

  if (arduinoPortName != null) {
    println("(can take a few seconds...)");
    try {
      found = new Serial( this, arduinoPortName, baud);
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
