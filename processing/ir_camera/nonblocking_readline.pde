public interface INonblockingReadLine {
  public String readLine();
  public void close();
  public void clear(); // whack any queued lines
  public int size(); // in que
  public void write(String x);
  public void write(char x);
}

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.io.Writer;

public class NonblockingBufferedReadLine implements INonblockingReadLine {
  private final BlockingQueue<String> lines = new LinkedBlockingQueue<String>();
  private volatile boolean closed = false;
  private Writer out = null;
  private Thread backgroundReaderThread = null;

  public NonblockingBufferedReadLine(final BufferedReader bufferedReader, Writer out) {
    backgroundReaderThread = new Thread(new Runnable() {
      @Override
        public void run() {
        try {
          while (!Thread.interrupted()) {
            String line = bufferedReader.readLine();
            if (line == null) {
              break;
            }
            lines.add(line);
          }
        } 
        catch (IOException e) {
          throw new RuntimeException(e);
        } 
        finally {
          try {
            bufferedReader.close();
          }
          catch (IOException e) {
            println("Couldn't close the buffered reader");
            println(e);
          } 
          closed = true;
        }
      }
    }
    );
    this.out = out;
    backgroundReaderThread.setDaemon(true);
    backgroundReaderThread.start();
  }

  public String readLine() {
    try {
      return closed || lines.isEmpty() ? null : lines.take();
    } 
    catch (InterruptedException e) {
      println("The BackgroundReaderThread was interrupted!", e);
      return null;
    }
  }

  public void write(String x) {
    try {
      out.write(x);
    }
    catch (IOException e) {
      println("Couldn't close the buffered reader");
      println(e);
    }
  }
  public void write(char x) {
    try {
      out.write(x);
    }
    catch (IOException e) {
      println("Couldn't close the buffered reader");
      println(e);
    }
  }

  public void clear() {
    lines.clear();
  }

  public int size() {
    return lines.size();
  }

  public void close() {
    if (backgroundReaderThread != null) {
      backgroundReaderThread.interrupt();
      backgroundReaderThread = null;
    }
  }
}

public class NonblockingSerialReadLine implements INonblockingReadLine {
  private final BlockingQueue<String> lines = new LinkedBlockingQueue<String>();
  private volatile boolean closed = false;
  private Thread backgroundReaderThread = null;
  private Serial serial;

  public NonblockingSerialReadLine(final Serial bufferedReader) {
    this.serial = bufferedReader;
    bufferedReader.buffer(1024);
    bufferedReader.bufferUntil(10); // lf

    backgroundReaderThread = new Thread(new Runnable() {
      @Override
        public void run() {
        try {
          while (!Thread.interrupted()) {
            String line = bufferedReader.readStringUntil(10); // lf
            if (line == null) {
              break;
            }
            if (line.contains("[") ) {
              lines.add(line);
            }
          }
        } 
        //catch (IOException e) {
        //  throw new RuntimeException(e);
        //} 
        finally {
          bufferedReader.stop();
          closed = true;
        }
      }
    }
    );
    backgroundReaderThread.setDaemon(true);
    backgroundReaderThread.start();
  }

  public String readLine() {
    try {
      return closed || lines.isEmpty() ? null : lines.take();
    } 
    catch (InterruptedException e) {
      println("The BackgroundReaderThread was interrupted!", e);
      return null;
    }
  }

  public void write(String x) {
    this.serial.write(x);
  }
  public void write(char x) {
    this.serial.write(x);
  }

  public void clear() {
    lines.clear();
  }

  public int size() {
    return lines.size();
  }

  public void close() {
    if (backgroundReaderThread != null) {
      backgroundReaderThread.interrupt();
      backgroundReaderThread = null;
    }
  }
}
