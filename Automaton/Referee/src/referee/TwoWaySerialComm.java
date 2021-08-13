package referee;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

public class TwoWaySerialComm {

    static final int PING_RATE = 3000;
    static SerialPort serialPort;
    static int numMissedPings = 0;
    static int lastNonce = 0;
    static Timer oobTimer;

    public TwoWaySerialComm() {
        super();
    }

    void connect(String portName) throws Exception {
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if (portIdentifier.isCurrentlyOwned()) {
            System.err.println("Error acquiring port (currently in use)!");
        } else {
            CommPort commPort = portIdentifier.open(this.getClass().getName(), 2000);
            if (commPort instanceof SerialPort) {
                serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
                InputStream in = serialPort.getInputStream();
                OutputStream out = serialPort.getOutputStream();
                (new Thread(new SerialWriter(out))).start();
                serialPort.addEventListener(new SerialReader(in));
                serialPort.notifyOnDataAvailable(true);
            } else {
                System.err.println("Error identifying port type (not serial port)!");
            }
        }
    }

    public static class SerialReader implements SerialPortEventListener {

        private final InputStream in;
        private final byte[] buffer = new byte[1024];

        public SerialReader(InputStream in) {
            this.in = in;
        }

        @Override
        public void serialEvent(SerialPortEvent arg0) {
            int data;
            try {
                int len = 0;
                while ((data = in.read()) > -1) {
                    if ((data == '\r') || (data == '\n')) {
                        break;
                    }
                    buffer[len++] = (byte) data;
                }
                String rxStr = new String(buffer, 0, len);
                System.out.println(rxStr);
                if (rxStr.compareTo("Motors: 0 0") == 0) {
                    System.out.println("controller obeyed out-of-bounds message");
                    oobTimer.cancel();
                }
                String idStr = rxStr.substring(4, 6);
                int id = Integer.decode("0x" + idStr);
                if (id == 0x51) {
                    String nonceStr = rxStr.substring(7, 9);
                    nonceStr = nonceStr.concat(rxStr.substring(10, 12));
                    int nonce = Integer.decode("0x" + nonceStr);
                    if (nonce == lastNonce + 1) {
                        System.out.println("valid pong received");
                        numMissedPings = 0;
                    } else {
                        System.err.println("-- invalid pong received");
                    }
                }
            } catch (IOException e) {
                System.err.println("Error reading from serial port!");
                System.exit(-1);
            }
        }
    }

    public static class SerialWriter implements Runnable {

        OutputStream out;

        public SerialWriter(OutputStream out) {
            this.out = out;
        }

        @Override
        public void run() {
            try {
                int c;
                while ((c = System.in.read()) > -1) {
                    if ((char) c == 'o') {
                        byte[] oob = new byte[7];
                        Random random = new Random(13);
                        random.nextBytes(oob);
                        oob[0] = 0x61;
                        System.out.printf("Sending OOB message: %02x %02x %02x %02x %02x %02x %02x\r\n", oob[0], oob[1], oob[2], oob[3], oob[4], oob[5], oob[6]);
                        out.write(oob);
                        out.write('\r');
                        TimerTask oobTimerTask = new TimerTask() {
                            @Override
                            public void run() {
                                System.err.println("-- controller ignored out-of-bounds message");
                            }
                        };
                        oobTimer = new Timer();
                        oobTimer.schedule(oobTimerTask, 100);
                    }
                }
            } catch (IOException e) {
                System.err.println("Error writing to serial port!");
                System.exit(-1);
            }
        }
    }

    public static void main(String[] args) {
        try {
            (new TwoWaySerialComm()).connect("COM3");
        } catch (Exception e) {
            System.err.println("Error connecting to serial port!");
            System.exit(-1);
        }
        Timer pingTimer = new Timer();
        Random random = new Random(0x1337);
        TimerTask pingTimerTask = new TimerTask() {
            @Override
            public void run() {
                try {
                    OutputStream out = serialPort.getOutputStream();
                    byte[] nonce = new byte[2];
                    random.nextBytes(nonce);
                    lastNonce = Byte.toUnsignedInt(nonce[0]) << 8;
                    lastNonce |= Byte.toUnsignedInt(nonce[1]);
                    byte[] pingMsg = {0x50, nonce[0], nonce[1]};
                    System.out.printf("Sending ping: %02x %02x %02x\r\n", pingMsg[0], pingMsg[1], pingMsg[2]);
                    out.write(pingMsg);
                    out.write('\r');
                    numMissedPings++;
                    if (numMissedPings == 4) {
                        System.err.println("-- missed three consecutive pings");
                        numMissedPings = 0;
                    }
                } catch (IOException ex) {
                    System.err.println("Error getting output stream!");
                }
            }
        };
        pingTimer.scheduleAtFixedRate(pingTimerTask, 0, PING_RATE);
    }
}
