# Hexapod Hardware Connection
#
# Gaurav Manek
import serial, struct;
import binascii;
import time;

TYPE_POSE_UPDATE = 42;

class SerialLink(object):
    def __enter__(self):
        return self;
        
    def __exit__(self, typ, value, traceback):
        self.ser.close();

    
    def __init__(self, config, hexa, port):
        self.legs = config.getLegs();
        # self.rad2enc = lambda sf, v: config.rad2enc(v);
        self.rad2enc = config.rad2enc;
        self.hexa = hexa;
        self.ser = serial.Serial(port, 38400, timeout=1);
        self.angle_invert = [1, -1, 1];
        time.sleep(1);
        if not self.ser:
            raise ValueError("Cannot connect to robot on given port.")
        
    # Note: bytes are packed as little-endian.
    def tick(self):
        # Get the hexapod state:
        st = self.hexa.getRawState();
        vals = [self.angle_invert[seg_no] * self.rad2enc(sa) for l in self.legs for seg_no, sa in enumerate(st[l])];
        wbuf = struct.pack('<' + 'h'*(3*len(self.legs)), *vals);
        checksum = 0
        for el in wbuf:
            checksum ^= ord(el)
        pkt = TYPE_POSE_UPDATE;
        checksum ^= pkt;
        sig = struct.pack('BB', pkt, checksum);
        
        self.ser.write(sig);
        self.ser.write(wbuf);
        self.ser.flush();
        
        def chksum(s):
            c = 0;
            for b in s:
                c ^= ord(b);
            return c;
        

        # Check received messages:
        pending = self.ser.inWaiting();
        if pending:
            p = self.ser.read(pending)
            print p;

            
            
        
        
        