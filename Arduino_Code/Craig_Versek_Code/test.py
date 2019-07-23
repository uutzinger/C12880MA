import os, glob, serial
from pylab import *

class MicroSpec(object):
    def __init__(self, port):
        self._ser = serial.Serial(port,baudrate=115200)
    def set_integration_time(self, seconds):
        cmd = "SPEC.INTEG %0.6f\n" % seconds
        self._ser.write(cmd.encode('utf8'))
    def read(self):
        self._ser.write(b"SPEC.READ?\n")
        sdata = self._ser.readline()
        sdata = array([int(p) for p in sdata.split(b",")])
        self._ser.write(b"SPEC.TIMING?\n")
        tdata = self._ser.readline()
        tdata = array([int(p) for p in tdata.split(b",")])
        return (sdata, tdata)

if __name__ == "__main__":
    DATASET_NAME = "LED Desk Lamp 1ft IT1e-6s Teensy 3.5 120MHz ADC synchronous"
    mpl.rcParams["savefig.directory"] = os.path.dirname(__file__)

    port = glob.glob("/dev/ttyACM*")[0]
    
    spec = MicroSpec(port)
    icc = spec.set_integration_time(1e-6)
    
    sdata, tdata = spec.read()
    print("Timings: %r" % (tdata - tdata[0],))
    
    plot(sdata)
    title(DATASET_NAME)
    show()
    savetxt("%s.csv" % DATASET_NAME,sdata,delimiter=",")
