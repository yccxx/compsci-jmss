__author__ = 'conroycheers'

from px4flow import PX4Flow
import time

class PX4FlowReader(PX4Flow):
    def __init__(self, port):

        PX4Flow.__init__(self, port)

        # No readings yet
        self.SensorX, self.SensorY = None, None
        self.X, self.Y = None, None
        self.H = None
        self.Quality = None

        # Create logfile named by current date / time
        filename = 'px4flow_' + time.strftime('%d_%b_%Y_%H_%M_%S') + '.csv'
        self.logfile = open(filename, 'w')
        self.write_and_flush('Time (sec), Ground Dist (m), Flow X, Flow Y, Flow Comp X (m), Flow Comp Y '
                             '(m), Quality (/255),,')
        self.write_and_flush('X accum (m), Y accum (m)\n')

        # These will get the accumulated X,Y distances in meters
        self.timeSecPrev = None
        self.X_accum = 0
        self.Y_accum = 0

        self.count = 0

    # Implements missing method in parent class
    def update(self):

        # Grab raw sensor X,Y
        self.SensorX, self.SensorY = self.getFlow()

        # Grab computed X,Y in meters
        self.X, self.Y = self.getFlowComp()

        # Grab ground distance (height) in meters
        self.H = self.getGroundDistance()

        # Grab quality in percent
        self.Quality = self.getQuality()

        # Time in seconds
        timeSec = self.getTime() / 1e6

        # Computed flow in meters per second
        flowCompX, flowCompY = self.getFlowComp()

        self.write_and_flush('%6.3f, %+3.3f, %d, %d, %+3.3f, %+3.3f, %d' % \
                     (timeSec, self.H, self.SensorX, self.SensorY, self.X, self.Y, self.Quality))

        # After first iteration, compute accumulated distances
        if self.count:

            # Compute distance if elapsed time available
            if self.timeSecPrev:

                elapsedSec = timeSec - self.timeSecPrev

                # Elapsed time should never be more than a small fraction of a second
                if elapsedSec < 0.1:
                    self.X_accum += flowCompX * elapsedSec
                    self.Y_accum += flowCompY * elapsedSec

                self.timeSecPrev = timeSec

                self.write_and_flush(',,%+3.3f, %+3.3f' % (self.X_accum, self.Y_accum))

        self.write_and_flush('\n')

        # Update count for speed reporting
        self.count += 1
        self.timeSecPrev = timeSec

    def write_and_flush(self, s):

        self.logfile.write(s)
        self.logfile.flush()

flow = PX4FlowReader('/dev/tty.usbmodemfd121')

while True:
    flow.refresh()
    time.sleep(0.01)
    try:
        flow.update()
    except Exception as e:
        print(e)
    print(flow.SensorX, flow.SensorY)