import serial
import time
import logging
import urwid

class Gamoto:
    """
    Author: Gnewt
    Contact: gnewt@gnewt.at

    A class to handle communications with the Gamoto board.
    Some code borrowed from the original gamoto.py by Randy Gamage
    """

    def __init__(self, serport, loglevel=logging.INFO, real = True):
        if type(serport) != serial.Serial and real == True:
            raise serial.SerialException('Invalid serial port for real-mode operation.')
        self.s = serport # pySerial object
        self.logger = logging.getLogger('gamoto')
        self.logger.setLevel(loglevel)
        self.real = real # if True, will actually write/read to/from serport

    # Register addresses follow.
    address = {'mVelocity': 57, 'Version': 178,\
    'setPosition': 47, 'dS': 42, 'Profile0': 180, 'setVelocity': 54,\
    'mPosition': 51, 'Analog4': 168, 'Trajectory': 59,\
    'Analog2': 164, 'Analog3': 166, 'Analog0': 160,\
    'Analog1': 162, 'FactoryReset': 0, 'SetHome': 3, 'SaveParams': 1,\
    'Mode2': 45, 'mPower': 60, 'pwrLimit': 44,\
    'iL': 40, 'Kd': 38, 'Reset': 2, 'Mode': 43, 'Ki': 36, 'Trajv': 183,\
    'Kp': 34, 'Traja': 185,\
    'Trajx': 180}
    # End register addresses.
    # Register lengths follow.
    length = {'mVelocity': 2, 'Version': 1,\
    'setPosition': 3, 'dS': 1, 'Profile0': 3, 'setVelocity': 2,\
    'mPosition': 3, 'Analog4': 2, 'Trajectory': 1,\
    'Analog2': 2, 'Analog3': 2, 'Analog0': 2,\
    'Analog1': 2, 'FactoryReset': 1, 'SetHome': 3, 'SaveParams': 1,\
    'Mode2': 1, 'mPower': 1, 'pwrLimit': 1,\
    'iL': 2, 'Kd': 2, 'Reset': 1, 'Mode': 1, 'Ki': 2, 'Trajv': 2,\
    'Kp': 2, 'Traja': 2,\
    'Trajx': 3}
    # End register lengths.

    """
    boardnum is the number of the board you're addressing
    if you have multiple boards.
    Just set this to 0 if you only have one board, but make
    sure it's correctly set on the DIP switches.
    """
    def writereg(self, boardnum, regadr, reglen, regvalue):
        # The following three instructions put together the buffer header.
        packet = [chr(0xAA + boardnum),chr(1 + reglen)]
        packet.append(chr(regadr))
        # The following for statement writes the actual register data.
        regval_bytes = regvalue
        for i in xrange(reglen):
            packet.append(chr(regval_bytes % 256))
            regval_bytes = regval_bytes >> 8
        # checksum the data, add the checkbit.
        packet.append(self.checksum(packet))

        # Write like mad!
        if self.real:
            self.s.write(''.join(packet))
        # Read like... less mad!
        if self.real:
            response = self.s.read(2)
            self.s.flushInput() # Flush the buffer. Sends random
                            # data sometimes
        else:
            response = '\x41\x41'

        return response == '\x41\x41'

    def readreg(self, boardnum, regadr, reglen):
        packet = [chr(0xAA + boardnum), '\x82']
        packet.append(chr(regadr))
        packet.append(chr(reglen))
        packet.append(self.checksum(packet))

        for ch in packet:
            self.logger.debug("WRITE: " + hex(ord(ch)))

        if self.real:
            self.s.write(''.join(packet))

        response = self.s.read(2+reglen)
        answer = 0
        self.logger.debug("Read register #%d", regadr)
        for i in xrange(reglen):
            for ch in response:
                self.logger.debug("READ: " + hex(ord(ch)))
            answer += ord(response[i+1])*(256**i)
            return answer
        else:
            return -1

    def set(self, boardnum, regname, regvalue):
        return self.writereg(boardnum, self.address[regname], self.length[regname], regvalue)
    def read(self, boardnum, regname):
        return self.readreg(boardnum, self.address[regname], self.length[regname])

    # Byte sum modulo 256
    def checksum(self, packet):
        cs = 0
        for ch in packet[1:]:
            cs += ord(ch[0])
        return chr(cs % 256)
    
class RobotControl(Gamoto):
    """Makes the robot arm do various cool things involving cooking our food."""
    def setPosition(self, a1, a2, a3):
        self.set(0, 'setPosition', a1)
        self.set(1, 'setPosition', a2)
        self.set(2, 'setPosition', a3)
    def readPosition(self):
        return (self.read(0, 'mPosition'), self.read(1, 'mPosition'), self.read(2, 'mPosition'))
    def setPower(self, v1, v2=None, v3=None):
        if not v2:
            v2 = v1
        if not v3:
            v3 = v1
        self.set(0, 'pwrLimit', v1)
        self.set(1, 'pwrLimit', v2)
        self.set(2, 'pwrLimit', v3)
    def waitUntilNear(self, a1, a2, a3, tol=5, timeout=2):
        beginTime = time.time()
        while time.time() < beginTime+timeout:
            (p1, p2, p3) = self.readPosition()
            if abs(p1 - a1) < tol and abs(p2 - a2) < tol and abs(p3 - a3) < tol:
                return
    def moveUntilNear(self, a1, a2, a3, tol=5, timeout=2):
        self.setPosition(a1, a2, a3)
        self.waitUntilNear(a1, a2, a3, tol, timeout)
    def learnFromMe(self, timeout=2):
        """You may have to remove motor power manually. Call this function and then move the arm, press enter for each waypoint.
        When you're done, press Ctrl-D and the robot should follow the waypoints in a loop infinitely."""
        self.setPower(0)
        self.s.flushInput()
        positions = []
        try:
            while True:
                raw_input()
                (a, b, c) = self.readPosition()
                self.logger.info("Coordinates: (%d, %d, %d)" % (a, b, c))
                positions.append((a, b, c))
        # Remember to plug the motors back in ;)
        except EOFError:
            self.setPower(150)
            while True:
                for i in positions:
                    self.moveUntilNear(i[0], i[1], i[2], timeout)
                    
    def moveFromList(self, positions, timeout=2):
        while True:
            for i in positions:
                self.moveUntilNear(i[0], i[1], i[2], timeout)
    
class UrwidControl(RobotControl):
    def addPosition(self, axis):
        self.axisPosition[axis] += 50
        self.setPosition(self.axisPosition[0], self.axisPosition[1], self.axisPosition[2])
        self.txt.set_text("Position: (%d, %d, %d)" % (self.axisPosition[0], self.axisPosition[1], self.axisPosition[2]))
    def subPosition(self, axis):
        self.axisPosition[axis] -= 50
        self.setPosition(self.axisPosition[0], self.axisPosition[1], self.axisPosition[2])
        self.txt.set_text("Position: (%d, %d, %d)" % (self.axisPosition[0], self.axisPosition[1], self.axisPosition[2]))
    def move_or_exit(self, input):
        if input in ('q', 'Q'):
            raise urwid.ExitMainLoop()
        elif input == 'up':
            self.addPosition(0)
        elif input == 'down':
            self.subPosition(0)
        elif input == 'left':
            self.subPosition(1)
        elif input == 'right':
            self.addPosition(1)
        elif input == 'a':
            if 0 < (self.axisPosition[2] + 50) < 1000:
                self.addPosition(2)
        elif input == 'f':
            if 0 < (self.axisPosition[2] - 50) < 1000:
                self.subPosition(2)

    def urwidTest(self):
        self.axisPosition = list(self.readPosition())
        self.txt = urwid.Text("Urwid Robot Arm Test")
        fill = urwid.Filler(self.txt, 'top')
        loop = urwid.MainLoop(fill, unhandled_input=self.move_or_exit)
        loop.run()