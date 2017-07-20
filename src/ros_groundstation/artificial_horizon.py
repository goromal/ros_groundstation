import sys, math, rospy, random
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt, QPointF,QRectF, QPoint
from PyQt5.QtGui import QColor, QBrush, QPen, QFont, QPolygon
from rosflight_msgs.msg import State, GPS
from std_msgs.msg import Float32
from .map_subscribers import *

class ArtificialHorizon(QtWidgets.QWidget):
    def __init__(self):
        super(ArtificialHorizon, self).__init__()
        self.initUI()

    def initUI(self):
        self.height = 600
        self.width = 600

        self.roll = 0     # degrees
        self.pitch = 0    # degrees
        self.speed = 0    # KIAS
        self.altitude = 0 # ft MSL
        self.heading = 0  # degrees
        self.numSat = 0   # Number of Satellites (GPS)

        self.pitchInterval = 0.013 # % of height used to display 1 degree

        self.setGeometry(300, 300, self.width, self.height)
        self.setWindowTitle('Artificial Horizon')
        self.show()

    def resizeEvent(self, newSize):
        self.width = newSize.size().width()
        self.height = newSize.size().height()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)
        self.drawArtificialHorizon(event, painter)
        painter.end()

    def drawArtificialHorizon(self, event, painter):
        # extract relevant values here from subscribers
        self.roll = int (math.floor(StateSub.phi*(180.0/math.pi)))
        self.pitch = int (math.floor(StateSub.theta*(180.0/math.pi)))
        self.speed = int (math.floor(StateSub.Va))
        self.altitude = int (math.floor(StateSub.alt))
        self.heading = int (math.floor(StateSub.psi*(180.0/math.pi))) % 360
        self.numSat = GPSDataSub.numSat

        self.drawSky(event, painter)

        painter.translate(self.width/2, self.height/2)
        painter.rotate(-self.roll)
        painter.translate(-self.width/2, -self.height/2)
        painter.translate(0,self.height*(self.pitch*self.pitchInterval))

        self.drawGround(event, painter)
        self.drawPitchIndicator(event, painter)

        painter.translate(0,self.height*(-1*self.pitch*self.pitchInterval))

        self.drawTurnIndicator(event, painter)

        painter.translate(self.width/2, self.height/2)
        painter.rotate(self.roll)
        painter.translate(-self.width/2, -self.height/2)

        self.drawAircraftSymbol(event, painter)
        self.drawAirspeedIndicator(event, painter)
        self.drawAltitudeIndicator(event, painter)
        self.drawHeadingIndicator(event, painter)
        self.drawNumSatellites(event, painter)
        #self.drawWaypointAccuracy(event, painter)

    def drawNumSatellites(self, event, painter):
        p1 = QPoint(0,0)
        p2 = QPoint(self.width*(0.25),self.height*0.1)
        rect = QRectF(p1,p2)
        if self.numSat < 4:
            painter.setPen(QPen(QBrush(Qt.red), 2, Qt.SolidLine))
        else:
            painter.setPen(QPen(QBrush(Qt.green), 2, Qt.SolidLine))
        painter.drawText(rect,QtCore.Qt.AlignCenter,"GPS: " + str(self.numSat) + " satellites")

    #def drawWaypointAccuracy(self, event, painter):
    #    p1 = QPoint(self.width*(0.65),0)
    #    p2 = QPoint(self.width,self.height*0.1)
    #    rect = QRectF(p1,p2)
    #    painter.drawText(rect,QtCore.Qt.AlignCenter,"Wp Accuracy: " + "%.2f" % self.latestWpAccuracy + " m")

    def drawSky(self, event, painter):
        brush = QtGui.QBrush(QtGui.QColor(38, 89, 242), QtCore.Qt.SolidPattern)
        painter.fillRect(QRectF(0,0,self.width, self.height), brush)

    def drawGround(self, event, painter):
        brush = QtGui.QBrush(QtGui.QColor(84, 54, 10), QtCore.Qt.SolidPattern)
        painter.fillRect(QRectF(-300,self.height/2,self.width+600, self.height*(0.5+self.pitchInterval*180)), brush)
        painter.setPen(QPen(QBrush(Qt.white), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(-300,self.height/2,self.width+600,self.height/2)


    def drawHeadingIndicator(self, event, painter):
        boxWidth = self.width*1.0
        boxHeight = self.height*0.1
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF((self.width-boxWidth)/2,self.height-boxHeight,boxWidth,boxHeight),brush)

        directions = {0:"N",45:"NE",90:"E",135:"SE",180:"S",215:"SW",270:"W",315:"NW"}
        scale = 0.01
        for i in range(self.heading-49,self.heading+49):
            if i % 10 == 0:
                x = self.width*0.5-((self.heading-i)*scale*self.width)
                y = self.height-boxHeight
                if i < 0:
                    i += 360
                i = i % 360
                text = str(i)
                if i in directions:
                    text = directions[i]
                painter.drawLine(x,y,x,y+5)
                painter.drawText(QPoint(x+7-8*len(text),y+22),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(self.width*(0.46),self.height)
        p2 = QPoint(self.width*(0.46),self.height - boxHeight*0.9)
        p3 = QPoint(self.width*(0.50),self.height - boxHeight)
        p4 = QPoint(self.width*(0.54),self.height - boxHeight*0.9)
        p5 = QPoint(self.width*(0.54),self.height)
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine, Qt.RoundCap))
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,str(self.heading) + u'\N{DEGREE SIGN}')

    def drawAirspeedIndicator(self, event, painter):
        boxWidth = self.width*0.13
        boxHeight = self.height*0.6
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF(0,(self.height-boxHeight)/2,boxWidth,boxHeight),brush)

        scale = 0.01
        for i in range(self.speed-29,self.speed+29):
            if i % 10 == 0 and i >= 0:
                x = boxWidth
                y = self.height*0.5+((self.speed-i)*scale*self.height)
                text = str(i)
                painter.drawLine(x-5,y,x,y)
                painter.drawText(QPoint(x-10-8*len(text),y+5),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(0,self.height*(0.46))
        p2 = QPoint(boxWidth*0.9,self.height*(0.46))
        p3 = QPoint(boxWidth,self.height*(0.5))
        p4 = QPoint(boxWidth*0.9,self.height*(0.54))
        p5 = QPoint(0,self.height*(0.54))
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine, Qt.RoundCap))
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,str(self.speed) + " kt")
        painter.drawText(QPoint(5,(self.height-boxHeight)/2-5),"Airspeed (KIAS)")

    def drawAltitudeIndicator(self, event, painter):
        boxWidth = self.width*0.13
        boxHeight = self.height*0.6
        brush = QtGui.QBrush(QColor(100,100,100,200))
        painter.setPen(QPen(QBrush(Qt.yellow), 2, Qt.SolidLine))
        painter.fillRect(QRectF(self.width-boxWidth,(self.height-boxHeight)/2,boxWidth,boxHeight),brush)

        scale = 0.01
        for i in range(self.altitude-29,self.altitude+29):
            if i % 10 == 0:
                x = self.width - boxWidth
                y = self.height*0.5+((self.altitude-i)*scale*self.height)
                text = str(i)
                painter.drawLine(x,y,x+5,y)
                painter.drawText(QPoint(x+10,y+5),text)

        painter.setBrush(Qt.black)
        p1 = QPoint(self.width,self.height*(0.46))
        p2 = QPoint(self.width-boxWidth*0.9,self.height*(0.46))
        p3 = QPoint(self.width-boxWidth,self.height*(0.5))
        p4 = QPoint(self.width-boxWidth*0.9,self.height*(0.54))
        p5 = QPoint(self.width,self.height*(0.54))
        poly = QPolygon([p1,p2,p3,p4,p5])
        painter.setPen(QPen(QBrush(QColor(0,0,0,0)), 2, Qt.SolidLine))
        painter.drawPolygon(poly)
        painter.setPen(QPen(QBrush(QColor(255,255,0)), 2, Qt.SolidLine))
        text = str(self.altitude) + " ft"
        rect = QRectF(p1,p4)
        painter.drawText(rect,QtCore.Qt.AlignCenter,text)
        painter.drawText(QPoint(self.width-boxWidth+5,(self.height-boxHeight)/2-5),"Altitude")

    def drawTurnIndicator(self, event, painter):
        painter.setBrush(Qt.white)
        painter.setPen(QPen(QBrush(Qt.white), 2, Qt.SolidLine, Qt.RoundCap))
        radius = self.width*(0.3)
        yOffset = self.height*(0.10)
        painter.drawArc(
            QRectF(self.width*(0.5)-radius,yOffset, 2*radius, 2*radius),
            16*30,16*120)

        height = self.height*0.02
        x = self.width/2
        y = yOffset
        x2 = x
        y2 = y-height
        x3 = x
        y3 = y2-5
        xCenter = self.width/2
        yCenter = radius + yOffset

        angles = [-60,-45,-30,-20,-10,0,10,20,30,45,60]
        for angle in angles:
            painter.translate(xCenter,yCenter)
            painter.rotate(angle)
            painter.translate(-xCenter,-yCenter)

            painter.drawLine(x,y,x2,y2)
            text = str(angle)
            painter.drawText(QPoint(x3-4*len(text),y3),text)

            painter.translate(xCenter,yCenter)
            painter.rotate(-angle)
            painter.translate(-xCenter,-yCenter)

        # Draw the arrow
        height = self.height*0.025
        poly = QPolygon([
            QPoint(x,y),
            QPoint(x-height/2,y+height),
            QPoint(x+height/2,y+height),])

        painter.translate(xCenter,yCenter)
        painter.rotate(self.roll)
        painter.translate(-xCenter,-yCenter)

        painter.drawPolygon(poly)

        painter.translate(xCenter,yCenter)
        painter.rotate(-self.roll)
        painter.translate(-xCenter,-yCenter)


    def drawAircraftSymbol(self, event, painter):
        brightYellow = QtGui.QColor(255, 255, 0)
        widthFraction = 0.10
        heightFraction = 0.05
        painter.setPen(QPen(QBrush(brightYellow), 5, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(brightYellow)
        poly = QPolygon([
            QPoint(self.width*0.5, self.height*0.5),
            QPoint(self.width*(0.5+widthFraction/2.0),self.height*(0.5+heightFraction)),
            QPoint(self.width*0.5, self.height*(0.5+heightFraction/2.0)),
            QPoint(self.width*(0.5-widthFraction/2.0),self.height*(0.5+heightFraction))
            ])
        painter.drawPolygon(poly)
        space = 0.25
        length = 0.1
        painter.drawLine(self.width*space, self.height/2, self.width*(space+length), self.height/2)
        painter.drawLine(self.width*(1-space-length), self.height/2, self.width*(1-space), self.height/2)


    def drawPitchIndicator(self, event, painter):
        minHeight = 0.15 - self.pitch*self.pitchInterval
        maxHeight = 0.85 - self.pitch*self.pitchInterval
        for i in range(-9,9):
            text = str(10*abs(i))
            height = 0.5-self.pitchInterval*10*i
            if height > minHeight and height < maxHeight:
                painter.drawLine(
                    self.width*(0.4),self.height*(height),
                    self.width*(0.6),self.height*(height))
                painter.drawText(QPoint(self.width*(0.6)+5,self.height*(height)+5),text)
                painter.drawText(QPoint(self.width*(0.4)-22,self.height*(height)+5),text)

            height = height - self.pitchInterval*5
            if height > minHeight and height < maxHeight:
                painter.drawLine(
                    self.width*(0.45),self.height*(height),
                    self.width*(0.55),self.height*(height))

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    ah = ArtificialHorizon()
    sys.exit(app.exec_())
