from tkinter import *
import math
import serial

# used blte.py from https://github.com/GunnarI/bluepy/tree/add-timeout-to-perhiperal
# as it has a timeout on connection available
import bluepy.btle as btle

from time import sleep
# from time import sleep

# ogólne funkcje i klasy
def myround(x, base=5):
    # rounding to the given base, used for snaps
    return base * round(x/base) 

# na potrzeby komunikacji z BLE bezpośrednio
# użycie biblioteki bluepy - nie do końca rozumiem jak to działa :)
class ReadDelegate(btle.DefaultDelegate):
    def handleNotification(self, cHandle, data):
        print(data.decode("utf-8"))

class Paint(object):
    # this is the main object that contain the entire app windo and object

    DEFAULT_PEN_SIZE = 5.0
    DEFAULT_COLOR = 'black'
    WIDTH = 800
    HEIGHT = 800

    def __init__(self):
        self.root = Tk()
        self.root.title('Dz3 Path Maker')

        self.connect_button = Button(self.root, text='Connect', command=self.connect)
        self.connect_button.grid(row=0, column=0)

        self.color_button = Button(self.root, text='Upload', command=self.sent_serial)
        self.color_button.grid(row=0, column=1)

        self.go_button = Button(self.root, text='GO', command=self.go)
        self.go_button.grid(row=0, column=2)

        self.loop_button = Button(self.root, text='GO Loop', command=self.goLoop)
        self.loop_button.grid(row=0, column=3)

        self.S_button = Button(self.root, text='STOP', command=self.S)
        self.S_button.grid(row=0, column=4)

        self.eraser_button = Button(self.root, text='clr Dz3', command=self.clearDz3)
        self.eraser_button.grid(row=0, column=5)

        self.eraser_button = Button(self.root, text='clr Canvas', command=self.clear)
        self.eraser_button.grid(row=0, column=6)

        self.pen_button = Button(self.root, text='undo', command=self.undo)
        self.pen_button.grid(row=0, column=7)
        
        self.snap_button = Button(self.root, text='snap to grid', command=self.toggle_snap)
        self.snap_button.grid(row=0, column=10, columnspan = 2)

        self.choose_size_button = Scale(self.root, from_=1, to=100, orient=HORIZONTAL, command=self.theGrid)
        self.choose_size_button.set(36)
        self.choose_size_button.grid(row=30, column=10, columnspan=4,sticky='NSEW')

        self.c = Canvas(self.root, bg='white', width=self.WIDTH, height=self.HEIGHT)
        self.c.grid(row=1, columnspan=9, rowspan=30)

        # control buttons 
        self.go_up = Button(self.root, text='Up')
        self.go_up.grid(row=8, column=11)
        self.go_up.bind("<ButtonPress>",self.goUp)
        self.go_up.bind("<ButtonRelease>", self.goStop)

        self.go_dn = Button(self.root, text='Dn')
        self.go_dn.grid(row=9, column=11)
        self.go_dn.bind("<ButtonPress>", self.goDn)
        self.go_dn.bind("<ButtonRelease>", self.goStop)

        self.go_left = Button(self.root, text='<<')
        self.go_left.grid(row=9, column=10)
        self.go_left.bind("<ButtonPress>", self.goLeft)
        self.go_left.bind("<ButtonRelease>", self.goStop)

        self.go_right = Button(self.root, text='>>')
        self.go_right.grid(row=9, column=12)
        self.go_right.bind("<ButtonPress>", self.goRight)
        self.go_right.bind("<ButtonRelease>", self.goStop)

        # turning by arc  stuff
        self.alphatxt = Label(self.root, text = "Turning by angle ")
        self.alphatxt.grid(row=1, column=10, columnspan=3)
        self.alpha_slider = Scale(self.root, from_=360, to=-360, orient=HORIZONTAL, command=self.getAlpha)
        self.alpha_slider.set(90)
        self.alpha_slider.grid(row=2, column=10, columnspan=4,sticky='NSEW')

        self.Rtxt = Label(self.root, text = "Turning Radius ")
        self.Rtxt.grid(row=3, column=10, columnspan=3)
        self.R_slider = Scale(self.root, from_=0, to=100, orient=HORIZONTAL, command=self.getR)
        self.R_slider.set(30)
        self.R_slider.grid(row=4, column=10, columnspan=4,sticky='NSEW')

        self.anglelabeltxt = Label(self.root, text = "Current Angle: ")
        self.anglelabeltxt.grid(row=6, column=10, columnspan = 2)
        self.anglelabel = Label(self.root, text = "0")
        self.anglelabel.grid(row=6, column=11)

        # insertArc trigger button
        self.go_right = Button(self.root, text='Add Turn')
        self.go_right.grid(row=5, column=10, columnspan = 4)
        self.go_right.bind("<ButtonPress>", self.insertArc)

        # somestuff to try straightline things
        self.points = [];
        self.lines = [];
        self.joints = [];
        self.txt = [];
        self.arclines = [];
        self.origin = None;
        self.snap = not True
        self.toggle_snap()


        self.globalAngle = 90; # startring angle 
        self.globalAngleLast = [self.globalAngle];

        self.commands = [];

        # stuff for grid
        self.gridlines= [];

        # moving the world around
        self.translate = (0,0);
        self.oldtranslate= (0,0);
        self.intranslate = False;
        self.translateclick= (0,0);
        
        #  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
        self.connect()

        self.setup()
        self.root.mainloop()

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = 5
        self.drivescale = self.choose_size_button.get()
        self.color = self.DEFAULT_COLOR
        self.eraser_on = False
        self.active_button = self.pen_button
        self.c.bind('<ButtonRelease-1>', self.pointerUp)
        self.c.bind('<B3-Motion>', self.movecanvas)
        self.c.bind('<Button-3>', self.setmove)
        self.c.bind('<MouseWheel>', self.zoom)
        self.c.bind('<Button-4>', self.zoomIn)
        self.c.bind('<Button-5>', self.zoomOut)
        self.c.bind('<Motion>', self.showTrajectory)

        self.alpha_slider.bind('<Button-5>', self.anle_dn)
        self.alpha_slider.bind('<Button-4>', self.anle_up)

        self.R_slider.bind('<Button-5>', self.R_dn)
        self.R_slider.bind('<Button-4>', self.R_up)

        self.theGrid(self.choose_size_button.get());   

    def connect(self):
        # procedure to connect ot the Arduino part of Dżordż
        # procedure is to try:
        # connect to a serial port on this machine /tmp/ttyBLE
        # if not successes trying to connect to the BLE directly 
        # the adress of the BT BLE of Dżordż is 88:25:83:f0:fe:e6
 
        print("Connecting...")
        try:
            self.ser = serial.Serial('/tmp/ttyBLE', timeout=1)
            self.ser.baudrate = 9600
            self.color_button.configure(background = "green")
            self.go_button.configure(background = "green")
            self.loop_button.configure(background = "green")
            self.is_serial = True
        except:
            print("Serial BT port error...")
            print("trying direct BLE connection...")
            try:
                self.BTEperihibal = btle.Peripheral("88:25:83:f0:fe:e6", timeout=3)
                print(f'BTE {self.BTEperihibal}')
                self.BTEservice = self.BTEperihibal.getServiceByUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
                self.Dzordz = self.BTEservice.getCharacteristics()[0]

                self.color_button.configure(background = "blue")
                self.go_button.configure(background = "blue")
                self.loop_button.configure(background = "blue")
                self.is_serial = False
                self.is_BLE = True
            except:
                self.color_button.configure(background = "red")
                self.go_button.configure(background = "red")
                self.loop_button.configure(background = "red")
                self.is_serial = False
                self.is_BLE = False

    def toggle_snap(self):
        self.snap = not self.snap
        if self.snap:
            self.snap_button.configure(background = "green")
        else:
            self.snap_button.configure(background = "lightgray")

    def BLE_sent(self, command):
        self.Dzordz.write(bytes(command, "utf-8"))
        
        self.BTEperihibal.withDelegate(ReadDelegate())
        while self.BTEperihibal.waitForNotifications(0.1):
            pass
            
    def anle_dn(self, *args):
        self.alpha_slider.set(myround(self.alpha_slider.get())-5)
    
    def anle_up(self, *args):
        self.alpha_slider.set(myround(self.alpha_slider.get())+5)

    def R_dn(self, *args):
        self.R_slider.set(myround(self.R_slider.get())-5)
    
    def R_up(self, *args):
        self.R_slider.set(myround(self.R_slider.get())+5)

    def getAlpha(self, *args):
        self.alpha_slider.set(myround(self.alpha_slider.get()))

    def getR(self, *args):
        self.R_slider.set(myround(self.R_slider.get()))

    def drawArc(self, start, theta, length, alpha ):
        # function for drawing an arc curve
        localscale = 1/(self.choose_size_button.get() /100)

        x0, y0 = start

        theta_d = theta
        theta = math.radians(theta)
        alpha_r = math.radians( alpha )

        # calculating the Rarius
        # lenght = alpha_r * R 
        R = length / abs(alpha_r)

        znak = -1 * alpha / abs(alpha)

        # print(f"Znak i tak {znak}  {alpha} {theta_d}")
        
        Cx = x0 + znak * R * math.sin( theta )
        Cy = y0 + znak * R * math.cos( theta )

        x1 = R * math.sin(alpha_r)
        y1 = -1* R * (1 - math.cos(alpha_r)) # bo liczymy y jak piksele w dół

        x2 =  x1 * math.cos(theta) + y1 * math.sin(theta)
        y2 = -x1 * math.sin(theta) + y1 * math.cos(theta)

        x2 =  x0 - znak * x2
        y2 =  y0 - znak * y2

        # skalowanie do kreślenia 
        Cx,Cy = self.projection((Cx,Cy))
        R = R * localscale
        
        if alpha == 360:
            alpha = 359
        if alpha == -360:
            alpha = 359

        if alpha > 0:
            self.lines.append( self.c.create_arc( Cx-R, Cy-R, Cx+R, Cy+R, start=theta_d-90, extent=alpha,
                                                  style=ARC, width=self.line_width, outline='red'))
        else:
            self.lines.append( self.c.create_arc( Cx-R, Cy-R, Cx+R, Cy+R, start=theta_d-270, extent=alpha, 
                                                  style=ARC, width=self.line_width, outline='red'))

        self.txt.append(self.c.create_text(Cx,Cy, fill="black",font="20",
                        text=f"arcL:{int(length)}cm"))

        return ( x2, y2 )

    def insertArc(self, *args):
        
        if len(self.points):
            alpha = myround(self.alpha_slider.get())
            R = myround(self.R_slider.get())
            dL = abs(int(math.radians(alpha) * R))
            
            x2, y2 = self.drawArc(self.points[-1], self.globalAngle, dL, alpha)
            
            cX, cY = self.projection(( x2, y2 ))

            self.joints.append( self.c.create_oval(cX-5, cY-5, cX+5, cY+5, outline='black',
                fill=None, width=2) )

            self.points.append((x2, y2))
            self.old_x = x2
            self.old_y = y2

            A = alpha
            self.globalAngleLast.append( self.globalAngle);
            self.globalAngle += A
            dA = self.globalAngle 

            # adding thisline as command
            self.commands.append((dL,A,700,dA))
            print(self.commands[-1])
            # adding command to reset the speed back after turn slowly
            self.commands.append((0,0,9999,dA))
        
    def goUp(self, *args):
        message = f'<1,200,0,10000>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)

    def goLeft(self, *args):
        message = f'<1,0,720,500>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)

    def goRight(self, *args):
        message = f'<1,0,-720,500>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)

    def goDn(self, *args):
        message = f'<1,-200,0,10000>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)


    def goStop(self, *args):
        message = f'<0,0,0,0>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)
    
    def S(self, *args):
        # sent the GO command
        message = f'<0,0,0,0>'
        if self.is_serial:
            # sent the GO command
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            self.BLE_sent(message)

    def zoom(self,event):
        self.choose_size_button.set(self.choose_size_button.get() + event.delta);
        pass

    def zoomIn(self, event):
        self.choose_size_button.set(self.choose_size_button.get() + 1);
        pass

    def zoomOut(self,event):
        self.choose_size_button.set(self.choose_size_button.get() - 1);
        pass

    def setmove(self, event):
        localscale = 1/(self.choose_size_button.get() /100)
        self.translateclick = (event.x / localscale, event.y / localscale)
        self.oldtranslate = self.translate
        print(self.translateclick)

    def resetmove(self):
        pass

    def movecanvas(self, event):
        # this procedure is to move the canvas around
        localscale = 1/(self.choose_size_button.get() /100)
        
        x0, y0 = self.translateclick
        x, y = self.oldtranslate 

        x += (event.x / localscale) - x0
        y += (event.y / localscale) - y0 
        
        self.translate = (x, y)

        self.theGrid(1)  

    def projection(self,point):
        """
        This function recalculate the real life coordinates
        to the screen canvas pixel coordinates
        """

        #  grabbing the scale
        localscale = 1/(self.choose_size_button.get() /100)
        shift_x, shift_y = self.translate;

        x,y = point;
        #  moving and scaling
        x = (x + shift_x) * localscale;
        y = (y + shift_y) * localscale;

        return (x, y)

    def projection_back(self, point):
        """
        This functions recalculate the screen canvas pixel coordinates
        to real life coordinates
        """
        #  grabbing the scale
        localscale = (self.choose_size_button.get() /100)
        
        shift_x, shift_y = self.translate;

        x,y = point;
        #  moving and scaling
        x = x * localscale - shift_x;
        y = y * localscale - shift_y;

        return (x, y)

    def theGrid(self, scaleval):
        """
        This procedure redraw the canvas with grid and all other stuff
        """        
        self.scrClr();

        if len(self.gridlines) > 0:
            for gline in self.gridlines:
                self.c.delete(gline)

        spc100cm = 10 * 100 / self.choose_size_button.get() #100cm in pixels

        nX = self.WIDTH // spc100cm
        nY = self.HEIGHT // spc100cm

        # for n in range(int(3*nX)):
        for n in range(120):
            
            # sx, sy = self.projection((10 * (n-1) - 10*(nX // 2), 10 * (n-1) - 10*(nX // 2)))
            sx, sy = self.projection((10 * (n-1-30), 10 * (n-1-30)))

            kolor = 'gray'
            grubosc = 1

            if (n % 10 == 0):
                kolor = 'blue'
                grubosc = 2

            self.gridlines.append( self.c.create_line( sx, 0, sx, self.HEIGHT,
                               width=grubosc, fill=kolor,
                               capstyle=ROUND, smooth=TRUE, splinesteps=36) )

            self.gridlines.append( self.c.create_line( 0, sy, self.WIDTH, sy,
                               width=grubosc, fill=kolor,
                               capstyle=ROUND, smooth=TRUE, splinesteps=36) )

        # self.reDraw();
        self.redraw_from_commands()

    def update(self):
        self.anglelabel.config(text = str(self.globalAngle) )

    def _distance(self, p0, p1):
        '''calculate distance between 2 points'''
        return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

    def reset(self, event):
        self.old_x, self.old_y = None, None

    def showTrajectory(self, event):
        """
        This procedure draws the gray line as
        prediction of the next piece of path
        """
        self.drivescale = self.choose_size_button.get() /100
        shift_x, shift_y = self.translate;

        if len(self.points):
            try:
                self.c.delete(self.trajectory)
            except:
                pass

            _x,_y = self.projection(self.points[-1])

            x = event.x * self.drivescale - shift_x
            y = event.y * self.drivescale - shift_y
            
            if self.snap:
                x = myround(x, 10)
                y = myround(y, 10)
            
            x,y= self.projection((x,y))

            self.trajectory = self.c.create_line(x, y, _x, _y,
                               width=self.line_width - 2, fill='gray',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36)

    def pointerUp(self,event):
        """
        This procedure is tha main part of reacting on the user mouse click
        """
        self.drivescale = self.choose_size_button.get() /100
        shift_x, shift_y = self.translate;

        if not self.old_x and not self.old_y:
            _x = event.x * self.drivescale - shift_x
            _y = event.y * self.drivescale - shift_y 
            
            if self.snap:
                _x = myround(_x, 10)
                _y = myround(_y, 10)

            self.old_x = _x
            self.old_y = _y 
            
            self.points.append((_x, _y))

            _x, _y = self.projection((_x, _y))   

            self.origin = self.c.create_oval(_x-5, _y-5, _x+5, _y+5, outline='red',
            fill=None, width=2)

        else:
            _x = event.x * self.drivescale - shift_x
            _y = event.y * self.drivescale - shift_y
            
            if self.snap:
                _x = myround(_x, 10)
                _y = myround(_y, 10)

            if self._distance(self.points[-1], (_x, _y)) >= 1:

                self.points.append((_x, _y))
                
                lx1, ly1 = self.projection( (self.old_x, self.old_y) )
                lx2, ly2 = self.projection( (_x,_y) )

                self.lines.append(self.c.create_line(lx1, ly1, lx2, ly2,
                                width=self.line_width, fill='red',
                                capstyle=ROUND, smooth=TRUE, splinesteps=36))
            
                self.old_x = _x
                self.old_y = _y 

                # figuring out the propper command for this click
                origin_x, origin_y = self.points[0]
                iA = 90;
                # handling this very point

                point = (_x, _y)
                cX = point[0]
                cY = point[1] 

                dX = point[0] - (self.points[-2][0])
                dY = (self.points[-2][1]) - point[1]

                dL = math.sqrt( dX**2 +dY**2 ) #* self.drivescale / 100

                # adding txt to the plot for easyreference.
                mX = ((point[0] + self.points[-2][0]) / 2)
                mY = ((point[1] + self.points[-2][1]) / 2)

                mX, mY = self.projection( (mX, mY) )

                self.txt.append(self.c.create_text(mX,mY, fill="black",font="20",
                            text=f"{int(dL)}cm"))

                cX, cY = self.projection( (cX, cY) )

                self.joints.append( self.c.create_oval(cX-5, cY-5, cX+5, cY+5, outline='black',
                fill=None, width=2) )
                
                dA = int(math.degrees(math.atan2(dY, dX)))
                if len(self.commands) > 0:
                    iA = self.commands[-1][3]
                else:
                    iA= 90

                A = dA - iA

                if A < -180: 
                    A = 360 + A
                elif A > 180:
                    A = A - 360

                # tracking theglobal angle od Dz3
                self.globalAngleLast.append( self.globalAngle);
                self.globalAngle += A
                self.update()
                

                # adding thisline as 2 commands - 1-turn - 2 move
                self.commands.append((0,A,0,dA))
                print(self.commands[-1])
                self.commands.append((dL,0,0,dA))
                print(self.commands[-1])
    
    def sent_serial(self):
        # first sent the set of sequence commands
        if self.is_serial:
            for command in self.commands:
                message = f'<20,{int(command[0])},{int(command[1])},{int(command[2])}>'
                self.ser.write(message.encode('utf-8'))
                print(message.encode('utf-8'))
                
                while (True):
                    response = str(self.ser.readline())
                    if "Step added" in response:
                        print("Command confirmed!")
                        break
        elif self.is_BLE:
            for command in self.commands:
                message = f'<20,{int(command[0])},{int(command[1])},{int(command[2])}>'
                self.BLE_sent(message)

    def redraw_from_commands(self):
        """
        the idea of this procedure is to be able 
        to redraw the path based on the commands list.
        """

        if len(self.commands):
            # lets bring the 1st point position
            starting_point = self.points[0]
            x, y = starting_point
            # global starting angle
            azimuth = 90

            # lets now go thru the membered commands:
            for command in self.commands:
                # marking the points
                cX, cY = self.projection( (x, y) )
                self.joints.append( self.c.create_oval(cX-5, cY-5, cX+5, cY+5, outline='black',
                fill=None, width=2) )

                # lets disassemble this one:
                #  command is (length, angle, speed, global angle)
                length, angle, speed, dA = command
                
                if length == 0:
                    azimuth += angle
                else:
                    if angle == 0:
                        # we just travel via straight line.
                        dX = length * math.cos(math.radians(azimuth))
                        dY = -1 * length * math.sin(math.radians(azimuth))
                        
                        x_end = x + dX
                        y_end = y + dY

                        _x  ,_y  = self.projection( (x, y) )
                        _xe ,_ye = self.projection( (x_end, y_end) )

                        self.lines.append(self.c.create_line(_x, _y, _xe, _ye,
                                width=self.line_width, fill='red',
                                capstyle=ROUND, smooth=TRUE, splinesteps=36))

                        mX = (_x + _xe) / 2
                        mY = (_y + _ye) / 2

                        self.txt.append(self.c.create_text(mX,mY, fill="black",font="20",
                        text=f"{int(length)}cm"))

                        x, y = x_end, y_end
                    
                    else:
                        # drawing an angle
                        x,y = self.drawArc((x,y), azimuth, length, angle)
                        azimuth += angle

            self.points[-1] = (x,y) # to compensate the shifts due to redraws angles 
            self.old_x = x
            self.old_y = y
            
    def go(self):
        """
        Sending the start sequence command to Dżordż
        """
        if self.is_serial:
            # sent the GO command
            message = f'<30,0,0,0>'
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
        elif self.is_BLE:
            # sent the GO command
            message = f'<30,0,0,0>'
            self.BLE_sent(message)

    def goLoop(self):
        """
        Sending the start sequence in loop command to Dżordż
        """
        if self.is_serial:
            # sent the GO command
            message = f'<33,0,0,0>'
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))

        elif self.is_BLE:
            # sent the GO command
            message = f'<33,0,0,0>'
            self.BLE_sent(message)

    def scrClr(self):
        try:
            self.c.delete(self.origin)
            self.c.delete(self.trajectory)
        except:
            pass

        for lin in self.lines:
            self.c.delete(lin)
        for txt in self.txt:
            self.c.delete(txt)
        for joint in self.joints:
            self.c.delete(joint)

    def clear(self):
        self.scrClr();

        self.points = [];
        self.lines = [];
        self.txt = [];
        self.commands = [];
        self.origin = None;

        self.old_x = None
        self.old_y = None

        self.globalAngle = 90;

        self.update();

    def clearDz3(self):
        if self.is_serial:
            message = f'<39,0,0,0>'
            # self.ser.write(message)
            self.ser.write(message.encode('utf-8'))
            # print(message.encode('utf-8'))
            print(message)

        elif self.is_BLE:
            # sent the GO command
            message = f'<39,0,0,0>'
            self.BLE_sent(message)


    def undo(self):
        if len(self.lines) > 0:
            self.c.delete(self.lines[-1])
            self.c.delete(self.txt[-1])
            self.c.delete(self.joints[-1])
            del self.lines[-1]
            del self.points[-1]
            del self.commands[-1] # 2x as we add two commands in each step.
            del self.commands[-1]
            del self.txt[-1]
            del self.joints[-1]

            self.globalAngle = self.globalAngleLast[-1];
            del self.globalAngleLast[-1];

            if len(self.lines) == 0:
                self.old_x, self.old_y = None, None
                self.c.delete(self.origin)
                self.origin = None
            else:
                self.old_x, self.old_y = self.points[-1]

            self.update();
            
if __name__ == '__main__':
    Paint()
