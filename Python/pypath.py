from tkinter import *
import math
import asyncio
from bleak import BleakClient, BleakScanner 
from bleak.exc import BleakError
from time import sleep, thread_time, time

# ogólne funkcje i klasy
def myround(x, base=5):
    # rounding to the given base, used for snaps
    return base * round(x/base) 


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

        self.c = Canvas(self.root, bg='darkgray', width=self.WIDTH, height=self.HEIGHT)
        self.c.grid(row=1, columnspan=9, rowspan=30)

        self.handc = Canvas(self.root, bg='white', width=200, height=200)
        self.handc.grid(row=5, column=14, columnspan=1, rowspan=1)

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

        # Hands controll 
        self.go_right = Button(self.root, text='\| |')
        self.go_right.grid(row=14, column=10)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,15,170,0> <41,14,0,0>"))

        self.go_right = Button(self.root, text='| |')
        self.go_right.grid(row=14, column=11)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,0,180,0> <41,1,45,0> <41,15,0,0> <41,14,99,0>"))

        self.go_right = Button(self.root, text='| |/')
        self.go_right.grid(row=14, column=12)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,0,10,0> <41,1,130,0>"))
        
        self.go_right = Button(self.root, text='( o o)')
        self.go_right.grid(row=10, column=12)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,7,155,0>"))

        self.go_right = Button(self.root, text='(o o)')
        self.go_right.grid(row=10, column=11)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,7,85,0>"))

        self.go_right = Button(self.root, text='(o o )')
        self.go_right.grid(row=10, column=10)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,7,5,0>"))
        
        self.head_txt = Label(self.root, text = "Turning head")
        self.head_txt.grid(row=11, column=10, columnspan=3)
        self.head_slider = Scale(self.root, from_=5, to=155, orient=HORIZONTAL, 
            command=lambda tmp=None: 
            self.command(f"<41, 7,{myround(self.head_slider.get())},0>"))
        self.head_slider.set(85)
        self.head_slider.grid(row=12, column=10, columnspan=4,sticky='NSEW')

        self.head_slider2 = Scale(self.root, from_=0, to=100, orient=HORIZONTAL, 
            command=lambda tmp=None: 
            self.command(f"<41, 6,{myround(self.head_slider2.get())},0>"))
        self.head_slider2.set(40)
        self.head_slider2.grid(row=13, column=10, columnspan=4,sticky='NSEW')

        self.go_right = Button(self.root, text='| |')
        self.go_right.grid(row=14, column=11)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: 
                self.command("<41,7,85,0> <41,0,180,0> <41,1,45,0> <41,15,0,0> <41,14,99,0>"))

        self.go_right = Button(self.root, text="\ ^^\ ")
        self.go_right.grid(row=14, column=10)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,7,160,0> <41,0,170,0> <41,1,0,0> <41,15,170,0> <41,14,5,0>"))
        
        self.go_right = Button(self.root, text="/ ^^/ ")
        self.go_right.grid(row=14, column=12)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: self.command("<41,7,20,0> <41,0,0,0> <41,1,160,0> <41,15,0,0> <41,14,160,0>"))

        self.go_right = Button(self.root, text="# ^^|")
        self.go_right.grid(row=15, column=10)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: 
                self.command("<41,0,180,0> <41,1,80,0> <41,15,110,0> <41,14,5,0>"))

        self.go_right = Button(self.root, text="|^^ #")
        self.go_right.grid(row=15, column=12)
        self.go_right.bind("<ButtonPress>",lambda tmp=None: 
                self.command("<41,0,60,0> <41,1,145,0> <41,15,0,0> <41,14,100,0>"))

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

        self.hand_lines = []
        self.hand_point = 0
        self.hand_zero = (150,100)
        self.hand_A = 100
        self.hand_B = 100
        self.hand_Alfa0 = 60


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

        self.last_cmd_time = time()
        self.cmd_delay = 20/1000;

        self.setup()
        self.BTsetup()
        self.root.mainloop()

        #  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
        # self.connect()

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = 5
        self.drivescale = self.choose_size_button.get()
        self.color = self.DEFAULT_COLOR
        self.eraser_on = False
        self.inmotion = False
        self.active_button = self.pen_button
        self.c.bind('<ButtonRelease-1>', self.pointerUp)
        self.c.bind('<B1-Motion>', self.movecanvas)
        self.c.bind('<Button-1>', self.setmove)
        self.c.bind('<MouseWheel>', self.zoom)
        self.c.bind('<Button-4>', self.zoomIn)
        self.c.bind('<Button-5>', self.zoomOut)
        self.c.bind('<Motion>', self.showTrajectory)

        self.handc.bind('<B1-Motion>', self.setHand)
        self.handc.bind('<Button-1>', self.setHand)

        self.alpha_slider.bind('<Button-5>', self.anle_dn)
        self.alpha_slider.bind('<Button-4>', self.anle_up)

        self.R_slider.bind('<Button-5>', self.R_dn)
        self.R_slider.bind('<Button-4>', self.R_up)

        self.theGrid(self.choose_size_button.get());   

        self.handc_set()
    

    def BTsetup(self):
        self.the_device = ""
        self.the_service = ""
        self.client = False
        self.loop = False

        self.is_serial = False
        self.is_BLE = False


    async def BTsearch(self):
        devices = await BleakScanner.discover()
        for i,d in enumerate(devices):
            print(f"[{i}]\t{d.name}\t{d.address}")
            if "BT05" in d.name:
                print(f"Potenitial robot found @ {d.address}")
                self.the_device = d.address


    async def BTgetServices(self ):
        device = await BleakScanner.find_device_by_address(self.the_device, timeout=20.0)
        
        if not device:
            raise BleakError(f"A device with address {self.the_device} could not be found.")

        async with BleakClient(device) as client:
            svcs = await client.get_services()
            print("Services:")
            for service in svcs:
                print(service)
                if "Vendor specific" in str(service):
                    print(service.characteristics)
                    for inside in service.characteristics:
                        print(f"Sub info properties: {inside.properties}")
                        print(f"Sub info uuid: {inside.uuid}")
                        self.the_service = inside.uuid

    async def BTconnect(self):
        """
        The class method to connecto tot the BT robot
        Based on the blake library for the BLE device operations.
        """
 
        print("Connecting...")
        print("trying direct BLE connection...")

        self.client = BleakClient(self.the_device,timeout=10)
        await self.client.connect()
        connection_status = self.client.is_connected
        print(f"Connection status: {connection_status}")

        if connection_status:
            print("Connected mode..")
            self.color_button.configure(background = "blue")
            self.go_button.configure(background = "blue")
            self.loop_button.configure(background = "blue")
            self.c.configure(bg='white')
            self.is_serial = False
            self.is_BLE = True
        else:
            self.color_button.configure(background = "red")
            self.go_button.configure(background = "red")
            self.loop_button.configure(background = "red")
            self.c.configure(bg="red")
            self.is_serial = False
            self.is_BLE = False

    async def BTdisconnect(self):
            if self.client:
                if self.client.is_connected:
                    await self.client.disconnect()
                    print("The robot have been disconnected...")
                else:
                    print("No robot to be disconnected!")
            else:
                print("No robot to be disconnected!")

    async def BTwrite(self, the_command, redial=True):
        if self.client.is_connected:
            await self.client.write_gatt_char(self.the_service,bytearray(the_command, "utf-8"), response=not True)
        else:
            print("No devce connected.")
            if redial and self.the_service:
                self.loop.run_until_complete(self.BTconnect()) 

    def connect(self):
        if not self.client:
            print("Scanning for BLE devices...")
            self.loop = asyncio.get_event_loop()
            self.loop.run_until_complete(self.BTsearch())

            if self.the_device:
                print(f"there is Mariola at {self.the_device}")
                self.loop.run_until_complete(self.BTgetServices())
                if self.the_service:
                    print(f"Found Vendor sercvice at {self.the_service}")
                    self.loop.run_until_complete(self.BTconnect())
            else:
                self.c.configure(bg='red')
        else:
            print("Shall be already connected...")

    def toggle_snap(self):
        self.snap = not self.snap
        if self.snap:
            self.snap_button.configure(background = "green")
        else:
            self.snap_button.configure(background = "lightgray")

    def BLE_sent(self, command):
        self.loop.run_until_complete(self.BTwrite(command))
            
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

    def command(self, cmd, *args):
        # sent the GO command
        message = cmd
        if time() > self.last_cmd_time + self.cmd_delay:
            print(f"sending: {message}")
            self.last_cmd_time = time();
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
        self.inmotion = True

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
        if self.inmotion:
            self.setmove(event)
            self.inmotion = False
        else:
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

    
    # for hand drawing
    def handc_set(self):
        line = self.handc.create_line(0,self.hand_zero[1],200,self.hand_zero[1],
                width=2, fill='red',
                capstyle=ROUND, smooth=TRUE, splinesteps=1)
        self.hand_lines.append(line)

        line = self.handc.create_line(self.hand_zero[0],0,self.hand_zero[0],200,
                width=2, fill='red',
                capstyle=ROUND, smooth=TRUE, splinesteps=1)
        self.hand_lines.append(line)

    def lineend(self, x,y,m,alpha):
        alpha_rad = alpha * math.pi / 180
        xe = m * math.cos(alpha_rad) + x
        ye = m * math.sin(alpha_rad) + y
        return xe, ye

    def setHandIK(self, x, y, h=0, a0=60):
        """
        Inverse Kinematic model for hands
        x,y palm position in relative space (-1,-1) to (1,1)
        h - hand number
        """
        m = math.sqrt(x**2 + y**2)

        if m > 1:
            fi = 180
        else:
            fi = math.degrees(math.acos((m**2 -0.25 -0.25)/(-0.5)))
        
        dzetta = math.degrees(math.atan2(-y,-x)) 
        if dzetta < 0: dzetta = 360 + dzetta

        psi = dzetta - (180 - fi)/2
        psi -= a0
        psi = max(0,min(180,psi))
        fi = max(0,min(180,fi))

        cmd_str = '' 

        if h==0 or h==3:
            cmd_str +=  f'<41,0,{int(max(0,180-(psi)))},0> <41,1,{int(fi)},0>'
            cmd_str += ' '

        if h==1 or h==3:
            cmd_str += f'<41,15,{int(max(0,(psi)))},0> <41,14,{int(180-fi)},0>'
        
        return cmd_str

        

    def setHand(self, event):
        x = event.x
        y = event.y
        x0, y0 = self.hand_zero[0],self.hand_zero[1]
        xx = x - x0
        yy = y - y0
        L1 = 50
        L2 = 50
        m_max = L1 + L2
        m = math.sqrt(xx*xx+yy*yy) / m_max
        r = 3

        if self.hand_point: self.handc.delete(self.hand_point)
        for l in self.hand_lines: self.handc.delete(l)

        self.handc_set()

        self.hand_point = self.handc.create_oval(x-r,y-r,x+r,y+r,outline='black',
                fill=None, width=2)

        line = self.handc.create_line(x0,y0,x,y,
                width=1, fill='red',
                capstyle=ROUND, smooth=TRUE, splinesteps=1)
        self.hand_lines.append(line)

        # A = 40
        A = math.degrees(math.atan2(y-y0, x-x0))
        if A < 0: A = 360 + A 
        dzetta = A #- self.hand_Alfa0

        if m > 1:
            fi = 180
        else:
            fi = math.degrees(math.acos(((m*m_max)**2 - L1*L1 - L2*L2)/(-2*L1*L2)))

        psi = dzetta - (180 - fi)/2

        fi = max(0,min(180,fi))

        ik_output = self.setHandIK(-xx/m_max, -yy/m_max, h=3,a0=60)
        print(f'IK: {ik_output}')

        print(f'm: {m} A:{A} fi:{fi} psi:{psi}')
        # cmd_str =  f'<41,0,{int(max(0,180-(psi-60)))},0> <41,1,{int(fi)},0>'
        # cmd_str += ' '
        # cmd_str += f'<41,15,{int(max(0,(psi-60)))},0> <41,14,{int(180-fi)},0>'
        self.command(ik_output)

        psi = max(self.hand_Alfa0,min(180+self.hand_Alfa0,psi))

        B = 90
        A1 = self.hand_Alfa0 + A
        A1 = max(self.hand_Alfa0, A)
        B1 = A1 + 180 - B

        xe,ye = self.lineend(x0,y0,50,psi) #self.hand_Alfa0)

        line = self.handc.create_line(x0,y0,xe,ye,
                width=4, fill='green',
                capstyle=ROUND, smooth=TRUE, splinesteps=1)
        self.hand_lines.append(line)

        xf,yf = self.lineend(xe,ye,50,psi + 180 - fi) #self.hand_Alfa0)

        line = self.handc.create_line(xe,ye,xf,yf,
                width=3, fill='blue',
                capstyle=ROUND, smooth=TRUE, splinesteps=1)
        self.hand_lines.append(line)
    
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
    window = Paint()
    if window.loop:
        window.loop.run_until_complete(window.BTdisconnect())
