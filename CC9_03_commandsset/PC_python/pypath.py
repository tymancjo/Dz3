from tkinter import *
from tkinter.colorchooser import askcolor
import math
import serial
from time import sleep

# ogólne funkcje
def elliptical_arc(canvas, x, y, r1, r2, t0, t1, width):
    return canvas.create_arc(x-r1, y-r2, x+r1, y+r2, start=t0, extent=t1-t0,
                             style='arc', width=width)

   

class Paint(object):

    DEFAULT_PEN_SIZE = 5.0
    DEFAULT_COLOR = 'black'
    WIDTH = 800
    HEIGHT = 800

    def __init__(self):
        self.root = Tk()
        self.root.title('Dz3 Path Maker')

        self.pen_button = Button(self.root, text='undo', command=self.undo)
        self.pen_button.grid(row=0, column=0)

        # self.brush_button = Button(self.root, text='make code', command=self.makecode)
        # self.brush_button.grid(row=0, column=1)

        self.color_button = Button(self.root, text='Upload', command=self.sent_serial)
        self.color_button.grid(row=0, column=1)

        self.go_button = Button(self.root, text='GO', command=self.go)
        self.go_button.grid(row=0, column=2)

        self.loop_button = Button(self.root, text='GO Loop', command=self.goLoop)
        self.loop_button.grid(row=0, column=3)

        self.S_button = Button(self.root, text='STOP', command=self.S)
        self.S_button.grid(row=0, column=4)

        self.eraser_button = Button(self.root, text='cl.Prog', command=self.clearDz3)
        self.eraser_button.grid(row=0, column=5)

        self.eraser_button = Button(self.root, text='Clear', command=self.clear)
        self.eraser_button.grid(row=0, column=6)

        self.choose_size_button = Scale(self.root, from_=1, to=100, orient=VERTICAL, command=self.theGrid)
        self.choose_size_button.set(36)
        self.choose_size_button.grid(row=1, column=9, rowspan=5,sticky='NSEW')

        self.c = Canvas(self.root, bg='white', width=self.WIDTH, height=self.HEIGHT)
        self.c.grid(row=1, columnspan=9, rowspan=5)

        # somestuff to try straightline things
        self.points = [];
        self.lines = [];
        self.joints = [];
        self.txt = [];
        self.arclines = [];
        self.origin = None;

        self.commands = [];

        # stuff for grid
        self.gridlines= [];

        # moving the world around
        self.translate = (0,0);
        self.oldtranslate= (0,0);
        self.intranslate = False;
        self.translateclick= (0,0);
        
        #  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
        try:
            self.ser = serial.Serial('/tmp/ttyBLE', timeout=1)
            self.ser.baudrate = 9600
        except:
            print("Serial BT port error...")

        self.setup()
        self.root.mainloop()

    def zoom(self,event):
        self.choose_size_button.set(self.choose_size_button.get() + event.delta);
        pass

    def zoomIn(self, event):
        self.choose_size_button.set(self.choose_size_button.get() + 1);
        pass

    def zoomOut(self,event):
        self.choose_size_button.set(self.choose_size_button.get() - 1);
        pass

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = 5
        self.drivescale = self.choose_size_button.get()
        self.color = self.DEFAULT_COLOR
        self.eraser_on = False
        self.active_button = self.pen_button
        self.c.bind('<B1-Motion>', self.pointer)
        self.c.bind('<ButtonRelease-1>', self.pointerUp)
        self.c.bind('<B3-Motion>', self.movecanvas)
        self.c.bind('<Button-3>', self.setmove)
        self.c.bind('<MouseWheel>', self.zoom)
        self.c.bind('<Button-4>', self.zoomIn)
        self.c.bind('<Button-5>', self.zoomOut)

        # trying to makepossible to draw arcs
        # self.c.bind('<ButtonRelease-3>', self.pointerArc)
        self.theGrid(self.choose_size_button.get());
    
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
        # x0 = x
        # y0 = y

        x += (event.x / localscale) - x0
        y += (event.y / localscale) - y0 
        
        self.translate = (x, y)

        self.theGrid(1)  

    def projection(self,point):
        #  grabbing the scale
        localscale = 1/(self.choose_size_button.get() /100)
        shift_x, shift_y = self.translate;

        x,y = point;
        #  moving and scaling
        x = (x + shift_x) * localscale;
        y = (y + shift_y) * localscale;

        return (x, y)

    def theGrid(self, scaleval):
        
        self.scrClr();

        if len(self.gridlines) > 0:
            for gline in self.gridlines:
                self.c.delete(gline)

        self.reDraw();

        spc100cm = 10 * 100 / self.choose_size_button.get() #100cm in pixels


        nX = self.WIDTH // spc100cm
        nY = self.HEIGHT // spc100cm

        print(spc100cm,nX);

        

        for n in range(int(2*nX)):
            
            sx, sy = self.projection((10 * (n-1) - 10*(nX // 2), 10 * (n-1) - 10*(nX // 2)))

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


        # for x in range(int(nX+2)):
        #     kolor = 'gray'
        #     grubosc = 1
        #     if (x % 10 == 0):
        #         kolor = 'blue'
        #         grubosc = 2

        #     self.gridlines.append( self.c.create_line( int(x*spc100cm), 0, int(x*spc100cm), self.HEIGHT,
        #                        width=grubosc, fill=kolor,
        #                        capstyle=ROUND, smooth=TRUE, splinesteps=36) )
        
        # for y in range(int(nY+2)):
        #     kolor = 'gray'
        #     grubosc = 1
        #     if (y % 10 == 0):
        #         kolor = 'blue'
        #         grubosc = 2

        #     self.gridlines.append( self.c.create_line( 0, int(y*spc100cm), self.WIDTH, int(y*spc100cm),
        #                        width=grubosc, fill=kolor,
        #                        capstyle=ROUND, smooth=TRUE, splinesteps=36) )


    def _create_arc(self, canvas, p0, p1, angle):
        extend_x = (self._distance(p0,p1) -(p1[0]-p0[0]))/2 # extend x boundary 
        extend_y = (self._distance(p0,p1) -(p1[1]-p0[1]))/2 # extend y boundary
        
        startAngle = math.atan2(p0[0] - p1[0], p0[1] - p1[1]) *180 / math.pi # calculate starting angle  
        
        canvas.create_arc(p0[0]-extend_x, p0[1]-extend_y , 
                               p1[0]+extend_x, p1[1]+extend_y, 
                               extent=angle, start=90+startAngle, style=ARC)

        '''use this rectangle for visualisation'''
        #self.canvas.create_rectangle(p0[0]-extend_x, p0[1]-extend_y, 
        #                                p1[0]+extend_x, p1[1]+extend_y)       

    def _distance(self, p0, p1):
        '''calculate distance between 2 points'''
        return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

    def activate_button(self, some_button, eraser_mode=False):
        self.active_button.config(relief=RAISED)
        some_button.config(relief=SUNKEN)
        self.active_button = some_button
        self.eraser_on = eraser_mode

    def reset(self, event):
        self.old_x, self.old_y = None, None

    def pointer(self,event):
        # if not self.old_x and not self.old_y:
        #     self.old_x = event.x
        #     self.old_y = event.y
        pass 

    def pointerArc(self, event):
        # lets check if this is maybe the first click
        if not self.old_x and not self.old_y:
            _x = event.x
            _y = event.y 
            
            self.old_x = _x
            self.old_y = _y 
            
            self.points.append((_x, _y))

            self.origin = self.c.create_oval(_x-5, _y-5, _x+5, _y+5, outline='red',
            fill=None, width=2)

        else:
            # skoro to juz kolejny klik, rysujemy elipsę przez approx przez dwa odcinki
            _x = event.x
            _y = event.y 

            self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.line_width, fill='blue',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36)

            self.arclines.append((_x,_y)) 
            self.old_x = event.x
            self.old_y = event.y
            
            # jeżeli to juz drugi odcinek generujemy łuk
            if len(self.arclines) == 2:
                
                p1 = (self.points[-1][0], self.points[-1][1])
                p0 = (self.arclines[-1][0], self.arclines[-1][1])

                self._create_arc(self.c, p0, p1,90)
                
                self.arclines = []


    def pointerUp(self,event):
        self.drivescale = self.choose_size_button.get() /100
        shift_x, shift_y = self.translate;

        if not self.old_x and not self.old_y:
            _x = event.x * self.drivescale - shift_x
            _y = event.y * self.drivescale - shift_y 
            
            self.old_x = _x
            self.old_y = _y 
            
            self.points.append((_x, _y))

            _x, _y = self.projection((_x, _y))   

            self.origin = self.c.create_oval(_x-5, _y-5, _x+5, _y+5, outline='red',
            fill=None, width=2)

        else:
            _x = event.x * self.drivescale - shift_x
            _y = event.y * self.drivescale - shift_y
            
            
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
            
            dA = math.degrees(math.atan2(dY, dX))
            if len(self.commands) > 0:
                iA = self.commands[-1][3]
            else:
                iA= 90

            A = dA - iA

            if A < -180: 
                A = 360 + A
            elif A > 180:
                A = A - 360

            # adding thisline as 2 commands - 1-turn - 2 move
            self.commands.append((0,A,0,dA))
            print(self.commands[-1])
            self.commands.append((dL,0,0,dA))
            print(self.commands[-1])


    

    def sent_serial(self):
        # first sent the set of sequence commands
        for command in self.commands:
            message = f'<20,{int(command[0])},{int(command[1])},{int(command[2])}>'
            self.ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))
            sleep(0.05)

    
            
    def go(self):
        # sent the GO command
        message = f'<30,0,0,0>'
        self.ser.write(message.encode('utf-8'))
        print(message.encode('utf-8'))

    def S(self):
        # sent the GO command
        message = f'<0,0,0,0>'
        self.ser.write(message.encode('utf-8'))
        print(message.encode('utf-8'))
  
    def goLoop(self):
        # sent the GO command
        message = f'<33,0,0,0>'
        self.ser.write(message.encode('utf-8'))
        print(message.encode('utf-8'))

    def scrClr(self):
        self.c.delete(self.origin)

        for lin in self.lines:
            self.c.delete(lin)
        for txt in self.txt:
            self.c.delete(txt)
        for joint in self.joints:
            self.c.delete(joint)

    def reDraw(self):
        
        # drawing origin oval
        if len(self.points) > 0:
            _x, _y = self.projection(self.points[0]) 
            
            self.origin = self.c.create_oval(_x-5, _y-5, _x+5, _y+5, outline='red',
                fill=None, width=2)
            
            for i,point in enumerate(self.points):
                if i > 0:
                    cX, cY = self.projection(point)
                    self.joints.append( self.c.create_oval(cX-5, cY-5, cX+5, cY+5, outline='black',
                                        fill=None, width=2) )
                    
                    _x,_y = self.projection(self.points[i-1])

                    self.lines.append(self.c.create_line(_x, _y, cX, cY,
                               width=self.line_width, fill='red',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36))

                    mX = (_x + cX) / 2
                    mY = (_y + cY) / 2

                    dL = self._distance(self.points[i-1], point)

                    self.txt.append(self.c.create_text(mX,mY, fill="black",font="20",
                        text=f"{int(dL)}cm"))



        # for lin in self.lines:
        #     self.c.delete(lin)
        # for txt in self.txt:
        #     self.c.delete(txt)
        # for joint in self.joints:
        #     self.c.delete(joint)


    def clear(self):
        self.scrClr();

        self.points = [];
        self.lines = [];
        self.txt = [];
        self.commands = [];
        self.origin = None;

        self.old_x = None
        self.old_y = None

    def clearDz3(self):
        
        message = f'<39,0,0,0>'
        # self.ser.write(message)
        self.ser.write(message.encode('utf-8'))
        # print(message.encode('utf-8'))
        print(message)


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

            if len(self.lines) == 0:
                self.old_x, self.old_y = None, None
                self.c.delete(self.origin)
                self.origin = None
            else:
                self.old_x, self.old_y = self.points[-1]

if __name__ == '__main__':
    Paint()
