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

        self.choose_size_button = Scale(self.root, from_=1, to=100, orient=VERTICAL)
        self.choose_size_button.set(10)
        self.choose_size_button.grid(row=1, column=9, rowspan=5,sticky='NSEW')

        self.c = Canvas(self.root, bg='white', width=800, height=800)
        self.c.grid(row=1, columnspan=9, rowspan=5)

        # somestuff to try straightline things
        self.points = [];
        self.lines = [];
        self.arclines = [];
        self.origin = None;

        self.commands = [];
        
        #  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
        try:
            self.ser = serial.Serial('/tmp/ttyBLE', timeout=1)
            self.ser.baudrate = 9600
        except:
            print("Serial BT port error...")

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
        self.c.bind('<B1-Motion>', self.pointer)
        self.c.bind('<ButtonRelease-1>', self.pointerUp)
        # trying to makepossible to draw arcs
        # self.c.bind('<ButtonRelease-3>', self.pointerArc)

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
        if not self.old_x and not self.old_y:
            _x = event.x
            _y = event.y 
            
            self.old_x = _x
            self.old_y = _y 
            
            self.points.append((_x, _y))

            self.origin = self.c.create_oval(_x-5, _y-5, _x+5, _y+5, outline='red',
            fill=None, width=2)

        else:
            self.lines.append(self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.line_width, fill='red',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36))
            self.old_x = event.x
            self.old_y = event.y
            self.points.append((event.x, event.y))

            # figuring out the propper command for this click
            origin_x, origin_y = self.points[0]
            iA = 90;
            self.drivescale = self.choose_size_button.get()

            # handling this very point

            point = (event.x, event.y)
            cX = point[0]
            cY = point[1] 

            dX = point[0] - (self.points[-2][0])
            dY = (self.points[-2][1]) - point[1]

            dL = math.sqrt( dX**2 +dY**2 ) * self.drivescale / 100
            
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
            sleep(0.02)

    
            
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

    def clear(self):
        self.c.delete(self.origin)
        for lin in self.lines:
            self.c.delete(lin)

        self.points = [];
        self.lines = [];
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
            del self.lines[-1]
            del self.points[-1]
            del self.commands[-1] # 2x as we add two commands in each step.
            del self.commands[-1]

            if len(self.lines) == 0:
                self.old_x, self.old_y = None, None
                self.c.delete(self.origin)
                self.origin = None
            else:
                self.old_x, self.old_y = self.points[-1]

if __name__ == '__main__':
    Paint()
