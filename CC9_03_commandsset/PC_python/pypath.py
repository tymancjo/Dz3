from tkinter import *
from tkinter.colorchooser import askcolor
import math
import serial
from time import sleep


class Paint(object):

    DEFAULT_PEN_SIZE = 5.0
    DEFAULT_COLOR = 'black'

    def __init__(self):
        self.root = Tk()
        self.root.title('Dz3 Path Maker')

        self.pen_button = Button(self.root, text='undo', command=self.undo)
        self.pen_button.grid(row=0, column=0)

        self.brush_button = Button(self.root, text='make code', command=self.makecode)
        self.brush_button.grid(row=0, column=1)

        self.color_button = Button(self.root, text='serial', command=self.sent_serial)
        self.color_button.grid(row=0, column=2)

        self.eraser_button = Button(self.root, text='GO', command=self.go)
        self.eraser_button.grid(row=0, column=3)

        self.eraser_button = Button(self.root, text='cl.Prog', command=self.clearDz3)
        self.eraser_button.grid(row=0, column=5)

        self.eraser_button = Button(self.root, text='Clear', command=self.clear)
        self.eraser_button.grid(row=0, column=6)

        self.choose_size_button = Scale(self.root, from_=1, to=10, orient=HORIZONTAL)
        self.choose_size_button.grid(row=0, column=4)

        self.c = Canvas(self.root, bg='white', width=800, height=800)
        self.c.grid(row=1, columnspan=9)

        # somestuff to try straightline things
        self.points = [];
        self.lines = [];
        self.origin = None;
        
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
        self.c.bind('<ButtonRelease-3>', self.pointerArc)

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
            # skoro to juz kolejny klik, rysujemy elipsę
            _x = event.x
            _y = event.y 

            

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


    def makecode(self):
        origin_x, origin_y = self.points[0]
        iA = 90;
        self.drivescale = self.choose_size_button.get()

        for i,point in enumerate(self.points[1:]):
            cX = point[0] - origin_x
            cY = origin_y - point[1] 
            # i-dlatego ze startujemy i=0 liczac od drugiego elementu
            dX = point[0] - self.points[i][0]
            dY = self.points[i][1] - point[1]
            dL = math.sqrt( dX**2 +dY**2 ) * self.drivescale / 10
            
            # dA = math.degrees(math.acos(dX/dL)) - 90 # -90 to make top a 0 azimuth
            dA = math.degrees(math.atan2(dY, dX))
            A = dA - iA
            if A < -180: 
                A = 360 + A
            elif A > 180:
                A = A - 360

            iA = dA

            print(f'{i} : {cX}:{cY}, L:{dL}, iA:{A}  A:{dA}')

    def sent_serial(self):

        if len(self.points) > 1:
            origin_x, origin_y = self.points[0]
            iA = 90;
            self.drivescale = self.choose_size_button.get()

            for i,point in enumerate(self.points[1:]):
                cX = point[0] - origin_x
                cY = origin_y - point[1] 
                # i-dlatego ze startujemy i=0 liczac od drugiego elementu
                dX = point[0] - self.points[i][0]
                dY = self.points[i][1] - point[1]
                dL = math.sqrt( dX**2 +dY**2 ) * self.drivescale / 10
                
                # dA = math.degrees(math.acos(dX/dL)) - 90 # -90 to make top a 0 azimuth
                dA = math.degrees(math.atan2(dY, dX))
                A = dA - iA
                if A < -180: 
                    A = 360 + A
                elif A > 180:
                    A = A - 360

                iA = dA

                message = f'<20,0,{int(A)},0>'
                self.ser.write(message.encode('utf-8'))
                print(message.encode())
                sleep(0.1)
                message = f'<20,{int(dL)},0,0>'
                self.ser.write(message.encode('utf-8'))
                print(message.encode('utf-8'))
                sleep(0.1)
            
    def go(self):
        message = f'<30,0,0,0>'
        # self.ser.write(message)
        self.ser.write(message.encode('utf-8'))
        # print(message.encode('utf-8'))
        print(message)

    def clear(self):
        self.c.delete(self.origin)
        for lin in self.lines:
            self.c.delete(lin)

        self.points = [];
        self.lines = [];
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
            
            if len(self.lines) == 0:
                self.old_x, self.old_y = None, None
                self.c.delete(self.origin)
                self.origin = None
            else:
                self.old_x, self.old_y = self.points[-1]

if __name__ == '__main__':
    Paint()
