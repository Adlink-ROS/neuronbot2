#!/usr/bin/env python3
import serial ## pip install pyserial
import time
import sys, getopt
import atexit

class Strip:
    def __init__(self, led_port, num):
        # led_port : port for the NeuronBot LED ttyUSB
        # baudrate : baud rate
        # num : the number of LED units
        baudrate=115200
        ser = serial.Serial(led_port, baudrate)
        if ser.isOpen():
            self.serial=ser
            self.num=num
            a=self.serial.readline()
            self.read=a.decode("utf-8")
            print(self.read)
        else :
            print('NeuronBot LED port is not opened.')

    def setPixelColor(self,i,r,g,b):
        ## i : the serial number for a LED unit, starting from No.0
        ## r : the intensity of red light
        ## g : the intensity of green light
        ## b : the intensity of blue light

        i_bytes = bytes(str(i), encoding='utf8')
        self.serial.write(b'i'+i_bytes)
        r_bytes = bytes(str(r), encoding='utf8')
        self.serial.write(b'r'+r_bytes)
        g_bytes = bytes(str(g), encoding='utf8')
        self.serial.write(b'g'+g_bytes)
        b_bytes = bytes(str(b), encoding='utf8')
        self.serial.write(b'b'+b_bytes+b'\n')
        return 0

    def delay(self,delay_time):
        # time : minisecond (10^-3)
        time.sleep(delay_time*0.001)
        return 0

    def show(self):
        show_bytes = bytes(str("show"), encoding='utf8') 
        self.serial.write(b'show\n')
        return 0

    def clear(self) :
        num=self.num
        for i in range(num) : 
            self.setPixelColor(i,0,0,0)
        self.show()

    def close(self):
        self.serial.close()
        return 0


    def breath(self):
        max_valx_val=180
        v=0
        num=self.num
        slope=6
        wait=50
        self.clear()

        while v<max_valx_val :
            for i in range(num):
                self.setPixelColor(i,v,v,v )
            self.show()
            self.delay(wait)
            v=v+slope

        while v>0 :
            v=v-slope
            for i in range(num):
                self.setPixelColor(i,v,v,v )
            self.show()
            self.delay(wait)
        return 0

    def forward(self,number=5):    
        # number : the number of lighting LED units
        max_val=180
        num=self.num
        wait=50
        gap=int(max_val/number)
        mid=int((number/2)+1)
        self.clear()
        t=int(1-mid)
        while t<num+mid :
            for i in range(int(1+(number/2))):
                self.setPixelColor((t+i), 0,(max_val-i*gap),(max_val-i*gap) )
                self.setPixelColor((t-i), 0,(max_val-i*gap),(max_val-i*gap ))
            self.setPixelColor( t-int(1+(number/2)), 0,0,0)
            self.show()
            self.delay(wait)
            t=t+1
        return 0


    def backward(self,number=5): 
        # number : the number of lighting LED units
        max_val=180
        num=self.num
        number=5
        wait=50
        gap=int(max_val/number)
        mid=int((number/2)+1)
        self.clear()
        t=int(num+mid-1)
        while t>(0-mid) :
            for i in range(int(1+(number/2))):
                self.setPixelColor((t+i), (max_val-i*gap),(max_val-i*gap),0 )
                self.setPixelColor((t-i), (max_val-i*gap),(max_val-i*gap),0)
            self.setPixelColor(t+(1+(number/2)), 0,0,0)
            self.show()
            self.delay(wait)
            t=t-1
        self.setPixelColor(0, 0,0,0)
        self.show()
        self.delay(wait)
        return 0
    
    def blink(self):
        wait=200
        num=self.num
        times=5
        self.clear()
        for t in range(times):
            for i in range(num):
                    self.setPixelColor(i,50,0,50 )
            self.show()
            self.delay(wait)
            self.clear()
            self.delay(wait)
        return 0

    def blink_red(self):
        wait=200
        num=self.num
        times=5
        self.clear()
        for t in range(times):
            for i in range(num):
                    self.setPixelColor(i,128,0,0)
            self.show()
            self.delay(wait)
            self.clear()
            self.delay(wait)
        return 0        

    def mode_clear(self, num):
        self.clear()      
        self.delay(100)  

    def mode_white(self, num):
        self.clear()      
        self.delay(100)  
        for i in range(num):
            self.setPixelColor(i,128,128,128)                
        self.show()
        self.delay(100)  

    def mode_amber(self, num):
        self.clear()      
        self.delay(100)  
        for i in range(num):
            self.setPixelColor(i,255,60,0)                
        self.show()
        self.delay(100)  

    def mode_red(self, num):
        self.clear()        
        self.delay(100)
        for i in range(num):
            self.setPixelColor(i,128,0,0)        
        self.show()
        self.delay(100)  
        
    def mode_green(self, num):
        self.clear()        
        self.delay(100)
        for i in range(num):
            self.setPixelColor(i,0,128,0)
        self.show()
        self.delay(100)  

    def mode_blue(self, num):
        self.clear()      
        self.delay(100)  
        for i in range(num):
            self.setPixelColor(i,0,0,128)                
        self.show()
        self.delay(100)  

    def mode_rainbow(self, num):
        for j in range(256):
            for i in range(num):
                value = (i+j) & 255                
                if value < 85:
                    self.setPixelColor(i, value*3, 255-value*3, 0)
                elif value < 170:
                    self.setPixelColor(i, 255-value*3, 0, value*3)
                else:
                    self.setPixelColor(i, 0, value*3, 255-value*3)
            self.show()
            self.delay(20)

    def demo(self):
        self.clear()
        print("clear")
        self.delay(1000)
        self.setPixelColor(1,128,0,0)
        print("No.1 unit is red")
        self.show()
        self.delay(1000)
        self.setPixelColor(2,0,128,0)
        print("No.2 unit is green")
        self.show()
        self.delay(1000)
        self.setPixelColor(3,0,0,128)
        print("No.3 unit is blue")
        self.show()
        self.delay(1000)
        self.clear()
        print("clear")
        self.delay(1000)
        print("blink")
        self.blink()
        print("move from the end to the start")
        self.backward()
        self.delay(1000)
        print("move from the start to the end")
        self.forward()
        self.delay(1000)
        print("breath")
        self.breath()
        self.clear()
        self.close()
        print("close port")

        return 0

def set_led(port, num, mode):
    s = Strip(port, num)
    if mode == 0:
        s.mode_clear(num)
    if mode == 1:
        s.mode_white(num)
    if mode == 2:
        s.mode_amber(num)
    if mode == 3:
        s.mode_red(num)
    if mode == 4:
        s.mode_green(num)
    if mode == 5:
        s.mode_blue(num)
    if mode == 6:
        s.mode_rainbow(num)
    if mode == 7:
        s.breath()
    if mode == 8:
        s.forward(num)
        s.backward(num)
    if mode == 9:
        s.blink_red()
    s.close()

# restore_led works only when program exits
@atexit.register
def restore_led():
    port = '/dev/neuronbotLED'
    num = 10
    mode = 2 # restore to amber
    set_led(port, num, mode)

def main():
    port = '/dev/neuronbotLED'
    num = 10
    mode = 5
    loop = False
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hlp:n:m:",["port=", "num=", "mode=", "loop"])
    except getopt.GetoptError:
        print ("python led_control.py -p <LED ttyUSB> -n <Number of LED units> -m <LED mode>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ("python led_control.py -p <LED ttyUSB> -n <Number of LED units> -m <LED mode>")
            sys.exit()
        elif opt in ("-p", "--port"):
            port = arg
        elif opt in ("-n", "--num"):
            num = int(arg)
        elif opt in ("-m", "--mode"):
            mode = int(arg)            
        elif opt in ("-l", "--loop"):
            loop = True

    if port=='' or num == '':
        print ("python led_control.py -p <LED ttyUSB> -n <Number of LED units>")
        sys.exit()

    set_led(port, num, mode)

    while (loop):
        # loop until receiving signal interrupt
        time.sleep(2)

if __name__ == "__main__":
    main()

