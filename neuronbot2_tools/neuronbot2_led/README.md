# NeuronBot2_LED (Arduino Nano for example)

Please refer to this GitHib repo: https://github.com/Adlink-ROS/neuronbot2_led

### Arduino IDE Setup and Sketch Upload (This step can be done on a laptop)

Click **Manage Libraries**  
<img src="https://github.com/Adlink-ROS/NeuronBot2_LED/blob/master/images/arduino_manage_library.png">


Install **Adafruit NeoPixel** for the Arduino Board.

<img src="https://github.com/Adlink-ROS/NeuronBot2_LED/blob/master/images/download_adafruit.png">


Set the **Board**, **Processor** and **Port** correctly.

<img src="https://github.com/Adlink-ROS/NeuronBot2_LED/blob/master/images/Nano_BoardSet.png"> 
<img src="https://github.com/Adlink-ROS/NeuronBot_LED/blob/master/images/processorset.png"> 
<img src="https://github.com/Adlink-ROS/NeuronBot2_LED/blob/master/images/portset.png">

Open **Arduino_Nano.ino** in Arduino IDE and click **Verify** and **Upload** buttons to write the sketch into UNO.

<img src="https://github.com/Adlink-ROS/NeuronBot2_LED/blob/master/images/verify.png">
<img src="https://github.com/Adlink-ROS/NeuronBot_LED/blob/master/images/upload.png">

### Hardware Setup (connect the Arduino board and NeuronBot)

Put the USB cable into the USB hub on NeuronBot.

Please find out to which port the Arduino board is connected, e.g., **/dev/neuronbotLED**.  


### Check the function

1. Install the dependence
    ```
    pip install pyserial
    ```
2. Execute the script
    ```
    python led_control.py --port /dev/neuronbotLED --mode 9
    # port : the port where the USB cable is, e.g., /dev/neuronbotLED.
    # mode : test built-in LED mode in this script
    ```
    <img src="https://github.com/Adlink-ROS/NeuronBot_LED/blob/master/demo_nano.gif">

### LED Define

The number of LED unit starts from 0 which is the unit closest to the wire.

<img src="https://github.com/Adlink-ROS/NeuronBot_LED/blob/master/images/nano_led.png">

### Build your own LED function
Import the class **Strip** at the beginning 

For example,
```python
#!/usr/bin/env python
from led_control import Strip
if __name__ == '__main__':
    port='/dev/neuronbotLED' ## Port for the Arduino Board
    num=30 ## The number of LED units
    s=Strip(port,num) ## Initialize the strip.
    
    s.clear() ## Turn all LED units off.
    s.delay(1000) ## Wait 1000 ms to let the strip have enough time to execute the command.
                  ## Feel free to change the delay time.  
    s.setPixelColor(1,180,0,0) ## Define NO.1 LED unit be red with intensity 100.
                               ## We only define the LED unit, but the light is still off.
    s.show() ## Turn on strip.
    s.delay(1000)
    s.demo() ## Display all basic function in class Strip.
    s.close() ## Disconnect the serial port object.
```


