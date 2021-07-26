# NeuronBot2_LED

### Dependency
```sh
pip install pyserial   

# if pip not found, use pip3 instead
pip3 install pyserial
```

### LED Example

```sh
cd <your_neuronbot2_workspace>/src/neuronbot2/neuronbot2_tools/neuronbot2_led/scripts
./led_control.py --port /dev/neuronbotLED --mode 6
```

You can look into `led_control.py` to see what else functions you can use to write your own program.
Currently, we have implemented 10 modes listed as below code snippets:

```python
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
```

### ROS Example

Use ROS launch file to run LED python [scripts](https://github.com/Adlink-ROS/neuronbot2/blob/noetic-devel/neuronbot2_tools/neuronbot2_led/launch/led_control.launch)
```xml
<launch>
	<node name="led_control" pkg="neuronbot2_led" type="led_control.py" args="-p /dev/neuronbotLED -m 5">
	</node>
</launch>
```

### Arduino Sketch

NeuronBot2 LED relies on **Adafruit NeoPixel** library of Arduino. If you are interested in modifying the Arduino sketch, you can get and build LED (Arduino) source code from this GitHb repo: https://github.com/Adlink-ROS/neuronbot2_led
