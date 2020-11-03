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
python led_control.py --port /dev/neuronbotLED --mode 6
```

You can look into `led_control.py` to see what else functions you can use to write your own program.

### ROS Example

Use ROS launch file to run LED python [scripts](https://github.com/Adlink-ROS/neuronbot2/blob/82b93ecd032b68db0be4b9691fee81f7903164c3/neuronbot2_tools/neuronbot2_led/launch/led_control.launch#L2)
```xml
<launch>
	<node name="led_control" pkg="neuronbot2_led" type="led_control.py" args="-p /dev/neuronbotLED -m 5">
	</node>
</launch>
```

### Arduino Sketch

NeuronBot2 LED relies on **Adafruit NeoPixel** library of Arduino. If you are interested in modifying the Arduino sketch, you can get and build LED (Arduino) source code from this GitHb repo: https://github.com/Adlink-ROS/neuronbot2_led
