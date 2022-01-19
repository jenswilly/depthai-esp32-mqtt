# ESP-MQTT and OAK-D-IoT-40 sample application
This example is a combination of the ESP32 example, "examples/protocols/mqtt/tcp" and the Luxonis example "esp32-spi-message-demo/parse_meta".

The DepthAI device runs inference and sends detections via SPI to the ESP32.

When the ESP32 receives an SPI message, the detections are decoded and sent to an MQTT broker.

### MQTT
Detections are sent as JSON on the `topic/detections` topic. 

Example JSON:

```
{
  "detections": [
    {
      "category": "sofa",
      "confidence": 0.6181640625,
      "label": 18
    }
  ],
  [
    {
      "category": "person",
      "confidence": 0.12640625,
      "label": 1
    }
  ]
}
```

## ESP32 side

### Configure the project

* Open the project configuration menu (`idf.py menuconfig`)
* Configure Wi-Fi or Ethernet under "Example Connection Configuration" menu
* Configure MQTT broker under "Example Configuration"

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py build
idf.py -p  /dev/cu.usbserial-1430 flash monitor
```

(To exit the serial monitor, type `Ctrl-T X`.)

## DepthAI side

Connect the OAK-D-IoT-40 device using an USB C cable.

Install prerequisites or activate appropriate Conda environment.

Go to the `esp32-spi-message-demo/parse_meta` directory and run:

```
python3 main.py
```

to start inference on the DepthAI device.


