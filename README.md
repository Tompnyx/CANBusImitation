# CAN Bus Imitation Project

### General Information and Acknowledgements

The following project has been created to imitate a CAN bus of a vehicle. It can:

- Send random CAN messages
- Receive CAN messages
- Imitate a vehicle

The CAN IDs and messages sent have been modelled after the Mazda RX 8, and the project has used [DaveBlackH](https://github.com/DaveBlackH/)'s research into the subject heavily (which can be found [here](https://github.com/DaveBlackH/MazdaRX8Arduino) and is accompanied by a [video walk](https://www.youtube.com/watch?v=HM5zCeXdATo)).

Inspiration to format the outputted messages is taken from [HCRL](https://ocslab.hksecurity.net/welcome)'s work on their CAN Dataset for intrusion detection ([OTIDS](https://ocslab.hksecurity.net/Dataset/CAN-intrusion-dataset)). It is formatted in the following way:
```
[If the message was Sent/Received]: Timestamp:  [Time since initialised in milliseconds]    ID: [Arbitration or CAN ID] DLC:    [Length of the data package sent]   [The data package sent]
```

### How to run

The code in this project was developed to run on the [Arduino Uno Rev3](https://store.arduino.cc/products/arduino-uno-rev3) and the [CAN-BUS Shield V2.0](https://wiki.dfrobot.com/CAN-BUS_Shield_V2__SKU__DFR0370_) (which runs on the [MCP2515 Chip](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf)), but any Arduino and CAN-BUS shield that supports the required libraries should work too.

The libraries used in this project include:

- [ArduinoJson](https://arduinojson.org/) Version 6.19.4
- [SD](https://www.arduino.cc/reference/en/libraries/sd/) Version 1.2.4
- [Seeed Arduino CAN](https://github.com/Seeed-Studio/Seeed_Arduino_CAN) Version 2.3.3

To run on your Arduino, make sure you have set the:

- baud
- SPI_CS pin
- CAN_INT pin
- SD_SPI_CS pin (Optionally, if you are using the **performRoute** option)

To perform the different functionality of the program, four different operational modes have been created. This modes can be switched between via setting the **op** parameter:

- **sendRandom** generates and sends a random CAN message
- **receiveOnly** receives only CAN messages
- **sendRandomAndReceive** is the combination of both **sendRandom** and **receiveOnly**
- **performRoute** generates CAN messages in accordance to a predetermined trip

Note that the **performRoute** mode requires: SD card capabilities, a SD card, and a JSON file containing the trip information on that SD card. The filename of the json file needs to replace the value of the variable **filename**. You will also need to calculate the memory pool (in bytes) needed to deserialize the json file. [ArduinoJsonAssistant](https://arduinojson.org/v6/assistant) is a useful tool for this. For further information, please refer to the [documentation](https://arduinojson.org/v6/doc/deserialization/) for deserialization. An example trip called **trip.json** can be found in the folder **ExampleTrips**.

An optional filter ID can also be set when receiving an Arbitration/ CAN ID.

### Development Environment
The IDE [CLion](https://www.jetbrains.com/clion/) was used with the plugins [PlatformIO for CLion](https://plugins.jetbrains.com/plugin/13922-platformio-for-clion) and [Serial Port Monitor](https://plugins.jetbrains.com/plugin/8031-serial-port-monitor).
