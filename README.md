# Project: skid_robot

## Introduction

The purpose of this project is to serve as a learning experience in developing an autonomous mobile robot controlled using [ROS] (Robotic Operating System). The experience includes desiging and integrating all the hardware and software components required to achieve autonomous operation of an outdoor _coverage task_. A good example of an outdoor coverage task for an autonomous robot is to go around the backyard of a house and cover as much of the area as possible (think of it as a [roomba] for the garden but without the vacuuming).  The Robotic Operating System is a very comprehensive ecosystem that covers many aspects in the operation of autonomous robots and there are a few manufactures that provide hardware platforms with fully operational systems using ROS (see, for example, [Jackal] and [TurtleBot3]). However, I wanted to learn how to design the hardware and develop the software by creating a hardware platform from the ground up.  So, here we go.

## Hardware Components

The following is a list of all the hardware components used in this project:
- 1x 4WD Aluminum Mobile Robot Platform: [AWSRobot]
- 4x Encoder Gearmotors: [motor]
- 1x BNO080: [IMU]
- 1x Raspberry Pi 3B: [RaspberryPi]
- 1x Pi-EzConnect Terminal Block Breakout HAT: [EzConnect]
- 2x Qwiic cable: [QwiicCable]
- 1x Rechargable Lithium-Ion Battery: [Battery]

### 3D Printed parts
- Raspberry PI base
- Component base

## Software Components

The software platform is basically ROS.  There are several platforms available to load on a Raspberry PI. I chose the one offered by Ubiquity Robotics ([ROS-image]). Their image is based on Ubuntu and uses pifi to manage the WiFi network. They provide nice documentation on how to install the image along with troubleshooting tips.

## License

Copyright (c) 2019 Juan C. Santamaria. All rights reserved.

Licensed under the [MIT] license.

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job.)
   [ROS]: https://www.ros.org/
   [roomba]: https://www.irobot.com/roomba
   [Jackal]: https://robots.ros.org/jackal/
   [TurtleBot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
   [AWSRobot]: http://shop.alsrobot.com/index.php?route=product/product&product_id=535
   [motor]: https://www.amazon.com/gp/product/B07GNGQ24C/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
   [IMU]: https://www.sparkfun.com/products/14686
   [RaspberryPi]: https://www.sparkfun.com/products/14643
   [EzConnect]: https://www.adafruit.com/product/2711
   [QwiicCable]: https://www.sparkfun.com/products/14425
   [Battery]: https://www.amazon.com/gp/product/B007RQW5WG/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1
   [ROS-image]: https://downloads.ubiquityrobotics.com/pi.html
   [MIT]: LICENSE
