# 🤖 Autonomous Ping Pong Collector

An autonomous robot designed to detect, collect, and avoid obstacles while retrieving stray ping pong balls using machine learning, computer vision, and robotics.

---

## 📸 Robot Gallery
*A closer look at the build and design*

![Bot Overview](bot_images/image_1.jpg)

---

## 🎬 Demo
Watch the robot in action:

![Demo](bot_demo/demo_1_gif)

---

## 🧠 Introduction
Meet the **Autonomous Ping Pong Collector**, a robot built to find and fetch rogue ping pong balls (preferably orange) using computer vision and machine learning.

No more post-match ball hunts—this bot navigates obstacles like a pro, making ball retrievers a thing of the past. The goal of this project is to demonstrate that even simple, repetitive tasks can be automated, so you can focus on what really matters: arguing over serves or gracefully losing the match.

---

## 🧰 Component List

| Item | Quantity |
|------|----------|
| NVIDIA Jetson Nano 4GB | 1 |
| SD-Card | 1 |
| Anker Portable Battery (5V/4A) | 1 |
| Arduino Nano | 1 |
| Arducam IMX519 | 1 |
| Slamtec RPLIDAR A1M8 | 1 |
| TT Motor (3–6V, 150–160 mA) | 1 |
| DC12V Reversible High Torque Gear Motor (62 RPM) | 1 |
| L298n Motor Driver (5V logic / 36mA) | 2 |
| 9g Servo (4.8–6V / 100–250 mA) | 1 |
| PCA9685 (2.3–5.5V / 25mA per servo) | 1 |
| 3mm Clear Acrylic (12"x12") | 6 |
| 3mm Clear Acrylic (12"x24") | 2 |
| 18650 2600 mAh Batteries | 4 |
| 18650 Battery Charger (4-slot) | 1 |
| 3.1A QC Micro USB Cable (3 Pack) | 1 |
| 16-Channel PWM Servo Driver | 1 |
| SG90 Servo Motor Kit (5 Pcs) | 1 |
| Keeyes L298n and Motor Kit | 1 |
| M3 x 10mm Phillips Screws (10 Pcs) | 1 |
| M3 x 20mm Phillips Screws (10 Pcs) | 1 |
| M3 x 25mm Phillips Screws (10 Pcs) | 1 |
| M3 Machine Hex Nuts (10 Pcs) | 2 |
| M2.5 x 6mm Phillips Screws (10 Pcs) | 1 |
| M2.5 x 20mm Phillips Screws (10 Pcs) | 1 |
| M2.5 Machine Hex Nuts | 2 |
| Pololu 1/2" Ball Caster (Plastic) | 2 |
| Pololu 3/8" Ball Caster (Metal) | 2 |
| SparkFun 22AWG Red Wire | 1 |
| SparkFun 22AWG Black Wire | 1 |
| 4x 18650 Battery Holder | 1 |
| 2 oz Applicator Bottle | 5 |
| Weld-On 4 (Fast Set) | 1 |
| Hitachi Brass Standoffs | 7 |
| 20mm Fuse (3A, 2-Pack) | 1 |
| SPST Toggle Switch | 1 |
| 20mm Fuse Holder | 1 |

---

## 🔧 Future Work & Improvements

- **Increase LiDAR Clearance**: Fine-tune the obstacle detection threshold (e.g., from 450 mm to 475 mm) or apply a dynamic lateral offset for better collision avoidance. Relocating the LiDAR to a more central position could also improve detection at shallow angles.

- **Refine Scoop Mechanism**: Adjust the sweeper arm’s angle and add a longer, more flexible lip to improve first-contact ball collection. Alternatively, explore a new mechanism such as vacuum-based suction.

- **Upgrade to Jetson Orin Nano**: The Jetson Nano 4GB limits performance and model complexity (e.g., restricted to MobileNet SSD v2). Upgrading to the Orin Nano would support real-time inference with more powerful models like YOLOv8. Also, the Jetson Nano has been officially discontinued by NVIDIA.

- **Add IMUs and Wheel Encoders**: The current setup uses LiDAR for reactive navigation only. Integrating IMUs and encoders would enable odometry, SLAM, and path planning—transforming the bot from a reactive to a globally aware autonomous system.

---

## ✅ Conclusion

We successfully built a robot capable of detecting and collecting ping pong balls while avoiding obstacles in real-time. Despite challenges such as memory limitations on the Jetson Nano and physical design issues like ball deflection off the sweeper blades, several design iterations helped us overcome these problems. This project offered valuable hands-on experience in robotics, machine learning, and iterative engineering—skills we’ll carry forward into our future endeavors.

---

> _Made with caffeine, ping pong balls, and perseverance._
