โปรเจคนี้เป็นโปรเจคที่ผมตั้งใจทำขึ้นมาหลังจากที่ผมได้ดูหนังหุ่นยนต์และเห็นบริษัทที่ทำหุ่นยนต์เจ๋งๆ เช่น รีลสตีล,Cyberpunk:Edgerunners,guardians of the galaxy,Boston dinamic, tesla, nasa และอีกหลายๆเรื่องและหลายๆบริษัท ผมจึงอยากสร้างหุ่นยนต์ของตัวเองดูครับ

# Components in this project

- Jetson nano developer kit
- Smart drive duo 30
- Raspberry pi camera
- Motor 12V DC
- Encoder 330 ppr
- RPLiDAR S1
- esp32

# ros2_smartdriveduo30_controller

today im starting with my new robot project (next gen of kalwtummad2).

but these code is just a small path of this project

so ,this path of project represent how to control smartdrive duo 30 using ros2 on jetson nano.

the concept to control smartdrive duo 30 is:

- 00000000 - 00111111 (0-63) is left wheel go forward
- 01000000 - 01111111 (64-127) is left wheel go backward
- 10000000 - 10111111 (128-191) is right wheel go forward
- 11000000 - 11111111 (192-255) is right wheel go backward

you can see code in /software/jetson/test_ros2/

- file "driver_service_client.py" is ros2 client to interact with user
- file "driver_service_server.py" is ros2 server control

# assembly
<img width="656" alt="9_12_2023_esp32_duo30_encoder_ultrasonic" src="https://github.com/aelde/mobileRobot_remake_jetson_esp32/assets/79216582/56ddf5d5-e786-4f22-8d9e-3e63d2c566e5">
