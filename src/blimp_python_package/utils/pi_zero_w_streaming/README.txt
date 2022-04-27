Setting up and running Pi-side streaming:
sh install_stream.sh
modify "udp_ip" to match server-side ip
python3 open_blimp_stream.py

--------------------------

Setting up and running server-side receiving:
sh install_rcv.sh
python3 stream_rcv.py

--------------------------

Setting up streaming receiver:
1) import ThreadedPiStream class from stream_rcv.py
2) create a ThreadedPiStream object and pass in three UDP ports, first for the camera, second for the imu, third for the altitude sensor
3) query images through get_image() -> stamp, image where stamp is a double and image is a numpy array
4) query imu data through get_bno() -> stamp, bno where stamp is a double and bno is a 2-tuple where the first element is a list with the quaternion and the second element is a list with the angular velocities
5) query altitude data through get_dist() -> stamp, distance where stamp is a double and distance is a floating point scalar 
