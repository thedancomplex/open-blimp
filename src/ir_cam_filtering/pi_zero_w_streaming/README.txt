Setting up and running Pi-side streaming:
sh install_stream.sh
python3 threaded_stream.py

--------------------------

Setting up and running server-side receiving:
sh install_rcv.sh
python3 threaded_stream_rcv.py

--------------------------

Setting up streaming receiver:
1) import ThreadedPiStream class from threaded_stream_rcv.py
2) create a ThreadedPiStream object and pass in two UDP ports, first for the camera port and second for the BNO055 port
3) query images through get_image() -> stamp, image where stamp is a double and image is a numpy array
4) query bno055 data through get_bno() -> stamp, bno where stamp is a double and bno is a 2-tuple where the first element is a list with the euler angles and the second element is a list with the accelerations
5) query both through get_pair() -> image_stamp, bno_stamp, image, bno

