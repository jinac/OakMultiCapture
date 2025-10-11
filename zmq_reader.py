import cv2
import zmq
import base64
import numpy as np
import pickle

import msgpack

context = zmq.Context()
footage_socket = context.socket(zmq.SUB)
footage_socket.bind('tcp://localhost:8080')
footage_socket.setsockopt_string(zmq.SUBSCRIBE, str(''))

while True:
    try:
        frame_data = footage_socket.recv()
        # print(frame_data)

        # frame = pickle.loads(frame)
        # source = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        msg = msgpack.unpackb(frame_data, use_list=False, raw=False)
        # print(np.dtype(msg['dtype']))
        # print(msg['data'])

        source = np.frombuffer(msg['data'], dtype=np.dtype(msg['dtype'])).reshape(msg['shape'])
        # print(source.shape)

        cv2.imshow("image", source)
        cv2.waitKey(1)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n\nBye bye\n")
        break
