from serial import Serial
import numpy as np
import matplotlib as plt
import time
from PIL import Image
import cv2

cascPath = "haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascPath)

def face_detect(imagePath):
	image = cv2.imread(imagePath)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	faces = faceCascade.detectMultieScale(
		gray, 
		scaleFactor = 1.1,
		minNeighbors = 5,
		minSize = (30, 30)
		#flags = cv2.CV_HAAR_SCALE_IMAGE
		)

	for (x,y,w,h) in faces:
		cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 2)
		image_name = imagePath.split('.')[0]
		cv2.imwrite(image_name+'_transformed.jpg', image)

with Serial('/dev/cu.usbmodem1411', 921600) as ser:
	data = bytes()
	keep_reading = 0

	counter = 0
	while True:
		ser_bytes = bytes()
		ser_bytes += ser.readline()

		if (ser_bytes.find(b'ACK CMD OV5642 detected') != -1):
			ser.write(b'20')

		if (ser_bytes.find('ACK' != -1)):
			print(ser_bytes)

		start_index = ser_bytes.find(b'\xff\xd8')
		if (start_index != -1):
			print('start_index found')
			keep_reading = 1
			counter += 1
			file = open(str(counter) + ".jpg", "wb")

		if (keep_reading):
			end_index = ser_bytes.find(b'\xff\xd9')

			if (end_index == -1):
				data += ser_bytes if (start_index == -1) else ser_bytes[start_index:]

			else:
				print('end_index found')
				data += (ser_bytes[:end_index+2]) if (start_index == -1) else ser_bytes[start_index:end_index]
				keep_reading = 0
				inp = np.asarray(bytearray(data), dtype = np.uint8)
				img = cv2.imdecode(inp, cv2.IMREAD_COLOR)
				file.write(img)
				file.close()
				face_detect(str(counter) + ".jpg")
				data = bytes()