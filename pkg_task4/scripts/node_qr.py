#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

shelf_id = ""
iterator_check = False
pkg_list = []

class Camera1:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

		rospy.get_param("pkg_details")
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
			self.img_process(cv_image)

		except CvBridgeError as e:
			rospy.logerr(e)

	def img_process(self, img):
		global iterator_check
		# Resize a 720x1280 image to 360x640 to fit it on the screen
		resized_image = cv2.resize(img, (400, 640)) 

		# convert the resized image to b&w
		gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
		thresh = 40
		img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

		# decode the obtained qrcode
		qr_result = decode(img_bw)
		#print qr_result

		if iterator_check == False:
			for qrcode in qr_result:
				self.id_and_priority_calc(qrcode, qr_result)
			cv2.waitKey(3)
			iterator_check = True
		
		#self.cv_window_print(qr_result, resized_image)

	def id_and_priority_calc(self, qrcode, qr_result):
		global shelf_id
		hor_line = qrcode.rect.top
		vert_line = qrcode.rect.left

		if vert_line <= 165:
			if hor_line <= 450 and hor_line > 371:
				shelf_id = "30"
			if hor_line <= 370 and hor_line > 296:
				shelf_id = "20"
			if hor_line <= 295 and hor_line > 221:
				shelf_id = "10"
			if hor_line <= 220 and hor_line > 136:
				shelf_id = "00"

		if vert_line <= 270 and vert_line > 166:
			if hor_line <= 450 and hor_line > 371:
				shelf_id = "31"
			if hor_line <= 370 and hor_line > 296:
				shelf_id = "21"
			if hor_line <= 295 and hor_line > 221:
				shelf_id = "11"
			if hor_line <= 220 and hor_line > 136:
				shelf_id = "01"

		if vert_line <= 355 and vert_line > 271:
			if hor_line <= 450 and hor_line > 371:
				shelf_id = "32"
			if hor_line <= 370 and hor_line > 296:
				shelf_id = "22"
			if hor_line <= 295 and hor_line > 221:
				shelf_id = "12"
			if hor_line <= 220 and hor_line > 136:
				shelf_id = "02"

		if qrcode.data == "red":
			priority = 1
		if qrcode.data == "yellow":
			priority = 2
		if qrcode.data == "green":
			priority = 3

		self.list_maker(qrcode, shelf_id, priority, qr_result)

	def list_maker(self, qrcode, shelf_id, priority, qr_result):
		global pkg_list
		pkg_id = "packagen"+shelf_id
		
		pkg_dict = {
			"priority": priority,
			"color": qrcode.data,
			"pkg_id": pkg_id,
			"shelf_id": shelf_id
		}

		pkg_list.append(pkg_dict)
		
		# for task5 fix up this messed up part lmao
		print len(pkg_list)
		print (pkg_dict)
		if len(pkg_list) == 12:
			rospy.set_param("/pkg_details", pkg_list)
			#foo = rospy.get_param("/pkg_details")
			#print foo

	def cv_window_print(self, qr_result, resized_image):
		for qrcode in qr_result:
			(x, y, w, h) = qrcode.rect
			# to draw the a box over qr code
			cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)

			# to enter the qrcode data and shelf id on the image
			text = "{} ({})".format(qrcode.data, shelf_id)
			cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


			# To draw reference boundaries to identify the shelf id
			# vertical lines
			cv2.line(resized_image, (60,0), (60,640), (255,255,255), 2)
			cv2.line(resized_image, (165,0), (165,640), (255,255,255), 2)
			cv2.line(resized_image, (270,0), (270,640), (255,255,255), 2)
			cv2.line(resized_image, (355,0), (355,640), (255,255,255), 2)

			# horizontal lines
			cv2.line(resized_image, (0,135), (400,135), (255,255,255), 2)
			cv2.line(resized_image, (0,220), (400,220), (255,255,255), 2)
			cv2.line(resized_image, (0,295), (400,295), (255,255,255), 2)
			cv2.line(resized_image, (0,370), (400,370), (255,255,255), 2)
			cv2.line(resized_image, (0,450), (400,450), (255,255,255), 2)

			#TODO: change these values into the actual values
			infotext1 = "no. of qrcodes: {}".format(len(qr_result))
			infotext2 = "SHELF IDs"
			infotext3 = "high Priority: {}".format(len(qr_result)) #this
			infotext4 = "med. priority: {}".format(len(qr_result)) #this
			infotext5 = "low priority: {}".format(len(qr_result)) #this

			cv2.putText(resized_image, infotext1, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
			cv2.putText(resized_image, infotext2, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
			cv2.putText(resized_image, infotext3, (0, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
			cv2.putText(resized_image, infotext4, (0, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
			cv2.putText(resized_image, infotext5, (0, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

			cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
		cv2.waitKey(3)

def main():

	rospy.init_node('node_eg3_qr_decode', anonymous=True)
	ic = Camera1()
	rospy.spin()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
