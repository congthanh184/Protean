import sys
import time
import os
import glob
import paho.mqtt.client as mqtt

from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive

PARENT_FOLDER_NAME = "protean_data"
device_id = ""

def getFolderID(dir_name):
	# Check & Get the folder id
	file_list = drive.ListFile({'q': "'root' in parents and trashed=false"}).GetList()
	# dir_id = None
	for file1 in file_list:
		# print str(file1['title']) 
		if file1['title'] == dir_name:
			return file1['id']
	return None

def getChildrenFolderID(parent_id, dir_name):
	# Check & Get the folder id
	file_list = drive.ListFile({'q': "'%s' in parents and trashed=false" % parent_id}).GetList()

	# dir_id = None
	for file1 in file_list:
		# print str(file1['title'])
		if file1['title'] == dir_name:
			return file1['id']
	return None

def getCurrentDirectory():
	return time.strftime("%d-%m-%Y")

#for mqtt
def on_connect(client, userdata, rc):
	print("Connected with result code " + str(rc))
	client.subscribe("jimmykin/listen")

def on_message(client, userdata, msg):
	# print "Topic:", msg.topic + "\nMessage:" + str(msg.payload)
	if msg.payload == 'r3p0':
		mqttc.publish("jimmykin/response", device_id + "online")

try:
	f = open("wecid.txt")
	device_id = f.readline()
	f.close()
except Exception, e:	
	pass

# establish connect to mqtt broker
mqttc = mqtt.Client()

mqttc.will_set("jimmykin/response", device_id + "_disconnected", 0, False);
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect("broker.mqttdashboard.com", 1883, 60)

mqttc.loop_start()

if device_id != "":
	device_id = device_id + "_"

dir_name = getCurrentDirectory()
current_dir = dir_name
search_key = str(dir_name + '/*.csv')
need_init_current_file = True

print dir_name + ' ' + search_key

while (1):
	# print "polling file"
	time.sleep(15)
	# time.sleep(500)

	#in case there is no file in folder
	try:
		last_file = max(glob.iglob(search_key), key=os.path.getctime)
	except Exception, e:
		# keep the same current file so it will not triggle upload		
		last_file = current_file
		pass

	if need_init_current_file:
		need_init_current_file = False
		current_file = last_file

	# print last_file
	# new file created, upload the file before
	if last_file != current_file:
		# print "upload" + current_file
		split_res = current_file.split('/')

		current_dir_name = split_res[0]
		current_file_upload = split_res[1]

		google_dir_name = device_id + current_dir_name
		google_file_upload = device_id + current_file_upload
		
		print "Upload " + current_file_upload + " to " + current_dir_name

		try:
			gauth = GoogleAuth()

			gauth.LoadCredentialsFile("mycreds.txt")

			if gauth.credentials is None:
				gauth.LocalWebserverAuth()
			elif gauth.access_token_expired:
				gauth.Refresh()	
			else:
				gauth.Authorize()

			gauth.SaveCredentialsFile("mycreds.txt")

			drive = GoogleDrive(gauth)
			
			# get folder ID of parent
			# folder path = /root/G3 Instrument/Protean_data
			# Protean_data id = "0BwSJa9VnFIuvVGNvRWZEYWZhYjg"
			parent_folder_id = "0BwSJa9VnFIuvVGNvRWZEYWZhYjg"

			# dir_id = getFolderID(google_dir_name)
			dir_id = getChildrenFolderID(parent_folder_id, google_dir_name)

			if dir_id==None:
				print "Google Drives, create " + google_dir_name
				dir_file = drive.CreateFile({'title': google_dir_name, 
					"mimeType": "application/vnd.google-apps.folder", "parents":[{"id": parent_folder_id}]})
				dir_file.Upload()
				dir_id = getChildrenFolderID(parent_folder_id, google_dir_name)
				
			# print "Upload " + current_file_upload
			file2 = drive.CreateFile({'title': google_file_upload, "parents":[{"kind": "drive#fileLink", "id": dir_id}]})
			file2.SetContentFile(str(current_dir_name + "/" + current_file_upload))
			file2.Upload({'convert': True})
			print "Finish upload file"
			print " "

		except Exception, e:
			print e
			pass
			
		current_file = last_file


	# search_key before updating dir_name to make sure check all the file
	# before move to new folder
	search_key = str(dir_name + '/*.csv')
	# print search_key
	# update dir_name, make sure the directory is created before update it
	dir_temp = getCurrentDirectory()
	if os.path.isdir(dir_temp):
		dir_name = dir_temp
		# print "update dir_name " + dir_name
