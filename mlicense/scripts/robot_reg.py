#!/usr/bin/env python3
'''
	File name: robot_reg.py
	Author: Baidyanath
	Brief: Registers robot to Mowito servers and get's license
	Date last modified: 5/6/2020
'''

__version__ = "dev" # This is required for working of mlicense_gen.py

import requests
import json
from glob import glob
import sys
import os


class ProcessBot:
	def __init__(self):
		self.reg_url = "https://9fzhk3jd3k.execute-api.ap-south-1.amazonaws.com/robo_reg/robot_registration"
		self.lic_url = "https://czkh6wsjq6.execute-api.ap-south-1.amazonaws.com/lice_gen/license_genrator"
		self.cred_file = "credentials_"
		self.lic_file = "license_"
		self.directory = ".mowito"
		self.version = __version__
		self.full_path = ""
		self.email = ""
		self.username = ""
		self.password = ""
		self.robot_name = ""
		self.mac_addresses=[]

	def gma(self):
		dirs = glob("/sys/class/net/*/")
		
		for dir in dirs:
			with open(dir+"/address", 'r') as file:
				mac_address=file.read()
				if(mac_address!="00:00:00:00:00:00\n"):
					self.mac_addresses.append(mac_address[:-1])
	
		if(len(self.mac_addresses) == 0):
			return False
		return True

	def take_input(self):
		print("\nPlease enter your credentials and a unique robot name for this bot")
		print("Enter your email:")
		self.email=input()
		print("Enter your username:")
		self.username=input()
		print("Enter your password:")
		self.password=input()
		print("Enter unique robot name:")
		self.robot_name=input()

	def write_creds(self):
		try:
			with open(self.full_path + "/" + self.cred_file, "w") as file:
				file.write(self.email+"\n")
				file.write(self.username+"\n")
				file.write(self.password+"\n")
				file.write(self.robot_name)
			return True
		except Exception:
			return False

	def write_license(self, license):
		try:
			with open(self.full_path + "/" + self.lic_file, "w") as file:
				file.write(license)
			return True
		except Exception:
			return False

	def process_creds(self):
		self.take_input()
		if not self.write_creds():
			print("\nError writing to file. Couldn't register robot.")
			return False
		else:
			print("\nCredentials written to file")
			return True

	def create_mowito_folder(self):
		parent_directory =os.path.expanduser('~')
		self.full_path = os.path.join(parent_directory, self.directory)
		try:
			os.mkdir(self.full_path)
		except FileExistsError:
			pass
		except:
			return False
		return True

	def send_reg_req(self):
		entry={
				"user_name":self.username,
				"user_email":self.email,
				"robot_name":self.robot_name,
				"mac_address":self.mac_addresses,
				"password":self.password
			}
		try:
			r = requests.post(self.reg_url, data=json.dumps(entry))
		except requests.exceptions.RequestException:
			return 0
		except Exception:
			return -1
		return r.status_code

	def get_license(self, mac_no):
		entry={
				"user_name":self.username,
				"user_email":self.email,
				"robot_name":self.robot_name,
				"mac_address":self.mac_addresses[0],
				"password":self.password,
				"software_version":self.version
			}
		try:
			r = requests.get(self.lic_url, data=json.dumps(entry))
		except requests.exceptions.RequestException:
			return 0
		except Exception:
			return -1
		return r.status_code, r.text

	def register(self):
		if not self.gma():
			print("\nNo mac address present on this system. Couldn't register robot")
			return False
	
		if not self.create_mowito_folder():
			print("\nError creating directory. Couldn't register robot")
			return False
	
		if not self.process_creds():
			return False
	
		retries=3
	
		while(retries>0):
			status = self.send_reg_req()
			if status == -1:
				print("\nCouldn't register robot online. Script will now exit.")
				return False
			elif status == 0:
				print("\nCould not connect to registration server. Please check your connection.")
				return False
			elif status == 200:
				print("\nRobot has been successfully registered.")
				return True
			elif status == 401:
				print("\n\nYour credentials are incorrect.")
				if not self.process_creds():
					return False
			elif status == 413:
				print("\nRobot name already exists in your account.")
				return True
	
			retries -= 1
	
		print("You have exceeded the maximum number of retries please run the script again")
		return False

	def license(self):
		retries = 3
		mac_no  = 0
		while(retries>0):
			status, text = self.get_license(mac_no)
			if status == -1:
				print("\nCouldn't get license. Script will now exit.")
				return False
			if status == 0:
				print("\nCould not connect to licensing server. Please check your connection.")
				return False
			elif status == 200:
				self.write_license(text[1:-1])
				print("\nLicense successfully downloaded.")
				return True
			elif status == 400:
				print("The software version doesn't seem correct please contact supplier.")
				return False
			elif status == 401:
				print("\nPlease contact supplier. Error Code:001")
				return False
			elif status == 403:
				print("\nYour account doesn't seem to have any licenses associated with it.")
				return False
			elif status == 411:
				print("\nPlease contact supplier. Error Code:002")
				return False
			elif status == 412:
				mac_no+=1
				if mac_no >= len(mac_addresses):
					print("\nRobot name was registered with different mac address. Please register this bot with a new name.");
					return False
				else:
					retries+=1
	
			retries -= 1
		print("You have exceeded the maximum number of retries please run the script again")
		return False


def main():
	proc_bot = ProcessBot()
	
	if not proc_bot.register():
		return

	proc_bot.license()



if __name__ == '__main__':
	main()