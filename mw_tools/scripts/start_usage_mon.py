#!/usr/bin/env python
import rosnode
import rosgraph
from datetime import datetime
import sys
import os
import rospy

# lots of things 'borrowed' from rosnode
def main():
	rospy.init_node('start_user_mon', anonymous=True)
	try:
	    from xmlrpc.client import ServerProxy
	except ImportError:
	    from xmlrpclib import ServerProxy

	ID = '/rosnode'
	master = rosgraph.Master(ID, master_uri=None)
	global process, loc
	process = {}
	loc = rospy.get_param("/script_location")

	while not rospy.is_shutdown():
		nodes = rosnode.get_node_names()
		
		for node in nodes:
		    name = " " + node
		    name = name[2:]
		    node_api = rosnode.get_api_uri(master, node)
		    if not node_api:
		        continue
		  
		    node = ServerProxy(node_api)
		    pid = rosnode._succeed(node.getPid(ID))
	
		    if process.has_key(pid):
		    	continue
		    process[pid] = name
		    print("nohup " + loc + "/usage-mon " + str(pid) + " " + name + " &")
		    os.system("nohup " + loc + "/usage-mon " + str(pid) + " " + name + " &")

if __name__ == '__main__':
	try:
		main()
	except:
		pass
	finally:
		global process
		out_dir = "~/.usage-mon/plots/" + str(int(datetime.timestamp(datetime.utcnow())))
		os.system("mkdir " + out_dir)
		for pid in process:
			name = process[pid]
			print("gnuplot -c " + loc + "/usage-plot.gp ~/.usage-mon/" + name + "." + str(pid) + ".dat " + name.replace("_","\\\\_") + " " + out_dir + "/" + name + ".svg")
			os.system("gnuplot -c " + loc + "/usage-plot.gp ~/.usage-mon/" + name + "." + str(pid) + ".dat " + name.replace("_","\\\\_") + " " + out_dir + "/" + name + ".svg")