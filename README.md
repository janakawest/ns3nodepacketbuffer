# ns3_node_packet_buffer

Introducing a packet buffer to ns-3 nodes
=========================================

The module is still under construction. However, the minimum functionalities are added to the module.
	-> All incoming packets are buffered in a FIFO queue
	-> Based on M/M/1 Queue model, packets are pulled out of the buffer
		 and forward to the protocol layer.
	-> Every time a packet comes to a NetDevice (i.e., PromiscReceiveFromDevice)
		 number of packets came to that particular device and average packet
		 the size of all incoming packets is calculated and add store respectively.
	-> For the experimentation purpose, the service rate of the route is 
		 Assumed as 50Mbps. Users can change it as their will.

Please use it at your own risk. Backup your ns-3 system before you add the module to your ns-3 system. This module is tested under, ns-2.18~.21.

Method of Installing
++++++++++++++++++++
Caution
-------
backup both your node.cc and node.h file prior to download

Download
========
Download following files:
	-> node.cc
	-> node.h
	-> node-packet-queue.cc
	-> node-packet-queue.h
and add to  '%ns-3-executable directory%/src/network/' directory.

Prepare your system
===================

Add both node-packet-queue.h and node-packet-queue.cc files to the wscript accordingl (i.e., 'model/node-packet-queue.cc', to the wscript file reside in '%ns-3-executable directory%/src/network/')

Configure the system to enable example and tests (i.e., ./waf --configure).
Build the system.

If any bug found, any modification needed, or any mistake found, please report us using janaka@west.sd.keio.ac.jp. Thus you can give a shot to edit, modify and update the file. 
