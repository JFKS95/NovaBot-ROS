# G-ROSSoftware
All code in relation to ROS development

For env.sh under devel:

#!/bin/bash

export ROS_MASTER_URI="http://192.168.1.101:11311"
export ROS_IP="192.168.1.102"

source /home/pi/ros_catkin_ws/devel/setup.bash

exec "$@"


For wpa_supplicant:

ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
	ssid="eduroam"
	key_mgmt=WPA-EAP
	eap=PEAP
	identity="eljfs2@lboro.ac.uk"
	anonymous_identity="anonymous@lboro.ac.uk"
	password=hash:3dc948de244ea28cd5ce23bdd5a99b8c
	ca_cert="/home/pi/certs/ca.pem"
	domain_suffix_match="radius.lboro.ac.uk"
	phase2="auth=MSCHAPV2"
	priority=1
	disabled=1
}

network={
	ssid="GalaxyWiFi"
	psk="milkyway"
	key_mgmt=WPA-PSK
}


