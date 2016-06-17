# Setup the Raspberry Pi for Xamla Egomo

This how-to assumes you use a linux pc and was tested with Ubuntu 14.04 and 16.04 but should work for other linux distributions, too.
It describes how to setup the SD card for the Raspberry Pi and provides a short overview how to operate the system.


## Prepare the SD card for the Raspberry Pi

The egomo sensor image for the Raspberry Pi can be obtained from the [Xamla homepage](http://xamla.com/downloads/egomo-1/rpi/xamla-egomo_v1_0.tar.gz). It is based on `Raspbian Jessie Lite`, version `May 2016`. In addition to the default image it contains amongst others a Lua+Torch installation, a basic ROS Jade installation and the Xamla Egomo tools. The image is optimized for fast boot and operates on read only partitions to protect the file system in case of a power failure.

The image can be installed to any SD card supported by the PI with a size of at least 8 GB. To achieve a short boot time and good operation performance of the Pi we recommend using class 10 SD cards or better.

Checksums of the image:

	MD5: dcfa77735607cd9b82e2f022387f87f7  xamla-egomo_v1_0.tar.gz
	SHA256: 1cad057e50a2ecf666028fc2797ee1846d1c4cb6a63e0c153f497c6268f39edc  xamla-egomo_v1_0.tar.gz


### Copying the Image to the SD Card

These steps are very similar to the steps required to [setup the usual Raspberry Pi images](https://www.raspberrypi.org/documentation/installation/installing-images/linux.md):

* Download the [SD card image](http://xamla.com/downloads/egomo-1/rpi/xamla-egomo_v1_0.tar.gz)
  * Connect the (new) SD card with your computer (SD card slot or via an SD card reader)
  * Unmount all partitions of the SD card (e.g. unmount /dev/sdd1 if sdd is your SD card device name) if any partition got mounted automatically
  * Write the content of the .img file to the SD card, e.g.

		`sudo dd bs=4M if=/some/path/on/your/disk/xamla-egomo.img of=/dev/sdd`

	***IMPORTANT***: Make sure to provide the *correct device name* as parameter for `of`, not just one of the partitions.
	You will loose all data on the device. So, double-check if the device (/dev/sdd) is really the SD card.
  * Wait until `dd` has finished. This might take some time and you will not see any output during the operation.
	With a typical class 10 SD card, the copy process takes about 6 minutes.
  * Remove the SD card and reconnect it with your PC. You should see two partitions on the SD card:
	A small one called boot and a second one with a size of about 6 GB containing the root file system. If the partitions don't get mounted automatically, mount them manually. When the root partition of the SD card is mounted, continue with the next section.


### Adopt the Config Files for Your Network/ROS Environment

The partition on the SD card are mounted read-only during operation to protected the file system in case of power failure. Therefore a few more files then usual have to be adopted.

  * `/etc/hosts`: Adopt the hostname to your requirement
  * `/etc/hostname`: Replace `pi-xamla` with the same hostname you set in /etc/hosts
  * `/etc/resolv.conf`: Set the correct domain name and the IP address of at least one name server. Usually this file would be updated automatically but since we are running a read-only filesystem the file has been filled by hand.
  * `/etc/ros-config`: Set the `ROS_MASTER_URI` to the URI of the system running the roscore/master server
  * `/etc/ntp.conf`: If the Pi has no internet access, provide the address of at least one reachable NTP server. By default, the debian.pool NTP servers are used.
  * `/etc/wpa_supplicant/wpa_supplicant.conf` By default, the pi uses the on-board wireless adapter for network access. The SSID and the password for the WLAN has to be configured in this file. If you want to use the Ethernet port instead of wireless, edit `/etc/network/interfaces`.


## First Boot

Plug in the SD card to your Raspberry Pi and power it up.

  * Log in to the pi with username `pi` and password `xamla`. (either via network or via a directly connected keyboard/monitor)
  * Remount the partition in read-write mode. To comfortable switch between read-only and read-write mode, two aliases `ro` and `rw` are defined. Just type `rw` at the prompt to remount the partition as read-write.
  * Change the password for user pi with `passwd`.
  * You might want to regenerate the SSH server keys:

        sudo rm -v /etc/ssh/ssh_host_*
        sudo dpkg-reconfigure openssh-server


## Useful Commands

  * By default, three ROS nodes are started during the system boot. The status of the nodes can be checked with

        sudo systemctl status rosgripper.service
        sudo systemctl status roswebcam.service
        sudo systemctl status rosdepthcam.service

  * Rebuilding the catkin workspace: Switch to ~/catkin_ws and execute

        source /home/pi/catkin_ws/devel/setup.bash
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/jade -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O3 install

## Remote Logging

The current setup uses the busybox syslogd daemon. It get's configured via `/etc/default/busybox-syslogd`. Open the file
and edit the line starting with `SYSLOG_OPTS`. Comment the line `SYSLOG_OPTS="-C128"` and uncomment `SYSLOG_OPTS="-C128 -L -R host:port"`. Host and port have to be set to your remote logging host. If you use the default port 512, the port number can be omitted. Afterwards, restart the syslog daemon via `sudo systemctl restart busybox-syslogd`

On the remote logging host, uncomment/add following lines in `/etc/rsyslog.conf`

        # provides UDP syslog reception
        $ModLoad imudp
        $UDPServerRun 514
        # make rsyslog listen on all ip addresses, you could specify an address
        $UDPServerAddress 0.0.0.0

**WARNING** Configuring the remote logging this way both expose all logging messages of the Pi to all PCs in your network and exposes the logging host so that any computer in the network can send log messages to the logging host. Use this setup only in a trusted network.
