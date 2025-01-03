FROM ubuntu:24.04

# Install necessary packages
# make - guess it is necessary... probably (cmake - too?)
# cppcheck - I guess it is necessary... or it will become 
#	     necessary at some point
# wget bzip2 unzip - not sure
RUN DEBIAN_FRONTEND=noninteractive \
	apt-get update \
	&& apt-get -y install make cppcheck git \
		      gcc libusb-0.1-4 libusb-dev \
		      avr-libc avrdude \
		      gcc-avr binutils-avr

# Create non root user named "ubuntu" (not certain if 
# username could be different). But put in root group since 
# GitHub actions needs permissions to create tmp files.
#RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo \
#	-u 1001 ubuntu
#USER ubuntu
#WORKDIR /home/ubuntu
RUN useradd -rm -d /home/avr_dev -s /bin/bash -g root -G sudo \
	-u 1001 avr_dev
USER avr_dev
WORKDIR /home/avr_dev

	
