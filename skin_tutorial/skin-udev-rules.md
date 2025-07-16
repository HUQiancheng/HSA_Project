# Install udev rules for skin interfaces on the Docker host

The FTDI skin driver (USB) needs special udev rules otherwise the system can
not detect and use the plugged device.

Udev rules do not work in a docker container because docker containers do not 
have a udev daemon running.

To get the USB devices running we need to install the udev rule on the docker
Host.


```bash
# To install the file on the host
docker run --rm --entrypoint cat tum-ics-ros:noetic-ics-skin-devel-vscode /lib/udev/rules.d/97-ft232H.rules | sudo tee /lib/udev/rules.d/97-ft232H.rules 1> /dev/null

# To print out the content of the installed file
cat /lib/udev/rules.d/97-ft232H.rules

# To remove the file on the host
sudo rm /lib/udev/rules.d/97-ft232H.rules


# To reload the udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```