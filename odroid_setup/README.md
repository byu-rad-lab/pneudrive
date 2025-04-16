## Setting Up an Odroid with Ubuntu for Pneumatic Drive Control

### Requirements

- [Download Balena Etcher](https://etcher.balena.io/#download-etcher)
- Ubuntu image: Find the image in JDrive at `groups/rad-software/odroid_pneudrive_22_04.zip`

### Flashing Ubuntu Image to SD Card

1. Open Balena Etcher.
2. Select the downloaded Ubuntu image.
3. Insert a 16 GB SD card.
4. Flash the image to the SD card.

### Setting Up Odroid

#### Connect Hardware

- Insert the flashed SD card into the Odroid.
- Connect the Odroid to power and Ansible.

#### SSH into Odroid

- SSH into the Odroid: `ssh ubuntu@192.168.0.200`
  - Password: `temppwd`

#### Ensure Correct File Structure

Ensure the directory contains the Pneudrive package. Ensure that it is the most up-to-date version.

#### Change Static IP Address

1. Check the [Lab Wiki](https://github.com/byu-rad-lab/rad-lab-wiki/blob/main/infrastructure/lab_ip_addresses.md) for an available IP address.
2. Update the Lab Wiki with your IP address and device information.
3. Print a label and attach it to the Odroid for future reference.
4. Determine the network interface with `ip addr` (likely `eth0`).
5. Edit the Netplan configuration file (Change ONLY the IP address): ```sudo nano /etc/netplan/radlabconfig.yaml ```
6. Go edit the .bashrc file so that ROS_IP is the ip address you just changed to.
7. Apply the changes: ```sudo netplan apply```
8. Reconnect to the Odroid with the new IP address.
9. Reboot and check the updated IP address:

```
sudo shutdown now -r
ssh ubuntu@<new_ip_address>
ip addr show
```

9. Confirm that the IP address has been updated successfully using `ip addr`.

### Getting Pneudrive to Run

1. Navigate to the `ros_ws` directory and delete the `devel` and `build` directories using the command `rm -rf devel build`.
2. Navigate to the home directory and source the `.bashrc` file by running the command `source .bashrc`.
3. Navigate back to the `ros_ws` directory and run the command `colcon build`.
4. Once the `colcon build` is complete, verify that the Pneudrive package can be found by running the command `ros2 pkg list`.
- You should see a list of packages including pneudrive.
- If pneudrive is not present in the list of packages, keep trying variations of what is listed above until it works.


