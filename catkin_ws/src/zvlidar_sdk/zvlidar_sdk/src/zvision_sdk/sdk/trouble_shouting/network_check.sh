#! /bin/sh

echo "This is a test script"

# Check network status
ifconfig

# Sleeo for 1 second
sleep 1s

# Check network status again
ifconfig

# Check firewall status
sudo ufw status numbered
