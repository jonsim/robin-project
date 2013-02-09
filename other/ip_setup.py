#!/usr/bin/env python
# DESCRIPTION: This script is designed to be run on the Pandaboard 
#              on startup to locate the laptop and notify it that 
#              the startup is complete.
# AUTHOR:      Jonathan Simmonds
import commands

HOSTNAME_SELF      = "jPanda"
HOSTNAME_TARGET    = "jLaptop"
HOSTNAME_SELF_FN   = ".ip_jpanda"
HOSTNAME_TARGET_FN = ".ip_jlaptop"
#HOSTNAME_SELF   = commands.getoutput("hostname")
RETRY_ATTEMPTS  = 3

# get the address of the wlan0 interface as a string
# the ifconfig command outputs something of the form:
# inet addr:192.168.0.17  Bcast:192.168.0.255  Mask:255.255.255.0
addr_self = commands.getoutput("ifconfig wlan0 | grep \"inet addr\"").split()[1].split(':')[1]

# generate the full address range
addr_range = '.'.join(addr_self.split('.')[0:3]) + ".*"

# try to search for the target 3 times before giving up
success = False
attempt_number = 0
while attempt_number < RETRY_ATTEMPTS and not success:
    try:
        # do a quick ping scan, looking for the pandaboard.
        # the nmap command outputs something of the form (after 15-20s):
        # Host jLaptop (192.168.0.15) is up (0.000099s latency).
        # NB: it looks different on the pandaboard, the addr is [5], not [2]
        nmap_result = commands.getoutput("nmap -sP " + addr_range + " | grep \"" + HOSTNAME_TARGET + "\"")
        addr_target = nmap_result.split()[5][1:-1]
        # connect to the remote target device and notify it of our success (exporting the addresses)
        wall_command = "echo \"FROM: %s @ %s\nTO:   %s @ %s\" | wall" % (HOSTNAME_SELF, addr_self, HOSTNAME_TARGET, addr_target)
        file1_command = "echo \\\"%s\\\" > ~/%s" % (addr_self,   HOSTNAME_SELF_FN)
        file2_command = "echo \\\"%s\\\" > ~/%s" % (addr_target, HOSTNAME_TARGET_FN)
        full_command = "ssh jon@%s '%s; %s; %s;'" % (addr_target, wall_command, file1_command, file2_command)
        remote_result = commands.getoutput(full_command)
        local_result  = commands.getoutput(file1_command)
        local_result  = commands.getoutput(file2_command)
        clock_command = "sudo date --set=\"`ssh jon@%s date`\"" % (addr_target)
        clock_result = commands.getoutput(clock_command)
        success = True
    except:
        # otherwise increment the attempt counter and retry (to account for timeouts)
        attempt_number += 1

if success:
    print "Successfully setup (after " + str(attempt_number+1) + " attempts)"
    print HOSTNAME_SELF, '@', addr_self
    print HOSTNAME_TARGET, '@', addr_target
else:
    print "Failed to locate " + HOSTNAME_TARGET
