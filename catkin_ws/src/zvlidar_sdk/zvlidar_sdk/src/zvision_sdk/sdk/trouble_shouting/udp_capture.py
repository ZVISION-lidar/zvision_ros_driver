#! /usr/bin/env python

import sys
import socket

#Specify the port number to capture.
UDP_PORT = 2368


if 2 != len(sys.argv):
    print("No specified port available, detect on default port ", UDP_PORT)
else:
    UDP_PORT = sys.argv[1]
    print("Detect on port ", UDP_PORT)



def udp_pkt_cap(port=None, cnt=125):
    """Capture udp packet on special port"""
    if not port:
        udp_port = 2368
    else:
        udp_port = port

    # type: udp socket
    udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	
    # Enable the SO_REUSEADDR option
    udpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    new_state = udpSocket.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR)
    # print("New sock state: %s" % new_state)

    # bind socket
    try:
        udpSocket.bind(("", udp_port))
        udpSocket.settimeout(0.06)
    except socket.error as err_msg:
        print("Udp socket message:%s" % err_msg)
        return

    udp_buf = 1304
    pkt_cnt = cnt
    messages = []
    while True:
        dat, add = udpSocket.recvfrom(udp_buf)
        if (dat[0:2] == b'\xaa\xaa') or (dat[0:2] == b'\xbb\xbb') or (dat[0:2] == b'\xcc\xcc'):
            # print(add[0], add[1], hex(ord(dat[2])) + '{:02x}'.format(ord(dat[3])) )
            # print(add[0], add[1], '0x{:02X} 0x{:02X}'.format(dat[2], dat[3]))
            print("Recv ok source ip and port is ", add)
            messages.append(dat + b'\n')
            if pkt_cnt > 1:
                for _ in range(pkt_cnt - 1):
                    datfrm, add = udpSocket.recvfrom(udp_buf)
                    messages.append(datfrm + b'\n')
                break
            else:
                break

    return messages # return string

print("Start to capture packet on port ", UDP_PORT)

data_cap = udp_pkt_cap(UDP_PORT, 320)

print("Cap packet ok, size is ", len(data_cap))
