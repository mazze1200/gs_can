import socket
import struct

def joinMcast(mcast_addr,port,if_ip):
    """
    Returns a live multicast socket
    mcast_addr is a dotted string format of the multicast group
    port is an integer of the UDP port you want to receive
    if_ip is a dotted string format of the interface you will use
    """

    #create a UDP socket
    mcastsock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #allow other sockets to bind this port too
    mcastsock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

    #explicitly join the multicast group on the interface specified
    mcastsock.setsockopt(socket.SOL_IP,socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(mcast_addr)+socket.inet_aton(if_ip))

    #finally bind the socket to start getting data into your socket
    mcastsock.bind((mcast_addr,port))

    return mcastsock


MCAST_GRP = '239.74.163.2'
MCAST_PORT = 43113
IS_ALL_GROUPS = False


sock = joinMcast(MCAST_GRP, MCAST_PORT, '192.168.16.32')
while True:
  print(sock.recv(10240))
