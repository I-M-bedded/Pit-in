
UDPport=3002
TCPport=2002

# trY="192.168.0.17"
# trX="192.168.0.18"
# rtZ="192.168.0.19"
# Lr="192.168.0.20"
# Rr="192.168.0.21"
# Lz="192.168.0.22"
# Rz="192.168.0.23"
trY="192.168.0.5"
trX="192.168.0.3"
rtZ="192.168.0.2"
Lr="192.168.0.1"
Rr="192.168.0.4"
Lz="192.168.0.10"
Rz="192.168.0.7"


udp_addresses=((trY,UDPport),(trX,UDPport),(rtZ,UDPport),(Lr,UDPport),(Rr,UDPport),(Lz,UDPport),(Rz,UDPport))

offsets=[[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]
homming_method=[2,0,0,1,1,0,0]
homming_dir=[0,0,0,0,1,0,0]

gains=[16,16,63,63,63,4,4]


