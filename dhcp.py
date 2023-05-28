import datetime
import random
import struct
from ryu.lib import addrconv
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import ipv4
from ryu.lib.packet import udp
from ryu.lib.packet import dhcp

class Config():
    controller_macAddr = '7e:49:b3:f0:f9:99' # don't modify, a dummy mac address for fill the mac enrty
    dns = '8.8.8.8' # don't modify, just for the dns entry
    start_ip = '192.168.1.2' # can be modified
    end_ip = '192.168.1.100' # can be modified
    ip_pool=['192.168.1.{}'.format(i) for i in range(2, 100)]
    ip_server='192.168.1.1'
    netmask = '255.255.255.0' # can be modified
    lease_time = 14 * 24 * 60 * 60
    # You may use above attributes to configure your DHCP server.
    # You can also add more attributes like "lease_time" to support bouns function.


class DHCPServer():
    hardware_addr = Config.controller_macAddr
    start_ip = Config.start_ip
    end_ip = Config.end_ip
    netmask = Config.netmask
    dns = Config.dns
    ip_pool=Config.ip_pool
    ip_server=Config.ip_server
    ip_to_time = {}
    lease_time=Config.lease_time
    bin_lease_time=struct.pack('!I',lease_time)
    offer_pool=[]
    @classmethod
    def renew_pool(cls):
        for ip in cls.ip_to_time:
            if datetime.datetime.now()> cls.ip_to_time[ip]:
                cls.ip_to_time.pop(ip)
                cls.ip_pool.append(ip)

    @classmethod
    def assemble_ack(cls, pkt, datapath, port):
        
        # TODO: Generate DHCP ACK packet here
        eth_pkt = pkt.get_protocol(ethernet.ethernet)
        if eth_pkt.ethertype != 0x0800:
            return
        ip_pkt = pkt.get_protocol(ipv4.ipv4)
        udp_pkt = pkt.get_protocol(udp.udp)
        if udp_pkt.src_port != 68 or udp_pkt.dst_port != 67:
            return
        dhcp_pkt = pkt.get_protocol(dhcp.dhcp)
        ip_addr=''
        for option in dhcp_pkt.options.option_list:
            if option.tag==dhcp.DHCP_REQUESTED_IP_ADDR_OPT:
                ip_addr=addrconv.ipv4.bin_to_text(option.value)
        cls.renew_pool
        if ip_addr not in cls.ip_pool:
                return
        if(ip_addr not in cls.offer_pool):
            return
        else : cls.offer_pool.remove(ip_addr)
        expire_time=datetime.datetime.now()+datetime.timedelta(seconds=cls.lease_time)
        cls.ip_to_time[ip_addr]=expire_time
        cls.ip_pool.remove(ip_addr)
        ack_pkt = packet.Packet()
        ack_pkt.add_protocol(ethernet.ethernet(
            ethertype=eth_pkt.ethertype,
            dst=eth_pkt.src,
            src=cls.hardware_addr))
        ack_pkt.add_protocol(ipv4.ipv4(
            dst='255.255.255.255',
            src=cls.ip_server,
            proto=ip_pkt.proto))
        ack_pkt.add_protocol(udp.udp(
            src_port=67,
            dst_port=68))
        dhcp_options=dhcp.options()
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_MESSAGE_TYPE_OPT,value=b'\x05'))
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_SERVER_IDENTIFIER_OPT,value=addrconv.ipv4.text_to_bin(cls.ip_server)))
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_SUBNET_MASK_OPT, value=addrconv.ipv4.text_to_bin(cls.netmask)))    
        dhcp_options.option_list.append(dhcp.option(tag=51,value=cls.bin_lease_time))
        ack_pkt.add_protocol(dhcp.dhcp(
            op=dhcp.DHCP_BOOT_REPLY,
            chaddr=dhcp_pkt.chaddr,
            yiaddr=ip_addr, 
            flags=dhcp_pkt.flags,
            giaddr=dhcp_pkt.giaddr,
            siaddr=cls.ip_server,
            xid=dhcp_pkt.xid,
            hlen=dhcp_pkt.hlen,
            sname=dhcp_pkt.sname,
            boot_file=dhcp_pkt.boot_file,
            options=dhcp_options
            ))
        print("sent ack")
        return ack_pkt

    @classmethod
    def assemble_offer(cls, pkt, datapath):
        eth_pkt = pkt.get_protocol(ethernet.ethernet)
        if eth_pkt.ethertype != 0x0800:
            return
        ip_pkt = pkt.get_protocol(ipv4.ipv4)
        udp_pkt = pkt.get_protocol(udp.udp)
        if udp_pkt.src_port != 68 or udp_pkt.dst_port != 67:
            return
        dhcp_pkt = pkt.get_protocol(dhcp.dhcp)
        cls.renew_pool
        if not cls.ip_pool:
            return
        ip_addr=random.choice(cls.ip_pool)
        offer_pkt = packet.Packet()
        offer_pkt.add_protocol(ethernet.ethernet(
            ethertype=eth_pkt.ethertype,
            dst=eth_pkt.src,
            src=cls.hardware_addr))
        offer_pkt.add_protocol(ipv4.ipv4(
            dst='255.255.255.255',
            src=cls.ip_server, 
            proto=ip_pkt.proto)) 
        offer_pkt.add_protocol(udp.udp( src_port=67, dst_port=68))        
        dhcp_options=dhcp.options()
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_MESSAGE_TYPE_OPT,value=b'\x02'))
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_SERVER_IDENTIFIER_OPT,value=addrconv.ipv4.text_to_bin(cls.ip_server)))
        dhcp_options.option_list.append(dhcp.option(tag=dhcp.DHCP_SUBNET_MASK_OPT, value=addrconv.ipv4.text_to_bin(cls.netmask)))
        dhcp_options.option_list.append(dhcp.option(tag=51,value=cls.bin_lease_time))
     
        dhcppkt=dhcp.dhcp( 
            op=dhcp.DHCP_BOOT_REPLY, 
            chaddr=dhcp_pkt.chaddr,  
            yiaddr=ip_addr,
            siaddr=cls.ip_server,
            flags=dhcp_pkt.flags,
            giaddr=dhcp_pkt.giaddr, 
            xid=dhcp_pkt.xid,
            hlen=dhcp_pkt.hlen,
            sname=dhcp_pkt.sname,
            boot_file=dhcp_pkt.boot_file,
            options=dhcp_options
            )
        offer_pkt.add_protocol(dhcppkt)
        print("sent offer")
        cls.offer_pool.append(ip_addr)
        return offer_pkt

        # TODO: Generate DHCP OFFER packet here

    @classmethod
    def handle_dhcp(cls, datapath, port, pkt):
        pkt_dhcp = pkt.get_protocols(dhcp.dhcp)
        for option in pkt_dhcp[0].options.option_list:
            if option.tag == 53:#dhcp.DHCP_MESSAGE_TYPE_OPT:
                if option.value == b'\x01':
                    offerpkt=cls.assemble_offer(pkt,datapath)
                    if offerpkt==None: return
                    cls._send_packet(datapath,port,offerpkt)
                elif option.value == b'\x03':
                    ackpkt=cls.assemble_ack( pkt,datapath,port)
                    if ackpkt==None: return 
                    cls._send_packet(datapath,port,ackpkt)

           
        # TODO: Specify the type of received DHCP packet
        # You may choose a valid IP from IP pool and genereate DHCP OFFER packet
        # Or generate a DHCP ACK packet
        # Finally send the generated packet to the host by using _send_packet method


    @classmethod
    def _send_packet(cls, datapath, port, pkt):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        if isinstance(pkt, str):
            pkt = pkt.encode()
        pkt.serialize()
        data = pkt.data
        actions = [parser.OFPActionOutput(port=port)]
        out = parser.OFPPacketOut(datapath=datapath,
                                  buffer_id=ofproto.OFP_NO_BUFFER,
                                  in_port=ofproto.OFPP_CONTROLLER,
                                      actions=actions,
                                  data=data)
        datapath.send_msg(out)






