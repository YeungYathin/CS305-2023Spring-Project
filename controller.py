from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.topology import event
from ryu.topology.switches import Switch,Host,HostState,Port,PortState,PortData,PortDataState,Link,LinkState
from ryu.topology.switches import Switches
from ryu.ofproto import ofproto_v1_0,ether, inet
from ryu.lib.packet import packet, ethernet, ether_types, arp
from ryu.lib.packet import dhcp
from ryu.lib.packet import ethernet
from ryu.lib.packet import ipv4
from ryu.lib.packet import packet
from ryu.lib.packet import udp
from dhcp import DHCPServer
from collections import defaultdict
import time
from ofctl_utilis import OfCtl,OfCtl_v1_0,OfCtl_after_v1_2,VLANID_NONE
import logging
import copy
import heapq

#用于计算最短路，nodes与class ControllerApp中的switch.dpid对应
class Graph:
    def __init__(self, nodes, adjacency_matrix):
        self.nodes = nodes
        self.adjacency_matrix = adjacency_matrix
        self.num_nodes = len(nodes)

        #存储某个临时节点的路由表，其元素是key:value键值对，具体为：dp:(distance,next_hop)
        #A   {A:None,B:(1,C)}
        #A ->.....->C->B
        self.routing_table = {}
        self.routing_tables = []#存储全部节点的路由表，其元素是self.routing_table

    #初始化某个节点的路由表
    def initialize_routing_table(self, source_node):
        for node in self.nodes:
            if node == source_node:
                self.routing_table[node] = (0, None)
            else:
                self.routing_table[node] = (float('inf'), None)

    #更新某个节点路由表
    def update_routing_table_DV(self, source_node):
        for node in self.nodes:
            if node != source_node:
                min_distance = self.routing_table[node][0]
                next_hop = self.routing_table[node][1]
                for neighbor in self.get_neighbors(node):
                    distance = self.routing_table[self.nodes[neighbor]][0] + self.adjacency_matrix[neighbor][self.nodes.index(node)]
                    if distance < min_distance or (next_hop==source_node and distance <=min_distance):
                        min_distance = distance
                        next_hop = self.nodes[neighbor]
                if next_hop==node:
                    self.routing_table[node] = (min_distance, source_node)
                else:
                    self.routing_table[node] = (min_distance, next_hop)

    def get_neighbors(self, node):
        neighbors = []
        for i in range(self.num_nodes):
            if self.adjacency_matrix[self.nodes.index(node)][i] != float('inf'):
                neighbors.append(i)
        return neighbors

    #对图中的每一个节点，求其路由表routing_table，并存入routing_tables中
    def run_DV(self):
        self.routing_tables=[]
        for source_node in self.nodes:
            self.initialize_routing_table(source_node)
            for _ in range(self.num_nodes - 1):
                self.update_routing_table_DV(source_node)
            #print(self.routing_table)
            tmp=copy.deepcopy(self.routing_table)
            self.routing_tables.append(tmp)
    
    class Dijkstra_Node:
        def __init__(self, node_num, neighbors, is_pushed, is_finished, parent, distance):
            self.node_num = node_num
            self.neighbors = neighbors
            self.is_pushed = is_pushed
            self.is_finished = is_finished
            self.parent = parent
            self.distance = distance
        
        def __lt__(self, other):
            if self.distance<other.distance:
                return True
            else:
                return False
            
    
    def Dijkstra(self, source_node):
        Dijkstra_Graph = {}
        for node in self.nodes:
            Dijkstra_Graph[node] = self.Dijkstra_Node(node_num=node, neighbors=[self.nodes[i] for i in self.get_neighbors(node)], is_pushed=False, is_finished=False, parent=None, distance=1000000000000)
        heap = []
        present = Dijkstra_Graph[source_node]
        heapq.heappush(heap, present)
        present.distance = 0
        present.is_pushed = True
        while len(heap)!=0:
            present = heapq.heappop(heap)
            present.is_finished = True
            for i in range(len(present.neighbors)):
                neighbor = Dijkstra_Graph[present.neighbors[i]]
                if not neighbor.is_finished and present.distance + self.adjacency_matrix[self.nodes.index(present.node_num)][self.nodes.index(neighbor.node_num)] < neighbor.distance:
                    neighbor.distance = present.distance + self.adjacency_matrix[self.nodes.index(present.node_num)][self.nodes.index(neighbor.node_num)]
                    neighbor.parent = present.node_num
                    if not neighbor.is_pushed:
                        neighbor.is_pushed = True
                        heapq.heappush(heap, neighbor)
                    else:
                        heapq.heapify(heap)
        for key, value in Dijkstra_Graph.items():
            if key == source_node:
                self.routing_table[key] = (0, None)
            elif value.parent is not None:
                self.routing_table[key] = (value.distance, value.parent)
            else:
                self.routing_table[key] = (float('inf'), None)
        
            
    def run_Dijkstra(self):
        self.routing_tables = []
        for source_node in self.nodes:
            self.initialize_routing_table(source_node=source_node)
            self.Dijkstra(source_node)
            tmp=copy.deepcopy(self.routing_table)
            self.routing_tables.append(tmp)

    def print_routing_table(self):
        #存储某个临时节点的路由表，其元素是key:value键值对，具体为：dp:(distance,next_hop)
        #A   {A:None,B:(1,C)}
        #A ->.....->C->B
        idx1=0
        for route_table in self.routing_tables:
            cur_node=self.nodes[idx1]
            print("\ns{} to other switch:".format(cur_node))
            idx1+=1
            idx2=-1
            for dpid,val in route_table.items():
                idx2+=1
                if dpid==cur_node:
                    continue
                #ans=[self.nodes[idx2]]
                ans=[dpid]
                dis,next_hop=val[0],val[1]
                flag=True
                while(next_hop!=cur_node):
                    if next_hop is None or dis==float('inf'):
                        print("s{} can not reach s{}".format(cur_node,dpid))
                        flag=False
                        break
                    ans.insert(0,next_hop)
                    next_hop=route_table[next_hop][1]
                    dis=route_table[next_hop][0]
                if flag:
                    formatted_str = '->'.join(['s{}'.format(num) for num in ans])
                    print(("s{}->".format(cur_node))+formatted_str+", cost is {}".format(len(ans)))



    #往图中添加节点，对应switch_add
    def add_node(self,node):
        if node in self.nodes:
            return 
        self.nodes.append(node)
        self.num_nodes=len(self.nodes)
        self.adjacency_matrix.append([float('inf')] * (self.num_nodes - 1) + [0])
        for i in range(self.num_nodes-1):
            self.adjacency_matrix[i].append(float('inf'))
    
    def delete_node(self,node):
        idx=self.nodes.index(node)
        self.nodes.remove(node)
        self.num_nodes=len(self.nodes)
        del self.adjacency_matrix[idx]
        for i in range(self.num_nodes):
            del self.adjacency_matrix[i][idx]
        self.run_Dijkstra()
        # self.run_DV()
    
    #往图中添加连接，对应link_add，同时更新整张图的路由表
    def modify_link(self,link):
        """
        links:[[src1,dst1,w1]]
        """
        idx_src=self.nodes.index(link[0])
        idx_dst=self.nodes.index(link[1])
        self.adjacency_matrix[idx_src][idx_dst]=link[2]
        self.adjacency_matrix[idx_dst][idx_src]=link[2]
        self.run_Dijkstra()
        # self.run_DV()


    #计算单个节点的转发表
    #类成员routing_tables存储的是某个节点通过哪个节点到目标节点
    #节点A的routing_table形如{'A': (0, None), 'B': (1, 'A'), 'C': (3, 'A'), 'D': (2, 'B'), 'E': (4, 'D')}
    #表示A->A:无需转发，    A->B：A->B(A直接转发给B),     A->C:A->C,    A->D:A->....->B->D

    #该函数功能为利用routing_table计算下一跳（转发给哪个交换机）
    def get_path(self,src_node,router_list):
        ans=[]
        for node in self.nodes:
            if node!=src_node:
                tmp=node
                while(router_list[tmp][1]!=src_node):
                    if router_list[tmp][0]==float('inf'):
                        ans.append(None)
                        break
                    tmp=router_list[tmp][1]
                if router_list[tmp][0]!=float('inf'):
                    ans.append(tmp)
            else:
                ans.append(src_node)
        return ans









class ControllerApp(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_0.OFP_VERSION]
    def __init__(self, *args, **kwargs):
        super(ControllerApp, self).__init__(*args, **kwargs)
        
        self.topo=Switches()#整个网络的拓扑结构，Switcthes中包含交换机，主机，连接，端口
        self.portState={}   #(port_dpid,port_no)->State
        self.ofctls={}#datapath.id与openflow控制器的映射。dp.id->ofctl
        self.graph=Graph([],[])
        self.pre_router={}
        self.port2port={}
        self.logger = logging.getLogger("ControllerApp")
        self.host2switch={}
        self.switch2host={}
        self.host2port={}
        self.arp_IP_MAC_mapping_table = {} # 整个网络拓扑中所有host的IP与MAC地址映射
        '''
        arp_IP_MAC_mapping_table用法:
        使用ipv4地址作为key 该主机所有信息作为value
        例如 
        arp_IP_MAC_mapping_table['10.0.0.3']可以获得以下信息
        {'mac': '00:00:00:00:00:03', 'port': {'dpid': '0000000000000003', 'port_no': '00000001', 'hw_addr': '36:5a:57:da:38:17', 'name': 's3-eth1'}}
        '''

    @set_ev_cls(event.EventSwitchEnter)
    def handle_switch_add(self, ev):
        # print(dir(ev))##switch
        # print(dir(ev.switch))##add_port del_port, dp, ports, to_dict
        print("event.EventSwitchEnter!!!!!")
        dp=ev.switch.dp#得到openflow协议控制的交换机

        #dp只是mininet中的一个虚拟的交换机，并不是switch，根据dp登记后获得switch
        self.topo._register(dp)#在网络拓扑登记交换机

        version=dp.ofproto.OFP_VERSION
        cur_OfCtl=None
            #cur_OfCtl = OfCtl.factory(dp, self.logger)
        cur_OfCtl=OfCtl_v1_0(dp,logging.getLogger("swicth"))
        
        self.ofctls[dp.id]=cur_OfCtl
        self.graph.add_node(dp.id)
        print("current topo:")
        print("switch dpid list:{}".format(self.graph.nodes))
        self.graph.print_routing_table()
        self.pre_router[dp.id]=[]

        if dp.id in self.switch2host:
            print(self.switch2host)
            print(dp.id)
            actions=[]
            sw_ofctl=self.ofctls[dp.id]
            ofp=sw_ofctl.dp.ofproto
            ofp_parser=sw_ofctl.dp.ofproto_parser
            mac=self.switch2host[dp.id]
            match=ofp_parser.OFPMatch(dl_dst=mac)
            output_port=self.host2port[mac]
            actions.append(ofp_parser.OFPActionOutput(int(output_port['port_no'])))
            m = ofp_parser.OFPFlowMod(sw_ofctl.dp, match=match,actions=actions)
            sw_ofctl.dp.send_msg(m)

        


    @set_ev_cls(event.EventSwitchLeave)
    def handle_switch_delete(self, ev):
        print("event.EventSwitchLeave!!!!!")
        #self.logger.info("Switch Leave %s" %ev.switch)
        dp=ev.switch.dp

        # self.topo._unregister(dp)

        # del self.ofctls[dp.id]
        # self.graph.delete_node(dp.id)
        # del self.pre_router[dp.id]
        # print("current topo:")
        # print("switch dpid list:{}".format(self.graph.nodes))
        # self.graph.print_routing_table()


        
    @set_ev_cls(event.EventHostAdd)
    def handle_host_add(self, ev):
        """
        Event handler indiciating a host has joined the network
        This handler is automatically triggered when a host sends an ARP response.
        """ 
        print("Handle_Host_Add Begin!!!!!!!!\n")
        
        msg = ev.host.to_dict()
        ipv4 = msg['ipv4'][0]
        mac = msg['mac']
        port = msg['port']
        #print(f'Info about the new host\nIPV4: {ipv4}, MAC: {mac}, port: {port}')
        #print(dir(ev.host.port))
        info_for_this_ip_host = {}
        info_for_this_ip_host['mac'] = msg['mac']
        
        self.arp_IP_MAC_mapping_table[msg['ipv4'][0]] = {'mac': msg['mac'], 'port': port}
        
        host = Host(msg['mac'], ev.host.port)
        host.ipv4.append(msg['ipv4'])
        self.topo.hosts.add(host=host)
        #self.host2port[host]=port
        # link_switch=self.topo._get_switch(int(port['dpid']))
        # print("link_switch is {}".format(link_switch))
        # print("topo.dps is {}".format(self.topo.dps))
        # print("host's port is {}".format(port))
        self.host2switch[host.mac]=int(port['dpid'])
        self.switch2host[int(port['dpid'])]=host.mac
        self.host2port[host.mac]=port

        actions=[]
        sw_ofctl=self.ofctls[int(port['dpid'])]
        ofp=sw_ofctl.dp.ofproto
        ofp_parser=sw_ofctl.dp.ofproto_parser
        match=ofp_parser.OFPMatch(dl_dst=mac)
        actions.append(ofp_parser.OFPActionOutput(int(port['port_no'])))
        m = ofp_parser.OFPFlowMod(sw_ofctl.dp, match=match,actions=actions)
        sw_ofctl.dp.send_msg(m)

        self.set_flow_table()
        #print("Handle_Host_Add End!!!!!!!!\n")
        




    def set_flow_table(self):
        router_table=self.graph.routing_tables
        nodes=self.graph.nodes
        for sw,sw_ofctl in self.ofctls.items():
            sw_router=self.graph.get_path(sw,router_table[nodes.index(sw)])
            pre_router=self.pre_router[sw]
            if pre_router==[]:
                pre_router=[None]*len(sw_router)
            # elif len(pre_router)<=len(sw_router):
            #     pre_router.extend([None]*(len(sw_router)-len(pre_router)))
            # elif len(pre_router)>len(sw_router):
            #     pre_router=[None]*len(sw_router)

            self.pre_router[sw]=sw_router
            src_dpid=sw
            ports=(self.topo._get_switch(sw)).ports
            sw_port_dict={}
            for port in ports:
                if port in self.port2port:
                    toPort=self.port2port[port]
                    toDpid=toPort.dpid 
                    sw_port_dict[toDpid]=port
            
            ofp=sw_ofctl.dp.ofproto
            ofp_parser=sw_ofctl.dp.ofproto_parser

            for i in range(len(pre_router)):
                pre_next_dpid=pre_router[i]
                pre_dst_dpid=self.graph.nodes[i]
                if src_dpid==pre_next_dpid or pre_next_dpid is None:
                        continue
                pre_hosts_mac=None
                if pre_dst_dpid in self.switch2host:
                    pre_hosts_mac=self.switch2host[pre_dst_dpid] 
                else:
                    continue    
                pre_match=ofp_parser.OFPMatch(dl_dst=pre_hosts_mac)
                actions=[]
                cmd = ofp.OFPFC_DELETE
                flow_mod = ofp_parser.OFPFlowMod(
                sw_ofctl.dp,match=pre_match,cookie=0, command=cmd, priority=0, actions=actions)
                sw_ofctl.dp.send_msg(flow_mod)
            # cmd = ofp.OFPFC_DELETE
            # actions = []
            # flow_mod = ofp_parser.OFPFlowMod(
            #     sw_ofctl.dp,cookie=0, command=cmd, priority=0, actions=actions)
            # sw_ofctl.dp.send_msg(flow_mod)
            for i in range(len(sw_router)):
                #if sw_router[i]!=pre_router:
                next_dpid=sw_router[i]
                dst_dpid=self.graph.nodes[i]
                if src_dpid==next_dpid or next_dpid is None:
                    continue
                hosts_mac=None
                if dst_dpid in self.switch2host:
                    hosts_mac=self.switch2host[dst_dpid] 
                else:
                    continue
                out_port=sw_port_dict[next_dpid].port_no#应该发往的端口
                if (sw,out_port) in self.portState:
                    if not self.portState[(sw,out_port)]:
                        continue
                #需指定终点交换机的地址，转发端口
                actions=[]
                
                match=ofp_parser.OFPMatch(dl_dst=hosts_mac)
                actions.append(ofp_parser.OFPActionOutput(out_port))
                m = ofp_parser.OFPFlowMod(sw_ofctl.dp, match=match,actions=actions)
                sw_ofctl.dp.send_msg(m)


    @set_ev_cls(event.EventLinkAdd)
    def handle_link_add(self, ev):
        #print("=================================================================")
        print("\nevent.EventLinkAdd!!!!!")

        #print(type(ev.link)) #ryu.topology.switches.Link
        #ev.link: ev.link.dst,ev.link.src
        #print(type(ev.link.dst)) #ryu.topology.switches.Port

        src,dst=ev.link.src,ev.link.dst
        self.port2port[src]=dst
        self.port2port[dst]=src
        switch_src,switch_dst=src.dpid,dst.dpid
        print("add link [{},{}]".format(switch_src,switch_dst))
        #print("src Port {}\nhw_addr {}\nits switch is {}\nip is {}".format(src,src.hw_addr,switch_src,self.topo.dps[switch_src].address))
        # print()
        #print("dst Port {}\nhw_addr {}\nits switch is {}\nip is {}".format(dst,dst.hw_addr,switch_dst,self.topo.dps[switch_dst].address))
        # print("///////////////////////////////////////")
        self.topo.links.update_link(src,dst)
        self.graph.modify_link([switch_src,switch_dst,1])
        print("switch dpid list:{}".format(self.graph.nodes))
        print()
        #print("=================================================================")

        print("current topo:")
        for element in self.graph.routing_tables:
            print(element)
        self.graph.print_routing_table()
        self.set_flow_table()
        


    @set_ev_cls(event.EventLinkDelete)
    def handle_link_delete(self, ev):
        print("event.EventLinkDelete!!!!!")
        src,dst=ev.link.src,ev.link.dst
        if src in self.port2port:
            del self.port2port[src]
        if dst in self.port2port:
            del self.port2port[dst]

        switch_src,switch_dst=src.dpid,dst.dpid
        self.topo.links.link_down(ev.link)
        self.graph.modify_link([switch_src,switch_dst,float('inf')])

        print("switch dpid list:{}".format(self.graph.nodes))
        print()
        #print("=================================================================")

        print("current topo:")
        for element in self.graph.routing_tables:
            print(element)
        self.graph.print_routing_table()
        self.set_flow_table()
        

    @set_ev_cls(event.EventPortModify)
    def handle_port_modify(self, ev):
        print("event.EventPortModify!!!!!")
        """
        Event handler for when any switch port changes state.
        This includes links for hosts as well as links between switches.
        """
        #print(dir(ev))
        # print(dir(ev.port))
        # print(type(ev.port))
        print(ev.port)
        print(ev.port.is_down())
        cur_port=ev.port
        cur_port_dpid=cur_port.dpid
        cur_port_no=cur_port.port_no
        self.portState[(cur_port_dpid,cur_port_no)]=ev.port.is_down()
        #topo_port=self.topo._get_port(cur_port_dpid,cur_port_no)
        self.set_flow_table()


    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        try:
            msg = ev.msg
            datapath = msg.datapath
            pkt = packet.Packet(data=msg.data)
            pkt_dhcp = pkt.get_protocols(dhcp.dhcp)
            inPort = msg.in_port
            
            if not pkt_dhcp:
                # TODO: handle other protocols like ARP 
                
                eth_pkt = pkt.get_protocol(ethernet.ethernet)
                # print(eth_pkt)
                if eth_pkt.ethertype == ether_types.ETH_TYPE_ARP:
                    arp_pkt = pkt.get_protocol(arp.arp)
                    if arp_pkt.opcode == arp.ARP_REQUEST:

                        self.handle_arp_request(datapath=datapath, msg=msg, arp_pkt=arp_pkt)
                
                pass
            else:
                print('\nDHCPServer\n')
                DHCPServer.handle_dhcp(datapath, inPort, pkt)      
            return 
        except Exception as e:
            self.logger.error(e)
    
    
    def handle_arp_request(self, datapath, msg, arp_pkt):
        #print('\nhandle_arp_request   start')
        # print(arp_pkt)
        ofctl = OfCtl_v1_0(datapath, logger=logging.getLogger('handle_arp_request'))
        ofctl.send_arp(arp_opcode=2,
                       vlan_id=VLANID_NONE,
                       dst_mac=arp_pkt.src_mac,
                       sender_mac=self.arp_IP_MAC_mapping_table[arp_pkt.dst_ip]['mac'],
                       sender_ip=arp_pkt.dst_ip,
                       target_ip=arp_pkt.src_ip,
                       target_mac=arp_pkt.src_mac,
                       src_port=ofproto_v1_0.OFPP_CONTROLLER,
                       output_port=msg.in_port)




