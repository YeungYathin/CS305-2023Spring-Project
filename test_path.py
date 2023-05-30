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
        # self.run_Dijkstra()
        self.run_DV()
    
    #往图中添加连接，对应link_add，同时更新整张图的路由表
    def modify_link(self,link):
        """
        links:[[src1,dst1,w1]]
        """
        idx_src=self.nodes.index(link[0])
        idx_dst=self.nodes.index(link[1])
        self.adjacency_matrix[idx_src][idx_dst]=link[2]
        self.adjacency_matrix[idx_dst][idx_src]=link[2]
        # self.run_Dijkstra()
        self.run_DV()


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

graph=Graph([],[])
graph.add_node(4)
graph.add_node(1)
graph.add_node(6)
graph.add_node(3)
graph.add_node(2)
graph.add_node(5)

graph.modify_link([1,2,1])
print("add link between 1,2")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([1,4,1])
print("add link between 1,4")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([1,5,1])
print("add link between 1,5")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([2,6,1])
print("add link between 2,6")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([3,4,1])
print("add link between 3,4")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([3,6,1])
print("add link between 3,6")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([4,5,1])
print("add link between 4,5")
for ele in graph.routing_tables:
    print(ele)
graph.modify_link([5,6,1])
print("add link between 5,6")
for ele in graph.routing_tables:
    print(ele)
    
    
    
# graph.modify_link([1,2,1])
# # print("add link between 1,2")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([1,4,1])
# # print("add link between 1,4")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([1,5,1])
# # print("add link between 1,5")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([2,6,1])
# # print("add link between 2,6")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([3,4,1])
# # print("add link between 3,4")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([3,6,1])
# # print("add link between 3,6")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([4,5,1])
# # print("add link between 4,5")
# # for ele in graph.routing_tables:
# #     print(ele)
# graph.modify_link([5,6,1])
# # print("add link between 5,6")
# # for ele in graph.routing_tables:
# #     print(ele)

# print(graph.get_neighbors(2))
# for row in graph.adjacency_matrix:
#     print(row)

#[4,1,6,3,2,5]
#[4,
# 1,
# 6,
# 3,
# 2,
# 5]

















# WYH版本Graph
# #用于计算最短路，nodes与class ControllerApp中的switch.dpid对应
# class Graph:
#     def __init__(self, nodes, adjacency_matrix):
#         self.nodes = nodes
#         self.adjacency_matrix = adjacency_matrix
#         self.num_nodes = len(nodes)

#         #存储某个临时节点的路由表，其元素是key:value键值对，具体为：dp:(distance,next_hop)
#         #A   {A:None,B:(1,C)}
#         #A ->.....->C->B
#         self.routing_table = {}
#         self.routing_tables = []#存储全部节点的路由表，其元素是self.routing_table

#     #初始化某个节点的路由表
#     def initialize_routing_table(self, source_node):
#         for node in self.nodes:
#             if node == source_node:
#                 self.routing_table[node] = (0, None)
#             else:
#                 self.routing_table[node] = (float('inf'), None)

#     #更新某个节点路由表
#     def update_routing_table_DV(self, source_node):
#         for node in self.nodes:
#             if node != source_node:
#                 min_distance = self.routing_table[node][0]
#                 next_hop = self.routing_table[node][1]
#                 for neighbor in self.get_neighbors(node):
#                     distance = self.routing_table[self.nodes[neighbor]][0] + self.adjacency_matrix[neighbor][self.nodes.index(node)]
#                     if distance < min_distance or (next_hop==source_node and distance <=min_distance):
#                         min_distance = distance
#                         next_hop = self.nodes[neighbor]
#                 if next_hop==node:
#                     self.routing_table[node] = (min_distance, source_node)
#                 else:
#                     self.routing_table[node] = (min_distance, next_hop)

#     def get_neighbors(self, node):
#         neighbors = []
#         for i in range(self.num_nodes):
#             if self.adjacency_matrix[self.nodes.index(node)][i] != float('inf'):
#                 neighbors.append(i)
#         return neighbors

#     #对图中的每一个节点，求其路由表routing_table，并存入routing_tables中
#     def run_DV(self):
#         self.routing_tables=[]
#         for source_node in self.nodes:
#             self.initialize_routing_table(source_node)
#             for _ in range(self.num_nodes - 1):
#                 self.update_routing_table_DV(source_node)
#             #print(self.routing_table)
#             tmp=copy.deepcopy(self.routing_table)
#             self.routing_tables.append(tmp)

#     def print_routing_table(self):
#         #存储某个临时节点的路由表，其元素是key:value键值对，具体为：dp:(distance,next_hop)
#         #A   {A:None,B:(1,C)}
#         #A ->.....->C->B
#         idx1=0
#         for route_table in self.routing_tables:
#             cur_node=self.nodes[idx1]
#             print("\ns{} to other switch:".format(cur_node))
#             idx1+=1
#             idx2=-1
#             for dpid,val in route_table.items():
#                 idx2+=1
#                 if dpid==cur_node:
#                     continue
#                 #ans=[self.nodes[idx2]]
#                 ans=[dpid]
#                 dis,next_hop=val[0],val[1]
#                 flag=True
#                 while(next_hop!=cur_node):
#                     if next_hop is None or dis==float('inf'):
#                         print("s{} can not reach s{}".format(cur_node,dpid))
#                         flag=False
#                         break
#                     ans.insert(0,next_hop)
#                     next_hop=route_table[next_hop][1]
#                     dis=route_table[next_hop][0]
#                 if flag:
#                     formatted_str = '->'.join(['s{}'.format(num) for num in ans])
#                     print(("s{}->".format(cur_node))+formatted_str+", cost is {}".format(len(ans)))



#     #往图中添加节点，对应switch_add
#     def add_node(self,node):
#         if node in self.nodes:
#             return 
#         self.nodes.append(node)
#         self.num_nodes=len(self.nodes)
#         self.adjacency_matrix.append([float('inf')] * (self.num_nodes - 1) + [0])
#         for i in range(self.num_nodes-1):
#             self.adjacency_matrix[i].append(float('inf'))
    
#     def delete_node(self,node):
#         idx=self.nodes.index(node)
#         self.nodes.remove(node)
#         self.num_nodes=len(self.nodes)
#         del self.adjacency_matrix[idx]
#         for i in range(self.num_nodes):
#             del self.adjacency_matrix[i][idx]
#         self.run_DV()
    
#     #往图中添加连接，对应link_add，同时更新整张图的路由表
#     def modify_link(self,link):
#         """
#         links:[[src1,dst1,w1]]
#         """
#         idx_src=self.nodes.index(link[0])
#         idx_dst=self.nodes.index(link[1])
#         self.adjacency_matrix[idx_src][idx_dst]=link[2]
#         self.adjacency_matrix[idx_dst][idx_src]=link[2]
#         self.run_DV()


#     #计算单个节点的转发表
#     #类成员routing_tables存储的是某个节点通过哪个节点到目标节点
#     #节点A的routing_table形如{'A': (0, None), 'B': (1, 'A'), 'C': (3, 'A'), 'D': (2, 'B'), 'E': (4, 'D')}
#     #表示A->A:无需转发，    A->B：A->B(A直接转发给B),     A->C:A->C,    A->D:A->....->B->D

#     #该函数功能为利用routing_table计算下一跳（转发给哪个交换机）
#     def get_path(self,src_node,router_list):
#         ans=[]
#         for node in self.nodes:
#             if node!=src_node:
#                 tmp=node
#                 while(router_list[tmp][1]!=src_node):
#                     if router_list[tmp][0]==float('inf'):
#                         ans.append(None)
#                         break
#                     tmp=router_list[tmp][1]
#                 if router_list[tmp][0]!=float('inf'):
#                     ans.append(tmp)
#             else:
#                 ans.append(src_node)
#         return ans