import pandas as pd
import numpy as np
import networkx as nx

folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_PGH_tdsp_node_cost/'
net_df = pd.read_csv(folder + 'graph', delimiter=' ', header=None, skiprows=1)
net_df.columns = ['edge_ID', 'from_node_ID', 'to_node_ID']

G=nx.from_pandas_edgelist(net_df, 'from_node_ID', 'to_node_ID', ['edge_ID'], create_using=nx.MultiDiGraph)
node_movement = []
for n in G.nodes:
    if G.in_degree(n) > 0 and G.out_degree(n) > 0:
        for e_in in G.in_edges(n, data=True):
            for e_out in G.out_edges(n, data=True):
                node_movement.append((n, e_in[2]['edge_ID'], e_out[2]['edge_ID']))


num_interval = 5000
num_node_cost = 10000
td_link_cost = np.zeros((G.number_of_edges(), num_interval), dtype=float)
td_node_cost = np.zeros((num_node_cost, num_interval), dtype=float)

td_link_cost = np.random.randint(1, 10, td_link_cost.shape)
td_node_cost = np.random.randint(1, 5, td_node_cost.shape)

link_ID = np.array([e[2]['edge_ID'] for e in G.edges(data=True)], dtype=int)
td_link_cost = np.concatenate((link_ID[:, np.newaxis], td_link_cost), axis=1)
assert(td_link_cost.shape[1] == num_interval + 1)

np.savetxt(folder + 'td_link_cost', td_link_cost, fmt='%d ' + (num_interval-1)*'%f ' + '%f')
f = open(folder + 'td_link_cost', 'r')
log = f.readlines()
f.close()
log.insert(0, 'link_ID td_cost\n')
f = open(folder + 'td_link_cost', 'w')
f.writelines(log)
f.close()

ind = np.random.choice(len(node_movement), num_node_cost, replace=False)
node_ID = np.array([node_movement[i] for i in ind], dtype=int)
td_node_cost = np.concatenate((node_ID, td_node_cost), axis=1)
assert(td_node_cost.shape[1] == num_interval + 3)

np.savetxt(folder + 'td_node_cost', td_node_cost, fmt='%d %d %d ' + (num_interval-1)*'%f ' + '%f')
f = open(folder + 'td_node_cost', 'r')
log = f.readlines()
f.close()
log.insert(0, 'node_ID in_link_ID out_link_ID td_cost\n')
f = open(folder + 'td_node_cost', 'w')
f.writelines(log)
f.close()