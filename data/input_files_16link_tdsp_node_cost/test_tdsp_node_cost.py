# %%
import MNMAPI
import numpy as np
import os

folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_16link_tdsp_node_cost"
max_interval = 100
num_rows_link_file = 23
num_rows_node_file = 10

link_tt_file_name = "td_link_tt"
node_tt_file_name = "td_node_tt"

link_cost_file_name = "td_link_cost"
node_cost_file_name = "td_node_cost"

dest_node_ID = 13
origin_node_ID = 1

# %%
tdsp_api = MNMAPI.tdsp_api()
tdsp_api.initialize(folder, max_interval, num_rows_link_file, num_rows_node_file)

# %%
# Two ways to read time-dependent node and link cost and tt files
# %% method 1: previous method, using plain txt
tdsp_api.read_td_cost_txt(folder, link_tt_file_name, node_tt_file_name, link_cost_file_name, node_cost_file_name)

#%% method 2: suppose the data is stored in numpy arrays
td_link_tt = np.load(os.path.join(folder, link_tt_file_name + '.npy'))
td_link_cost = np.load(os.path.join(folder, link_cost_file_name + '.npy'))
td_node_tt = np.load(os.path.join(folder, node_tt_file_name + '.npy'))
td_node_cost = np.load(os.path.join(folder, node_cost_file_name + '.npy'))

tdsp_api.read_td_cost_py(td_link_tt, td_link_cost, td_node_tt, td_node_cost)

# %%
tdsp_api.build_tdsp_tree(dest_node_ID)

# %%
timestamp = 5
tdsp = tdsp_api.extract_tdsp(origin_node_ID, timestamp)
# tdsp : number of nodes * 4
# first col: node sequence
# second col: link sequence with last element being -1
# third col: first element being the travel cost
# fourth col: first element being the travel time
print(tdsp)
# %%
