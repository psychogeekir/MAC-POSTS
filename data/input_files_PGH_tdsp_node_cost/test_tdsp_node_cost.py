import MNMAPI
import numpy as np

folder = "/home/qiling/Documents/MAC-POSTS/data/input_files_PGH_tdsp_node_cost"
max_interval = 5000
num_rows_link_file = 12976
num_rows_node_file = 10000
link_cost_file_name = "td_link_cost"
node_cost_file_name = "td_node_cost"

dest_node_ID = 150361
origin_node_ID = 100264

# %%
tdsp_api = MNMAPI.tdsp_api()
tdsp_api.initialize(folder, max_interval, num_rows_link_file, num_rows_node_file, link_cost_file_name, node_cost_file_name)

# %%
tdsp_api.build_tdsp_tree(dest_node_ID)

# %%
timestamp = 1000
tdsp = tdsp_api.extract_tdsp(origin_node_ID, timestamp)
# tdsp : number of nodes * 3
# first col: node sequence
# second col: link sequence with last element being -1
# third col: first element being the travel time
print(tdsp)