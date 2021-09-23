# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%



# %%
import os
import numpy as np
import pandas as pd

from MNM_mmnb import MNM_network_builder
import MNMAPI


# %%
data_folder = os.path.join('.', 'side_project', 'network_builder', 'MNM_cache', 'input_files_7link_multimodal_dode_generated')


# %%
nb = MNM_network_builder()
nb.load_from_folder(data_folder)
nb.ID2path


# %%
a = MNMAPI.mmdta_api()
a.initialize(data_folder)
a


# %%
link_driving_array = np.array([e.ID for e in nb.link_driving_list], dtype=np.int)
link_driving_array
a.register_links_driving(link_driving_array)


# %%
link_bus_array = np.array([e.ID for e in nb.link_bus_list], dtype=np.int)
a.register_links_bus(link_bus_array)


# %%
link_walking_array = np.array([e.ID for e in nb.link_walking_list], dtype=np.int)
a.register_links_walking(link_walking_array)


# %%
path_array = np.array(list(nb.ID2path.keys()), dtype=np.int)
a.register_paths(path_array)


