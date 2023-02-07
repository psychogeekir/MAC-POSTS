# %%
import MNMAPI
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

# %%
folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_due'
gap_file = os.path.join(folder, 'record/gap_iteration')
path_flow_file = os.path.join(folder, 'record/path_table_buffer')

# %%
max_iter = 100
verbose = False
with_dtc = False
# method = 'GP'
method = 'MSA'

# %%
dta = MNMAPI.dta_api()
dta.run_due(max_iter, folder, verbose, with_dtc, method)
# %%
gap_iters = np.loadtxt(gap_file)
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), gap_iters, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Gap', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

# plt.savefig(os.path.join(folder, fig_name))

plt.show()
# %%
