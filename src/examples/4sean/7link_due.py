# %%
import MNMAPI
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

# %%
# nohup python 7link_due.py > 7link_due_output.log 2> 7link_due_output.err &
# folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_7link_due'
# folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_16link_due'
folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_PGH_due'
gap_file = os.path.join(folder, 'record/gap_iteration')
path_flow_file = os.path.join(folder, 'record/path_table_buffer')

# %%
max_iter = 100
verbose = False
with_dtc = False
method = 'GP'
# method = 'MSA'

# %%
dta = MNMAPI.dta_api()
dta.run_due(max_iter, folder, verbose, with_dtc, method)
# %%
result_iters = np.loadtxt(gap_file, delimiter=" ")
gap_iters = result_iters[:, 0]
tt_iters = result_iters[:, 1]
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), gap_iters, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Gap', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record/gap_iters.png"))

plt.show()
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), tt_iters, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Total travel time', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record/tt_iters.png"))

plt.show()

# %%
