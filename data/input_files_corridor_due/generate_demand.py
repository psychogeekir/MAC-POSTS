# %%
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# %%
header = '#Origin_ID Destination_ID <demand by interval>\n'
num_assign_inters = 60
slope = 0.1
# %%
# many to many
od_demand_total = pd.DataFrame()
od_demand_total['origin_ID'] = [11, 11, 11, 12, 12, 12, 13, 13]
od_demand_total['dest_ID'] = [14, 15, 16, 14, 15, 16, 15, 16]
od_demand_total['total_demand'] = [400., 600., 1000., 200., 400., 600., 200., 400.]
od_demand_total['slope'] = slope
od_demand_total.loc[od_demand_total['total_demand'] == 0, 'slope'] = 0
for i in range(num_assign_inters):
    od_demand_total[i] = od_demand_total['total_demand'] / num_assign_inters - od_demand_total['slope'] * (num_assign_inters / 2 - 0.5 - i)

# %%
np.min(od_demand_total.loc[:, list(range(num_assign_inters))].values)
# %%
od_demand_total.loc[:, list(range(num_assign_inters))].sum(axis=1)

# %%
od_demand_total.loc[:, ['origin_ID', 'dest_ID'] + list(range(num_assign_inters))].to_csv('/home/qiling/Documents/MAC-POSTS/data/input_files_corridor_due/MNM_input_demand', sep=" ", index=False, header=False)
# %%
f = open('/home/qiling/Documents/MAC-POSTS/data/input_files_corridor_due/MNM_input_demand', 'r')
log = f.readlines()
f.close()
# %%
log = [header] + log
# %%
f = open('/home/qiling/Documents/MAC-POSTS/data/input_files_corridor_due/MNM_input_demand', 'w')
f.writelines(log)
f.close()


# %%
folder = '/home/qiling/Documents/MAC-POSTS/data/input_files_corridor_due'
max_iter = 100
# %%
# due
result_iters = np.loadtxt(os.path.join(folder, 'record_due/gap_iteration'), delimiter=" ")
gap_iters_due = result_iters[:, 0]
tt_iters_due = result_iters[:, 1]
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), gap_iters_due, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Gap', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record_due/gap_iters.png"))

plt.show()
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), tt_iters_due, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Total travel time', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record_due/tt_iters.png"))

plt.show()

# %%
# dso
result_iters = np.loadtxt(os.path.join(folder, 'record_dso/gap_iteration'), delimiter=" ")
gap_iters_dso = result_iters[:, 0]
tt_iters_dso = result_iters[:, 1]
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), gap_iters_dso, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Gap', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record_dso/gap_iters.png"))

plt.show()
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), tt_iters_dso, color = 'r', marker = 'o', linewidth = 3)

plt.ylabel('Total travel time', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
# plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record_dso/tt_iters.png"))

plt.show()
# %%
# %%
plt.figure(figsize = (16, 9), dpi=300)
plt.plot(np.arange(max_iter), tt_iters_due, color = 'r', marker = 'o', linewidth = 3, label='DUE')
plt.plot(np.arange(max_iter), tt_iters_dso, color = 'b', marker = '*', linewidth = 3, label='DSO')

plt.ylabel('Total travel time', fontsize = 20)
plt.xlabel('Iteration', fontsize = 20)
plt.legend()
# plt.ylim([0, 1])
plt.xlim([0, max_iter])

plt.savefig(os.path.join(folder, "record_dso/tt_comparison.png"))

plt.show()
# %%
