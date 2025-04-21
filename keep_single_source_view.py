# -*- coding: utf-8 -*-
input_path = 'data_splits/custom/city_sim_AB_test_eight_view_deepvmvs_dense.txt'
output_path = 'data_splits/custom/city_sim_AB_test_eight_view_deepvmvs_dense.txt'

with open(input_path, 'r') as fin:
    lines = fin.readlines()

filtered_lines = []
for line in lines:
    tokens = line.strip().split()
    if len(tokens) < 2:
        continue
    source_id = int(tokens[1])
    if source_id % 2 == 1:
        filtered_lines.append(line)

with open(output_path, 'w') as fout:
    fout.writelines(filtered_lines)

print("筛选完成，结果保存在:", output_path)
