import json
import numpy as np
import scipy.io as sio
import glob

json_files = glob.glob('*.json')
print(json_files)

# 每个文件对应的效率区间
eta_band_map = {
    '80': 78,
    '80to85': 82.5,
    '85to88': 86.5,
    '88to90': 89,
    '90to92': 91,
    '92to93': 92.5,
    '93to94': 93.5,
    '94to95': 94.5,
    '95to96': 95.5,
    '96to97': 96.5,
    '97': 98
}
# ============================

rpm_all = []
torque_all = []
eta_all = []

for jf in json_files:
    with open(jf, 'r', encoding='utf-8') as f:
        data = json.load(f)

    points = data['datasetColl'][0]['data']

    for key in eta_band_map:
        if key in jf:
            eta_band = eta_band_map[key]
            break

    for p in points:
        rpm_all.append(p['value'][0])
        torque_all.append(p['value'][1])
        eta_all.append(eta_band)

eff_data = {
    'rpm': np.array(rpm_all),
    'torque': np.array(torque_all),
    'eta': np.array(eta_all)
}

sio.savemat('efficiency_map_all.mat', {'eff_data': eff_data})
print('Merged efficiency map saved.')