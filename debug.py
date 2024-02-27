import torch
import time
import numpy as np

t0 = time.time()
for i in range(500):
    pose = np.random.rand(4,4)
    pose_torch = torch.Tensor(pose)
    pose_numpy = pose_torch.numpy()
duration = time.time()-t0
print(duration)
