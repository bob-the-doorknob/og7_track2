"""Locate map/visibility rejection distances in recorded turn-exit previews."""
import sys
from pathlib import Path
import numpy as np
from team_driver.pure_pursuit import PurePursuit
from team_driver.map_guard import MapGuard
from team_driver.safety import LidarSafety

root = Path('/hackathon/.worktrees/task1-residual-rl-baseline')
path = PurePursuit(root/'maps/icra26_compete_racing_line.csv', lookahead_max=1.5)
guard = MapGuard(root/'maps/icra26_compete.yaml', path)
safety = LidarSafety(horizon=10, max_deceleration=2.8)
capture = Path(sys.argv[1])
raw = np.load(capture)
limits = np.load(capture.with_suffix('.preview.npz'))['values']
for arc in (19, 20, 21):
    indices = np.flatnonzero((limits[:,3]>=arc)&(limits[:,3]<arc+1))
    idx = indices[len(indices)//2]
    sample = raw['samples'][idx]
    _, x,y,yaw,speed,*_ = sample
    steering = limits[idx,9]
    points = path.preview_trajectory((x,y),yaw,max(speed,limits[idx,7]),steering)
    obstacles, valid = safety._scan_points(raw['scans'][idx],raw['angles'])
    print('arc', arc, 'pose',sample.tolist(), 'preview_points',len(points), 'raw_clearance',
          safety._trajectory_clearance(obstacles,points))
    print('prefix certifications',[(round(n*.1,1),guard.certifies_trajectory((x,y),yaw,points[:n+1]))
          for n in range(10,len(points),10)])
    rotation=np.asarray(((np.cos(yaw),np.sin(yaw)),(-np.sin(yaw),np.cos(yaw))))
    world=np.asarray((x,y))+points@rotation
    for n, point in enumerate(world):
        if not guard._free_disk(point):
            print('first nonfree disk',n*.1,point.tolist())
            break
