import numpy as np
import rerun as rr
import rerun.blueprint as rrb

def load_traj(path):
  ts = []
  pos = []
  with open(path, "r") as f:
    for line in f:
      parts = line.strip().split()
      if len(parts) != 8:
        continue
      t = float(parts[0])
      x, y, z = map(float, parts[1:4])
      ts.append(t)
      pos.append([x, y, z])
  return np.array(ts), np.array(pos)


def flatten_xy(positions):
  # z 무시하고 XY만 사용
  return positions[:, :2]


def main():
  f1 = "/home/tw/mf/project_manage_vcs/spagri/perception/mflib/slam3d/glim/dump/traj_imu.txt"
  f2 = "/home/tw/mf/project_manage_vcs/spagri/perception/mflib/slam3d/glim/dump/dump2/traj_imu.txt"

  t1, p1 = load_traj(f1)
  t2, p2 = load_traj(f2)

  rr.init("traj_2d_compare", spawn=True)

  # Flatten
  p1_2d = flatten_xy(p1)
  p2_2d = flatten_xy(p2)

  rr.log("traj1/points2d", rr.Points2D(p1_2d, colors=[255,0,0]))
  rr.log("traj2/points2d", rr.Points2D(p2_2d, colors=[0,255,0]))

  # Optionally arrows2d for motion direction
  if len(p1_2d) >= 2:
    origins = p1_2d[:-1]
    vecs = p1_2d[1:] - p1_2d[:-1]
    rr.log("traj1/arrows2d", rr.Arrows2D(origins=origins, vectors=vecs, colors=[255,0,0]))

  if len(p2_2d) >= 2:
    origins = p2_2d[:-1]
    vecs = p2_2d[1:] - p2_2d[:-1]
    rr.log("traj2/arrows2d", rr.Arrows2D(origins=origins, vectors=vecs, colors=[0,255,0]))

  # Setup 2D view
  blueprint = rrb.Blueprint(
    rrb.Spatial2DView(
      origin="/",
      name="TopDown 2D View",
      # optional: set background, visual_bounds 등
      background=[30, 30, 30],
      # e.g. x and y bounds — adjust based on your data
      visual_bounds=rrb.VisualBounds2D(x_range=[-5, 5], y_range=[-5, 5]),
    ),
    collapse_panels=True,
  )
  rr.send_blueprint(blueprint)

  print("Logged 2D trajectories. Open viewer and use the 2D view.")

if __name__ == "__main__":
  main()
