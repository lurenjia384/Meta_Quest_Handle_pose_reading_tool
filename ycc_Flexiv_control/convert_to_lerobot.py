import argparse
import csv
from pathlib import Path

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def load_episode(ep_dir: Path):
    """Yield (state, action, top_img, front_img) for each row of an episode."""
    csv_path = ep_dir / "data.csv"
    img_dir = ep_dir / "images"
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(row["frame"])
            state = np.array([
                float(row["s_x"]), float(row["s_y"]), float(row["s_z"]),
                float(row["s_qw"]), float(row["s_qx"]), float(row["s_qy"]), float(row["s_qz"]),
                float(row["s_gripper"]),
            ], dtype=np.float32)
            action = np.array([
                float(row["a_x"]), float(row["a_y"]), float(row["a_z"]),
                float(row["a_qw"]), float(row["a_qx"]), float(row["a_qy"]), float(row["a_qz"]),
                float(row["a_gripper"]),
            ], dtype=np.float32)
            top = np.asarray(Image.open(img_dir / f"top_{frame:05d}.png"))
            front = np.asarray(Image.open(img_dir / f"front_{frame:05d}.png"))
            yield state, action, top, front


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True,
                        help="raw recording session directory")
    parser.add_argument("--dataset-root", default="datasets",
                        help="root directory for the lerobot dataset")
    parser.add_argument("--dataset-name", default="flexiv_vr_001",
                        help="lerobot repo_id / dataset directory name")
    parser.add_argument("--task", default="teleop_task",
                        help="task label for all episodes")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    session = Path(args.session_dir)
    ep_dirs = sorted(session.glob("episode_*"))
    if not ep_dirs:
        print(f"no episode_* dirs found in {session}")
        return 1
    print(f"found {len(ep_dirs)} episodes in {session}")

    # probe first episode to get shapes
    s0, a0, top0, front0 = next(load_episode(ep_dirs[0]))
    img_shape = top0.shape
    print(f"state shape {s0.shape}, image shape {img_shape}")

    features = {
        "observation.state": {"dtype": "float32", "shape": tuple(s0.shape),
                              "names": ["x", "y", "z", "qw", "qx", "qy", "qz", "gripper"]},
        "observation.image.top": {"dtype": "video", "shape": tuple(img_shape)},
        "observation.image.front": {"dtype": "video", "shape": tuple(img_shape)},
        "action": {"dtype": "float32", "shape": tuple(a0.shape),
                   "names": ["x", "y", "z", "qw", "qx", "qy", "qz", "gripper"]},
    }

    root = Path(args.dataset_root) / args.dataset_name
    if root.exists():
        print(f"ERROR: dataset dir already exists: {root}")
        print("       choose a new --dataset-name or remove the old dir.")
        return 1
    root.parent.mkdir(parents=True, exist_ok=True)
    ds = LeRobotDataset.create(
        repo_id=args.dataset_name,
        fps=args.fps,
        features=features,
        root=root,
        robot_type="flexiv_rizon4",
        use_videos=True,
    )
    print(f"dataset created at {root / args.dataset_name}")

    total_frames = 0
    for ep_dir in ep_dirs:
        frames = 0
        for state, action, top, front in load_episode(ep_dir):
            ds.add_frame({
                "observation.state": state,
                "observation.image.top": top,
                "observation.image.front": front,
                "action": action,
                "task": args.task,
            })
            frames += 1
        ds.save_episode()
        total_frames += frames
        print(f"  episode {ep_dir.name}: {frames} frames")

    try:
        ds.stop_image_writer()
    except Exception:
        pass
    print(f"Done: {len(ep_dirs)} episodes, {total_frames} frames at {root / args.dataset_name}")


if __name__ == "__main__":
    main()
