#!/usr/bin/env python3
"""Batch runner for stereo_ksm over multiple KSM datasets and configs.

For each (dataset, config) pair:
  1. runs stereo_ksm with cwd set to the per-run result directory
  2. collects CameraTrajectoryTUM.txt / CameraTrajectory.txt and copies the GT poses.txt
  3. evaluates with evo (ATE via evo_ape, RPE via evo_rpe) and saves plots (no GUI)
Per dataset it also renders one plot with all config trajectories vs GT (evo_traj)
and writes metrics_summary.csv aggregating ATE/RPE for every config.

Usage: edit DATASET_LIST / CONFIG_LIST below, then
    python3 scripts/run_ksm_experiments.py
"""

import csv
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

# --------------------------------------------------------------------------
# User configuration
# --------------------------------------------------------------------------
DATASET_LIST = [
    "/root/nas/dataset/KSM/scene07/scene07_cs02_s1p8_p060",
    # add more dataset sequence directories here
]

CONFIG_LIST = [
    "/root/ORB_SLAM2_with_rerun/Examples/Stereo/KSM_offline.yaml",
    "/root/ORB_SLAM2_with_rerun/Examples/Stereo/KSM_online.yaml",
]

EXECUTABLE_PATH = "/root/ORB_SLAM2_with_rerun/Examples/Stereo/stereo_ksm"
VOC_PATH = "/root/ORB_SLAM2_with_rerun/Vocabulary/ORBvoc.txt"
RESULT_PATH = "~/share/Results/"

T_MAX_DIFF = 0.01
RPE_DELTA = 1.0
RPE_DELTA_UNIT = "m"   # relative error per 1 m of travelled distance
PLOT_MODE = "xz"

# --------------------------------------------------------------------------

TUM_TRAJECTORY_NAME = "CameraTrajectoryTUM.txt"
KITTI_TRAJECTORY_NAME = "CameraTrajectory.txt"
STAT_KEYS = ("rmse", "mean", "median", "std", "min", "max")
STAT_PATTERN = re.compile(r"^\s*(rmse|mean|median|std|min|max)\s+([0-9.eE+-]+)\s*$", re.MULTILINE)

EVO_ENV = {**os.environ, "MPLBACKEND": "Agg"}


def log(message):
    print(f"[run_ksm] {message}", flush=True)


def parse_evo_stats(stdout):
    """Extract the rmse/mean/... block from evo_ape / evo_rpe stdout."""
    return {key: float(value) for key, value in STAT_PATTERN.findall(stdout)}


def run_evo(command, stats_file):
    """Run an evo command headless, save stdout, return parsed stats (None on failure)."""
    result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=EVO_ENV
    )
    stats_file.write_text(result.stdout)
    if result.returncode != 0:
        log(f"evo command failed (rc={result.returncode}): {' '.join(map(str, command))}")
        log(f"  see {stats_file}")
        return None
    return parse_evo_stats(result.stdout)


def run_slam(config, dataset, run_dir):
    """Run stereo_ksm inside run_dir so trajectories land there. Returns True on success."""
    log_file = run_dir / "slam_log.txt"
    with log_file.open("w") as handle:
        result = subprocess.run(
            [EXECUTABLE_PATH, VOC_PATH, str(config), str(dataset)],
            cwd=run_dir, stdout=handle, stderr=subprocess.STDOUT,
        )
    if result.returncode != 0:
        log(f"stereo_ksm failed (rc={result.returncode}), see {log_file}")
        return False
    if not (run_dir / TUM_TRAJECTORY_NAME).is_file():
        log(f"stereo_ksm produced no {TUM_TRAJECTORY_NAME} in {run_dir}")
        return False
    return True


def evaluate_run(run_dir, gt_file, est_file):
    """Run evo_ape + evo_rpe (trans/rot) for one run. Returns dict of stats per metric."""
    evo_dir = run_dir / "evo"
    evo_dir.mkdir(parents=True, exist_ok=True)
    metrics = {}

    ape_cmd = [
        "evo_ape", "tum", str(gt_file), str(est_file),
        "--align", "--t_max_diff", str(T_MAX_DIFF), "--no_warnings",
        "--plot_mode", PLOT_MODE,
        "--save_plot", str(evo_dir / "ape_plot.png"),
        "--save_results", str(evo_dir / "ape_results.zip"),
    ]
    metrics["ate"] = run_evo(ape_cmd, evo_dir / "ape_stats.txt")

    for relation, tag in (("trans_part", "rpe_trans"), ("angle_deg", "rpe_rot")):
        rpe_cmd = [
            "evo_rpe", "tum", str(gt_file), str(est_file),
            "--align", "--t_max_diff", str(T_MAX_DIFF), "--no_warnings",
            "--pose_relation", relation,
            "--delta", str(RPE_DELTA), "--delta_unit", RPE_DELTA_UNIT,
            "--save_plot", str(evo_dir / f"{tag}_plot.png"),
            "--save_results", str(evo_dir / f"{tag}_results.zip"),
        ]
        metrics[tag] = run_evo(rpe_cmd, evo_dir / f"{tag}_stats.txt")

    return metrics


def plot_all_configs(result_dataset_dir, gt_file, trajectories):
    """One overlay plot of every config trajectory vs GT via evo_traj.

    evo_traj labels curves by file name, so copy each trajectory to <config_name>.tum first.
    """
    if not trajectories:
        return
    plot_dir = result_dataset_dir / ".evo_traj_tmp"
    plot_dir.mkdir(parents=True, exist_ok=True)
    labelled = []
    for config_name, est_file in trajectories:
        labelled_file = plot_dir / f"{config_name}.tum"
        shutil.copy2(est_file, labelled_file)
        labelled.append(str(labelled_file))

    command = [
        "evo_traj", "tum", *labelled,
        "--ref", str(gt_file), "--align", "--t_max_diff", str(T_MAX_DIFF),
        "--no_warnings", "--plot_mode", PLOT_MODE,
        "--save_plot", str(result_dataset_dir / "all_configs_trajectory.png"),
    ]
    run_evo(command, result_dataset_dir / "evo_traj_log.txt")
    shutil.rmtree(plot_dir, ignore_errors=True)


def write_summary_csv(result_dataset_dir, rows):
    columns = ["config", "slam_status"]
    for metric, keys in (("ate", STAT_KEYS), ("rpe_trans", STAT_KEYS[:4]), ("rpe_rot", STAT_KEYS[:4])):
        columns += [f"{metric}_{key}" for key in keys]

    csv_path = result_dataset_dir / "metrics_summary.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in columns})
    log(f"wrote {csv_path}")


def summary_row(config_name, status, metrics):
    row = {"config": config_name, "slam_status": status}
    for metric, stats in metrics.items():
        if not stats:
            continue
        for key, value in stats.items():
            row[f"{metric}_{key}"] = f"{value:.6f}"
    return row


def check_prerequisites():
    problems = []
    for tool in ("evo_ape", "evo_rpe", "evo_traj"):
        if shutil.which(tool) is None:
            problems.append(f"missing evo tool: {tool}")
    if not Path(EXECUTABLE_PATH).is_file():
        problems.append(f"missing executable: {EXECUTABLE_PATH}")
    if not Path(VOC_PATH).is_file():
        problems.append(f"missing vocabulary: {VOC_PATH}")
    for config in CONFIG_LIST:
        if not Path(config).is_file():
            problems.append(f"missing config: {config}")
    for dataset in DATASET_LIST:
        if not Path(dataset).is_dir():
            problems.append(f"missing dataset: {dataset}")
    return problems


def main():
    problems = check_prerequisites()
    if problems:
        for problem in problems:
            log(f"ERROR: {problem}")
        return 1

    result_root = Path(RESULT_PATH).expanduser()
    result_root.mkdir(parents=True, exist_ok=True)

    failures = []
    overview = []  # (dataset_name, config_name, status, ate_rmse)

    for dataset in DATASET_LIST:
        dataset_dir = Path(dataset).expanduser()
        dataset_name = dataset_dir.name
        result_dataset_dir = result_root / dataset_name
        result_dataset_dir.mkdir(parents=True, exist_ok=True)

        gt_path = dataset_dir / "poses.txt"
        if not gt_path.is_file():
            log(f"ERROR: no ground truth {gt_path}, skipping dataset {dataset_name}")
            failures.append(f"{dataset_name}: missing poses.txt")
            continue

        log(f"=== dataset {dataset_name} ===")
        summary_rows = []
        trajectories = []  # (config_name, est_file) for the combined plot
        gt_copy_for_plot = None

        for config in CONFIG_LIST:
            config_path = Path(config)
            config_name = config_path.stem
            run_dir = result_dataset_dir / config_name
            trajectory_dir = run_dir / "trajectory"
            run_dir.mkdir(parents=True, exist_ok=True)
            trajectory_dir.mkdir(parents=True, exist_ok=True)

            log(f"--- {dataset_name} / {config_name}: running stereo_ksm ---")
            if not run_slam(config_path, dataset_dir, run_dir):
                failures.append(f"{dataset_name}/{config_name}: SLAM run failed")
                summary_rows.append(summary_row(config_name, "failed", {}))
                overview.append((dataset_name, config_name, "failed", ""))
                continue

            est_file = trajectory_dir / TUM_TRAJECTORY_NAME
            shutil.move(str(run_dir / TUM_TRAJECTORY_NAME), est_file)
            kitti_file = run_dir / KITTI_TRAJECTORY_NAME
            if kitti_file.is_file():
                shutil.move(str(kitti_file), trajectory_dir / KITTI_TRAJECTORY_NAME)
            gt_file = trajectory_dir / "poses.txt"
            shutil.copy2(gt_path, gt_file)
            gt_copy_for_plot = gt_file

            log(f"--- {dataset_name} / {config_name}: evaluating with evo ---")
            metrics = evaluate_run(run_dir, gt_file, est_file)
            if any(stats is None for stats in metrics.values()):
                failures.append(f"{dataset_name}/{config_name}: evo evaluation failed")
                status = "evo_failed"
            else:
                status = "ok"

            summary_rows.append(summary_row(config_name, status, metrics))
            ate_rmse = (metrics.get("ate") or {}).get("rmse", "")
            overview.append((dataset_name, config_name, status, ate_rmse))
            trajectories.append((config_name, est_file))

        if trajectories and gt_copy_for_plot is not None:
            log(f"--- {dataset_name}: combined trajectory plot ---")
            plot_all_configs(result_dataset_dir, gt_copy_for_plot, trajectories)
        write_summary_csv(result_dataset_dir, summary_rows)

    print()
    print(f"{'dataset':<40} {'config':<25} {'status':<12} {'ate_rmse':<12}")
    for dataset_name, config_name, status, ate_rmse in overview:
        ate = f"{ate_rmse:.6f}" if isinstance(ate_rmse, float) else str(ate_rmse)
        print(f"{dataset_name:<40} {config_name:<25} {status:<12} {ate:<12}")

    if failures:
        print()
        for failure in failures:
            log(f"FAILURE: {failure}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
