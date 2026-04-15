#!/usr/bin/env python3
import csv
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output"
SWEEP_DIR = OUTPUT_DIR / "isd-sweep"
SUMMARY_CSV = SWEEP_DIR / "isd-sweep-summary.csv"
SCENARIO = "scratch/nr-x2-handover-measures-channel-test"

ISD_VALUES = [41600.0, 41750.0, 41900.0]


def percentile(values, p):
    values = sorted(values)
    idx = max(0, min(len(values) - 1, int(len(values) * p)))
    return values[idx]


def summarize_metrics(csv_path: Path):
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))

    sinr = [float(r["sinrDb"]) for r in rows]
    cl = [float(r["servingDlCouplingLossDb"]) for r in rows]

    return {
        "ue_count": len(rows),
        "sinr_p5": percentile(sinr, 0.05),
        "sinr_p50": percentile(sinr, 0.50),
        "sinr_p95": percentile(sinr, 0.95),
        "cl_p5": percentile(cl, 0.05),
        "cl_p50": percentile(cl, 0.50),
        "cl_p95": percentile(cl, 0.95),
    }


def run_one(isd_value: float):
    cmd = [
        "./ns3",
        "run",
        (
            f'{SCENARIO} --snapshotOnce=1 --snapshotDurationMs=30 '
            f'--snapshotUesPerCell=100 --snapshotCellRadiusMeters=25000 '
            f'--shadowingEnabled=1 --interSiteDistanceMeters={isd_value}'
        ),
    ]
    subprocess.run(cmd, cwd=ROOT, check=True)

    source_csv = OUTPUT_DIR / "channel-test-all-ue-sinr.csv"
    target_csv = SWEEP_DIR / f"channel-test-all-ue-sinr-isd-{isd_value:.4f}.csv"
    shutil.copy2(source_csv, target_csv)
    return summarize_metrics(source_csv)


def main():
    SWEEP_DIR.mkdir(parents=True, exist_ok=True)

    with SUMMARY_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "isd_m",
                "ue_count",
                "cl_p5",
                "cl_p50",
                "cl_p95",
                "sinr_p5",
                "sinr_p50",
                "sinr_p95",
            ],
        )
        writer.writeheader()

        for isd in ISD_VALUES:
            summary = run_one(isd)
            writer.writerow({"isd_m": isd, **summary})
            print(
                f"isd={isd:.4f} "
                f"CL=({summary['cl_p5']:.3f}, {summary['cl_p50']:.3f}, {summary['cl_p95']:.3f}) "
                f"SINR=({summary['sinr_p5']:.3f}, {summary['sinr_p50']:.3f}, {summary['sinr_p95']:.3f})"
            )


if __name__ == "__main__":
    main()
