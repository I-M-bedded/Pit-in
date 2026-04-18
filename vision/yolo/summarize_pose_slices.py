#!/usr/bin/env python
"""
Format slice_evaluation.json into readable markdown/CSV tables.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional


def fmt(value, digits: int = 4) -> str:
    if value is None:
        return "-"
    if isinstance(value, (int,)):
        return str(value)
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def load_results(path: Path) -> List[dict]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError("Expected a list of model results in slice evaluation JSON.")
    return data


def make_metric_row(result: dict) -> Dict[str, object]:
    metrics = result.get("results_dict", {})
    overall = result.get("slice_metrics", {}).get("overall", {})
    return {
        "model": result.get("model_name"),
        "mAP50_box": metrics.get("metrics/mAP50(B)"),
        "mAP50_95_box": metrics.get("metrics/mAP50-95(B)"),
        "mAP50_pose": metrics.get("metrics/mAP50(P)"),
        "mAP50_95_pose": metrics.get("metrics/mAP50-95(P)"),
        "recall": overall.get("recall"),
        "precision": overall.get("precision"),
        "kp_cov": overall.get("keypoint_coverage"),
        "pck05": overall.get("pck@0.05"),
        "pck10": overall.get("pck@0.10"),
        "mean_err": overall.get("mean_norm_error_matched"),
    }


def flatten_group(result: dict, group_name: str, keys: Iterable[str]) -> List[Dict[str, object]]:
    group = result.get("slice_metrics", {}).get(group_name, {})
    rows = []
    for key in keys:
        metrics = group.get(key, {})
        rows.append(
            {
                "model": result.get("model_name"),
                "slice": key,
                "image_count": metrics.get("image_count"),
                "recall": metrics.get("recall"),
                "precision": metrics.get("precision"),
                "kp_cov": metrics.get("keypoint_coverage"),
                "pck05": metrics.get("pck@0.05"),
                "pck10": metrics.get("pck@0.10"),
                "mean_err": metrics.get("mean_norm_error_matched"),
            }
        )
    return rows


def build_markdown_table(rows: List[Dict[str, object]], headers: List[str]) -> str:
    header_line = "| " + " | ".join(headers) + " |"
    sep_line = "| " + " | ".join(["---"] * len(headers)) + " |"
    body = []
    for row in rows:
        body.append("| " + " | ".join(fmt(row.get(h, row.get(h.lower()))) for h in headers) + " |")
    return "\n".join([header_line, sep_line, *body])


def write_csv(path: Path, rows: List[Dict[str, object]], headers: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow({h: row.get(h) for h in headers})


def summarize_results(input_path: Path, out_dir: Path) -> None:
    results = load_results(input_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    main_rows = [make_metric_row(result) for result in results]
    vis_keys = ["vis_0", "vis_1", "vis_2"]

    vis_rows: List[Dict[str, object]] = []
    vis_focus_rows: List[Dict[str, object]] = []
    light_block_rows: List[Dict[str, object]] = []

    for result in results:
        vis_rows.extend(flatten_group(result, "visibility_metrics", vis_keys))
        vis_focus_rows.extend(flatten_group(result, "visibility_metrics", ["vis_1", "vis_2"]))
        group = result.get("slice_metrics", {}).get("lighting_block_metrics", {})
        for key in sorted(group.keys()):
            light_block_rows.extend(flatten_group(result, "lighting_block_metrics", [key]))

    main_headers = [
        "model",
        "mAP50_box",
        "mAP50_95_box",
        "mAP50_pose",
        "mAP50_95_pose",
        "recall",
        "precision",
        "kp_cov",
        "pck05",
        "pck10",
        "mean_err",
    ]
    slice_headers = ["model", "slice", "image_count", "recall", "precision", "kp_cov", "pck05", "pck10", "mean_err"]

    markdown_parts = [
        "# Slice Evaluation Summary",
        "",
        "## Overall",
        build_markdown_table(main_rows, main_headers),
        "",
        "## Visibility Slices",
        build_markdown_table(vis_rows, slice_headers),
        "",
        "## Visibility Focus",
        build_markdown_table(vis_focus_rows, slice_headers),
        "",
        "## Lighting Blocks",
        build_markdown_table(light_block_rows, slice_headers),
        "",
    ]

    markdown_path = out_dir / "slice_summary.md"
    markdown_path.write_text("\n".join(markdown_parts), encoding="utf-8")

    write_csv(out_dir / "overall.csv", main_rows, main_headers)
    write_csv(out_dir / "visibility.csv", vis_rows, slice_headers)
    write_csv(out_dir / "visibility_focus.csv", vis_focus_rows, slice_headers)
    write_csv(out_dir / "lighting_blocks.csv", light_block_rows, slice_headers)

    print(f"Saved markdown summary to {markdown_path.resolve()}")
    print(f"Saved CSV tables to {out_dir.resolve()}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Summarize slice evaluation JSON into markdown/CSV tables")
    ap.add_argument(
        "--input",
        type=Path,
        default=Path("vision/result/baseline_slice_eval/slice_evaluation.json"),
    )
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=Path("vision/result/baseline_slice_eval/summary"),
    )
    args = ap.parse_args()

    summarize_results(args.input, args.out_dir)


if __name__ == "__main__":
    main()
