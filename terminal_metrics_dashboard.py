#!/usr/bin/env python3
"""Live terminal view for ROS 2 architecture-recovery metrics.

This script does not call LLM APIs. It watches metrics_results.json and prints
the current metric values plus short explanations of what data each section
represents.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple


ERROR_CATEGORIES = [
    "interface_mismatch",
    "hallucinated_node",
    "subsystem_boundary_confusion",
    "missing_node",
    "wrong_topic_name",
    "lifecycle_violation",
]


def pct(value: Any) -> str:
    try:
        return f"{float(value) * 100:.2f}%"
    except (TypeError, ValueError):
        return "n/a"


def ms(value: Any) -> str:
    try:
        return f"{float(value):.2f} ms"
    except (TypeError, ValueError):
        return "n/a"


def load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def avg_by_model(values: Dict[str, Any]) -> Dict[str, float]:
    grouped: Dict[str, list[float]] = {}
    for pair_key, value in values.items():
        model = pair_key.split(" :: ", 1)[0]
        try:
            grouped.setdefault(model, []).append(float(value))
        except (TypeError, ValueError):
            continue
    return {
        model: sum(items) / len(items)
        for model, items in sorted(grouped.items())
        if items
    }


def top_items(values: Dict[str, Any], limit: int = 6) -> Iterable[Tuple[str, float]]:
    parsed = []
    for key, value in values.items():
        try:
            parsed.append((key, float(value)))
        except (TypeError, ValueError):
            continue
    return sorted(parsed, key=lambda item: item[1], reverse=True)[:limit]


def clear_screen() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def print_model_table(title: str, values: Dict[str, float], suffix: str = "") -> None:
    print(title)
    if not values:
        print("  n/a")
        return
    for model, value in values.items():
        print(f"  {model:<12} {pct(value) if not suffix else f'{value * 100:.2f}{suffix}'}")


def render(results: Dict[str, Any], source_path: Path) -> None:
    metrics = results.get("metrics", {})
    repos = results.get("repos", {})
    models = results.get("models", {})
    gt = results.get("ground_truth", {})
    source_packages = results.get("source_packages", {}).get("repositories", {})
    yaml_evidence = results.get("yaml_evidence", {})
    proposal = results.get("proposal_status", {})
    tool_audit = results.get("tool_audit", {})

    precision = metrics.get("m1_node_level_precision", {})
    sbrg = metrics.get("m2_sbrg", {})
    ecd = metrics.get("m3_ecd", {})
    ierv = metrics.get("m4_ierv", {})
    tcr = metrics.get("m5_tcr", {})
    timing = results.get("timing", {})

    total_gt_nodes = sum(len(nodes or []) for nodes in gt.values())
    total_gt_topics = 0
    for nodes in gt.values():
        for node in nodes or []:
            total_gt_topics += len(node.get("publishes", []) or [])
            total_gt_topics += len(node.get("subscribes", []) or [])

    print("=" * 78)
    print("ROS 2 LLM Architecture Recovery - Terminal Metrics Dashboard")
    print("=" * 78)
    print(f"Source file:      {source_path}")
    print(f"Run ID:           {results.get('run_id', 'unknown')}")
    print(f"Created UTC:      {results.get('created_utc', 'unknown')}")
    print(f"Real data only:   {'yes' if results.get('real_data_only') else 'no'}")
    print(f"Repos / models:   {len(repos)} repositories / {len(models)} models")
    print(f"GT architecture:  {total_gt_nodes} nodes, {total_gt_topics} pub/sub edges")
    print()

    print("M1 - Node-Level Precision: generated nodes that are real")
    print_model_table("  Average by model:", avg_by_model(precision))
    print()

    print("M2 - Subsystem Boundary Recall Gap: node recall minus subsystem recall")
    print_model_table("  Average by model:", avg_by_model(sbrg), " percentage points")
    print()

    print("M3 - Error Category Distribution: what mistakes the LLMs made")
    counts = ecd.get("counts", {})
    dist = ecd.get("distribution", {})
    if counts:
        for category, count in sorted(counts.items(), key=lambda item: int(item[1]), reverse=True):
            print(f"  {category:<30} {int(count):>4} errors  {pct(dist.get(category, 0.0)):>8}")
    else:
        print("  n/a")
    print()

    print("M4 - Inter-LLM Error Rate Variation: where model choice changes risk")
    for category, value in top_items(ierv, limit=6):
        print(f"  {category:<30} {float(value) * 100:.2f} percentage points")
    if not ierv:
        print("  n/a")
    print()

    print("M5 - Tool Coverage Ratio: categories detected automatically")
    proposal_tcr = tcr.get("proposal_tcr", {})
    extended_tcr = tcr.get("extended_tcr", {})
    print(f"  Proposal tools only: {pct(proposal_tcr.get('tcr', tcr.get('tcr', 0.0)))}")
    print(f"  Extended/local:      {pct(extended_tcr.get('tcr', 0.0))}")
    adequate = tcr.get("adequate_detectors_by_category", {})
    for category in ERROR_CATEGORIES:
        detectors = adequate.get(category) or ["none"]
        print(f"  {category:<30} {', '.join(detectors)}")
    print()

    print("M6 - Time-to-Verify: local parse/check/validation time")
    print(f"  Current pipeline TtV: {ms(metrics.get('m6_ttv_ms'))}")
    print(f"  Total wall clock:      {ms(timing.get('total_wall_clock_ms'))}")
    representative = metrics.get("m6_representative_ttv", {})
    if representative:
        print(f"  Five-repo average:     {ms(representative.get('average_total_ttv_ms'))}")
    print()

    print("Tool / Evidence Status")
    for name, data in sorted(tool_audit.items()):
        if isinstance(data, dict):
            print(f"  {name:<12} {data.get('status', 'unknown'):<36} {ms(data.get('elapsed_ms'))}")
    gaps = proposal.get("remaining_gaps", [])
    if gaps:
        print("  Proposal gaps:")
        for gap in gaps:
            print(f"    - {gap}")
    print()

    print("Data Sources In This Run")
    print("  ground_truth:      static source-parser answer key: nodes, topics, lifecycle, subsystems")
    print("  llm_outputs:       architecture JSON returned by each configured model")
    print("  comparisons:       TP/FP/FN plus classified errors per model and repo")
    print("  tool_audit:        AS2FM/ROSClaw smoke or validator evidence")
    print("  source_packages:   full source manifests used for traceability")
    print("  yaml_evidence:     YAML/launch/config files with architecture signals")
    print()

    print("Repository Coverage")
    for repo, data in sorted(source_packages.items()):
        files = data.get("source_file_count", 0)
        prompts = data.get("prompt_file_count", 0)
        yaml_count = len(yaml_evidence.get(repo, []) or [])
        gt_count = len(gt.get(repo, []) or [])
        status = "source-unavailable" if files == 0 else "source-loaded"
        print(
            f"  {repo:<30} {status:<18} files={files:<4} "
            f"prompt_files={prompts:<3} yaml={yaml_count:<3} gt_nodes={gt_count:<3}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch or print ROS 2 architecture-recovery metrics.")
    parser.add_argument("metrics_file", nargs="?", default="metrics_results.json")
    parser.add_argument("--watch", action="store_true", help="refresh until Ctrl+C")
    parser.add_argument("--interval", type=float, default=2.0, help="watch refresh interval in seconds")
    args = parser.parse_args()

    path = Path(args.metrics_file)
    if not path.exists():
        raise SystemExit(f"Metrics file not found: {path}")

    while True:
        try:
            results = load_json(path)
            clear_screen()
            render(results, path)
            if not args.watch:
                return 0
            print()
            print(f"Refreshing every {args.interval:.1f}s. Press Ctrl+C to stop.")
            time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nStopped.")
            return 0
        except json.JSONDecodeError:
            clear_screen()
            print(f"{path} is currently being written; waiting...")
            time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())

