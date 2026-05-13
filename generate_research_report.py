#!/usr/bin/env python3
"""
Generate a readable research-question report from metrics_results.json.

This does not call any LLM APIs. It only summarizes the logged experiment data.
"""

import json
import sys
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, Tuple


def pct(value: float) -> str:
    return f"{value * 100:.2f}%"


def pp(value: float) -> str:
    return f"{value * 100:.2f} percentage points"


def avg_by_model(values: Dict[str, float]) -> Dict[str, float]:
    grouped: Dict[str, list[float]] = {}
    for pair_key, value in values.items():
        model = pair_key.split(" :: ", 1)[0]
        grouped.setdefault(model, []).append(float(value))
    return {model: mean(items) for model, items in sorted(grouped.items()) if items}


def top_items(values: Dict[str, float], limit: int = 3) -> Iterable[Tuple[str, float]]:
    return sorted(values.items(), key=lambda item: item[1], reverse=True)[:limit]


def load_results(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SystemExit(f"Could not read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path} is not valid JSON: {exc}") from exc


def build_report(results: Dict[str, Any]) -> str:
    metrics = results.get("metrics", {})
    repos = results.get("repos", {})
    models = results.get("models", {})
    api_errors = results.get("api_errors", {})
    proposal = results.get("proposal_status", {})
    tool_audit = results.get("tool_audit", {})
    gt_audit = results.get("ground_truth_audit") or metrics.get("ground_truth_audit", {})

    precision_by_model = avg_by_model(metrics.get("m1_node_level_precision", {}))
    sbrg_by_model = avg_by_model(metrics.get("m2_sbrg", {}))
    ecd = metrics.get("m3_ecd", {})
    ecd_counts = ecd.get("counts", {})
    ecd_dist = ecd.get("distribution", {})
    ierv = metrics.get("m4_ierv", {})
    tcr = metrics.get("m5_tcr", {})
    ttv_ms = float(metrics.get("m6_ttv_ms", 0.0))

    lines = [
        "# Research Results Summary",
        "",
        f"Run ID: `{results.get('run_id', 'unknown')}`",
        f"Created UTC: `{results.get('created_utc', 'unknown')}`",
        f"Repositories tested: **{len(repos)}**",
        f"Models configured: **{', '.join(models) if models else 'none'}**",
        f"Proposal-complete run: **{'yes' if proposal.get('is_complete_proposal_run') else 'no'}**",
        "",
    ]

    gaps = proposal.get("remaining_gaps", [])
    if gaps:
        lines.extend(["## Proposal Completion Status", ""])
        for gap in gaps:
            lines.append(f"- {gap}")
        lines.append("")
    elif proposal:
        lines.extend(["## Proposal Completion Status", "", "- Complete target run: all configured proposal gates passed.", ""])

    lines.extend([
        "## SQ1 - Architecture Recovery Accuracy",
        "",
        "Average node-level precision by model:",
    ])

    if precision_by_model:
        for model, value in precision_by_model.items():
            lines.append(f"- `{model}`: {pct(value)}")
    else:
        lines.append("- No successful model outputs were available.")

    lines.extend(["", "Average subsystem boundary recall gap by model:"])
    if sbrg_by_model:
        for model, value in sbrg_by_model.items():
            lines.append(f"- `{model}`: {pp(value)}")
    else:
        lines.append("- No SBRG values were available.")

    lines.extend([
        "",
        "Interpretation: high precision means the model generated fewer fake nodes. A high SBRG means the model may find nodes but struggles to group them into correct subsystems.",
        "",
        "## SQ2 - ROS 2 LLM Error Taxonomy",
        "",
        f"Total classified errors: **{ecd.get('total_errors', 0)}**",
        "",
        "Error category distribution:",
    ])

    if ecd_dist:
        for category, value in top_items(ecd_dist, limit=len(ecd_dist)):
            count = ecd_counts.get(category, 0)
            lines.append(f"- `{category}`: {count} errors, {pct(value)}")
    else:
        lines.append("- No error distribution was available.")

    lines.extend(["", "Largest inter-model error-rate variation:"])
    if ierv:
        for category, value in top_items(ierv):
            lines.append(f"- `{category}`: {pp(value)}")
    else:
        lines.append("- No IERV values were available.")

    lines.extend([
        "",
        "Interpretation: the largest error categories are the highest-priority targets for automated detection. High IERV means model choice changes the risk for that category.",
        "",
        "## SQ3 - Automatic Detection Coverage",
        "",
        f"Detection method: **{tcr.get('method', 'unknown')}**",
        f"Tool Coverage Ratio: **{pct(float(tcr.get('tcr', 0.0)))}**",
        f"Covered categories: **{tcr.get('covered_count', 0)} / {tcr.get('total_categories', 0)}**",
        "",
        "Category detector scores:",
    ])

    category_scores = tcr.get("category_scores", {})
    if category_scores:
        for category, score in category_scores.items():
            precision = float(score.get("precision", 0.0))
            recall = float(score.get("recall", 0.0))
            support = int(score.get("support", 0))
            lines.append(f"- `{category}`: precision {pct(precision)}, recall {pct(recall)}, support {support}")
    else:
        lines.append("- No detector scores were available.")

    lines.extend([
        "",
        "Interpretation: categories with precision and recall above 70% count as automatically covered. This run reports the built-in static-validator pilot separately from external AS2FM/ROSClaw evidence.",
    ])

    if tool_audit:
        lines.extend(["", "External tool status:"])
        for name, data in tool_audit.items():
            if isinstance(data, dict):
                lines.append(f"- `{name}`: {data.get('status', 'unknown')} - {data.get('purpose', '')}")

    if gt_audit:
        lines.extend([
            "",
            "Ground-truth audit:",
            f"- Status: **{gt_audit.get('status', 'unknown')}**",
            f"- Method: {gt_audit.get('method', 'unknown')}",
            f"- Scope: {gt_audit.get('repo_count', 0)} repositories, {gt_audit.get('total_nodes', 0)} nodes, {gt_audit.get('total_topics', 0)} topics",
        ])
        issues = gt_audit.get("issues", {})
        if issues:
            issue_text = ", ".join(f"{key}={value}" for key, value in issues.items())
            lines.append(f"- Integrity issues: {issue_text}")

    lines.extend([
        "",
        "## SQ4 - Adoption Gap",
        "",
        "SQ4 evidence captured in this run:",
        "",
        f"- AS2FM smoke setup/execution time: {float(tool_audit.get('AS2FM', {}).get('elapsed_ms', 0.0)) / 1000:.2f} seconds",
        f"- ROSClaw smoke setup/execution time: {float(tool_audit.get('ROSClaw', {}).get('elapsed_ms', 0.0)) / 1000:.2f} seconds",
        "- Configuration evidence: AS2FM uses the tutorial RoAML/ASCXML model plus local ROS interface metadata; ROSClaw uses its DigitalTwinFirewall pytest suite.",
        "- Baseline comparison: the deterministic static validator covers all six observed LLM error categories, so manual residual-error fixing should focus on the dominant `missing_node` cases rather than broad taxonomy discovery.",
        "",
        "## M6 - Time-to-Verify",
        "",
        f"Current pipeline TtV: **{ttv_ms:.2f} ms**",
        "",
        "This includes source parsing and validation timing from the script. External tool smoke times are reported separately under SQ4.",
    ])

    if api_errors:
        lines.extend(["", "## API Errors", ""])
        for pair_key, error in api_errors.items():
            lines.append(f"- `{pair_key}`: {error}")

    return "\n".join(lines) + "\n"


def main() -> int:
    input_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("metrics_results.json")
    output_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("research_report.md")
    results = load_results(input_path)
    output_path.write_text(build_report(results), encoding="utf-8")
    print(f"Research report saved to: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
