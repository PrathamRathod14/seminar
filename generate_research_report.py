#!/usr/bin/env python3
"""
Generate a readable research-question report from metrics_results.json.

This does not call any LLM APIs. It only summarizes the logged experiment data.
"""

import json
import re
import sys
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Tuple


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


def slugify(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return slug.strip("_") or "repo"


def puml_text(value: Any) -> str:
    text = str(value)
    return text.replace("\\", "/").replace('"', "'").replace("\r", " ").replace("\n", "\\n")


def repo_error_counts(results: Dict[str, Any], repo: str) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for pair_key, comparison in results.get("comparisons", {}).items():
        if pair_key.split(" :: ", 1)[-1] != repo:
            continue
        for err in comparison.get("errors", []):
            category = str(err.get("category", "unknown"))
            counts[category] = counts.get(category, 0) + 1
    return counts


def repo_model_rows(results: Dict[str, Any], repo: str) -> List[str]:
    metrics = results.get("metrics", {})
    precision = metrics.get("m1_node_level_precision", {})
    sbrg = metrics.get("m2_sbrg", {})
    comparisons = results.get("comparisons", {})
    rows: List[str] = []
    for model in results.get("models", {}):
        pair_key = f"{model} :: {repo}"
        comp = comparisons.get(pair_key, {})
        rows.append(
            f"{model}: precision {pct(float(precision.get(pair_key, 0.0)))}, "
            f"SBRG {pp(float(sbrg.get(pair_key, 0.0)))}, "
            f"TP/FP/FN {comp.get('tp', 0)}/{comp.get('fp', 0)}/{comp.get('fn', 0)}"
        )
    return rows


def build_repo_puml(results: Dict[str, Any], repo: str) -> str:
    gt_summary = results.get("ground_truth_audit", {}).get("repo_summaries", {}).get(repo, {})
    source_count = len(results.get("source_files_sent", {}).get(repo, []))
    full_source_count = results.get("source_packages", {}).get("repositories", {}).get(repo, {}).get("source_file_count", 0)
    yaml_count = len(results.get("yaml_evidence", {}).get(repo, []))
    error_counts = repo_error_counts(results, repo)
    model_rows = repo_model_rows(results, repo)
    tcr = results.get("metrics", {}).get("m5_tcr", {})
    tool_audit = results.get("tool_audit", {})
    annotation = results.get("annotation_audit", {})
    top_errors = sorted(error_counts.items(), key=lambda item: item[1], reverse=True)
    if top_errors:
        error_text = "\\n".join(f"{puml_text(cat)}: {count}" for cat, count in top_errors)
    else:
        error_text = "No classified errors"
    model_text = "\\n".join(puml_text(row) for row in model_rows) or "No model outputs"
    tools_text = "\\n".join(
        f"{puml_text(name)}: {puml_text(data.get('status', 'unknown'))}"
        for name, data in tool_audit.items()
        if isinstance(data, dict)
    ) or "No tool audit"
    return "\n".join([
        "@startuml",
        "!pragma layout smetana",
        "skinparam backgroundColor #FFFFFF",
        "skinparam defaultFontName Arial",
        "skinparam rectangle {",
        "  RoundCorner 8",
        "  BorderColor #4B5563",
        "}",
        f"title ROS 2 LLM Architecture Recovery\\n{puml_text(repo)}",
        f"rectangle \"INPUT\\nRepo: {puml_text(repo)}\\nFull source files: {full_source_count}\\nPrompt files: {source_count}\\nYAML evidence files: {yaml_count}\\nGT nodes/topics: {gt_summary.get('nodes', 0)}/{gt_summary.get('topics', 0)}\" as input #E0F2FE",
        f"rectangle \"SQ1: LLM architecture recovery\\n{model_text}\" as sq1 #ECFDF5",
        f"rectangle \"SQ2: Error taxonomy\\nTotal repo errors: {sum(error_counts.values())}\\n{error_text}\" as sq2 #FFF7ED",
        f"rectangle \"SQ3: Tool detection\\nTCR: {pct(float(tcr.get('tcr', 0.0)))}\\n{tools_text}\" as sq3 #F5F3FF",
        f"rectangle \"SQ4 / Outputs\\nAnnotation: {puml_text(annotation.get('status', 'not recorded'))}\\nRun: {puml_text(results.get('run_id', 'unknown'))}\\nTtV: {float(results.get('metrics', {}).get('m6_ttv_ms', 0.0)):.2f} ms\" as sq4 #F8FAFC",
        "input --> sq1",
        "sq1 --> sq2",
        "sq2 --> sq3",
        "sq3 --> sq4",
        "footer Generated from metrics_results.json",
        "@enduml",
        "",
    ])


def _render_repo_diagram_matplotlib(results: Dict[str, Any], repo: str, jpg_path: Path) -> bool:
    """Render a per-repo metrics diagram locally using matplotlib — no network required."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        return False

    metrics = results.get("metrics", {})
    precision = metrics.get("m1_node_level_precision", {})
    sbrg = metrics.get("m2_sbrg", {})
    comparisons = results.get("comparisons", {})
    gt_summary = results.get("ground_truth_audit", {}).get("repo_summaries", {}).get(repo, {})
    tcr_data = metrics.get("m5_tcr", {})
    tool_audit = results.get("tool_audit", {})
    yaml_count = len(results.get("yaml_evidence", {}).get(repo, []))
    source_count = len(results.get("source_files_sent", {}).get(repo, []))
    error_counts = repo_error_counts(results, repo)
    models = list(results.get("models", {}).keys())

    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    fig.suptitle(f"ROS 2 LLM Architecture Recovery\n{repo}", fontsize=13, fontweight="bold", y=1.01)
    fig.patch.set_facecolor("#F8FAFC")

    # Panel 1 — M1 Precision + M2 SBRG per model
    ax1 = axes[0]
    ax1.set_facecolor("#ECFDF5")
    if models:
        prec_vals = [float(precision.get(f"{m} :: {repo}", 0.0)) * 100 for m in models]
        sbrg_vals = [float(sbrg.get(f"{m} :: {repo}", 0.0)) * 100 for m in models]
        x = range(len(models))
        bars1 = ax1.bar([i - 0.2 for i in x], prec_vals, width=0.35, label="Precision %", color="#3B82F6", alpha=0.85)
        bars2 = ax1.bar([i + 0.2 for i in x], sbrg_vals, width=0.35, label="SBRG pp", color="#F59E0B", alpha=0.85)
        ax1.set_xticks(list(x))
        ax1.set_xticklabels(models, rotation=15, ha="right", fontsize=8)
        ax1.set_ylim(0, 110)
        ax1.set_ylabel("Value (%)")
        ax1.legend(fontsize=8)
        for bar in list(bars1) + list(bars2):
            h = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width() / 2, h + 1, f"{h:.1f}", ha="center", va="bottom", fontsize=7)
    else:
        ax1.text(0.5, 0.5, "No model outputs", ha="center", va="center", transform=ax1.transAxes)
    ax1.set_title("SQ1: Node Precision & SBRG", fontsize=10, fontweight="bold")

    # Panel 2 — M3 Error Category Distribution
    ax2 = axes[1]
    ax2.set_facecolor("#FFF7ED")
    if error_counts:
        cats = list(error_counts.keys())
        counts = [error_counts[c] for c in cats]
        colors = ["#EF4444", "#F97316", "#EAB308", "#22C55E", "#3B82F6", "#8B5CF6"][:len(cats)]
        wedges, texts, autotexts = ax2.pie(
            counts, labels=None, autopct="%1.0f%%", colors=colors,
            startangle=140, pctdistance=0.75,
        )
        for t in autotexts:
            t.set_fontsize(8)
        ax2.legend(wedges, [f"{c} ({v})" for c, v in zip(cats, counts)],
                   loc="lower center", bbox_to_anchor=(0.5, -0.28), fontsize=7, ncol=2)
    else:
        ax2.text(0.5, 0.5, "No classified errors", ha="center", va="center", transform=ax2.transAxes)
    ax2.set_title("SQ2: Error Category Distribution", fontsize=10, fontweight="bold")

    # Panel 3 — SQ3 tool status + repo info
    ax3 = axes[2]
    ax3.set_facecolor("#F5F3FF")
    ax3.axis("off")
    lines = [
        f"SQ3 — Tool Detection",
        f"  TCR: {float(tcr_data.get('tcr', 0.0)) * 100:.1f}%  "
        f"({tcr_data.get('covered_count', 0)}/{tcr_data.get('total_categories', 6)} categories)",
    ]
    for name, data in tool_audit.items():
        if isinstance(data, dict):
            lines.append(f"  {name}: {data.get('status', 'unknown')}")
    lines += [
        "",
        "Dataset",
        f"  GT nodes: {gt_summary.get('nodes', 0)}",
        f"  GT topics: {gt_summary.get('topics', 0)}",
        f"  Prompt files: {source_count}",
        f"  YAML evidence: {yaml_count}",
        "",
        f"Run: {results.get('run_id', 'unknown')}",
        f"TtV: {float(metrics.get('m6_ttv_ms', 0.0)):.1f} ms",
    ]
    ax3.text(0.05, 0.95, "\n".join(lines), transform=ax3.transAxes,
             fontsize=9, verticalalignment="top", fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#EDE9FE", alpha=0.8))
    ax3.set_title("SQ3 + Dataset Summary", fontsize=10, fontweight="bold")

    plt.tight_layout()
    jpg_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(jpg_path), format="jpeg", dpi=120, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    # Validate the saved file is a real JPEG (starts with FF D8)
    with open(jpg_path, "rb") as f:
        magic = f.read(2)
    return magic == b"\xff\xd8"


def write_result_diagrams(results: Dict[str, Any], output_root: Path | None = None) -> Dict[str, Any]:
    run_id = str(results.get("run_id", "unknown_run"))
    root = output_root or Path("results") / run_id
    root.mkdir(parents=True, exist_ok=True)
    manifest: Dict[str, Any] = {"run_id": run_id, "root": str(root), "repositories": {}}

    for repo in results.get("repos", {}):
        repo_dir = root / slugify(repo)
        repo_dir.mkdir(parents=True, exist_ok=True)
        puml_path = repo_dir / "architecture_metrics.puml"
        jpg_path = repo_dir / "architecture_metrics.jpg"
        puml_path.write_text(build_repo_puml(results, repo), encoding="utf-8")
        jpg_rendered = _render_repo_diagram_matplotlib(results, repo, jpg_path)
        manifest["repositories"][repo] = {
            "folder": str(repo_dir),
            "puml": str(puml_path),
            "jpeg": str(jpg_path),
            "jpeg_rendered": jpg_rendered,
        }

    manifest_path = root / "diagram_manifest.json"
    manifest["manifest"] = str(manifest_path)
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def build_report(results: Dict[str, Any]) -> str:
    metrics = results.get("metrics", {})
    repos = results.get("repos", {})
    models = results.get("models", {})
    api_errors = results.get("api_errors", {})
    proposal = results.get("proposal_status", {})
    tool_audit = results.get("tool_audit", {})
    detection_audit = results.get("tool_detection_audit", {})
    adoption_gap = results.get("adoption_gap", {})
    annotation_audit = results.get("annotation_audit", {})
    verification_artifacts = results.get("verification_artifacts", {})
    yaml_evidence = results.get("yaml_evidence", {})
    source_packages = results.get("source_packages", {})
    gt_audit = results.get("ground_truth_audit") or metrics.get("ground_truth_audit", {})
    real_data_only = bool(results.get("real_data_only", proposal.get("real_data_only", False)))

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
        f"Real-data-only mode: **{'yes' if real_data_only else 'no'}**",
        f"Annotation artifact: **{annotation_audit.get('status', 'not recorded')}**",
        f"Proposal-complete run: **{'yes' if proposal.get('is_complete_proposal_run') else 'no'}**",
        f"YAML evidence files with architecture signals: **{sum(len(items) for items in yaml_evidence.values())}**",
        f"Full-repo source package manifests: **{len(source_packages.get('repositories', {}))}**",
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
        "Interpretation: high precision means the model generated fewer hallucinated nodes. A high SBRG means the model may find nodes but struggles to group them into correct subsystems.",
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
        "Interpretation: categories with precision and recall above 70% count as automatically covered. This run records per-tool detection logs and counts a category as covered when at least one detector is adequate.",
    ])

    adequate = tcr.get("adequate_detectors_by_category", {})
    if adequate:
        lines.extend(["", "Adequate detector by category:"])
        for category, tools in adequate.items():
            lines.append(f"- `{category}`: {', '.join(tools) if tools else 'none'}")

    tool_details = detection_audit.get("tools", {})
    if tool_details:
        lines.extend(["", "Per-tool detector status:"])
        for name, data in tool_details.items():
            if isinstance(data, dict):
                lines.append(f"- `{name}`: {data.get('status', 'unknown')} - {data.get('mode', '')}")
    if verification_artifacts:
        lines.extend([
            "",
            "Verification artifacts:",
            f"- AS2FM/JANI files: {len(verification_artifacts.get('as2fm_jani_files', []))}",
            f"- Native AS2FM repo models: {len(verification_artifacts.get('native_as2fm_models', {}).get('models', {}))}",
            f"- ROSClaw per-pair error files: {len(verification_artifacts.get('rosclaw_error_files', []))}",
            f"- Manifest: {verification_artifacts.get('manifest', 'not recorded')}",
        ])
        first_record = None
        for log in detection_audit.get("logs", []):
            records = log.get("error_records", [])
            if records:
                first_record = records[0]
                break
        before_call = (first_record or {}).get("rosclaw_before_call", {})
        if before_call:
            request = before_call.get("request", {})
            lines.append(
                "- ROSClaw before-call sample: "
                f"{request.get('method', 'unknown')} / "
                f"{request.get('params', {}).get('name', 'unknown')} -> "
                f"{before_call.get('status', 'unknown')}, detected={before_call.get('detected', 'unknown')}"
            )

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
            f"- Filter policy: {gt_audit.get('filter_policy', 'unknown')}",
            f"- Scope: {gt_audit.get('repo_count', 0)} repositories, {gt_audit.get('total_nodes', 0)} nodes, {gt_audit.get('total_topics', 0)} topics",
            f"- Annotation file: {annotation_audit.get('path', 'not recorded')}",
            f"- YAML evidence files: {sum(len(items) for items in yaml_evidence.values())}",
            f"- Full source package manifest: {source_packages.get('manifest', 'not recorded')}",
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
        "- Baseline comparison: the observed residual LLM errors are counted and classified rather than treated as one undifferentiated manual-fix bucket.",
    ])

    baseline = adoption_gap.get("industry_baseline", {})
    if baseline:
        lines.append(f"- Industry baseline: {pct(float(baseline.get('manual_residual_error_rate', 0.0)))} residual manual correction rate.")

    tools = adoption_gap.get("tools", {})
    for name, data in tools.items():
        github = data.get("github", {})
        setup = data.get("setup_complexity", {})
        if github:
            lines.append(
                f"- `{name}` adoption proxy: stars={github.get('stars', 'n/a')}, forks={github.get('forks', 'n/a')}, open_issues={github.get('open_issues', 'n/a')}, setup_command_lines={setup.get('setup_command_lines', 'n/a')}."
            )

    rosa = adoption_gap.get("rosa_error_evidence", {})
    if rosa:
        lines.append(
            f"- ROSA external error evidence: {rosa.get('classified_issue_count', 0)} classified issues/PRs from {rosa.get('source', 'unknown')}."
        )

    lines.extend([
        f"- AS2FM operational check time: {float(tool_audit.get('AS2FM', {}).get('elapsed_ms', 0.0)) / 1000:.2f} seconds",
        f"- ROSClaw validator check time: {float(tool_audit.get('ROSClaw', {}).get('elapsed_ms', 0.0)) / 1000:.2f} seconds",
        "- Configuration evidence: AS2FM conversion, generated RoAML/JANI architecture-property checks, and ROSClaw-compatible per-error validation logs ran locally.",
    ])

    lines.extend([
        "",
        "## M6 - Time-to-Verify",
        "",
        f"Current pipeline TtV: **{ttv_ms:.2f} ms**",
        "",
        "This includes source parsing and validation timing from the script.",
    ])

    representative = metrics.get("m6_representative_ttv", {})
    if representative:
        lines.extend([
            "",
            "Representative five-repo TtV:",
            f"- Method: {representative.get('method', 'unknown')}",
            f"- Average total TtV: {float(representative.get('average_total_ttv_ms', 0.0)):.2f} ms",
        ])
        for record in representative.get("records", []):
            lines.append(
                f"- `{record.get('repo', 'unknown')}`: {float(record.get('total_ttv_ms', 0.0)):.2f} ms "
                f"(AS2FM {record.get('as2fm_native_status', 'unknown')} in {float(record.get('as2fm_native_ms', 0.0)):.2f} ms)"
            )
        ubuntu_humble = representative.get("ubuntu_22_04_ros2_humble", {})
        if ubuntu_humble:
            probe = ubuntu_humble.get("probe", {})
            probe_text = str(probe.get("stdout_tail", "")).replace("\n", "; ").strip("; ")
            lines.extend([
                "",
                "Ubuntu 22.04 + ROS 2 Humble TtV audit:",
                f"- Status: **{ubuntu_humble.get('status', 'unknown')}**",
                f"- Humble present: {ubuntu_humble.get('humble_present', 'unknown')}",
                f"- Probe: {probe_text or 'not recorded'}",
            ])
            for record in ubuntu_humble.get("records", []):
                lines.append(
                    f"- `{record.get('repo', 'unknown')}`: {record.get('status', 'unknown')} "
                    f"({float(record.get('elapsed_ms', 0.0)):.2f} ms)"
                )

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
    manifest = write_result_diagrams(results)
    print(f"Result diagrams saved to: {manifest['root']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
