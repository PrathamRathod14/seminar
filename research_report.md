# Research Results Summary

Run ID: `20260513T175513Z`
Created UTC: `2026-05-13T17:58:07.415886+00:00`
Repositories tested: **15**
Models configured: **llama-4, groq-large, groq-small, qwen-groq**
Real-data-only mode: **yes**
Annotation artifact: **source_parser_seed_for_manual_review**
Proposal-complete run: **no**
YAML evidence files with architecture signals: **13**
Full-repo source package manifests: **15**

## Proposal Completion Status

- AS2FM category detection not run on real repo behavior models

## SQ1 - Architecture Recovery Accuracy

Average node-level precision by model:
- `groq-large`: 10.00%
- `groq-small`: 5.56%
- `llama-4`: 6.67%
- `qwen-groq`: 0.00%

Average subsystem boundary recall gap by model:
- `groq-large`: 4.44 percentage points
- `groq-small`: 4.44 percentage points
- `llama-4`: 2.22 percentage points
- `qwen-groq`: 0.00 percentage points

Interpretation: high precision means the model generated fewer hallucinated nodes. A high SBRG means the model may find nodes but struggles to group them into correct subsystems.

## SQ2 - ROS 2 LLM Error Taxonomy

Total classified errors: **106**

Error category distribution:
- `hallucinated_node`: 47 errors, 44.34%
- `missing_node`: 43 errors, 40.57%
- `interface_mismatch`: 6 errors, 5.66%
- `subsystem_boundary_confusion`: 5 errors, 4.72%
- `lifecycle_violation`: 3 errors, 2.83%
- `wrong_topic_name`: 2 errors, 1.89%

Largest inter-model error-rate variation:
- `missing_node`: 72.31 percentage points
- `hallucinated_node`: 58.31 percentage points
- `interface_mismatch`: 10.00 percentage points

Interpretation: the largest error categories are the highest-priority targets for automated detection. High IERV means model choice changes the risk for that category.

## SQ3 - Automatic Detection Coverage

Detection method: **proposal_tools_separated_from_local_adapters**
Proposal Tool Coverage Ratio: **16.67%**
Proposal covered categories: **1 / 6**
Extended/local Tool Coverage Ratio: **100.00%**

Proposal category detector scores:
- `interface_mismatch`: precision 100.00%, recall 100.00%, support 3
- `hallucinated_node`: precision 0.00%, recall 0.00%, support 21
- `subsystem_boundary_confusion`: precision 0.00%, recall 0.00%, support 5
- `missing_node`: precision 0.00%, recall 0.00%, support 28
- `wrong_topic_name`: precision 0.00%, recall 0.00%, support 2
- `lifecycle_violation`: precision 0.00%, recall 0.00%, support 3

Interpretation: proposal TCR counts only AS2FM and ROSClaw. The extended/local TCR includes generated JANI-style property checks and the deterministic static validator as baseline evidence.

Adequate detector by category:
- `interface_mismatch`: ROSClaw
- `hallucinated_node`: none
- `subsystem_boundary_confusion`: none
- `missing_node`: none
- `wrong_topic_name`: none
- `lifecycle_violation`: none

Extended/local adequate detector by category:
- `interface_mismatch`: ROSClaw, AS2FM_JANI_adapter, LocalStaticValidator
- `hallucinated_node`: AS2FM_JANI_adapter, LocalStaticValidator
- `subsystem_boundary_confusion`: AS2FM_JANI_adapter, LocalStaticValidator
- `missing_node`: AS2FM_JANI_adapter, LocalStaticValidator
- `wrong_topic_name`: AS2FM_JANI_adapter, LocalStaticValidator
- `lifecycle_violation`: AS2FM_JANI_adapter, LocalStaticValidator

Per-tool detector status:
- `ROSClaw`: run - ROSClaw before-call DigitalTwinFirewall validation; adequate only where the validator actually intercepts the category
- `AS2FM`: not_applicable_for_current_repo_inputs - Real AS2FM category detection requires repo-specific RoAML/ASCXML behaviour models; current ROS 2 source repositories provide architecture graphs, not executable AS2FM behaviour models.
- `AS2FM_JANI_adapter`: run - Generated JANI-style taxonomy property evaluation over architecture graph; evidence for formalizable checks, not counted as real AS2FM proposal TCR.
- `LocalStaticValidator`: run - Deterministic ROS 2 architecture graph validator baseline; not counted as existing-tool proposal TCR.

Verification artifacts:
- AS2FM/JANI files: 60
- Native AS2FM repo models: 15
- ROSClaw per-pair error files: 60
- Manifest: C:\Users\prath\Desktop\seminar\verification_artifacts\20260513T175513Z\verification_manifest.json
- ROSClaw before-call sample: tools/call / ur5_validate_trajectory -> run, detected=False

External tool status:
- `AS2FM`: run - AS2FM operational RoAML/ASCXML to JANI conversion check using local ROS interface metadata
- `ROSClaw`: run - ROSClaw DigitalTwinFirewall validator test suite
- Note: external tool status records installation/smoke execution only; proposal TCR above records category-level detection.

Ground-truth audit:
- Status: **recorded**
- Method: deterministic source-parser integrity audit
- Filter policy: real_data_only
- Scope: 15 repositories, 12 nodes, 4 topics
- Annotation file: C:\Users\prath\Desktop\seminar\annotations\ground_truth_annotations_20260513T175513Z.json
- YAML evidence files: 13
- Full source package manifest: C:\Users\prath\Desktop\seminar\source_packages\20260513T175513Z\source_package_manifest.json
- Integrity issues: empty_node_names=0, invalid_topic_names=0

## SQ4 - Adoption Gap

SQ4 evidence captured in this run:

- Baseline comparison: the observed residual LLM errors are counted and classified rather than treated as one undifferentiated manual-fix bucket.
- Industry baseline: 20.00% residual manual correction rate.
- `AS2FM` adoption proxy: stars=16, forks=2, open_issues=20, setup_command_lines=0.
- `ROSA` adoption proxy: stars=1514, forks=160, open_issues=3, setup_command_lines=2.
- ROSA external error evidence: 8 classified issues/PRs from nasa-jpl/rosa.
- AS2FM operational check time: 1.25 seconds
- ROSClaw validator check time: 2.06 seconds
- Configuration evidence: AS2FM conversion, generated RoAML/JANI architecture-property checks, and ROSClaw-compatible per-error validation logs ran locally.

## M6 - Time-to-Verify

Current pipeline TtV: **26582.23 ms**

This includes source parsing and validation timing from the script.

Representative five-repo TtV:
- Method: source parsing + prompt evidence collection + JANI artifact generation + native AS2FM conversion + category validation
- Average total TtV: 1444.03 ms
- `ros2/rviz`: 2468.08 ms (AS2FM failed in 1429.34 ms)
- `ros2/rosbag2`: 1447.62 ms (AS2FM failed in 961.44 ms)
- `ros2/geometry2`: 1218.25 ms (AS2FM failed in 976.07 ms)
- `ros2/diagnostics`: 1019.54 ms (AS2FM failed in 1003.94 ms)
- `ros2/robot_state_publisher`: 1066.67 ms (AS2FM failed in 1043.71 ms)

Ubuntu 22.04 + ROS 2 Humble TtV audit:
- Status: **environment_unavailable**
- Humble present: False
- Probe: ubuntu_version=22.04; humble_setup=missing; ros_distro=missing; python_version=Python 3.10.12
