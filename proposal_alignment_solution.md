# Proposal Alignment Solution: Tool Evaluation and SQ4

## Tool Evaluation / SQ3 Alignment

The current implementation already produces a ROS 2-specific error taxonomy and computes Tool Coverage Ratio (TCR), but the proposal requires the detection step to be attributed specifically to existing tools, especially AS2FM and the ROSClaw validator. To align the implementation with the proposal, the tool evaluation stage should be separated into three explicit detector columns:

1. `ROSClaw_validator_detected`
2. `AS2FM_detected`
3. `local_static_validator_detected`

The local static validator may remain in the pipeline, but it should be treated as a baseline detector rather than as a substitute for AS2FM or ROSClaw. TCR should therefore be reported twice:

- **Proposal TCR:** a category is covered only if AS2FM or ROSClaw detects it with precision >= 70% and recall >= 70%.
- **Extended TCR:** a category is covered if AS2FM, ROSClaw, or the local static validator detects it with precision >= 70% and recall >= 70%.

For each error category from SQ2, the pipeline should run the same labelled error instances through all available detectors and record true positives, false positives, false negatives, precision, and recall per detector. If AS2FM cannot be run on a category because the required behavioural model is unavailable, that category should be marked as `not_applicable_for_AS2FM`, not as detected. If ROSClaw cannot evaluate a category outside manipulation or interface-call contexts, it should likewise be marked as `not_applicable_for_ROSClaw`.

This solves the current mismatch because the reported SQ3 result will no longer imply that AS2FM and ROSClaw fully detected every category unless they actually did. The local validator can still demonstrate that automatic detection is possible, while the proposal-specific result remains tied to existing formal verification and ROSClaw-style validation tools.

## SQ4 Adoption Gap Alignment

The current run records limited adoption evidence, but SQ4 asks why formally verified open-source tools are not widely adopted in industry despite working well. To align SQ4 with the proposal, adoption should be measured as a separate friction study rather than only as GitHub popularity metadata.

For AS2FM and ROSA, the pipeline should record:

- **Setup Complexity Score:** number of manual installation commands, required dependencies, environment variables, ROS distribution assumptions, and failed setup attempts.
- **Pipeline Complexity Score:** number of translation steps from ROS 2 source/launch files to the tool's accepted input format, including any required manual model-writing.
- **Time-to-Verify (TtV):** end-to-end wall-clock time from source code and launch files to a verification result, including parsing, model translation, configuration, and checking.
- **Adoption Proxy Score:** GitHub stars, forks, open issues, release activity, documentation completeness, and evidence of real deployments or user reports.
- **Manual-Fix Baseline Comparison:** compare the measured tool friction against the Siemens-style baseline where approximately 20% of LLM-generated automation output is manually corrected instead of formally checked.

The SQ4 answer should then be framed as follows: AS2FM may be fast once a valid formal model exists, but adoption is limited by the work required before checking can begin, especially model translation, environment setup, and property specification. ROSA shows stronger community visibility, but it is an LLM-ROS interface rather than a complete formal detection pipeline for the full taxonomy. Therefore, the adoption gap is not mainly caused by slow verification; it is caused by integration friction before verification and by the industrial acceptability of manually fixing the residual 20% of LLM errors.

This solves the current mismatch because SQ4 no longer claims broad industrial adoption evidence from limited local execution data. Instead, it explicitly measures the friction factors that explain why working verification tools are not automatically adopted in LLM-generated ROS 2 workflows.

