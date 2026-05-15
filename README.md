# ROS 2 LLM Architecture Recovery

This project evaluates whether Large Language Models can recover ROS 2 software architecture from real source code, and what kinds of mistakes they make.

The pipeline clones real ROS 2 repositories, extracts source-derived ground truth, asks configured LLMs to recover the architecture, compares the model output against the ground truth, classifies errors, and writes metrics/reports for analysis.

## What This Project Measures

- ROS 2 nodes recovered by each model
- Publishers, subscribers, topics, message/interface types, and lifecycle hints
- Subsystem grouping quality
- Hallucinated nodes and missing nodes
- Interface mismatches, wrong topic names, lifecycle violations, and subsystem boundary confusion
- Tool coverage using ROSClaw, AS2FM status, generated JANI-style checks, and a local static validator
- Time-to-verify for the architecture recovery and validation pipeline

## Repository Layout

```text
ros2_llm_arch_recovery.py      Main experiment pipeline
generate_research_report.py    Creates research_report.md from metrics_results.json
terminal_metrics_dashboard.py  Shows metrics in the terminal
requirements.txt               Python dependencies
.env.example                   Safe example configuration
metrics_results.json           Latest generated metrics artifact
research_report.md             Latest generated readable report

annotations/                   Ground-truth audit/annotation files
results/                       Generated architecture diagram artifacts
source_packages/               Source package manifests used for audit
verification_artifacts/        JANI, ROSClaw, AS2FM, and TtV evidence
repos/                         Cloned ROS 2 repositories, generated locally
audit_llm_responses/           Raw model responses, generated locally
.llm_cache/                    Cached model responses, generated locally
```

`repos/`, `audit_llm_responses/`, and `.llm_cache/` are generated locally and are intentionally not committed.

## Requirements

- Python 3.10 or newer
- Git
- Internet access for cloning ROS 2 repositories
- A Groq API key for the default model set

The project is easiest to run from Windows PowerShell because the external-tool audit currently expects the local Windows virtual environment path `.venv\Scripts\python.exe`.

## Quick Start

Clone the repository:

```powershell
git clone https://github.com/PrathamRathod14/seminar.git
cd seminar
```

Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Install dependencies:

```powershell
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Create your local config:

```powershell
Copy-Item .env.example .env
```

Open `.env` and add your real Groq key:

```env
GROQ_API_KEY=your_real_groq_key_here
```

Do not commit `.env`; it is ignored by Git.

## Run The Experiment

Run the full pipeline:

```powershell
python ros2_llm_arch_recovery.py
```

Generate the readable report:

```powershell
python generate_research_report.py
```

View the metrics in the terminal:

```powershell
python terminal_metrics_dashboard.py metrics_results.json
```

Watch the metrics with live refresh:

```powershell
python terminal_metrics_dashboard.py metrics_results.json --watch
```

## First Test Run

For a faster first run, edit `.env`:

```env
MAX_REPOS=2
ACTIVE_MODELS=llama-4,groq-small
PROMPT_CHAR_BUDGET=30000
LLM_CACHE=1
REAL_DATA_ONLY=1
```

Then run:

```powershell
python ros2_llm_arch_recovery.py
python generate_research_report.py
python terminal_metrics_dashboard.py metrics_results.json
```

After that works, increase `MAX_REPOS` or add more models.

## Configuration

Main `.env` settings:

```env
GROQ_API_KEY=your_real_groq_key_here
ACTIVE_MODELS=llama-4,groq-large,groq-small,qwen-groq
MAX_REPOS=15
PROMPT_CHAR_BUDGET=30000
LLM_CACHE=1
LLM_CACHE_DIR=.llm_cache
LLM_CHUNK_PAUSE_SECONDS=2
REAL_DATA_ONLY=1
```

Optional settings:

```env
GITHUB_TOKEN=your_optional_github_token
OPENROUTER_SITE_URL=http://localhost
OPENROUTER_APP_NAME=ros2-llm-arch-recovery
```

`GITHUB_TOKEN` is optional. It only helps avoid GitHub API rate limits when collecting adoption-gap evidence.

## Default Models

The default configured models are Groq-backed labels:

- `llama-4`
- `groq-large`
- `groq-small`
- `qwen-groq`

All default labels require `GROQ_API_KEY`.

The code also contains support for additional future labels, but the default reproducible setup uses Groq only.

## Real-Data-Only Mode

By default:

```env
REAL_DATA_ONLY=1
```

This excludes tutorial, demo, example, test, mock, fake, dummy, stub, fixture, simulation, simulator, and loopback-style inputs from the evaluation. The goal is to evaluate real implementation code rather than synthetic or teaching examples.

## Pipeline Steps

1. Select ROS 2 repositories from the configured list.
2. Clone missing repositories into `repos/`.
3. Extract source-derived ground truth with the static parser.
4. Send bounded source evidence to each configured LLM.
5. Parse the returned architecture JSON.
6. Compare LLM output with ground truth.
7. Classify errors into the ROS 2 taxonomy.
8. Generate diagrams, annotations, manifests, validation artifacts, and metrics.
9. Write `metrics_results.json`.
10. Generate `research_report.md`.

## Output Files

Important generated files:

```text
metrics_results.json
research_report.md
annotations/ground_truth_annotations_<run_id>.json
results/<run_id>/
source_packages/<run_id>/
verification_artifacts/<run_id>/
audit_llm_responses/<run_id>/
```

The most important file is:

```text
metrics_results.json
```

It contains the run ID, selected repositories, selected models, ground truth, LLM outputs, comparisons, tool audit, validation artifacts, and all metrics.

## Metrics

The experiment reports six main metrics:

| ID | Metric | Meaning |
| --- | --- | --- |
| M1 | Node-Level Precision | How many generated nodes are real |
| M2 | Subsystem Boundary Recall Gap | Drop from node recall to subsystem recall |
| M3 | Error Category Distribution | Which error types appear most often |
| M4 | Inter-LLM Error Rate Variation | How differently models fail |
| M5 | Tool Coverage Ratio | Which error categories are automatically detected |
| M6 | Time-to-Verify | Runtime cost of verification |

Error categories:

- `interface_mismatch`
- `hallucinated_node`
- `subsystem_boundary_confusion`
- `missing_node`
- `wrong_topic_name`
- `lifecycle_violation`

## Tool Validation Notes

The pipeline separates proposal-level tool coverage from extended local validation:

- Proposal TCR counts AS2FM and ROSClaw.
- Extended/local TCR also includes generated JANI-style checks and the deterministic local static validator.

AS2FM may be marked not applicable for category-level detection on the current real ROS 2 repositories because AS2FM needs repo-specific RoAML/ASCXML behavior models. The repository source code provides architecture evidence, not always executable AS2FM behavior models.

ROSClaw firewall checks can run when the local `tools/rosclaw` checkout is available.

## Optional External Tool Checkouts

Some historical artifacts were generated with local checkouts under:

```text
tools/AS2FM/
tools/rosa/
tools/rosclaw/
```

Those are external repositories and are not required to inspect the committed metrics or reports. For a fresh clone, the main LLM architecture-recovery flow still works with the Python dependencies and Groq key. External-tool smoke checks may report `failed` or `not_applicable` until those tools are installed locally.

## WSL/Linux Notes

For WSL/Linux, use Linux activation commands:

```bash
cd /mnt/c/Users/prath/Desktop/seminar
python3 -m venv .venv_wsl
source .venv_wsl/bin/activate
python3 -m pip install -r requirements.txt
python3 terminal_metrics_dashboard.py metrics_results.json
```

The dashboard and report reader work well in WSL. The full external-tool audit is currently easiest from Windows PowerShell because parts of the script use `.venv\Scripts\python.exe`.

## Common Commands

Run the main pipeline:

```powershell
python ros2_llm_arch_recovery.py
```

Regenerate the report:

```powershell
python generate_research_report.py
```

Print metrics dashboard:

```powershell
python terminal_metrics_dashboard.py metrics_results.json
```

Watch dashboard:

```powershell
python terminal_metrics_dashboard.py metrics_results.json --watch --interval 5
```

Check Git status:

```powershell
git status
```

## Troubleshooting

If `python` is not found in WSL, use `python3`.

If PowerShell blocks virtual environment activation, run:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

If API calls fail, check that `.env` contains:

```env
GROQ_API_KEY=your_real_groq_key_here
```

If the run is slow or expensive, reduce:

```env
MAX_REPOS=2
ACTIVE_MODELS=llama-4
```

If GitHub API evidence fails, add an optional `GITHUB_TOKEN` or ignore the adoption-gap GitHub counters.

## Research Interpretation

Use `research_report.md` for a professor-friendly summary organized by research question:

- SQ1: architecture recovery accuracy
- SQ2: ROS 2 LLM error taxonomy
- SQ3: automatic detection coverage
- SQ4: adoption-gap evidence
- M6: time-to-verify

The key conclusion from the latest real-data run is that LLMs can attempt ROS 2 architecture recovery, but their outputs need automatic validation because missing nodes, hallucinated nodes, and model-to-model variation are significant.
