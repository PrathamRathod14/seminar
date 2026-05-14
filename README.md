# ROS 2 LLM Architecture Recovery

This project tests a simple research question:

**Can LLMs recover ROS 2 software architecture from real source code, and what mistakes do they make?**

This runnable setup uses real Groq-hosted models only. By default, `REAL_DATA_ONLY=1`, so tutorial, example, demo, test, mock, dummy, fake, stub, fixture, simulation, simulator, and loopback inputs are excluded from the evaluation data.

In proposal terms, this is the current implementation:

- **Now:** run SQ1, SQ2, and SQ3 static validation on real-data-only ROS 2 source inputs. AS2FM and ROSClaw smoke checks are skipped in real-data-only mode because the available local inputs are tutorial/test/simulation fixtures.

## Current Setup

The current test uses Groq-hosted free/available models:

- `llama-4`
- `groq-small`
- `groq-large`
- `qwen-groq`

These are configured in `.env`:

```env
GROQ_API_KEY=your_real_key_here
ACTIVE_MODELS=llama-4,groq-large,groq-small,qwen-groq
MAX_REPOS=15
PROMPT_CHAR_BUDGET=30000
LLM_CACHE=1
REAL_DATA_ONLY=1
```

For this Groq-only target set, provide `GROQ_API_KEY`. Do not put real API keys in `.env.example`; keep real keys only in `.env`.

## Install

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
Copy-Item .env.example .env
```

Then edit `.env` and paste your Groq key.

## Run

```powershell
python ros2_llm_arch_recovery.py
```

Then generate a readable research summary:

```powershell
python generate_research_report.py
```

This creates:

```text
research_report.md
```

## Easy Explanation

Think of the project like an exam.

- The ROS 2 repositories are the exam questions.
- The script's static parser creates the answer key.
- The LLMs give their answers.
- The script marks the answers and writes the marksheet to `metrics_results.json`.

## How The Test Works

1. **Input**

   The script clones real ROS 2 repositories into `repos/`.

   Examples:

   - `ros2/examples`
   - `ros2/demos`
   - `navigation2`
   - `ros2/rclcpp`
   - `ros2/rclpy`

2. **Ground Truth**

   The script reads the code and extracts the real architecture:

   - nodes
   - publishers
   - subscribers
   - topic names
   - message types
   - lifecycle nodes
   - launch-file executables

3. **LLM Architecture Recovery**

   The script sends selected source files to each active Groq model.

   Each model is asked to return JSON containing:

   - node names
   - subsystem names
   - published topics
   - subscribed topics
   - lifecycle status

4. **Comparison**

   The model output is compared with the ground truth.

   The script checks whether the model:

   - found real nodes
   - missed nodes
   - hallucinated fake nodes
   - used wrong topic names
   - used wrong message/interface types
   - confused subsystem boundaries
   - got lifecycle status wrong

5. **Output**

   Results are written to:

   - `metrics_results.json`: final metrics and comparisons
   - `audit_llm_responses/<run_id>/`: raw model outputs
   - `.llm_cache/`: cached LLM responses to avoid repeated API calls

## Research Questions Mapping

### SQ1: Architecture Accuracy

The script measures how accurately each active Groq model recovers ROS 2 nodes and subsystem groupings.

Main metrics:

- **Node-Level Precision**
- **Subsystem Boundary Recall Gap**

### SQ2: Error Taxonomy

The script classifies LLM mistakes into ROS 2-specific error categories:

- `interface_mismatch`
- `hallucinated_node`
- `subsystem_boundary_confusion`
- `missing_node`
- `wrong_topic_name`
- `lifecycle_violation`

Main metrics:

- **Error Category Distribution**
- **Inter-LLM Error Rate Variation**

### SQ3: Automatic Detection

The script now reports SQ3 in two layers:

- **Proposal Tool Coverage Ratio:** counts only categories detected with adequate precision and recall by AS2FM or ROSClaw.
- **Extended/local Tool Coverage Ratio:** also includes the generated JANI-style AS2FM adapter and the deterministic ROS 2 static architecture validator.

This distinction is important because the current real ROS 2 source repositories provide architecture graphs, but not repo-specific RoAML/ASCXML behaviour models that AS2FM can execute directly. Therefore the real AS2FM column is marked as not applicable for those inputs unless such behaviour models are supplied. The generated JANI-style adapter remains in the output as evidence that the category checks are formalizable, but it is not counted as real AS2FM proposal coverage.

### SQ4: Adoption Gap

The run records practical adoption-friction evidence from the local smoke checks:

- real-data-only filter status
- skipped status for external smoke checks whose available inputs are tutorial/test/simulation fixtures
- comparison against the baseline of manually triaging residual LLM errors

GitHub stars/forks/issues can still be added as an adoption proxy, but they are not required for the proposal completion gate recorded in `metrics_results.json`.

## Metrics

### M1: Node-Level Precision

How many generated nodes are real?

```text
TP / (TP + FP)
```

Example:

```text
LLM outputs 10 nodes.
9 are real.
Precision = 90%.
```

### M2: Subsystem Boundary Recall Gap

How much accuracy drops when moving from node detection to subsystem grouping.

```text
node recall - subsystem recall
```

A large gap means the model finds nodes but groups them badly.

### M3: Error Category Distribution

Which error type appears most often.

Example:

```text
807 of 975 errors are missing_node.
missing_node = 82.77%.
```

### M4: Inter-LLM Error Rate Variation

How different the models are for each error category.

A high value means model choice matters a lot.

### M5: Tool Coverage Ratio

How many error categories can be detected automatically.

```text
covered categories / total categories
```

### M6: Time-to-Verify

How long the full checking pipeline takes.

This includes:

- parsing source code
- comparing model output
- validating errors

## Suggested First Run

Use a small run first:

```env
MAX_REPOS=2
ACTIVE_MODELS=llama-4,groq-small,groq-large
PROMPT_CHAR_BUDGET=30000
LLM_CHUNK_PAUSE_SECONDS=2
LLM_CACHE=1
```

Then run:

```powershell
python ros2_llm_arch_recovery.py
python generate_research_report.py
```

After it finishes, open `metrics_results.json` and look at:

```json
"metrics"
```

That section tells you which model performed best and what errors happened most.

For a more readable answer, open:

```text
research_report.md
```

That file is organized directly by SQ1, SQ2, SQ3, SQ4, and TtV.

## Full Proposal Target

Your pasted plan says the final study should do this:

1. Select 15 ROS 2 repositories.
2. Build or verify ground-truth node, topic, and subsystem annotations.
3. Feed each repo to the configured real Groq models.
4. Log TP, FP, FN for nodes and subsystem groupings.
5. Classify every error into a ROS 2-specific taxonomy.
6. Test whether AS2FM or ROSClaw detects each error category.
7. Measure end-to-end TtV.
8. Measure adoption friction for AS2FM and ROSA.

The current pipeline covers steps 1-5 with real Groq model calls/cache and real static validation. In real-data-only mode it does not use the bundled AS2FM tutorial model or ROSClaw firewall tests as metric evidence, because those are not real production source inputs.

## Future Expansion

When you get more API keys, update `.env` like this:

```env
ACTIVE_MODELS=llama-4,groq-large,groq-small,qwen-groq,gemini-flash,mistral-small,openrouter-free,gpt-nano
```

Supported future labels already in the code:

- `llama-4`
- `groq-large`
- `groq-small`
- `qwen-groq`
- `gemini-flash`
- `mistral-small`
- `openrouter-free`
- `gpt-nano`
