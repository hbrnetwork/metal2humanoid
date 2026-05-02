# CLAUDE.md

This file gives Claude instructions for helping maintain this repository’s learning wiki.

The repository is not just for code. It is also a living technical wiki and learning journal. Your job is to help turn rough work, implementation notes, debugging sessions, experiments, and concepts into organized `.mdx` lesson files.

The goal is not to produce homework-style reports. The goal is to create a compounding knowledge base: every implementation, bug, explanation, experiment, and lesson should make the next student faster.

---

## 1. Primary responsibility

When the student asks for help, Claude should help them produce clear, truthful, organized `.mdx` notes.

Prioritize:

1. Technical accuracy
2. Reproducibility
3. Clear explanations
4. Honest uncertainty
5. Good file organization
6. Reusable examples

Do **not** invent results, fake citations, pretend code worked, or hide uncertainty.

If something is unclear, mark it as:

```md
TODO: needs verification
```

or:

```md
Unclear: this still needs to be tested.
```

---

## 2. Required file organization

All notes should live under:

```txt
materials/notes/{topic}/{subtopic}.mdx
```

Where:

- `{topic}` is a broad category, such as `robotics`, `ml`, `rl`, `simulation`, `experiments`, `debugging`, or `references`.
- `{subtopic}` is a self-contained lesson, concept, implementation, debugging note, or experiment.

Use lowercase filenames with hyphens.

Good examples:

```txt
materials/notes/robotics/configuration-space.mdx
materials/notes/robotics/forward-kinematics.mdx
materials/notes/ml/decision-trees.mdx
materials/notes/rl/deep-q-networks.mdx
materials/notes/simulation/mujoco-basics.mdx
materials/notes/debugging/pybullet-install-issues.mdx
materials/notes/experiments/rrt-parameter-sweep.mdx
```

Bad examples:

```txt
materials/notes/stuff.mdx
materials/notes/random-notes.mdx
materials/notes/My Notes.mdx
materials/notes/final_final_notes.mdx
```

Each `.mdx` file should be a self-contained lesson with examples.

---

## 3. Suggested topic folders

Use these folders when appropriate:

```txt
materials/
  notes/
    robotics/
    ml/
    rl/
    simulation/
    experiments/
    debugging/
    references/
```

Add new topic folders only when they represent a real broad category.

Prefer:

```txt
materials/notes/robotics/configuration-space.mdx
```

over:

```txt
materials/notes/robotics/configuration-space-notes-final.mdx
```

---

## 4. What should become a note

Create or update an `.mdx` note whenever the student works on something meaningful.

Examples:

- A new robotics concept
- A new ML/RL concept
- A new algorithm
- A simulator, robot, sensor, package, or tool
- A bug that took time to understand
- A design decision
- A paper, tutorial, or repo that helped
- A failed approach and why it failed
- A working example someone else can reproduce
- A visualization, diagram, table, or experiment result

Rule of thumb:

> If the student spent more than 30 minutes learning, debugging, designing, or implementing something, it probably deserves a note.

---

## 5. Standard MDX lesson format

When creating a serious concept or implementation note, use this structure unless the user asks otherwise:

````mdx
---
title: "Clear Human-Readable Title"
tags: ["robotics", "motion-planning"]
topic: "robotics"
subtopic: "configuration-space"
lastUpdated: YYYY-MM-DD
difficulty: beginner
---

# Clear Human-Readable Title

One short paragraph explaining what this note is about and why it matters.

## 1. Motivation

What problem are we solving?

Why does this matter for the project?

What should the reader understand by the end?

## 2. Intuition

Explain the core idea in plain English.

Use examples, analogies, diagrams, or simple cases when helpful.

## 3. Formal idea

Give the more precise technical version.

Use definitions, equations, or pseudocode when useful.

For example:

$$
\mathcal{C}_{free} = \{q \in \mathcal{C} \mid \mathcal{A}(q) \cap \mathcal{O} = \emptyset\}
$$

## 4. Minimal example

Include a small example that makes the idea concrete.

This can be:

- A toy Python example
- A simple geometry example
- A small robot example
- A command someone can run
- A diagram or figure

```python
def example():
    return "small reproducible example"
```

## 5. Implementation notes

Describe what was implemented.

Include relevant file paths, functions, classes, commands, and dependencies.

```txt
Relevant files:
- src/planning/rrt.py
- src/collision/checker.py
- scripts/run_rrt_demo.py
```

Example command:

```bash
python scripts/run_rrt_demo.py
```

## 6. Results

What happened?

Include screenshots, tables, plots, logs, or observations when useful.

Be honest about whether the result is complete, partial, or broken.

## 7. Lessons learned

What should the next person understand?

What was surprising?

What mistakes should they avoid?

## 8. Open questions / next steps

- [ ] Thing to try next
- [ ] Bug to investigate
- [ ] Experiment to run
- [ ] Concept to understand better

## 9. References

Link to helpful papers, websites, repos, videos, or related notes.

- [[robotics/configuration-space]]
- [[motion-planning/collision-checking]]
````

---

## 6. Debugging note format

For debugging sessions, use this structure:

````mdx
---
title: "Debugging: Short Description"
tags: ["debugging"]
topic: "debugging"
subtopic: "short-description"
lastUpdated: YYYY-MM-DD
difficulty: beginner
---

# Debugging: Short Description

## Symptom

What went wrong?

Include the error message, screenshot, or command output.

```txt
Paste relevant error here.
```

## Context

What was the student trying to do?

What command did they run?

What files were involved?

## Failed attempts

What did they try that did not work?

- Attempt 1:
- Attempt 2:
- Attempt 3:

## Root cause

What was actually wrong?

If the root cause is unknown, say so clearly.

## Fix

What fixed the issue?

```bash
command_that_fixed_it
```

## Lesson learned

How can someone avoid this problem next time?

## Related notes

- [[simulation/mujoco-basics]]
- [[debugging/python-environment-setup]]
````

---

## 7. Experiment note format

For experiments, use this structure:

````mdx
---
title: "Experiment: Short Description"
tags: ["experiment"]
topic: "experiments"
subtopic: "short-description"
lastUpdated: YYYY-MM-DD
difficulty: intermediate
---

# Experiment: Short Description

## Question

What question is this experiment trying to answer?

## Hypothesis

What did the student expect to happen?

## Setup

Describe the environment, robot, simulator, parameters, code, and dataset.

```txt
Relevant files:
- path/to/file.py
- path/to/config.yaml
```

## Command

```bash
python scripts/run_experiment.py --arg value
```

## Results

What happened?

Include tables, screenshots, plots, or logs.

## Interpretation

What do the results mean?

Did they support the hypothesis?

## Limitations

What is incomplete or uncertain?

## Next steps

- [ ] Follow-up experiment
- [ ] Parameter to sweep
- [ ] Bug to check
````

---

## 8. Claude behavior guidelines

Claude should:

- Turn rough notes into clean `.mdx`
- Ask for missing context only when needed
- Preserve technical details
- Keep uncertainty visible
- Suggest better filenames and folder locations
- Add examples when useful
- Add TODOs instead of inventing missing information
- Split giant notes into smaller notes
- Link related notes using wiki links
- Make notes readable by a future student

Claude should not:

- Invent experiment results
- Invent file paths
- Invent citations
- Pretend code was run
- Remove useful debugging context
- Over-polish away uncertainty
- Put everything into one large note
- Use vague language when specific commands, files, or functions are known

---

## 9. Formatting conventions

Use Markdown / MDX formatting consistently.

### Headings

Use numbered sections for long notes:

```md
## 1. Motivation

## 2. Intuition

## 3. Formal idea

## 4. Minimal example

## 5. Implementation notes
```

### Callouts

Use callouts for important ideas:

```md
> [!note] Key idea
> This is an important concept.

> [!warning] Common mistake
> This is something that often goes wrong.

> [!tip] Practical advice
> This is a useful implementation tip.

> [!math] Definition
> Use this for formal definitions, equations, or theorem-like statements.
```

### Code blocks

Use fenced code blocks with the language name.

````md
```python
def example():
    return "hello"
```
````

Use `bash` for terminal commands:

````md
```bash
python scripts/run_demo.py
```
````

Use `txt` for logs or file trees:

````md
```txt
materials/
  notes/
    robotics/
      configuration-space.mdx
```
````

### Math

Use inline math:

```md
The configuration is $q \in \mathcal{C}$.
```

Use display math:

```md
$$
\mathcal{C}_{free} = \{q \in \mathcal{C} \mid \mathcal{A}(q) \cap \mathcal{O} = \emptyset\}
$$
```

### Wiki links

Use wiki links to connect notes:

```md
See [[robotics/configuration-space]] and [[motion-planning/collision-checking]].
```

Use display text when helpful:

```md
See [[robotics/configuration-space|configuration space]] for background.
```

---

## 10. Recommended workflow with the student

When the student gives rough notes, Claude should usually respond with:

1. Suggested file path
2. A polished `.mdx` draft
3. Any TODOs or missing information
4. Suggested related notes to link

Example:

```txt
Suggested file:
materials/notes/robotics/configuration-space.mdx
```

Then provide the `.mdx` content.

When the student gives a debugging transcript, turn it into a debugging note.

When the student gives experiment notes, turn them into an experiment note.

When the student gives a large mixed note, split it into multiple proposed files.

---

## 11. Quality bar

A good note is:

- Clear enough that another student can learn from it
- Specific enough that someone can reproduce the work
- Honest about what worked and what did not
- Connected to related notes
- Organized under `materials/notes/{topic}/{subtopic}.mdx`
- Written as a self-contained lesson
- Includes examples when possible
- Includes commands, file paths, screenshots, or references when useful

A bad note is:

- A random collection of thoughts
- Missing commands, file paths, or context
- Only understandable to the original author
- Full of unexplained screenshots
- Written after the fact with important details forgotten
- Overly polished but technically vague
- Generated without careful review

---

## 12. Final rule

If it would save the next student time, write it down.

If it was annoying to debug, write it down.

If it is reusable, write it down.

If it is confusing, write down what is confusing.

The wiki should become the memory of the project.
