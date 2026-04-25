# AGENTS.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

## Project-Specific: Thesis Work

For this thesis repository:

- Do not assume thesis content, metadata, dates, keywords, structure changes, or translations that the user did not explicitly provide.
- If required information is missing, ask instead of inventing placeholders with guessed values.
- When the user asks to update only a specific part, restrict edits to that part and dependent (if needed) only.
- Do not write thesis body text, abstracts, contributions, or other chapter content unless explicitly requested.
- Do not enable optional sections or change template behavior unless explicitly requested.


### Core Rules for Thesis Writing

1. **Answer one clear research question**
   Everything in your thesis should serve this question.

2. **Be precise, not fancy**
   Clarity beats complex wording. Avoid fluff.

3. **Structure matters**
   Follow a logical flow: *Introduction → Literature → Method → Results → Discussion → Conclusion*.

4. **Back every claim with evidence**
   No unsupported statements—cite or show data.

5. **Stay consistent**
   Use the same terms, tense, formatting, and citation style throughout.

6. **Explain your method so others can replicate it**
   If someone can’t repeat your work, it’s incomplete.

7. **Separate results from interpretation**
   First show what you found, then explain what it means.

8. **Be critical, not just descriptive**
   Analyze sources and results—don’t just summarize.

9. **Edit aggressively**
   Cut redundancy, tighten sentences, and remove anything irrelevant.

10. **Follow your university guidelines exactly**
    Formatting and citation mistakes can cost easy points.
