# CS3050 Lab 6: Advanced Route Planning - Multi-Constraint Pathfinding

## Assignment Overview

In this lab, you will extend the route planning system to handle **real-world routing constraints** that go beyond simple shortest path finding. This assignment requires you to make informed algorithmic decisions, prove correctness properties, and justify your design choices.

**Due Date:** [Your date here]  
**Points:** 300 points in four sections/submissions
**Submission:** Code + Written Report (Markdown, LaTeX, or PDF)

---

## Learning Objectives

By completing this lab, you will:

- Understand the limitations of standard pathfinding algorithms in real-world scenarios
- Implement constraint satisfaction in graph algorithms
- Analyze algorithm correctness and performance tradeoffs
- Make and justify architectural decisions
- Debug and optimize existing algorithmic code

---

## Background: The Real-World Routing Problem

Emergency services, delivery companies, and ride-sharing apps don't just need the shortest path—they need paths that satisfy multiple constraints:

- **Time windows**: Arrive at nodes within specific time ranges
- **Vehicle capacity**: Can't exceed weight/volume limits
- **Priority levels**: Some destinations are more urgent than others
- **Turn restrictions**: Some routes are one-way or have turn prohibitions
- **Dynamic costs**: Edge weights change based on time of day (traffic)

Your task is to extend the existing system to handle these real-world complications.

---

## Part 1: Implementation (120 points)

### Task 1.1: Time-Window Constrained Routing (60 points)

Extend ONE of the three implementations (C, Python, or Go) to support time windows.

**Requirements:**

1. Modify the node CSV format to include optional time windows:

   ```csv
   id,lat,lon,earliest,latest
   1,38.9072,-77.0369,0,100
   2,38.9100,-77.0400,30,60
   3,38.9050,-77.0300,50,150
   ```

2. Implement a modified shortest path algorithm that finds paths where you arrive at each node within its time window `[earliest, latest]`

3. If no feasible path exists (constraints cannot be satisfied), your program should:
   - Report "No feasible path satisfying time constraints"
   - Identify which constraint(s) were violated
   - Suggest the "closest" path (minimize constraint violations)

**Key Challenge:** You CANNOT simply run the existing algorithms unchanged. Think about:

- How do time windows affect the optimality guarantee of Dijkstra?
- When can you prune paths that are shorter but arrive too early/late?
- How does this change the state space you need to track?

**Deliverable:** 

- Modified source code with clear comments explaining your approach
- New CSV test files that demonstrate:
  - A feasible path
  - An infeasible path
  - A case where the shortest distance path violates constraints

### Task 1.2: Priority-Based Multi-Destination Routing (60 points)

Implement a system that routes through multiple destinations with different priority levels.

**Requirements:**

1. Accept a list of destinations with priorities:

   ```
   destination,priority
   3,HIGH
   5,MEDIUM
   7,LOW
   ```

2. Find a route that:
   - Starts at a given source
   - Visits all destinations
   - Minimizes total distance
   - Visits HIGH priority destinations before MEDIUM before LOW

3. Handle the case where strict priority ordering makes the path significantly longer

**Key Challenge:** This is NOT the traveling salesman problem. You need to decide:

- How much longer can the path be to respect priority order?
- Should you allow priority violations if the distance savings is significant?
- How do you balance optimality vs. priority satisfaction?

**Deliverable:**

- Implementation that takes a threshold parameter (e.g., "allow 20% longer path")
- Function that outputs: route, total distance, priority violations (if any)
- Test cases showing different threshold behaviors

---

## Part 2: Algorithmic Analysis (90 points)

This section requires mathematical reasoning and cannot be completed by AI alone without your understanding.

### Task 2.1: Correctness Proof (45 points)

**Question:** Under what conditions does your time-window algorithm guarantee finding a feasible path if one exists?

Write a formal proof that includes:

1. **State your assumptions** clearly (e.g., non-negative weights, specific constraint properties)
2. **Proof sketch** showing your algorithm's correctness using one of:
   - Loop invariants
   - Induction
   - Exchange argument
3. **Counterexample**: Provide a specific graph configuration where your algorithm might fail to find a feasible path (even if one exists), OR prove that no such configuration exists

**What we're looking for:**

- Not a code walkthrough—we want mathematical reasoning
- Specific graph examples (draw them)
- Clear logical flow: assumptions → reasoning → conclusion

### Task 2.2: Performance Analysis (45 points)

For your time-window algorithm:

1. **Time Complexity Analysis:**
   - Derive the worst-case time complexity as a function of |V|, |E|, and W (max time window range)
   - Explain what contributes to this complexity (what's the bottleneck?)
   - Compare to standard Dijkstra: when is yours faster? slower?

2. **Space Complexity Analysis:**
   - What additional state must you track compared to standard shortest path?
   - Derive space complexity

3. **Experimental Validation:**
   - Create test graphs of varying sizes: |V| = 10, 50, 100, 500
   - Measure actual runtime (in microseconds/milliseconds)
   - Plot: Graph size vs. Runtime
   - Does your empirical result match your theoretical analysis? Explain any discrepancies.

**Deliverable:** 

- Written analysis (LaTeX, Markdown or well-formatted PDF)
- Graphs/plots showing experimental results
- Discussion of theoretical vs. empirical performance

---

## Part 3: Design Justification (60 points)

### Task 3.1: Algorithm Selection (30 points)

You implemented time-window routing by modifying ONE of the three algorithms (Dijkstra, A*, or Bellman-Ford).

**Question:** Why did you choose that algorithm as your base?

Write a 1-2 page analysis that:

1. **Explains your choice** - Why is it better suited than the alternatives?
2. **Discusses tradeoffs** - What did you sacrifice by not choosing the others?
3. **Proposes modifications** - How would you modify a DIFFERENT algorithm to handle the same constraints?
4. **Comparative analysis** - In what scenarios would the alternative be better?

**Evaluation criteria:**

- Depth of algorithmic understanding
- Concrete examples supporting your arguments
- Discussion of real-world implications
- Quality of comparative reasoning

### Task 3.2: Alternative Approaches (30 points)

**Scenario:** Your manager says "customers are complaining that our routes take too long. Can you make it faster?"

Propose and analyze TWO different approaches to speed up your time-window routing:

For each approach:

1. **Describe the optimization** in detail
2. **Analyze the tradeoff** (what do you lose for the speedup?)
3. **Estimate the speedup** (order of magnitude: 2x? 10x? 100x?)
4. **Identify when it fails** (what graph properties make it ineffective?)

Possible directions (you can propose your own):

- Preprocessing/indexing
- Approximation algorithms
- Heuristic pruning
- Parallel processing
- Different data structures

**What we're looking for:**

- Creative but grounded solutions
- Understanding of algorithmic tradeoffs
- Quantitative reasoning about performance
- Recognition of limitations

---

## Part 4: Debugging and Optimization (30 points)

### Task 4.1: Bug Hunt (15 points)

We've introduced a subtle bug into the Bellman-Ford implementation in ONE of the three languages (we'll tell you which one). The bug causes incorrect results only under specific graph configurations.

**Your Task:**

1. **Identify the bug** through testing and code analysis
2. **Explain why it occurs** - what specific input triggers it?
3. **Fix the bug** and verify with test cases
4. **Document your process** - how did you find it?

**Hint:** The bug is not a syntax error or obvious crash—it produces "reasonable but wrong" results.

### Task 4.2: Performance Optimization (15 points)

Profile your time-window implementation and identify the bottleneck operation (the part that takes the most time).

**Requirements:**

1. **Profile your code** using appropriate tools:
   - C: `gprof` or `valgrind --tool=callgrind`
   - Python: `cProfile`
   - Go: `pprof`

2. **Identify the hotspot** - which function/operation dominates runtime?

3. **Optimize it** - make the code at least 20% faster through:
   - Better data structures
   - Algorithmic improvements
   - Avoiding redundant computation

4. **Document your optimization:**
   - Before/after performance measurements
   - What changed and why
   - Any tradeoffs (memory? code complexity?)

---

## Submission Requirements

### Code Submission (via Canvas, a zip file)
```
lab6_submission/
├── src/
│   ├── route_planner.[c|py|go]  # Your modified implementation
│   ├── Makefile or requirements.txt
│   └── README.md               # Build/run instructions
├── tests/
│   ├── test_time_windows.csv
│   ├── test_priorities.csv
│   └── test_results.txt        # Expected outputs
├── profiling/
│   ├── profile_before.txt
│   ├── profile_after.txt
│   └── optimization_notes.md
└── report/
    └── lab6_report.pdf
```

### Written Report (PDF, 8-12 pages)

Your report should include:
1. **Introduction** (0.5 pages)
   - Problem overview
   - Your approach summary

2. **Implementation Details** (2-3 pages)
   - Algorithm modifications
   - Design decisions
   - Key challenges and solutions

3. **Correctness Proof** (2-3 pages)
   - Formal proof from Task 2.1
   - Assumptions and reasoning

4. **Performance Analysis** (2-3 pages)
   - Complexity analysis
   - Experimental results with graphs
   - Discussion

5. **Design Justification** (2-3 pages)
   - Algorithm selection reasoning
   - Alternative approaches analysis

6. **Debugging and Optimization** (1-2 pages)
   - Bug discovery process
   - Performance optimization results

7. **Conclusion** (0.5 pages)
   - Lessons learned
   - Future improvements

---

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Time-Window Implementation** | 60 | Correctness, handles edge cases, code quality |
| **Priority Routing** | 60 | Correct logic, parameter handling, testing |
|-----------|--------|----------|
| **Correctness Proof** | 45 | Mathematical rigor, clear reasoning, completeness |
| **Performance Analysis** | 45 | Accurate complexity, good experiments, insightful discussion |
|-----------|--------|----------|
| **Algorithm Selection** | 30 | Depth of understanding, comparative analysis |
| **Alternative Approaches** | 30 | Creativity, feasibility, tradeoff analysis |
|-----------|--------|----------|
| **Bug Hunt** | 15 | Found bug, explained clearly, good test cases |
| **Optimization** | 15 | Measurable improvement, well documented |
|-----------|--------|----------|
| **Report Quality** | 15 | Clear writing, good organization, proper citations |
| **Code Quality** | 15 | Comments, structure, documentation |
|-----------|--------|----------|
| **TOTAL** | **330** | *Extra 30 points available* |

---

## Academic Integrity Notice

**Collaboration Policy:**
- You MAY discuss high-level approaches with classmates
- You MAY use reference materials (textbooks, papers, documentation)
- You MAY use AI tools (ChatGPT, GitHub Copilot) for LEARNING and DEBUGGING
- You MUST write all code yourself
- You MUST write all analysis and proofs yourself

**What AI tools CAN'T do for you:**
- Make informed design decisions for your specific problem
- Prove correctness without understanding the algorithm
- Justify tradeoffs in your specific context
- Debug subtle logical errors in algorithmic code
- Analyze performance characteristics accurately

**We will check for:**
- Suspiciously perfect code that doesn't match your explanation in the report
- Analysis that doesn't align with your implementation
- Proofs that are generic and don't address your specific algorithm
- Multiple students with identical design decisions and justifications

**If you use AI assistance, you must:**
- Document what you asked and how you used the response
- Demonstrate understanding by explaining it in your own words
- Make it your own by adapting it to your specific needs

---

## Hints and Tips

### For Time Windows:

- Think about how time windows change the "state" in your search
- Consider: is a node just identified by its ID, or by (ID, arrival_time)?
- What does "shortest path" even mean with time constraints?

### For Proofs:

- Start with simple examples—can you prove it works on a 3-node graph?
- Look at loop invariants: what's true before/after each iteration?
- Common pitfall: assuming your algorithm does what you want without proving it

### For Performance Analysis:

- Generate random graphs programmatically for testing at scale
- Use consistent hardware/environment for timing experiments
- Run multiple trials and report averages (systems are noisy!)
- I'm not looking for perfection, and your explanation of what you did is important if you do not find measurable variation on your system (though you "should"). 

### For Optimization:

- Profile BEFORE optimizing—don't guess where the bottleneck is
- Sometimes the best optimization is a better algorithm
- Measure carefully: did you really make it faster?

---

## Resources

- *Introduction to Algorithms* (CLRS), Chapter 24 (Shortest Paths)
- *Algorithm Design* by Kleinberg & Tardos, Chapter 6 (Dynamic Programming)
- Research paper: "Time-Dependent Shortest Paths" by Orda & Rom
- Profiling tutorials: [Links TBD on Canvas]

---

## Frequently Asked Questions

**Q: Can I use external libraries for graph algorithms?**  
A: No. You must implement the core algorithm logic yourself. You may use standard library data structures (heaps, hashmaps, etc.).

**Q: What if my time-window algorithm is exponential time?**  
A: That's okay IF you can prove it and explain why the problem requires it. Some variants are NP-hard.

**Q: How formal does the proof need to be?**  
A: Remember, I'm looking to figure out what you **understand**. More formal than pseudocode walkthrough, less formal than a published paper. Think: undergraduate algorithm textbook level, but focus on making it clear, and make sure your description of the proof, however you do it, matches your code. 

**Q: Can I do the implementation in a different language?**  
A: No.

**Q: Do I have to find the bug in Part 4.1?**  
A: Yes, it's required. Start early and use systematic debugging techniques.

**Q: What if I can't optimize by 20%?**  
A: Document what you tried and why it didn't work. Credit is given for soundly reasoned, and good attempts.

---

## Getting Started Checklist

- [ ] Fork the repository
- [ ] Clone YOUR FORK of the repository and verify all three implementations work
- [ ] Read through all existing code to understand the structure
- [ ] Design your time-window data structures on paper first
- [ ] Write test cases BEFORE implementing
- [ ] Start the written analysis early (it takes longer than you think)
- [ ] Profile early and often
- [ ] Leave time for debugging and optimization
- [ ] Proofread your report

---

**Good luck! This is a challenging assignment aimed at deepening your understanding of graph algorithms and to prepare you to approach challenges like this one in a pragmatic manner.**