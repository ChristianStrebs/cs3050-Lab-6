# Lab 6 Student Hints and FAQ

## Getting Started

### "I don't know where to begin!"

**Start small:**
1. First, understand the existing code thoroughly
2. Draw a 4-node graph on paper
3. Trace through time-window routing manually
4. THEN start coding

**Don't jump straight to implementation!**

### "Should I use Dijkstra, A*, or Bellman-Ford as my base?"

Think about:
- Do time windows create negative edges? (No)
- Will a heuristic help? (Maybe—depends on your approach)
- Does order of relaxation matter? (Think about this carefully!)

**Each choice has tradeoffs. There's no single "right" answer.**

---

## Part 1: Implementation Hints

### Time Windows

#### Hint 1: What is a "state"?
In regular shortest path: state = node ID
With time windows: state = ??? (think about what else matters)

#### Hint 2: Can you arrive early and wait?
- If yes: affects your state space
- If no: simpler but might miss feasible paths

**Design decision:** Document what you choose and why.

#### Hint 3: When do you prune?
You can ignore a path if:
- You arrive after `latest[v]` → path is infeasible
- You have another path to `v` that arrives earlier → can dominate

**Be careful:** A path with longer distance might arrive earlier!

#### Hint 4: Testing infeasibility
Create a graph where:
```
Node 2 has window [10, 20]
Node 3 has window [15, 25]
Edge 2→3 takes 30 time units
→ No path from 2→3 satisfies constraints
```

### Priority Routing

#### Hint 1: Not the Traveling Sales Person Problem!

You don't need to find the optimal tour. You need:

- A tour that visits all nodes
- Respects priority order (mostly)
- Is "reasonably short"

#### Hint 2: Greedy approach

One approach:

1. Visit HIGH priority nodes in distance order
2. Then MEDIUM priority nodes
3. Then LOW priority nodes

**But** this might be very suboptimal. How much worse is it?

#### Hint 3: Threshold parameter

If threshold = 20%, you might:

- Allow swapping adjacent different-priority nodes if saves >20% distance
- OR add up to 20% distance to maintain priority order

**Design choice:** What makes sense for the use case?

---

## Part 2: Proof Hints

### "I don't know how to start a proof!"

**Template:**
```
Claim: [State what you're proving]

Assumptions:
- [List all assumptions clearly]
- e.g., "All edge weights are non-negative"
- e.g., "Time windows are consistent (earliest ≤ latest)"

Proof sketch:
1. [Define a loop invariant OR use induction]
2. [Show your algorithm maintains this property]
3. [Show this property guarantees correctness]

Base case: [If using induction]
Inductive step: [If using induction]
Conclusion: [Therefore...]
```

### Loop Invariant Approach

**For time-window Dijkstra variant:**

Invariant: "After k iterations, for all nodes in the visited set, we have found the earliest feasible arrival time."

**Prove:**

1. Initially true (start node has arrival time 0)
2. Maintained by each iteration (why?)
3. Upon termination, all reachable nodes are visited

### "What if I can't prove it's correct?"

That might mean:

- Your algorithm has a subtle bug
- Your algorithm only works under certain conditions
- The problem is harder than you thought

**All of these are valuable discoveries!** Document what you found.

---

## Part 3: Performance Analysis Hints

### Time Complexity

**Wrong approach:**
"My algorithm is O(V log V) because I use a priority queue."

**Right approach:**
"Standard Dijkstra is O((V + E) log V). In my time-window variant:

- I track states as (node, arrival_time)
- Arrival times can be discretized into W values
- So I have up to V × W states
- Each state processes E edges
- Therefore: O((VW + E) log(VW))"

**Show your reasoning step by step!**

### Experimental Validation

**Minimum requirements:**

```python
import time

sizes = [10, 50, 100, 500]
times = []

for n in sizes:
    graph = generate_random_graph(n)
    start = time.time()
    run_algorithm(graph)
    elapsed = time.time() - start
    times.append(elapsed)

plot(sizes, times)
```

**Better:**

- Run multiple trials (10+) and average
- Control for other processes
- Test different graph densities
- Compare algorithms side-by-side

---

## Part 4: Bug Hunt Hints

### "I can't find the bug!"

**Systematic approach:**

1. Create simple test cases
2. Compare output with correct implementation (Dijkstra)
3. Add print statements to see what the algorithm is doing
4. Look for patterns: when does it fail?

### Debugging Checklist

Test with:

- [ ] Simple 3-node graph
- [ ] Graph with no path
- [ ] Graph with multiple paths
- [ ] Larger random graphs
- [ ] The provided test case

### "I think I found it, but I'm not sure"

**Verify:**

1. Can you explain WHY it causes wrong results?
2. Can you create a test case that triggers it?
3. Does fixing it make all tests pass?

### "What kind of bug is it?"

It's NOT:

- A syntax error
- A crash
- An obvious logic error

It IS:

- A subtle algorithmic mistake
- Something that seems like it should work
- Related to the order of operations
- Only visible with specific graph structures

---

## Design Justification Hints

### "Why did I choose Dijkstra?"

**Bad answer:**
"Because it's faster."

**Good answer:**
"I chose Dijkstra because:

1. Time windows don't create negative weights
2. Priority queue is efficient for my state space size
3. A* heuristic wouldn't help much because time constraints are stricter than distance

Tradeoffs:

- If we had negative weights, Bellman-Ford would be necessary
- If time windows were very loose, A* might explore fewer nodes

Alternative: I could use A* with a combined distance+time heuristic..."

### "What are alternative approaches?"

Think about:

- **Preprocessing**: Can you precompute anything?
- **Approximation**: What if you allow small errors?
- **Heuristics**: What if you prioritize promising paths?
- **Parallelization**: Can you explore multiple paths at once?

**For each:** Explain the tradeoff!

---

## Optimization Hints

### "I don't know what to optimize"

**Profile first!**
```bash
# Python
python -m cProfile -o profile.stats route_planner.py ...
python -c "import pstats; p = pstats.Stats('profile.stats'); p.sort_stats('cumulative'); p.print_stats(10)"

# C
gcc -pg route_planner.c -o route_planner
./route_planner ...
gprof route_planner gmon.out > analysis.txt
```

**gprof does not work with OSX, but other tools are available**


**The bottleneck might surprise you!**

### Common Bottlenecks

1. **Priority queue operations**
   - Are you using the right data structure?
   - Can you reduce number of insertions?

2. **State duplication**
   - Are you tracking the same state multiple times?
   - Can you merge equivalent states?

3. **Path reconstruction**
   - Are you rebuilding paths repeatedly?
   - Can you cache intermediate results?

### "Nothing is faster than 20%"

**Options:**

1. Try a different optimization
2. Consider algorithmic improvements (better than micro-optimization)
3. Document what you tried—partial credit available!

---

## Common Mistakes

### Implementation

"I'll just modify the existing code slightly"
- Time windows fundamentally change the problem
- You need to rethink the state space

"AI generated perfect code on first try"
- There WILL be bugs
- Testing reveals them
- Iteration is part of learning

"I'll handle edge cases later"
- Edge cases are WHERE the learning happens
- They reveal misunderstandings

### Proofs

Proving something you didn't implement
- Your proof must match YOUR code

Hand-waving at the hard parts
- "It's obvious that..." → not a proof

Generic textbook proof
- Must address YOUR specific modifications

### Performance Analysis

"It ran in 0.5 seconds"
- Need multiple sizes to see complexity
- Need multiple trials for accuracy

Ignoring discrepancies
- Theory says O(n²), experiments show O(n³)?
- Investigate! Don't just report.

---

## Testing Strategies

### Create Diverse Test Cases

1. **Simple cases** (3-4 nodes)
   - Verify basic correctness
   - Easy to trace manually

2. **Edge cases**
   - No feasible path
   - Multiple equal-length paths
   - Very tight time windows
   - Very loose time windows

3. **Stress tests**
   - Large graphs (100+ nodes)
   - Dense graphs (many edges)
   - Sparse graphs (few edges)

4. **Real-world-ish**
   - Mimic actual city street networks
   - Use realistic time windows

### Automated Testing

```python
def test_time_windows():
    # Known input
    graph = create_test_graph()
    # Expected output
    expected_path = [1, 2, 3]
    expected_distance = 10.5
    
    # Actual output
    actual_path, actual_dist = your_algorithm(graph, 1, 3)
    
    # Verify
    assert actual_path == expected_path
    assert abs(actual_dist - expected_distance) < 0.01
```

---

## Time Management

### Realistic Schedule (3 weeks)

**Week 1:** Understand existing code, design approach
**Week 1:** Implement time windows
**Week 2:** Implement priority routing, start proofs
**Week 2:** Complete proofs, performance analysis
**Week 3:** Bug hunt, optimization, testing
**Week 3:** Write report, polish everything

**Don't do it all at the end!**

---

## Getting Help

### Before Asking

1. **Read the error message carefully**
2. **Search the course materials**
3. **Try to debug it yourself**
4. **Prepare a specific question**

### Good Questions

"My time-window algorithm gives incorrect results on this graph [show graph]. I expect path [1,2,3] but get [1,4,3]. Here's my reasoning about why it should be [1,2,3]..."

"I'm stuck on the proof. I can show X and Y, but I can't connect them to conclude Z. Here's what I've tried..."

### Less Helpful Questions: 

- "My code doesn't work, can you fix it?"
- "Is this proof correct?" [without showing reasoning]
- "Which algorithm should I use?"

---

## Final Advice

### On Using AI Tools

**DO:**

- Use AI to understand concepts
- Ask AI to explain algorithms
- Use AI to debug syntax errors
- Use AI to learn about data structures

**DON'T:**

- Copy AI-generated code without understanding it
- Let AI make design decisions for you
- Submit AI-generated proofs
- Rely on AI for correctness checking

**Remember:** You need to defend your work. Can you explain it?

### On Working Through Frustration

**When stuck:**

1. Take a break
2. Draw pictures
3. Work through a small example manually
4. Explain it to a rubber duck
5. Sleep on it

**Complex problems require time to marinate in your brain.**

### On Learning

This assignment is hard by design. The goal is not just to complete it, but to:

- Understand deeply
- Make thoughtful decisions
- Learn to analyze and justify
- Develop problem-solving skills

**The struggle is where the learning happens.**

---

## Success Checklist

Before submitting:

- [ ] Code compiles/runs without errors
- [ ] All test cases pass
- [ ] Code is well-commented
- [ ] Proof addresses your specific algorithm
- [ ] Performance analysis includes experiments
- [ ] Design decisions are justified with examples
- [ ] Bug is found and fixed
- [ ] Optimization is measured
- [ ] Report is well-written and organized
- [ ] You can explain everything in your submission

**If you can't check all boxes, you can still submit—but document what's incomplete. Remember, these are setup as four different Canvas assignments.**

---

**Good luck! Start early, think deeply, and don't hesitate to ask questions!**