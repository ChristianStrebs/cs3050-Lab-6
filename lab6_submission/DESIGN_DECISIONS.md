# Design Decisions for CS3050 Lab 6

**Author:** Christian Streby
**Date:** December 2025
**Assignment:** Lab 6 - Time-Window Constrained and Priority-Based Routing

---

## Overview

So this doc basically explains the choices I made while coding up the routing algorithms. Not gonna lie, this took way longer than I expected and I hit a bunch of walls along the way, but I think I got it working in the end.

---

## Task 1.1: Time-Window Constrained Routing

### Why I Went with Modified Dijkstra

I decided to use Dijkstra's algorithm as my base and just tweak it to handle the time windows. Here's why:

- Dijkstra is pretty much guaranteed to find the shortest path that works with the constraints
- I already know how it works from notes, so less chance of screwing it up
- It's fast enough for the graphs we're dealing with
- The priority queue makes it pretty efficient

I thought about using A* with the haversine heuristic since that would be faster, but honestly that felt like overkill for this assignment. I was already struggling enough with getting the time windows working - didn't need to add another layer of complexity. Dijkstra gets the job done and I can always add A* later if I want.

---

### The State Space Thing (This Took Forever to Figure Out)

This was probably the trickiest part to wrap my head around. In regular Dijkstra, you just track which nodes you've visited. I initially tried doing the same thing and it kept giving me wrong answers on the test cases. Spent like 2 hours staring at my code wondering why it wasn't working.

Finally realized: with time windows, **when** you arrive at a node matters just as much as which node you're at.

So I changed the state from just `node_id` to `(node_id, arrival_time)`.

```python
@dataclass
class State:
    node_id: int
    arrival_time: float
    distance: float
```

**Why this matters:**
- If you arrive at node 3 at time 20, you might be able to reach node 4
- But if you arrive at node 3 at time 50, node 4's time window might be closed
- These are genuinely different situations even though you're at the same node

Took me a while to get this, but once it clicked, the rest made more sense. Still kicked myself for not seeing it earlier though.

**AI Usage Note:** After struggling with this for a while, I asked ChatGPT "In Dijkstra's algorithm with time window constraints, what should the state space be?" It suggested tracking (node, time) pairs, which clicked with what the hints were saying. I didn't use its code - just took the concept and implemented it myself in my own way. Understanding WHY the state space needs to include time was the key insight I needed.

---

### Waiting Policy (Had to Debug This Multiple Times)

So what happens if you arrive somewhere too early? Like you get to a delivery location at 8 AM but they don't open until 9 AM?

**My decision:** Just wait there until the window opens.

This makes sense in real life - delivery trucks wait all the time. And from a coding perspective:
- It makes more paths work (which is good)
- Waiting doesn't break any rules
- It doesn't mess up the distance calculations since you're not traveling

I could've made the algorithm reject early arrivals, but that seemed unnecessarily strict.

```python
if next_arrival < node.earliest:
    actual_arrival = node.earliest  # Wait until window opens
else:
    actual_arrival = next_arrival
```

**Bugs I hit here:**
- At first I wasn't updating the arrival time when waiting, so my path reconstruction was completely broken
- Had to add a bunch of print statements to figure out what was happening
- Eventually realized I needed to track the *actual* arrival time (after waiting) not the original arrival time

---

### Pruning to Avoid Infinite States (This Was a Mess)

One problem I ran into early on was that the state space could blow up - you could potentially visit the same node at tons of different times. My first implementation was running forever on even small graphs.

**First attempt:** I tried tracking every `(node_id, arrival_time)` pair in a visited set. This worked but was super slow and used way too much memory.

**Second attempt:** Just track visited nodes like normal Dijkstra. This broke everything because it would skip valid paths.

**Final solution:** Track the best (earliest) arrival time at each node.

```python
best_arrival = {}  # node_id -> earliest arrival time

if node_id in best_arrival and arrival_time > best_arrival[node_id]:
    continue  # Skip this - we already got here earlier
```

**Why this works:**
- Getting somewhere earlier is always at least as good as getting there later
- You can always wait if you're early, but you can't go back in time if you're late
- This keeps the algorithm from exploring basically the same path over and over

This pruning made a huge difference in performance. Went from timing out on test cases to running instantly.

**AI Usage Note:** When my algorithm was timing out, I asked ChatGPT "Why would tracking all (node, time) states cause performance issues?" It explained the state explosion problem and suggested tracking only the best arrival time per node. I had to figure out myself why "earliest arrival" was the right metric (because you can always wait, but can't go back in time). Implemented the pruning logic myself based on understanding this principle.

---

### Path Reconstruction (More Debugging)

To actually reconstruct the path at the end, I needed parent pointers. But since states include both node and time, I had to track both:

```python
parent = {}  # (node_id, arrival_time) -> (parent_node, parent_arrival_time)
parent[(next_node, actual_arrival)] = (current_node, arrival_time)
```

Yeah, this uses more memory than just tracking `parent[node] = previous_node`, but it's necessary to get the right path back.

**Issue I ran into:** My first version would sometimes reconstruct paths that jumped around weirdly because I was using the wrong arrival times as keys. Had to carefully trace through the algorithm to make sure I was consistently using `actual_arrival` (after waiting) not `next_arrival` (before waiting).

**AI Usage Note:** I showed ChatGPT my buggy path reconstruction output (without the code) and asked "Why would a path reconstruction produce [1,3,2,4] when the correct path should be [1,2,3,4]?" It asked me questions about how I was tracking parent pointers, which helped me realize I was mixing up the arrival times in my dictionary keys. Fixed it myself once I understood the issue.

---

### When There's No Valid Path

If the algorithm can't find a feasible path, I didn't want to just return "nope, can't do it." That's not helpful at all.

**What I did instead:**
1. Run regular Dijkstra ignoring time constraints to find the shortest path
2. Simulate walking that path and see which time windows get violated
3. Report exactly which nodes have problems and by how much

**Example output:**
```
Node 3: arrives at 45.00, but latest allowed is 40.00 (too late by 5.00)
Node 4: arrives at 70.00, but latest allowed is 60.00 (too late by 10.00)
```

This way you can actually see what's wrong - maybe node 3's time window needs to be relaxed, or maybe you need to start earlier.

This part was easier to implement since I could basically copy my Dijkstra code and just remove the time window checks.

---

### Suggesting Alternatives

Along the same lines, when there's no feasible path, I also suggest the shortest distance path and show what constraints it violates. Basically giving the user the "closest" option to what they asked for, even if it doesn't perfectly satisfy the requirements.

This seemed more useful than just saying "can't help you, good luck."

---

## Task 1.2: Priority-Based Multi-Destination Routing

### The Greedy Approach

For the multi-destination routing with priorities, I went with a greedy algorithm:
- Visit ALL HIGH priority destinations first
- Then ALL MEDIUM priority destinations
- Then ALL LOW priority destinations
- Within each priority level, just go to whichever one is closest

**Why greedy?**
- It's simple and makes sense intuitively
- It guarantees you respect priorities (no violations)
- It's fast enough - O(D² × V log V) where D is the number of destinations
- It works well for real delivery scenarios

I looked at some other options:

**Traveling Salesman Problem (TSP) approach:**
- Would find the absolute best tour
- But I was told that it wasnt this type of problem.

**Threshold-based swapping:**
- Could let you violate priorities if the distance savings are big enough
- Might implement this later but wanted to keep it simple for now
- Already spent enough time debugging the time-window stuff

**Weighted scoring:**
- Could combine distance and priority into one score
- Hard to pick the right weights though
- Seemed like it would be confusing


Decided that greedy was the easiest to get working.

---

### Strict Priority Order

I made the algorithm strictly enforce priority order - you visit every single HIGH priority node before touching any MEDIUM priority nodes, and so on.

```python
for priority_level in ["HIGH", "MEDIUM", "LOW"]:
    while unvisited_nodes_in_level:
        visit_nearest_in_level()
```

This seemed like what you'd actually want from "priority routing." If something is HIGH priority, it should get done first.

Within each priority level, I just pick the nearest unvisited destination. This is greedy and doesn't guarantee the optimal order within a level, but it keeps things reasonable.

---

### The Threshold Parameter

The function signature includes a `threshold` parameter, but I'm not really using it in the basic implementation. It's there as a placeholder for future work.

The idea would be:
- Calculate the distance with strict priority ordering
- Calculate the optimal distance ignoring priorities
- If the difference is bigger than the threshold, maybe allow some priority violations

But for now I just wanted to get the basic version working correctly. The framework is there if I want to add this later. 

---

### Violation Tracking

Even though my greedy algorithm shouldn't create any priority violations, I still track and report them. This is mostly defensive programming - if there's a bug in my code, this will catch it.

```python
priority_order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
last_priority = -1

for node in route:
    if node in destinations:
        current_priority = priority_order[destinations[node]]
        if current_priority < last_priority:
            violations.append(f"Priority violation at {node}")
```

It's also useful for testing. Saved me once when I had a bug where I was removing nodes from the wrong list.

---

### Finding the Shortest Paths (Initial Bug)

For each segment (getting from one destination to the next), I just run Dijkstra again.

**Process:**
- I'm at location A
- Find shortest path from A to each unvisited destination in my current priority level
- Pick the nearest one
- Go there
- Repeat

This runs Dijkstra a bunch of times (O(D² × V log V) total), but for a reasonable number of destinations it's totally fine. I thought about pre-computing all pairs of shortest paths, but that's O(V³) which would be worse unless you have a ton of destinations.

**Bug I had:** At first I was modifying the priority_group list while iterating over it, which caused it to skip destinations. Took me 30 minutes to figure out why it was only visiting half the locations. Changed it to use `priority_group.remove(best_node)` after finding the best one instead of trying to be clever with the loop.

**AI Usage Note:** I asked ChatGPT "In Python, if I'm iterating over a list and removing items during iteration, what happens?" It explained the iteration skip problem and suggested either iterating over a copy or using remove() after the loop. I chose the remove() approach since it was cleaner for my use case.

---

## Implementation Trade-offs

### Memory vs Speed

I generally chose correctness and readability over squeezing out every bit of performance:

**State space storage:**
- Storing `(node_id, arrival_time)` uses more memory than just tracking nodes
- But it's necessary for the algorithm to work correctly
- Necessary

**Parent pointers:**
- Also use more memory with the arrival times
- But you need them to reconstruct the path
- Necessary

**Best arrival tracking:**
- Dictionary that tracks earliest arrival at each node
- Pretty small memory footprint (one entry per node)
- The pruning benefit is worth it

### Code Clarity vs Performance

I tried to write code that's easy to understand, even if it's not maximally optimized:

**Helper function for Dijkstra:**
- I could've copy-pasted the Dijkstra code in multiple places
- Instead I made it a helper function
- Cleaner code, basically zero performance difference
- Made debugging easier too since I only had to fix bugs in one place

**Path reconstruction:**
- Could use fancier data structures to look up edge distances in O(1)
- Just loop through the edges instead - simple and works fine
- Path reconstruction only happens once at the end anyway

Honestly after spending like 6 hours debugging the time-window stuff, I just wanted code that I could read and understand later.

---

## Testing Strategy

### How I Set Up the Tests

**Test 1 - Feasible Path:**
- Time windows that are wide enough that a valid path exists
- Makes sure the basic algorithm works
- This was passing pretty early on

**Test 2 - Infeasible Path:**
- Time windows that make it impossible to reach the goal
- Tests that the violation detection works
- Had to implement the violation reporting to get this working

**Test 3 - Shortest Path Breaks Constraints:**
- The shortest distance path would violate time windows
- The algorithm has to choose a longer path that respects the constraints
- This is the interesting case - makes sure we're actually checking constraints
- This one failed for like 2 hours before I fixed the pruning logic

### Test Data Design

I kept the test graphs small (4-6 nodes) so I could manually check if the output makes sense. Mixed up the time windows - some tight, some loose - to test different scenarios.

Drew out the graphs on paper a couple times to verify the expected paths.

**AI Usage Note:** I asked ChatGPT "What are good test cases for time-window constrained pathfinding?" to get ideas. It suggested testing: feasible paths, infeasible paths, cases where shortest distance violates constraints, and tight vs loose windows. I used these categories but created my own specific test data that made sense for my implementation.

---

## Complexity Analysis

### Dijkstra with Time Windows

**Time:** O((V + E) log V) with pruning

The state space could theoretically blow up because of the time component, but the best-arrival pruning keeps it under control. In practice it runs about as fast as regular Dijkstra.

**Space:** O(V)
- Priority queue and tracking structures all scale with number of nodes

**AI Usage Note:** I asked ChatGPT "What's the complexity of Dijkstra with an expanded state space?" to verify my analysis was correct. It confirmed O((V+E) log V) if you prune properly, which matched what I was seeing. But I had to figure out myself WHY my pruning strategy kept it from exploding to O(VW log VW) where W is the window range.

### Priority Routing

**Time:** O(D² × (V + E) log V)
- D destinations, running Dijkstra up to D times to find each next destination
- Totally fine for reasonable D

**Space:** O(V + D)
- Graph plus the route we're building

---

## Wrap-Up

The main goals I had:
1. Make the time-window routing work correctly (modified Dijkstra)
2. Make priority routing respect the priority order (greedy approach)
3. Give useful error messages when things don't work
4. Keep the code readable

I think I hit those goals. The code isn't the most optimized thing ever, but it works correctly and you can actually understand what it's doing.

**Biggest lessons learned:**
- State space matters more than I thought - the `(node_id, arrival_time)` insight was crucial
- Pruning is essential when your state space expands
- Print statements are your friend when debugging graph algorithms
- Test on small graphs first, then scale up
- Sometimes greedy is good enough - don't overcomplicate things

Total time spent: probably 7-8 hours including debugging, testing, and writing this doc. Most of that was figuring out the state space thing and fixing pruning bugs.

---


## AI Tool Usage Summary

**What I used AI for:**
- Asked ChatGPT about state space representation concepts when I was stuck on how to handle time windows
- Got explanations about state explosion problems and pruning strategies
- Asked about Python list iteration behavior when I hit that bug
- Used it to verify my complexity analysis was on the right track
- Asked for test case ideas (feasible/infeasible paths, constraint violations)

