---
title: 'Notes on Optimizing Compilers'
date: 2024-09-15
permalink: /posts/2024/optimizing-compilers/
---

Notes from [15-745: Optimizing Compilers](https://www.cs.cmu.edu/~15745/).

## Lecture 1: Overview of Optimizations

Three-address code
```
A := B op C
A := unaryop B
A := B
GOTO s
IF A relop B GOTO s
CALL f
RETURN
```

Basic block
- sequence of 3-addr statements
- only first statement can be reached from outside the block (no branches into middle)
- all statements are executed consecutively if first one is executed (no branches out or halts except maybe at end of block)
- are maximal
- optimizations within a basic block are local optimizations

Flow graph
- Nodes: basic blocks
- Edges: \\(b_i \rightarrow b_j\\) if there is a branch from \\(b_i\\) to \\(b_j\\). Also works if \\(b_j\\) physically follows \\(b_i\\) which does not end in an unconditional goto.

Optimization types:
- local: within bb -- across instrs
- global: within a flow graph -- across bbs
- interprocedural analysis: within a program -- across procedures (flow graphs)

Local
- common subexpression elimination
- constant folding or elimination
- dead code elimination

Global
- Global versions of local
  - global common subexpression elimination
  - global constant propagation
  - dead code elimination
- Loop opts
  - reduce code to be executed in each iter
  - code motion
  - induction var elimination
- Other control structures
  - Code hoisting: eliminate copies of idential code on parallel paths in flow graph to reduce code size

Induction Variable Elimination
- Loop indices are induction var
- Linear function of loop indices are also induction var
- Analysis: detection of induction var
- opts
  - strength reduction: replace mult with add
  - elim loop index: replace termination by tests on other induction vars

Loop invariant code motion
- computation is done within loop and result is same as long as we keep going around the loop
- move computation outside of loop

Machine dependent optimizations
- regalloc
- instr schd
- memory hierarchy optz

## Lecture 2: Local Optimizations
Outline
- bb/flow graphs
- abstr 1: dag
- abstr 2: value numbering
- phi in ssa

Partitionining into BBs
- identify leader of eahc bb. 
  - first instr
  - target of a jump
  - any instr after a jump
- bb starts at leadr & ends at instr immediately before a leader or last instr

Common subexpression elimination
- array expressions
- field access in records
- access to parameters

CSE cont:
- consider Parse Tree for `a+a * (b-c) + (b-c) * d`
- notice that subtrees `b-c` and `a` are duplicated
- turn parse tree to expression DAG

DAG bad:
- worked for one statment but not so good for multiple statments

Alternative: Value Numbering Scheme
- var2value where each value has its own number
- common subexpression means same value number
- `r1 + r2 => var2value(r1) + var2value(r2)`
```
VALUES : (var, op, var) -> var = {}
BIN_VALUES : (var, op, var) -> var = {}

for dst, src1, op, src2 in instrs:
  val1 = var2value(src1)
  val2 = var2value(src2)
  if val1, op, val2 in BIN_VALUES:
    print(f`{dst} = {BIN_VALUES[src1, op, src2]}`)
  else:
    tmp = newtemp()
    BIN_VALUES[val1, op, val2] = tmp
    print(f`{tmp} = {VALUES[src1]} {op} {VALUES[src2]}`)
    print(f`{dst} = {tmp}`)
  VALUES[dst] = tmp

# uhhh im sort of lost, above is wrong
```

phi functions in SSA
- appears in llvm
- copy prop
  - for a given use of X
    - are all reaching definitions of X:
      - copies from same variable: eg, X = Y
      - where Y is not redefined since that copy?
    - if so, substitute X for Y
- Static single assignments: IR where every variable is assigned a value at most once in the program rext
- Easy for basic block
  - For instr:
    - LHS: assign to fresh version of variable
    - RHS: use most recent version of variable
- What about joins in CFG?
```
c = 12
if (i) {
  a = x + y
  b = a + x
} else {
  a = b + 2
  c = y + 1
}
a = c + a
```
to 
```
c1 = 12
if (i) {
  a1 = x + y
  b1 = a1 + x
} else {
  a2 = b + 2
  c2 = y + 1
}
a4 = c? + a?
```
- use notational convention: phi function
```
a3 = phi(a1,a2)
c3 = phi(c1,c2)
b2 = phi(b1,b)
b3 = c3 + a3
```

## Lecture 3: Overview of LLVM Compiler
LLVM Compiler Infrastructure
- provides resable components for building compilers
- reduces time/cost
- build static compilers, jits, trace-based optz, etc
LLVM Compiler Framework
- e2e compilers using LLVM infra
- C and C++ are robust and aggressive
- Emit C code or native code

Parts
- LLVM Virtual Instructio Set
- ...

System:
- front end turns C/C++/Java/... into LLVM IR. One front end is Clang
- LLVM optimizer opterates on LLVM IR in series of passes
- Back end turns LLVM IR into machine code

Analysis passes do not change code

LLVM instruction set
- RISC-like three address code
- infinite virtual register set in SSA form
- Simple, low-level control flow constructs
- Load/store instructions with typed-pointers

```
for (int i = 0; i < N; i++) {
  Sum(&A[i], &P);
}
```

```
loop:   ; prefs = %bb0, %loop
  %i.1 = phi i32 [ 0, %bb0 ], [ %i.2, %loop ]
  #AiAddr = getelementptr float* %A, i32 %i.1
  call void @Sum(float %AiAddr, %pair* %P)
  %i.2 = add i32 %i.1, 1
  %exitcond = icmp eq i32 %i.1, %N
  br i1 %exitcond, label %outloop, label %loop
```

- explicit dataflow through SSA
- explicit cfg, even for exceptions
- explicit language independent type-information
- explicit typed pointer arithmetic
  - preserve array subscript and struture indexing

Type system:
- Primitives: label, void, float, integer (incld arbitrary bitwidth integers i1, i32, ...)
- Derived: pointer, array, structure, function
- No high-level types

Stack allocation is explicit in LLVM
```
int caller() {
  int T;
  T = 4;
  ...
}

int %caller() {
  %T = alloca i32
  store i32 4, i32* %T
  ...
}
```

LLVM IR is almost all nested doubly-linked lists
```
Function &M = ...
for (Function::iterator I = M->begin(); I != M->end; ++I) {
  BasicBlock &BB = I;
  ...
}
```

- Modules contains functions and globals (unit of compilation/analysis/optimization)
- Functions contain basic blocks and arguments
- Basic blocks contain instructions
- Instructions contain opcodes and vector of operands

Pass Manager
- ImmutablePass: doesn't do much
- LoopPass
- RegionPass: process single-enty, single-exit regions
- ModulePas
- CallGraphSCCPass: bottom up on the call graph
- FunctionPass
- BasicBlockPass: process basic blocks

Tools:
- llvm-as: .ll (text) to .bc (binary)
- llvm-dis: .bc to .ll
- llvm-link: link multiple .bc files
- llvm-prof: profile output to human readers
- opt: run one or more optz on bc

Aggregate tools:
- bugpoint
- clang

mostly use clang, opt, llvm-dis

## Lecture 4: Dataflow Analysis

Reaching definitions
- every assignment is a definition
- a definition reaches a point p if there exists a path from the point immediately following d to p such that d is not killed along that path

Live variable analysis:
- a variable v is live at point p if the value of v is used along some path in the flow graph starting at p
- otws the variable is dead

For each basic block, track uses and defs

reaching definitions vs live variables:
- sets of definitions vs variables
- forward vs backward
- ...

Reaching:
- consider `a = b + c`. at what previous statments did variable c acquire a value that can reach line 10?
Live range:
- starting from definition of c, go to the next definition of variable or end of scope that variable c exists.