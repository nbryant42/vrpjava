# Comb Inequalities

[Lysgaard, Letchford, and Eglese](https://www.lancaster.ac.uk/staff/letchfoa/articles/2004-cvrp-exact.pdf)
define strengthened comb inequalities for the CVRP. This note considers whether an exact MILP separator for a small
subfamily would be useful in this solver.

## Summary

The [original draft](CombInequalities.md) fixed the number of teeth at two and replaced the bin-packing value $r(S)$
with the rounded capacity value

$$
k(S) := \left\lceil \frac{q(S)}{Q} \right\rceil.
$$

That subfamily is valid, but it cannot strengthen the closure of the ordinary rounded-capacity inequalities (RCIs).
Every violated two-tooth inequality implies that at least one simpler RCI is still violated. An exact two-tooth MILP
separator would therefore be an unnecessarily expensive way to rediscover a capacity or connectivity cut.

The smallest potentially useful restriction has three teeth. The weakest practical starting point is narrower still:
three completely disjoint, two-customer teeth, each with one endpoint in the handle. These are 2-matching inequalities.
They have a much smaller formulation, a known polynomial-time exact separation algorithm, and a simple exhaustive
oracle for checking a first implementation.

This conclusion depends on using $k$, as this solver does. It does not rule out a two-tooth inequality based on the
exact bin-packing value $r$, but embedding exact bin packing inside set selection would be a substantially harder
problem than the proposal considered.

## Restricted definition and validity

Let $V_c=V\setminus\{0\}$ be the customer vertices, with depot $0$. For the restricted family, take a handle
$H\subset V_c$ and depot-free teeth $T_1,\ldots,T_t\subseteq V_c$. Require

* $A_j:=T_j\cap H\ne\varnothing$ and $B_j:=T_j\setminus H\ne\varnothing$ for every tooth; and
* the teeth to be pairwise disjoint.

Complete tooth disjointness is stronger than the condition in the paper, which permits certain intersections. It makes
the validity argument and any initial separator unambiguous. Define

$$
S(H,T_1,\ldots,T_t)
  := \sum_{j=1}^t \bigl(k(A_j)+k(B_j)+k(T_j)\bigr).
$$

When $S$ is odd, the strengthened comb inequality is

$$
x(\delta(H))+\sum_{j=1}^t x(\delta(T_j)) \ge S(H,T_1,\ldots,T_t)+1. \tag{1}
$$

The restriction remains valid with $k$. For each tooth, the three RCIs on $A_j$, $B_j$, and $T_j$, together with

$$
x(\delta(A_j))+x(\delta(B_j))
  = x(\delta(T_j))+2x(E(A_j:B_j)),
$$

give

$$
x(\delta(T_j))+x(E(A_j:B_j))
  \ge k(A_j)+k(B_j)+k(T_j). \tag{2}
$$

Every edge in $E(A_j:B_j)$ crosses the handle. Complete tooth disjointness makes these edge sets disjoint, so summing
(2) gives

$$
x(\delta(H))+\sum_{j=1}^t x(\delta(T_j)) \ge S.
$$

For an integer CVRP solution, every customer-set boundary has even value, since

$$
x(\delta(U))=2|U|-2x(E(U)).
$$

The left-hand side is therefore even. If $S$ is odd, it must be at least $S+1$, proving (1).

## Why two teeth add nothing beyond RCIs

For each tooth, let

$$
d_j:=k(A_j)+k(B_j)-k(T_j).
$$

For positive demands,

$$
\lceil a\rceil+\lceil b\rceil-\lceil a+b\rceil\in\{0,1\},
$$

so $d_j\in\{0,1\}$, and

$$
S=2\sum_{j=1}^t k(T_j)+\sum_{j=1}^t d_j. \tag{3}
$$

With $t=2$, odd $S$ means exactly one of $d_1,d_2$ is one. Consequently,

$$
S+1=2k(T_1)+2k(T_2)+2. \tag{4}
$$

But the RCIs for $H,T_1,T_2$ imply

$$
x(\delta(H))+x(\delta(T_1))+x(\delta(T_2))
  \ge 2k(H)+2k(T_1)+2k(T_2)
  \ge S+1, \tag{5}
$$

because $H\ne\varnothing$ gives $k(H)\ge1$. Thus every fixed-two-tooth $k$-comb is dominated by three ordinary
RCIs.

This matters even though the current implementation does not explicitly store every RCI. When its subproblem finishes,
`RccSepCVRPCuts` is designed to separate the $k\ge2$ cases exactly, while `SubtourCuts` finds disconnected $k=1$
cases but is not a full minimum-cut separator. If a two-tooth comb were violated, the direct response would be to
separate the violated member of

$$
\{\delta(H)\ge2k(H),\ \delta(T_1)\ge2k(T_1),\ \delta(T_2)\ge2k(T_2)\},
$$

including an exact minimum-cut routine for the $k=1$ case if necessary. That would be cheaper and at least as strong.

## The smallest non-redundant case

Equation (3) also characterizes when more teeth can help. Let $D=\sum_j d_j$. The comb right-hand side is

$$
2\sum_j k(T_j)+D+1,
$$

whereas the RCIs for the handle and teeth give at least

$$
2\sum_j k(T_j)+2k(H).
$$

The comb can be stronger than those RCIs only when

$$
D+1>2k(H). \tag{6}
$$

For $t=3$, odd parity gives $D\in\{1,3\}$. Condition (6) is possible only when

$$
D=3\quad\text{and}\quad k(H)=1.
$$

The narrowest example gives every tooth two vertices: one in $H$, one outside $H$. If each tooth's combined demand
is at most $Q$, all three terms associated with a tooth have $k=1$. Then $S=9$, and (1) becomes

$$
x(\delta(H))+\sum_{j=1}^3x(\delta(T_j))\ge10. \tag{7}
$$

This is the ordinary three-tooth comb, or 2-matching inequality. Requiring $q(H)\le Q$ and
$q(T_j)\le Q$ is not needed for validity of the ordinary comb, but it focuses separation on candidates not already
dominated by the obvious RCIs.

## A small exact MILP for the three-tooth case

Although Padberg and Rao give a polynomial-time exact separator for 2-matching inequalities, a fixed-three-tooth MILP
would be a reasonable educational prototype.

For customers $i=1,\ldots,n$, introduce binary $h_i$ indicating membership in $H$. For each customer edge
$e=\{i,j\}$, initially introduce binary $f_e$ indicating that the edge is one of the three two-vertex teeth. Impose

$$
\sum_e f_e=3,
$$

$$
\sum_{e\ni i}f_e\le1 \qquad(i=1,\ldots,n),
$$

and, for $e=\{i,j\}$,

$$
f_e\le h_i+h_j,
\qquad
f_e\le2-h_i-h_j.
$$

Thus the selected tooth edges form a matching and each crosses the handle. To target the least RCI-dominated cases,
omit $f_{ij}$ when $q_i+q_j>Q$, and impose

$$
\sum_i q_i h_i\le Q.
$$

For each customer edge, a continuous variable $z_{ij}\in[0,1]$ can model $h_i h_j$ with the usual constraints

$$
z_{ij}\le h_i,
\qquad z_{ij}\le h_j,
\qquad z_{ij}\ge h_i+h_j-1.
$$

Given the current relaxation $x^*$, maximize the violation of the standard 2-matching form of (7):

$$
\max
\left(
  \sum_{1\le i<j\le n}x^*_{ij}z_{ij}
  +\sum_{1\le i<j\le n}x^*_{ij}f_{ij}
  -\sum_{i=1}^n h_i
  -1
\right). \tag{8}
$$

A positive objective identifies the violated cut

$$
x(E(H))+x(F)\le |H|+1, \tag{9}
$$

where $F$ is the selected three-edge matching. Equations (7) and (9) are equivalent under the customer degree
equations. Variables $z_{ij}$ whose $x^*_{ij}=0$ do not affect the objective and may simply be omitted.

For a fixed integer handle, the feasible $f$ variables form a cardinality-three matching polytope on a bipartite graph,
so they can in principle be continuous; a separate matching step can recover an integral optimum in the event of a
tie. Keeping them binary initially makes cut extraction simpler. Either way, this model uses only one set of internal-
edge product variables and is structurally clearer than selecting arbitrary tooth pieces. It still has potentially
$O(n^2)$ edge variables, so it should be treated as an experiment, not enabled by default.

## If arbitrary teeth are revisited

For fixed $t\ge3$ and completely disjoint teeth, a full set-selection MILP can use binary categories for each customer:

* the handle core $H\setminus\bigcup_jT_j$;
* each intersection $A_j=T_j\cap H$;
* each outside part $B_j=T_j\setminus H$; and
* none of the selected sets.

At most one category may be selected per customer, and every $A_j$ and $B_j$ must be nonempty. If
$c_i$, $a_{ji}$, and $b_{ji}$ denote the first three kinds of membership, respectively, then the derived memberships
are $y^H_i=c_i+\sum_j a_{ji}$ and $y^{T_j}_i=a_{ji}+b_{ji}$. If
$k(U)=\alpha_U+1$, then $\alpha_U$ is a **nonnegative** integer, not a positive one. To represent $k$ exactly,
scale $Q$ and all demands to integers $\bar Q$, $\bar q_i$, and impose for every
$U\in\{A_j,B_j,T_j:j=1,\ldots,t\}$

$$
\bar Q\alpha_U+1\le\sum_i\bar q_i y^U_i
\le\bar Q(\alpha_U+1). \tag{10}
$$

The upper and lower demand bands matter. A one-sided RCC-Sep-style constraint relies on maximizing one $\alpha$; parity
can otherwise make a nonmaximal $\alpha$ attractive. The general parity equation is

$$
\sum_U\alpha_U+3t=2\beta+1. \tag{11}
$$

For $t=2$, the earlier equation $2\beta-\sum_U\alpha_U=1$ happened to enforce the same odd parity after shifting
$\beta$, but it did not make $\beta$ equal to $(S-1)/2$. Equation (11) states the intended relationship directly.

Finally, a variable $z^U_{ij}=y^U_i y^U_j$ marks an **internal** edge, not a boundary edge. With the same McCormick
constraints used above,

$$
x^*(\delta(U))=
\sum_i x^*_{0i}y^U_i+
\sum_{1\le i<j\le n}x^*_{ij}
\left(y^U_i+y^U_j-2z^U_{ij}\right). \tag{12}
$$

The complete objective is therefore

$$
1+\sum_U(\alpha_U+1)
-x^*(\delta(H))-\sum_{j=1}^t x^*(\delta(T_j)). \tag{13}
$$

This finishes the algebra that was missing from the original draft, but it also exposes the likely computational
problem: for $t=3$, the model has seven membership categories per customer and four sets' worth of edge-product
variables. Its search space is much larger than RCC-Sep's binary in/out selection, before accounting for tooth-label
symmetry.

## Verification and next steps

If comb separation is pursued, the safe order is:

1. Add a pure evaluator for (7)/(9), plus an exhaustive tiny-instance oracle over handles and three-edge matchings.
2. Validate every generated inequality against every feasible CVRP solution on tiny instances.
3. Compare the MILP optimum with the exhaustive separation optimum on tiny fractional degree-feasible points.
4. Add an exact minimum-cut separator for $k=1$ RCIs before attributing any gain to comb cuts.
5. Run the separator only at the root, behind an experimental parameter and a strict time budget. A timed-out separator
   may contribute validated feasible incumbent cuts, but failure to finish must never affect proof completeness.
6. Benchmark root bound, total nodes, cuts, and runtime. The paper reports no comb-bound improvement for EIL33 but does
   report one for EIL51, so EIL51 is the more plausible initial target.

The original fixed-two-tooth MILP should not be implemented. If the three-tooth 2-matching experiment shows no useful
bound improvement, the larger arbitrary-tooth MILP is unlikely to justify its cost.
