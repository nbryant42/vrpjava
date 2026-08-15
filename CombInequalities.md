# Comb Inequalities

[Lysgaard et al.](https://www.lancaster.ac.uk/staff/letchfoa/articles/2004-cvrp-exact.pdf) defines a set of strengthened
comb inequalities that can be applied to the CVRP.

This document discusses a possible (not yet implemented in code) MILP-based exact approach to cut-separation for a
limited subset of these. We'll start simple by restricting the search space to two non-intersecting teeth, and require
that the teeth satisfy $T_i ∩ T_j ∩ H = ∅$, thereby avoiding a disjunctive constraint.

## Problem definition

We define a *comb* to be a vertex set $H ⊂ V_c$, called the handle, and $t ≥ 2$ other vertex sets $T_1,... ,T_t$,
called teeth, such that:

* $H ∩ T_j \neq ∅$ and $T_j \setminus H \neq ∅$ for $j = 1,... ,t;$
* for each pair $\{i, j\}⊂\{1,... ,t\}: Ti ∩ Tj ∩ H = ∅$ (this differs from Lysgaard et al. for simplicity's sake.)

Lysgaard et al. defines that for any set $S ⊂ V$, let $\tilde{r}(S)$ equal $r(S)$ if $0 \notin S$,
and $r(V \setminus S)$ otherwise.

Our choice to exclude the depot from teeth means that $\tilde{r}(S) = r(S)$ in the following, so we define the quantity:
$$
S(H, T_1,... ,T_t) := \sum_{j=1}^{t}(r(T_j ∩ H) + r(T_j \setminus H) + r(T_j))
$$

## Deriving a MILP formulation

We'll also replace $r(S)$ with $k(S)$.

To separate constraints, we need a way to measure the size of the violation. Given the inequality:

$$
x(δ(H )) + \sum_{j=1}^{t} x(δ(T_j)) ≥ S(H, T1,... ,Tt) + 1
$$

Which can be rewritten as:

$$
S(H, T1,... ,Tt) + 1 - x(δ(H )) - \sum_{j=1}^{t} x(δ(T_j)) \le 0
$$

If the constraint is violated, the size of the violation can be expressed as a positive number:

$$
S(H, T1,... ,Tt) + 1 - x(δ(H )) - \sum_{j=1}^{t} x(δ(T_j))
$$

We'll need decision variables for set membership. There are three minimal sets from which all others derive as set-union
relationships. These are:

* $H'$: The "core" of $H$, excluding all teeth intersections.
* $T_j ∩ H$
* $T_j \setminus H$

Respectively, when deciding whether a vertex $j$ is in a set, we will call those decision variables:

* $h'_j$
* $t_{0j}$
* $t_{1j}$

By various substitutions, we arrive at:

$$
\textrm{Maximize:}
$$
$$
1 - x(δ(H)) + \sum_{j=1}^{t}(k(T_j ∩ H) + k(T_j \setminus H) + k(T_j) - x(δ(T_j)))
$$

Along similar lines to [RCC-Sep](https://onlinelibrary.wiley.com/doi/10.1002/net.22183), we will also need to introduce
an $α + 1$ term for every $k(S)$ calculation. There are 3 of these terms, so we'll just number them from left to right
as $α_{0j}$, $α_{1j}$, and $α_{2j}$, which gets us to:

$$
\textrm{Maximize:}
$$
$$
1 - x(δ(H)) + \sum_{j=1}^{t}(α_{0j} + α_{1j} + α_{2j} - x(δ(T_j)) + 3)
$$

We're not quite there yet. If the two edge-crossing terms $x(δ(H))$ and $x(δ(T_j))$ correspond to sufficiently-large
sets of potentially-fractional decision variables $\{x_{ij}^*, \{i, j\} \in E\}$, we don't have a violation, but if most
of those variables are zero, we do; that is, every corresponding $x_{ij}^*$ set to 1 in the candidate solution reduces
the size of the violation. So it's understood that the function $x()$ includes a multiplication by the candidate
solution's decision variable.

More explicitly, along the lines of RCC-Sep, define variables $\gamma$ and $\gamma'$ for those two terms. Via
constraints, $\gamma_{ij}$ will be set to 1 if vertices $i$ and $j$ are both in $H$, or 0 otherwise. $\gamma'_{ijk}$
models the same condition for set $T_k$, leading to:

$$
\textrm{Maximize:}
$$
$$
1 - x(δ(H)) + \sum_{j=1}^{t}(α_{0j} + α_{1j} + α_{2j} - x(δ(T_j)) + 3)
$$

## Constraining $S(H, T_1,... ,T_t)$ to be odd

That 1 term is a problem. It must be 0 if $S(H, T_1,... ,T_t)$ is even. So introduce positive integer variable
$\beta$ such that $2\beta - \sum_{j=1}^{t}(α_{0j} + α_{1j} + α_{2j}) = 1$

## Status and next steps

This derivation is unfinished. Under the intended restriction $t = 2$, the final parity constraint is sound: because each
$k(S)$ is represented by $α + 1$, we have $S(H,T_1,T_2) = \sum α + 6$, so $S$ is odd exactly when $\sum α$ is odd.
The constraint should therefore be understood as excluding even-$S$ candidates, rather than as justification for simply
removing the $+1$ from the strengthened inequality. If $t$ is allowed to vary, the parity condition should instead be
written in its general form, $\sum α + 3t = 2\beta + 1$.

Before implementing this, clarify whether the teeth must be completely disjoint or merely cannot intersect inside the
handle. Then specify all variable domains, depot exclusions, non-emptiness and disjointness constraints, the membership
relationships for each tooth, the demand constraints defining every $α$, and the full linearized objective for the handle
and tooth boundary terms using the $\gamma$ variables. A small exhaustive checker should validate every generated cut on
small CVRP instances before the separator is integrated, and the restricted exact MILP approach should be benchmarked
against the published heuristic separation procedure.
