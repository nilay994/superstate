- ACADO is the best thing - however not when codegening for my case
- Mayer terms are not the standard least squares optimization in blackbox codegen that seems to be supported. `[ACADO] Error: Only standard LSQ objective supported for code generation`
- Infact, not even the horizon thing is a problem. Lets say I fixed a constant horizon, I still can't optimize for saturated control inputs by adding maximizeMayerterm. 
- No examples in codegen use mayer terms.
- profiling the code on Bebop yeilds 2ms using my dynamic equations, 8ms using theirs. 
- Profiling on pc takes 200 microseconds!! (too good)
- maybe lets move to profiling the qpOASES code, and lets come up with a standard method of performing time optimization.
- another contribution in thesis - use secant method for time optimization - of your own quadprog that does so well.


