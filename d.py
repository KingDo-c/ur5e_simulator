import sympy as sp

# Define symbols for the variables
q, d0, a1, a2, d3, d4, d5 = sp.symbols('q d0 a1 a2 d3 d4 d5')

# Define each matrix
M1_0 = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, d0],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

M2_1 = sp.Matrix([
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

M3_2 = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, a1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

M4_3 = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, a2],
    [0, 0, 1, d3],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

M5_4 = sp.Matrix([
    [0, 1, 0, 0],
    [0, 0, 1, d4],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

M6_5 = sp.Matrix([
    [0, 0, 1, d5],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
]) * sp.Matrix([
    [sp.cos(q), -sp.sin(q), 0, 0],
    [sp.sin(q), sp.cos(q), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Calculate the final transformation matrix by multiplying all matrices
final_transformation_matrix = M1_0 * M2_1 * M3_2 * M4_3 * M5_4 * M6_5
final_transformation_matrix.simplify()
