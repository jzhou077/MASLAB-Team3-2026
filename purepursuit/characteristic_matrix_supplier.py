from math.matrix import Matrix

bezier_matrices = dict()
initialized = False

# Caches commonly used bezier curves
def initialize():
    if not initialized:
        get_bezier_characteristic_matrix(2)
        get_bezier_characteristic_matrix(3)

    initialized = True

def get_bezier_characteristic_matrix(degree):
    if degree not in bezier_matrices:
        bezier_matrices[degree] = generate_bezier_characteristic_matrix(degree)

    return bezier_matrices[degree]

def generate_bezier_characteristic_matrix(degree):
    output = Matrix(arr=generate_pascal_triangle(degree + 1))

    sampledRow = output.get_row(output.row_count - 1) # last row
    i = 1
    while i < output.row_count - 1:
        j = 0
        while j <= i:
            output.matrix[i][j] = output.matrix[i][j] * abs(sampledRow[i])
            j += 1
        i += 1
    
    return output

def generate_pascal_triangle(layers):
    output = [[0 for _ in range(layers)] for _ in range(layers)]

    prevSign = 1
    for i in range(layers):
        output[i][0] = prevSign
        output[i][i] = 1
        prevSign *= -1

    i = 2
    while i < len(output):
        j = 1
        while j < i:
            output[i][j] = output[i-1][j-1] - output[i-1][j]
            j += 1
        i += 1

    return output
