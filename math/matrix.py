

class Matrix:
    def __init__(self, row_count=0, col_count=0, arr=None):
        if arr is None:
            self.row_count = row_count
            self.col_count = col_count
            self.matrix = [[0 for _ in range(col_count)] for _ in range(row_count)]
        else:
            self.matrix = arr
            self.row_count = len(self.matrix)
            self.col_count = len(self.matrix[0])

    def copy(self):
        output = Matrix(self.row_count, self.col_count)
        for i in range(self.row_count):
            output.set_row(self.get_row(i))
        
        return output

    def deep_copy(self, copyMatrix):
        output = [[] for _ in range(len(copyMatrix))]
        for i in range(len(copyMatrix)):
            output = copyMatrix[i].copy()
        
        return output

    def get_matrix(self):
        return self.matrix
    
    def get_row(self, row_index):
        return self.matrix[row_index].copy()

    def set_row(self, row_index, new_row):
        self.matrix[row_index] = new_row

    def get_col(self, col_index):
        output = [0.0 for _ in range(self.row_count)]
        for i in range(self.row_count):
            output[i] = self.matrix[i][col_index]

        return output

    def setMatrix(self, newMatrix):
        num_cols = len(newMatrix[0])
        for i in range(len(newMatrix)):
            if len(newMatrix[i]) != num_cols:
                return
        matrix = self.copy()

    def multiply(self, other):
        if other.row_count != self.col_count:
            raise ArithmeticError

        output = Matrix(self.row_count, other.col_count)

        for i in range(output.row_count):
            rowSample = self.get_row(i)

            for j in range(output.col_count):
                dpSum = 0
                colSample = other.get_col(j)

                for k in range(len(rowSample)):
                    if not (rowSample[k] == 0 or colSample[k] == 0):
                        dpSum += rowSample[k] * colSample[k]

                output.matrix[i][j] = dpSum
        
        return output