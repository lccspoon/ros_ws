#ifndef _BUBBLE_SORT_H
#define _BUBBLE_SORT_H

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class BubbleSort 
{
public:
        MatrixXd sort(MatrixXd matrix) 
        {
            int rows = matrix.rows();
            int cols = matrix.cols();

            for (int i = 0; i < rows; ++i) {
                
                
                for (int j = 0; j < cols - 1; ++j) {
                    for (int k = 0; k < cols - j - 1; ++k) {
                        if (matrix(i, k) > matrix(i, k + 1)) {
                            std::swap(matrix(i, k), matrix(i, k + 1));


                            
                        }
                    }
                }
            }

            return matrix;
        }
};

// int main() {
//     MatrixXi matrix(3, 3);
//     matrix << 5, 2, 9,
//               1, 8, 4,
//               7, 6, 3;

//     std::cout << "Original matrix:\n" << matrix << std::endl;

//     BubbleSort bubbleSort;
//     MatrixXi sortedMatrix = bubbleSort.sort(matrix);

//     std::cout << "Sorted matrix:\n" << sortedMatrix << std::endl;

//     return 0;
// }



#endif