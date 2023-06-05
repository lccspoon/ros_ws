#ifndef _BUBBLE_SORT_H
#define _BUBBLE_SORT_H

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class BubbleSort 
{

    private:
    double data_record=0;
    int count=0;

    public:

    MatrixXd sort(MatrixXd matrix) //lcc  传入一个固定维度的矩阵，返回排序后的矩阵
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

    double sort_continuet(double data_in)  //lcc 一直传入数据，返回最大的数
    {

        // if( fabs(data_record-data_in)<=0.1 * 0.01 ) 
        // {
        //     data_record=data_record;
        //     // count++;
        // }
        // else 
        if(data_record>data_in) 
        {
            data_record=data_record;
            // count=0;
        }
        else
        {
            data_record=data_in;
            // count=0;
        }
        // printf("count:%d\n",count);
        return data_record;       
    }

    // int ret_sort_continuet_max_flag(void)
    // {
    //     int fff=0;
    //     if(count>=20)
    //     {
    //         fff=1;
    //     }    
    //     return fff;
    // }

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