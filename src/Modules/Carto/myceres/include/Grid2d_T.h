#ifndef _GRID2D_H_
#define _GRID2D_H_
#include <vector>
#include <stdint.h>

struct Grid2D_T {
 public:
 Grid2D_T(const std::vector<uint16_t> &data,std::vector<double> &intTodouble,
         int row_begin,  int row_end,
         int col_begin,  int col_end,double max_x,double max_y,double resolution)
      : data_(data),intTodouble_(intTodouble),
        row_begin_(row_begin), row_end_(row_end),
        col_begin_(col_begin), col_end_(col_end),
        num_rows_(row_end - row_begin), num_cols_(col_end - col_begin),
        num_values_(num_rows_ * num_cols_), limits_max_x(max_x),limits_max_y(max_y),limits_resolution(resolution)
	{
    }

   void GetValue(int r, int c, double* f){
    int row_idx =
        std::min(std::max(row_begin_, r), row_end_ - 1) - row_begin_;
    int col_idx =
        std::min(std::max(col_begin_, c), col_end_ - 1) - col_begin_;

    int n = num_rows_ * col_idx + row_idx;
    *f =(double)(data_[n]);

	}

	double GetCorrespondenceCost(int r,int c) 
	{
		int index = c * num_rows_ + r;
		double 	value = 0.9;	
		if(index< data_.size())
		{
			if(data_[index] < intTodouble_.size())
			{
				if(data_[index] == 0)
				{
					return 0.9;
				}
				value = intTodouble_[data_[index]];
			}
		}
		return value;
	}


 public:
    const std::vector<uint16_t> &data_;
	std::vector<double> &intTodouble_;
    int row_begin_;
    int row_end_;
    int col_begin_;
    int col_end_;
    int num_rows_;
    int num_cols_;
    int num_values_;


    double limits_max_x;
    double limits_max_y;
	double limits_resolution;
};

#endif
