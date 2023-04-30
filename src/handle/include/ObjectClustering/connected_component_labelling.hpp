#ifndef CONNECTED_COMPONENT_LABELLING_H
#define CONNECTED_COMPONENT_LABELLING_H

#include <vector>
#include <queue>

#define IDX_OK(x,L) (x>=0) & (x < L)

namespace lidar {

class ConnectedComponentLabelling
{
public:
    static int label(std::vector<int>& outputLabelledImg, const int nRows, const int nCols,const std::vector<bool>& binaryImg)
    {
        std::vector<int> labelledImg(nRows * nCols, 0);
        outputLabelledImg = labelledImg;
        const int dy[8] = {1, 0, -1, 0, 1, -1, -1, 1};
        const int dx[8] = {0, 1, 0, -1, -1, -1, 1, 1};
        int label = 0;
        std::queue<int> qi, qj;

        for (int i = 0; i < nRows; i++)
        {
            for (int j = 0; j < nCols; j++)
            {
                unsigned int idx = i * nCols + j;
                if (binaryImg[idx] && outputLabelledImg[idx] == 0)
                {
                    outputLabelledImg[idx] = ++label;
                    qi.push(i); qj.push(j);
                    while (!qi.empty() && !qj.empty())
                    {
                        int ii = qi.front(), jj = qj.front();
                        qi.pop(); qj.pop();
                        for (int k = 0; k < 8; k++)
                        {
                            int y = ii + dy[k], x = jj + dx[k];
                            if ((IDX_OK(x, nCols) && IDX_OK(y, nRows)))
                            {
                                unsigned int idx2 = y * nCols + x;
                                if (binaryImg[idx2] && outputLabelledImg[idx2] == 0)
                                {
                                    outputLabelledImg[idx2] = label;
                                    qi.push(y); qj.push(x);
                                }
                            }
                        }
                    }
                }
            }
        }
        return label;
    }
};
}//namespace lidar

#endif
