#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace fsHelper
{
template <class T>
void readOrDefault(const FileNode &node, T &x, const T &default_value = T())
{
    if (node.empty())
        x = default_value;
    else
        node >> x;
};
}; // namespace fsHelper
