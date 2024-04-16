#include "node.hpp"

bool Node::operator==(const Node& n) const{
    return x_ == n.x_ && y_ == n.y_;
}

bool Node::operator!=(const Node& n) const{
    return !operator==(n);
}

bool Node::compare_cost::operator()(const Node& n1, const Node& n2) const{
    return (n1.g_ + n1.h_ > n2.g_ +n2.h_) || ((n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_));
};

namespace math{
    
    double euclidean_distance(const Node &n1, const Node &n2)
    {
        return std::hypot(n1.x_ - n2.x_, n1.y_ - n2.y_);
    }
}