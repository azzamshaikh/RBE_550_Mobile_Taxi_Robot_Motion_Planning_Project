#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <array>
#include <vector>
#include <map>

class Node{
    public:
        Node(int x=0, int y=0, double g=0.0, double h=0.0, int index=0, int parent= 0){
            x_ = x;
            y_ = y;
            g_ = g;
            h_ = h;
            index_ = index;
            parent_ = parent;
        }

        bool operator==(const Node& n) const;

        bool operator!=(const Node& n) const;

        struct compare_cost{
            bool operator()(const Node& n1, const Node& n2) const;
        };

    public:
        int x_, y_, index_, parent_;
        double g_, h_;
};

namespace math {
    double euclidean_distance(const Node& n1, const Node& n2);
}

#endif