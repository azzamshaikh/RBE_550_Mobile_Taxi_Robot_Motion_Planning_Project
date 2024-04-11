#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <array>
#include <vector>
#include <map>

class Node{
    public:
        Node(int x=0, int y=0, double g=0.0, double h=0.0, int id=0, int parentID= 0);

        Node operator+(const Node& n) const;

        Node operator-(const Node& n) const;

        bool operator==(const Node& n) const;

        bool operator!=(const Node& n) const;

        static std::vector<Node> getMotion();

        struct compare_cost{
            bool operator()(const Node& n1, const Node& n2) const;
        };

        struct compare_coordinates{
            bool operator()(const Node& n1, const Node& n2) const;
        };

        void distance(const Node& goal);

    public:
        int x_, y_;
        double g_, h_;
        int id_, parentID_;

};

namespace math {
    double euclidean_distance(const Node& n1, const Node& n2);
    double angle(const Node& n1, const Node& n2);
}

#endif